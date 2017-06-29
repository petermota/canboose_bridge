/* -------------------------------------------------------------
 *  Can bus bridge (PC <-> Can Bus)
 *  
 *  This is how it works:
 *  
 *  (1) All incoming messages from PC via USB will be validated
 *  and sent to CAN Bus  
 *  
 *  (2) All incoming messages from CAN Bus will be sent to PC 
 *  via USB interface
 *  
 *  There are no blocking I/O calls. No delay calls
 *  Everything is async or interrupt driven
 *  This is how we can achieve a high throughput
 *  
 *  Pedro Mota Gonzalez - peter.mota@gmail.com 
 *  January 2017
 * 
 *  -------------------------------------------------------------
 * 
 *  Format of CANMessages received from USB are in ASCII format.
 *  Hex numbers in ASCII
 *  Frames start with ':', end with ';'
 *
 *  Standard frame:
 *
 *  :ShhhhNd0d1d2d3d4d5d6d7;
 *
 *  for example: 
 *  
 *  :S4FE4Nd0d1;
 *
 *  Extended frame
 *
 *  :XhhhhhhhhNd0d1d2d3d4d5d6d7;
 *
 *  for example:
 *  
 *  :X7FE4AFE4Nd0d1d2d3d4d5d6d7;
 *
 *  S - Standard CAN frame
 *  X - Extended CAN frame
 *  hhhh -    2 byte header for Standard frames
 *  hhhhhhh - 4 byte header for Extended frames
 *  N - Normal frame
 *  R - Remote frame
 *  d0...d7 - Up to 8 data bytes
 */

#include <FlexCAN.h>

class AsyncReceiverClass : public CANListener {
  public:
    void printFrame(CAN_message_t &frame, int mailbox);
    bool frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller);

  private:
    char intToAsciiHex(uint8_t nibble);
};

void AsyncReceiverClass::printFrame(CAN_message_t &frame, int mailbox) {
  String result = "CANBus message -> ID: ";
  result.concat(String(frame.id, HEX));
  result.concat(" X:");
  result.concat(frame.flags.extended);
  result.concat(" R:");
  result.concat(frame.flags.remote);
  result.concat(" OvR:");
  result.concat(frame.flags.overrun);
  result.concat(" Len:");
  result.concat(frame.len);
  result.concat(" Data: ");
  for (int c = 0; c < frame.len; c++) {
    result.concat(String(frame.buf[c], HEX));
    result.concat(" ");
  }
  Serial.println(result);
}

char AsyncReceiverClass::intToAsciiHex(uint8_t nibble) {
  if (nibble >= 0 && nibble <= 9)
    return nibble + '0';
  else if (nibble >= 10 && nibble <= 15)
    return nibble + 'A' - 10;
  else
    return '0';
}

bool AsyncReceiverClass::frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller) {  
  // index to write in the array
  uint8_t index = 0;
  uint8_t array2send[29];

  // All messages start with :
  array2send[index++] = ':';
  String header = String(frame.id, HEX);
  uint8_t adding = header.length();
  if (frame.flags.extended) {
    array2send[index++] = 'X';
    while (adding++ < 8) array2send[index++] = '0';
  }
  else {
    array2send[index++] = 'S';
    while (adding++ < 4) array2send[index++] = '0';
  }
  for (uint8_t i = 0; i < header.length(); i++) array2send[index++] = header[i];
  frame.flags.remote ? array2send[index++] = 'R' : array2send[index++] = 'N';

  for (int c = 0; c < frame.len; c++) {
    array2send[index++] = intToAsciiHex((frame.buf[c] & 0xF0) >> 4);
    array2send[index++] = intToAsciiHex(frame.buf[c] & 0x0F);
  }
  array2send[index++] = ';';
  array2send[index++] = '\n';
  Serial.write(array2send, index);
  Serial.send_now();

  return true;
}

AsyncReceiverClass asyncReceiverCAN;

/* -------------------------------------------------------------
 *  Inicializacion del bus 0. Pines 3 y 4
 */
void setup(void) {
  // Teensy 3.2 USB speed is always 12 Mbit/s, so speed will
  // be ignored
  Serial.begin(9600);

  // Initialize CAN bus and attach asynchronous listener class
  Can0.begin(125000);  
  Can0.attachObj(&asyncReceiverCAN);

  // There are 16 mailboxes. This will be the configuration:
  // 0 - To receive normal frames
  // 1 to 14 - To receive extended frames
  // 15 - To send frames
  Can0.setNumTxBoxes(1);

  // We will receive only extended frames
  CAN_filter_t allPassFilter;
  allPassFilter.id=0;
  allPassFilter.ext=1;
  allPassFilter.rtr=0;
  for (int filterNum = 1; filterNum < 15; filterNum++) {
    Can0.setFilter(allPassFilter, filterNum); 
  }
  asyncReceiverCAN.attachGeneralHandler();
}


/* -------------------------------------------------------------
 *  Main event loop, nothing to do here
 */
void loop(void) {

}

/*
 * Buffer to store usb incoming message from computer
 */
#define EMERGENCY_LED 13
#define SIZE_USB_BUFFER 26
char usbBufferedInput[SIZE_USB_BUFFER];
int  usbIndex = -1;

/*
 * We will control timeouts from USB. A partial received 
 * message will have 50 milliseconds to receive the rest 
 * of the message. After this timeout partial received
 * message will be discarded
 */
#define USB_MESSAGE_TIMEOUT 50
unsigned long timeReceived = 0;

/*
 * Function to convert one Ascii Hexadecimal digit to 
 * numeric value
 */
uint8_t AsciiHexToInt(char c) {
  if (c >= 48 && c <= 57) {
    return c - '0';
  }
  else if (c >= 65 && c <= 70) {
    return c - 'A' + 10;
  }
  else if (c >= 97 && c <= 102) {
    return c - 'a' + 10;
  }
  else return 0;
}

/*
 * Validate USB message (all fields) and send it to
 * CAN Bus
 */

void processUSBMessage(char m[], int frameLength) {
  /* We will create a new message. But first we have to check everything is ok */
  CAN_message_t message = {0, 0, {0, 0, 0, 0}, 0};

  /* 99% of the time message will be extended */
  int normalBitPosition = 9;

  /* First bit should be X or S */
  if (m[0] == 'X') {
    message.flags.extended = true;
  }
  else if (m[0] == 'S') {
    normalBitPosition = 5;
    message.flags.extended = false;
  }
  else return;

  /* Bit 5 for S frame or 9 for X frame should be N or R */
  if (m[normalBitPosition] == 'N') {
    message.flags.remote = false;
  }
  else if (m[normalBitPosition] == 'R') {
    message.flags.remote = true;
  }
  else return;

  /* Now convert header to 16 or 32 bit number */
  int headerSize = normalBitPosition - 1;
  char dest[headerSize];
  for (int i = 1; i < headerSize + 1; i++) dest[i - 1] = m[i];
  message.id = 0;
  for (int i = 0; i < headerSize; i++) {
    message.id += AsciiHexToInt(dest[i]) << 4 * (headerSize - 1 - i);
  }

  /* Nibbles should be even. We cannot have 1.5 bytes for example  */
  int howManyNibbles = frameLength - normalBitPosition - 1;
  if (howManyNibbles % 2 == 0) {
    message.len = howManyNibbles / 2;
  }
  else return;

  // Fill data
  for (int i = 0; i < howManyNibbles / 2; i++) {
    message.buf[i] = (AsciiHexToInt(m[i * 2 + normalBitPosition + 1]) << 4) + AsciiHexToInt(m[i * 2 + normalBitPosition + 2]);
  }

  // Send it to CAN bus. We will turn on led if we can not buffer
  // the message
  Can0.write(message) == 1 ? digitalWrite(EMERGENCY_LED, LOW) : digitalWrite(EMERGENCY_LED, HIGH);
}

/*
 * This function will be called every time we have data on 
 * USB port. So it is interrupt driven.
 */
void serialEvent() {  
  /* Give me all your bytes serial baby */
  while (Serial.available()) {
    char c = Serial.read();

    // Maybe we have read start of message ':' but we will not wait forever 
    // for the rest of the message
    if (usbIndex >= 0 && millis() - timeReceived > USB_MESSAGE_TIMEOUT) {
      // Timeout
      usbIndex = -1;
    }

    // First char of message is ':'
    if (c == ':') {
      timeReceived = millis();
      usbIndex = 0;
    }
    // Next characters can be anything
    // Take care of overflows
    else if (usbIndex >= 0 && usbIndex < SIZE_USB_BUFFER && c != ';') {
      usbBufferedInput[usbIndex++] = c;
    }
    // Last character should be ';'
    else if (usbIndex > 0 && usbIndex < SIZE_USB_BUFFER + 1 && c == ';') {
      // We have a CANMessage from computer, process it
      processUSBMessage(usbBufferedInput, usbIndex);
      usbIndex = -1;
    }
    // Discard completely
    else {
      usbIndex = -1;
    }
  }
}
