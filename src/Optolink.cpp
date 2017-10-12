#include "Optolink.h"


Optolink::Optolink():
  _stream(nullptr),
  _address(0),
  _length(0),
  _value{0},
  _writeMessageType(false),
  _rcvBuffer{0},
  _rcvBufferLen(0),
  _rcvLen(0),
  _debugMessage(true),
  _state(INIT),
  _action(WAIT),
  _lastMillis(0),
  _numberOfTries(5),
  _errorCode(0),
  _debugPrinter(nullptr)
  {}

/*
//begin serial @ 4800 baud, 8 bits, even parity, 2 stop bits
void Optolink::begin(int8_t rx, int8_t tx) {
  SoftwareSerial* serial = new SoftwareSerial(rx, tx, false, 64);
  serial->begin(4800);
  serial->setParity(1);
  serial->setStopBits(2);
  _stream = serial;
  //serial->flush();
}
*/
#ifdef ARDUINO_ARCH_ESP32
void Optolink::begin(HardwareSerial* serial, int8_t rxPin, int8_t txPin) {
  serial->begin(4800, SERIAL_8E2, rxPin, txPin);
  _stream = serial;
  //serial->flush();
}
#endif
#ifdef ESP8266
void Optolink::begin(HardwareSerial* serial) {
  serial->begin(4800, SERIAL_8E2);
  _stream = serial;
  //serial->flush();
}
#endif


void Optolink::loop() {
  if (_numberOfTries < 1) {
    _state = IDLE;
    _action = RETURN_ERROR;
  }
  else if (_state == INIT) {
    _initHandler();
  }
  else if (_state == IDLE) {
    _idleHandler();
  }
  /*
  else if (_state == SYNC) {
    _syncHandler();
  }
  else if (_state == SEND) {
    _sendHandler();
  }
  */
  else if (_state == RECEIVE) {
    _receiveHandler();
  }
}


// reset new devices to KW state by sending 0x04.
void Optolink::_initHandler() {
  if (_stream->available()) {
    if(_stream->peek() == 0x05) {
      _state = IDLE;
      _idleHandler();
      _debugPrinter->println("INIT done.");
    }
    else _stream->read();
  }
  else {
    if (millis() - _lastMillis > 1000UL) {
      _lastMillis = millis();
      const uint8_t buff[] = {0x04};
      _stream->write(buff, 1);
    }
  }
}


// idle state, waiting for sync from Vito
void Optolink::_idleHandler() {
  if (_stream->available()) {
    if (_stream->read() == 0x05) {
      _lastMillis = millis();
      _debugPrinter->println("0x05 received");
      if (_action == PROCESS) {
        _state = SYNC;
        _syncHandler();
      }
    }
    else _debugPrinter->println("something wrong received");
  }
  else if (_action == PROCESS && (millis() - _lastMillis < 10UL)) {  // don't wait for 0x05 sync signal, send directly after last request
    _state = SEND;
    _sendHandler();
  }
  else if (millis() - _lastMillis > 10 * 1000UL) {
    _state = IDLE;
    _errorCode = 1;
    --_numberOfTries;
  }
}


void Optolink::_syncHandler() {
  const uint8_t buff[1] = {0x01};
  _stream->write(buff, 1);
  _state = SEND;
  _sendHandler();
}

//
void Optolink::_sendHandler() {
  uint8_t buff[6];
  if (_writeMessageType) {
    //type is WRITE
    //has length of 8 chars + length of value
    buff[0] = 0xF4;
    buff[1] = (_address >> 8) & 0xFF;
    buff[2] = _address & 0xFF;
    buff[3] = _length;
    //add value to message
    memcpy(&buff[4], _value, _length);
    _stream->write(buff, 4 + _length);
  }
  else {
    //type is READ
    //has fixed length of 8 chars
    buff[0] = 0xF7;
    buff[1] = (_address >> 8) & 0xFF;
    buff[2] = _address & 0xFF;
    buff[3] = _length;
    //set length of expected answer
    _rcvLen = _length;
    _stream->write(buff, 4);
  }
  _clearInputBuffer();
  _rcvBufferLen = 0;
  --_numberOfTries;
  _state = RECEIVE;
  if (_writeMessageType) _debugPrinter->print(F("WRITE "));
  else _debugPrinter->print(F("READ"));
  _debugPrinter->print(F(" request on address "));
  _printHex(_debugPrinter, &buff[1], 2);
  _debugPrinter->print(F(", length "));
  _debugPrinter->println(_length);
}


void Optolink::_receiveHandler() {
  while (_stream->available() > 0) {  //while instead of if: read complete RX buffer
    _rcvBuffer[_rcvBufferLen] = _stream->read();
    ++_rcvBufferLen;
  }
  if (_rcvBufferLen == _rcvLen) {  //message complete, check message
    _state = IDLE;
    _action = RETURN;
    _lastMillis = millis();
    _errorCode = 0;  //succes
    _debugPrinter->println(F("succes"));
    return;
  }
  else if (millis() - _lastMillis > 10 * 1000UL) {  //Vitotronic isn't answering, try again
    _rcvBufferLen = 0;
    _errorCode = 1;  //Connection error
    memset(_rcvBuffer, 0, 2);
    _state = IDLE;
    _action = RETURN_ERROR;
  }
}


//set properties for datapoint and move state to SEND
bool Optolink::readFromDP(uint16_t address, uint8_t length) {
  if (_action != WAIT) {
    _debugPrinter->println(F("Optolink not available, skipping action."));
    return false;
  }
  //setup properties for next state in communicationHandler
  _address = address;
  _length = length;
  _writeMessageType = false;
  _rcvBufferLen = 0;
  _numberOfTries = 5;
  memset(_rcvBuffer, 0, 2);
  _action = PROCESS;
  return true;
}


//set properties datapoint and move state to SEND
bool Optolink::writeToDP(uint16_t address, uint8_t length, uint8_t value[]) {
  if (_action != WAIT) {
    _debugPrinter->println(F("Optolink not available, skipping action."));
    return false;
  }
  //setup variables for next state
  _address = address;
  _length = length;
  memcpy(_value, value, _length);
  _writeMessageType = true;
  _rcvBufferLen = 0;
  _numberOfTries = 5;
  memset(_rcvBuffer, 0, 2);
  _action = PROCESS;
  return true;
}


const int8_t Optolink::available() const {
  if (_action == RETURN_ERROR) return -1;
  else if (_action == RETURN) return 1;
  else return 0;
}

const bool Optolink::isBusy() const {
  if (_action == WAIT) return false;
  else return true;
}


//return value and reset comunication to IDLE
void Optolink::read(uint8_t value[]) {
  if (_action != RETURN) {
    _debugPrinter->println(F("No reading available"));
    return;
  }
  if (_writeMessageType) {  //return original value in case of WRITE command
    memcpy(value, &_value, _length);
    _action = WAIT;
    return;
  }
  else {
    memcpy(value, &_rcvBuffer, _length);
    _action = WAIT;
    return;  //added for clarity
  }
}


const uint8_t Optolink::readError() {
  _action = WAIT;
  return _errorCode;
}


//clear serial input buffer
inline void Optolink::_clearInputBuffer() {
  while(_stream->available() > 0) {
    _stream->read();
  }
}

void Optolink::setDebugPrinter(Print* printer) {
  _debugPrinter = printer;
}


//Copied from Arduino.cc forum --> (C) robtillaart
inline void Optolink::_printHex(Print* printer, uint8_t array[], uint8_t length) {
  char tmp[length * 2 + 1];
  byte first;
  uint8_t j = 0;
  for (uint8_t i = 0; i < length; ++i) {
    first = (array[i] >> 4) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first ;
    ++j;

    first = (array[i] & 0x0F) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first;
    ++j;
  }
  tmp[length * 2] = 0;
  printer->print(tmp);
}
