#include "grove-can-bus.h"

void WioCAN::clearBuffer() {
  unsigned long timer_s = millis();
  while(1) {
    if(millis() - timer_s > 50) return;
    while(canSerial->available()) {
      canSerial->read();
      timer_s = millis();
    }
  }
}

bool WioCAN::sendCommand(const char* cmd) {
  unsigned long timer_s = millis();
  unsigned char len = 0;
  
  canSerial->println(cmd);
  while(1) {
    if(millis() - timer_s > 500) {
      return false;
    }
    
    while(canSerial->available()) {
      tempBuffer[len++] = canSerial->read();
      timer_s = millis();
    }
    
    if(len >= 4 && tempBuffer[len-1] == '\n' && tempBuffer[len-2] == '\r' && 
       tempBuffer[len-3] == 'K' && tempBuffer[len-4] == 'O') {
      clearBuffer();
      return true;
    }
  }
}

bool WioCAN::enterConfigMode() {
  canSerial->print("+++");
  clearBuffer();
  return true;
}

bool WioCAN::exitConfigMode() {
  clearBuffer();
  bool ret = sendCommand("AT+Q\r\n");
  clearBuffer();
  return ret;
}

void WioCAN::begin() {
  canSerial = &Serial1;
  canSerial->begin(CAN_BAUDRATE);
  Serial.println("CAN Bus module initialized");
}

bool WioCAN::setCanRate(unsigned char rate) {
  enterConfigMode();
  sprintf(tempBuffer, "AT+C=%d\r\n", rate);
  
  bool ret = sendCommand(tempBuffer);
  exitConfigMode();
  return ret;
}

bool WioCAN::send(unsigned long id, unsigned char ext, unsigned char rtr, unsigned char len, const unsigned char* buf) {
  unsigned char data[14] = {0};
  
  data[0] = id >> 24;       // id3
  data[1] = (id >> 16) & 0xff; // id2
  data[2] = (id >> 8) & 0xff;  // id1
  data[3] = id & 0xff;      // id0
  
  data[4] = ext;
  data[5] = rtr;
  
  for(int i = 0; i < len; i++) {
    data[6 + i] = buf[i];
  }
  
  for(int i = 0; i < 14; i++) {
    canSerial->write(data[i]);
  }
  
  return true;
}

void WioCAN::sendPid(unsigned char __pid) {
    unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};
    
#if STANDARD_CAN_11BIT
    this->send(CAN_ID_PID, 0, 0, 8, tmp);   // SEND TO ID:0X55
#else
    this->send(CAN_ID_PID, 1, 0, 8, tmp);   // SEND TO ID:0X55
#endif
}

bool WioCAN::receive(unsigned long* id, unsigned char* buf) {
  if(!canSerial->available()) {
    return false;
  }
  
  unsigned long timer_s = millis();
  int len = 0;
  unsigned char data[20];
  
  while(1) {
    while(canSerial->available()) {
      data[len++] = canSerial->read();
      if(len == 12) break;
      
      if((millis() - timer_s) > 100) {
        canSerial->flush();
        return false;
      }
    }
    
    if(len == 12) {
      unsigned long receivedId = 0;
      
      for(int i = 0; i < 4; i++) {
        receivedId <<= 8;
        receivedId += data[i];
      }
      
      *id = receivedId;
      
      for(int i = 0; i < 8; i++) {
        buf[i] = data[i + 4];
      }
      return true;
    }
    
    if((millis() - timer_s) > 100) {
      canSerial->flush();
      return false;
    }
  }
}

void WioCAN::debugMode() {
  String inputBuffer = "";
  while(Serial.available()) {
    char inkey = Serial.read();
    inputBuffer += inkey;
    canSerial->write(inkey);
  }
  if (inputBuffer.length()) {
    Serial.println(inputBuffer);
  }
  while(canSerial->available()) {
    Serial.write(canSerial->read());
  }
}

void WioCAN::debugPID() {
  String inputBuffer = "";
  while(Serial.available()) {
    char inkey = Serial.read();
    inputBuffer += inkey;
  }
  if (inputBuffer.length() > 2 && inputBuffer.startsWith("0x")) {
    //inputBuffer.remove(0, 2); // Remove the "0x" prefix
    String pidStr = inputBuffer.substring(2); // "0x"以降を取得
    sendPid(strtol(pidStr.c_str(), nullptr, 16));
    Serial.print("Sent PID: ");
    Serial.println(strtol(pidStr.c_str(), nullptr, 16));
  }
  if (inputBuffer.length()) {
    Serial.println(inputBuffer);
  }
  while(canSerial->available()) {
    Serial.write(canSerial->read());
  }
}

bool WioCAN::setMask(uint8_t n, uint8_t ext, unsigned long value) {
  enterConfigMode();
  sprintf(tempBuffer, "AT+M=[%d][%d][%08lX]\r\n", n, ext, value);
  bool ret = sendCommand(tempBuffer);
  exitConfigMode();
  return ret;
}

bool WioCAN::setFilt(uint8_t n, uint8_t ext, unsigned long value) {
  enterConfigMode();
  sprintf(tempBuffer, "AT+F=[%d][%d][%08lX]\r\n", n, ext, value);
  bool ret = sendCommand(tempBuffer);
  exitConfigMode();
  return ret;
}
