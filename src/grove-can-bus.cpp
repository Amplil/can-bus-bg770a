#include "grove-can-bus.h"

void WioCAN::clearBuffer() {
  const uint32_t start=millis();uint32_t quiet=start;
  while(uint32_t(millis()-start)<200 && uint32_t(millis()-quiet)<50) {
    if(canSerial->available()) { canSerial->read();quiet=millis(); }
  }
}

bool WioCAN::sendCommand(const char* cmd) {
  const uint32_t start=millis();
  char tail[7]={};size_t count=0;
  canSerial->println(cmd);
  // Bound both elapsed time and storage even with echo or continuous noise.
  while(uint32_t(millis()-start)<500) {
    if(!canSerial->available())continue;
    const char c=char(canSerial->read());
    if(count<sizeof(tail))tail[count++]=c;
    else { memmove(tail,tail+1,sizeof(tail)-1);tail[sizeof(tail)-1]=c; }
    if(count>=4 && memcmp(tail+count-4,"OK\r\n",4)==0) { clearBuffer();return true; }
    if(count>=7 && memcmp(tail+count-7,"ERROR\r\n",7)==0)return false;
  }
  return false;
}

bool WioCAN::enterConfigMode() {
  canSerial->print("+++");
  clearBuffer();
  return true;
}

bool WioCAN::exitConfigMode() {
  clearBuffer();
  bool ret = sendCommand("AT+Q\r\n");
  if(!ret)Serial.println("CAN config exit ACK missing; data mode unconfirmed");
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
  const bool exited=exitConfigMode();
  return ret && exited;
}

bool WioCAN::send(unsigned long id, unsigned char ext, unsigned char rtr, unsigned char len, const unsigned char* buf) {
  if(len>8 || (len && !buf) || ext>1 || rtr>1 || id>(ext?0x1FFFFFFFUL:0x7FFUL))return false;
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
    if(canSerial->write(data[i])!=1) {
      Serial.println("CAN UART write incomplete; framing unconfirmed");return false;
    }
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

void WioCAN::sendDtc(unsigned char __dtc_mode) {
    // DTC services have one application byte (no PID); remaining bytes are padding.
    unsigned char tmp[8] = {0x01, __dtc_mode, 0x00, 0, 0, 0, 0, 0};

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
  const bool exited=exitConfigMode();
  return ret && exited;
}

bool WioCAN::setFilt(uint8_t n, uint8_t ext, unsigned long value) {
  enterConfigMode();
  sprintf(tempBuffer, "AT+F=[%d][%d][%08lX]\r\n", n, ext, value);
  bool ret = sendCommand(tempBuffer);
  const bool exited=exitConfigMode();
  return ret && exited;
}
