#ifndef WIOCAN_H
#define WIOCAN_H

#include <Arduino.h>

// CAN BUS Module settings
#define CAN_BAUDRATE    9600
//#define CAN_BAUDRATE   38400
//#define CAN_BAUDRATE   115200

// CAN bus rates (based on Serial_CAN_Arduino library)
#define CAN_RATE_5      1
#define CAN_RATE_10     2
#define CAN_RATE_20     3
#define CAN_RATE_25     4
#define CAN_RATE_31_2   5
#define CAN_RATE_33     6
#define CAN_RATE_40     7
#define CAN_RATE_50     8
#define CAN_RATE_80     9
#define CAN_RATE_83_3   10
#define CAN_RATE_95     11
#define CAN_RATE_100    12
#define CAN_RATE_125    13
#define CAN_RATE_200    14
#define CAN_RATE_250    15
#define CAN_RATE_500    16
#define CAN_RATE_666    17
#define CAN_RATE_1000   18

// CAN Module class for Grove CAN BUS Module based on GD32E103
class WioCAN {
private:
  HardwareSerial* canSerial;
  char tempBuffer[100];
  
  void clearBuffer();
  bool sendCommand(const char* cmd);
  bool enterConfigMode();
  bool exitConfigMode();

public:
  void begin();
  bool setCanRate(unsigned char rate);
  bool send(unsigned long id, unsigned char ext, unsigned char rtr, unsigned char len, const unsigned char* buf);
  bool receive(unsigned long* id, unsigned char* buf);
  void debugMode();
};

#endif // WIOCAN_H
