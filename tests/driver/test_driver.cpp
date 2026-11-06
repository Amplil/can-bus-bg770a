#include "grove-can-bus.h"
#include <cassert>
#include <iostream>
static uint32_t nowMs=0;
uint32_t millis(){return nowMs++;}
HardwareSerial Serial,Serial1;
int main(){
 WioCAN can;can.begin();assert(Serial1.baud==9600);
 assert(can.setCanRate(CAN_RATE_500));
 const std::string wire(Serial1.tx.begin(),Serial1.tx.end());
 assert(wire.find("+++AT+C=16\r\n\r\n")!=std::string::npos);
 assert(wire.find("AT+Q\r\n\r\n")!=std::string::npos);
 Serial1.configAck=false;assert(!can.setCanRate(CAN_RATE_500));
 Serial1.configAck=true;Serial1.exitAck=false;assert(!can.setCanRate(CAN_RATE_500));
 assert(!can.setMask(0,0,0));assert(!can.setFilt(0,0,0));
 Serial1.exitAck=true;Serial1.noise=std::string(300,'X');
 assert(can.setCanRate(CAN_RATE_500));assert(can.setMask(0,0,0));assert(can.setFilt(0,0,0));
 Serial1.tx.clear();can.sendPid(PID_ENGIN_PRM);
 const std::vector<unsigned char> expected={0,0,7,0xDF,0,0,2,1,0x0C,0,0,0,0,0};
 assert(Serial1.tx==expected);
 Serial1.rx={0,0,7,0xE8,4,0x41,0x0C,0x1F,0x40,0,0,0};
 unsigned long id=0;unsigned char data[8]={};
 assert(can.receive(&id,data));assert(id==0x7E8 && data[1]==0x41 && data[4]==0x40);
 Serial1.tx.clear();can.sendDtc(PID_DTC_READ);assert(Serial1.tx.size()==14 && Serial1.tx[6]==1 && Serial1.tx[7]==3);
 Serial1.tx.clear();Serial1.failAfter=6;
 const unsigned char request[]={2,1,0x0C,0,0,0,0,0};
 assert(!can.send(0x7DF,0,0,8,request));assert(Serial1.tx.size()==6);
 assert(!can.send(0x7DF,0,0,9,request));
 std::cout<<"Actual driver config commands/exit failures/bounded noise and legacy PID UART bytes passed\n";
}
