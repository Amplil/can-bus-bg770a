#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <deque>
#include <vector>
uint32_t millis();
struct String:std::string {
 using std::string::string;
 String(const std::string& s):std::string(s){}
 bool startsWith(const char* s)const{return compare(0,std::strlen(s),s)==0;}
 String substring(size_t n)const{return substr(n);}
};
struct HardwareSerial {
 bool configAck=true,exitAck=true;
 std::string noise;std::vector<unsigned char> tx;
 std::deque<unsigned char> rx;
 unsigned baud=0;
 size_t failAfter=static_cast<size_t>(-1);
 void begin(unsigned value){baud=value;}
 void print(const char* s){for(;*s;++s)tx.push_back(static_cast<unsigned char>(*s));}
 template<class T>void print(T){}
 void println(const char* s){
  print(s);print("\r\n");
  const bool exit=std::strncmp(s,"AT+Q",4)==0;
  if(std::strncmp(s,"AT+",3)!=0)return;
  const std::string reply=noise+((exit?exitAck:configAck)?"OK\r\n":"ERROR\r\n");
  for(auto c:reply)rx.push_back(static_cast<unsigned char>(c));
 }
 template<class T>void println(const T&){}
 int available(){return static_cast<int>(rx.size());}
 int read(){if(rx.empty())return -1;const auto c=rx.front();rx.pop_front();return c;}
 size_t write(unsigned char c){if(tx.size()>=failAfter)return 0;tx.push_back(c);return 1;}
 void flush(){}
};
extern HardwareSerial Serial,Serial1;
