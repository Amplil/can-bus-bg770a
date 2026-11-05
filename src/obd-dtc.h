#pragma once
#include <stdint.h>
namespace bg770a {
// ISO 15765 Mode 03 application data: 43, DTC count, then two bytes per DTC.
// Only a complete single frame is decoded here; no ISO-TP flow control is sent.
inline bool dtcSingleFrame(const uint8_t* d) {
  if(d[1]!=0x43 || d[0]<2 || d[0]>6 || d[2]>2 || d[0]!=2+2*d[2])return false;
  for(unsigned n=0;n<d[2];++n)if(d[3+2*n]==0 && d[4+2*n]==0)return false;
  return true;
}
}
