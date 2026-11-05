#include "can-diagnostics.h"
#include "obd-dtc.h"
#include "obd-session.h"
#include <cassert>
#include <iostream>
int main(){
 bg770a::CanDiagnostics d;
 for(unsigned n=0;n<64;++n)assert(d.detail(0));
 assert(!d.detail(999) && d.suppressed==1);assert(d.detail(1000));
 assert(d.summary(0));assert(!d.summary(4999));assert(d.summary(5000));
 bg770a::CanDiagnostics wrap;assert(wrap.detail(0xFFFFFFF0U));assert(wrap.detail(0x3E0U));
 using S=bg770a::ObdSession;S s;uint8_t pid;assert(s.next(0,pid));
 uint8_t data[]={6,0x41,0,0,0,0,0,0};
 assert(!s.accept(0x123,data,8,1));assert(s.rejection()==S::Reject::Responder);
 data[1]=0x7F;assert(!s.accept(0x7E8,data,8,2));assert(s.rejection()==S::Reject::Service);
 data[1]=0x41;data[2]=1;assert(!s.accept(0x7E8,data,8,3));assert(s.rejection()==S::Reject::Pid);
 data[2]=0;data[0]=5;assert(!s.accept(0x7E8,data,8,4));assert(s.rejection()==S::Reject::Length);
 data[0]=6;assert(!s.accept(0x7E8,data,8,200));assert(s.rejection()==S::Reject::Deadline);assert(s.takeTimeout());assert(!s.takeTimeout());
 const uint8_t one[]={4,0x43,1,0x01,0xAF,0,0,0};assert(bg770a::dtcSingleFrame(one));
 const uint8_t zero[]={2,0x43,0,0,0,0,0,0};assert(bg770a::dtcSingleFrame(zero));
 const uint8_t two[]={6,0x43,2,0x01,0xAF,0xC0,0x10,0};assert(bg770a::dtcSingleFrame(two));
 const uint8_t uncouned[]={3,0x43,0x01,0xAF,0,0,0,0};assert(!bg770a::dtcSingleFrame(uncouned));
 const uint8_t mismatch[]={4,0x43,2,0x01,0xAF,0,0,0};assert(!bg770a::dtcSingleFrame(mismatch));
 const uint8_t padded[]={7,0x43,1,0x01,0xAF,0xFF,0xFF,0xFF};
 assert(bg770a::dtcSingleFrame(padded));assert(!bg770a::dtcSingleFrame(padded,7));
 const uint8_t paddedZero[]={7,0x43,0,0xFF,0xFF,0xFF,0xFF,0xFF};assert(bg770a::dtcSingleFrame(paddedZero));
 std::cout<<"Diagnostic rejection reasons, bounded raw logs, timers and counted CAN Mode03 SF validation passed\n";
}
