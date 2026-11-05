#include "obd-session.h"
#include <cassert>
#include <iostream>
using namespace bg770a;
int main() {
 uint8_t p=99;
 uint8_t bitmap[]={6,0x41,0,0,0x18,0,1,0}; // RPM, speed, continuation
 ObdSession s;
 assert(s.support(0x0C)==Support::Unknown);
 assert(s.next(0,p)&&p==0);
 assert(!s.accept(0x123,bitmap,8,1));
 auto bad=bitmap[1];bitmap[1]=0x7F;assert(!s.accept(0x7E8,bitmap,8,2));bitmap[1]=bad;
 bitmap[0]=0x10;assert(!s.accept(0x7E8,bitmap,8,3));bitmap[0]=6;
 assert(!s.accept(0x7E8,bitmap,6,4));
 assert(!s.accept(0x7E8,bitmap,8,5));assert(s.selected());
 assert(s.support(0x0C)==Support::Supported);
 assert(s.support(0x0D)==Support::Supported);
 assert(s.support(1)==Support::Unsupported);
 assert(s.support(0x21)==Support::Unknown);
 assert(s.support(0x20)==Support::Supported);
 assert(!s.next(104,p));assert(s.next(105,p)&&p==0x20);
 uint8_t page[]={6,0x41,0x20,0,0,0,1,0};
 assert(!s.accept(0x7E9,page,8,106));assert(!s.accept(0x7E8,page,8,107));
 assert(s.next(207,p)&&p==0x40);
 page[2]=0x40;page[3]=0x40;page[6]=0;
 assert(!s.accept(0x7E8,page,8,208));
 assert(s.support(0x42)==Support::Supported);assert(s.support(0x60)==Support::Unsupported);
 assert(s.support(0)==Support::Unknown);assert(s.support(0x61)==Support::Unknown);
 assert(!s.next(308,p));
 assert(s.next(309,p)&&p==0x0C);
 uint8_t rpm[]={4,0x41,0x0C,0x12,0x34,0,0,0};
 rpm[2]=0x0D;assert(!s.accept(0x7E8,rpm,8,309));rpm[2]=0x0C;
 rpm[0]=3;assert(!s.accept(0x7E8,rpm,8,310));rpm[0]=4;
 assert(!s.accept(0x7E9,rpm,8,311));assert(s.accept(0x7E8,rpm,8,312));
 ObdSession timeout;assert(timeout.next(0xFFFFFFF0U,p));timeout.tick(0xB8U);
 assert(timeout.support(0x0C)==Support::Unknown);assert(timeout.next(0x11CU,p)&&p==0);
 assert(!timeout.accept(0x7E8,bitmap,8,0x1E4));
 assert(timeout.next(0x248,p)&&p==0);timeout.tick(0x310);
 assert(!timeout.next(0x374,p));
 ObdSession ext(true);assert(ext.requestId()==0x18DB33F1UL);
 assert(ext.next(0,p));assert(!ext.accept(0x7E8,bitmap,8,1));
 assert(!ext.accept(0x18DA01F1,bitmap,8,2));
 assert(!ext.accept(0x38DAF110,bitmap,8,2));
 assert(!ext.accept(0x18DAF110,nullptr,8,2));
 assert(!ext.accept(0x18DAF110,bitmap,8,3));assert(ext.ecu()==0x18DAF110);
 ObdSession none;
 uint8_t empty[]={6,0x41,0,0,0,0,0,0};
 assert(none.next(0,p));assert(!none.accept(0x7E8,empty,8,1));
 assert(none.support(0x20)==Support::Unsupported);
 assert(!none.next(101,p));assert(!none.next(201,p));
 assert(none.support(0x42)==Support::Unknown);
 ObdSession late;assert(late.next(0,p));
 assert(!late.accept(0x7E8,bitmap,8,200));assert(!late.selected());
 ObdSession odometer;
 uint32_t t=0;
 for(unsigned page=0;page<6;++page) {
  assert(odometer.next(t,p) && p==page*32);
  uint8_t bits[]={6,0x41,p,0,0,0,1,0};
  if(page==5) { bits[3]=4;bits[6]=0; }
  assert(!odometer.accept(0x7E8,bits,8,t+1)); t+=101;
 }
 assert(odometer.support(0xA6)==Support::Supported);
 assert(odometer.support(0xC0)==Support::Unsupported);
 assert(odometer.next(t,p) && p==0xA6);
 uint8_t distance[]={6,0x41,0xA6,0,0,0x10,0,0};
 assert(odometer.accept(0x7E8,distance,8,t+1));
 for(unsigned variant=0;variant<3;++variant) {
  ObdSession compatibility(false,true);uint32_t now=0;
  for(unsigned attempt=0;attempt<3;++attempt) {
   assert(compatibility.next(now,p)&&p==0);
   if(variant==1) {uint8_t bad[]={6,0x7F,0,0,0,0,0,0};assert(!compatibility.accept(0x7E8,bad,8,now+50));bad[0]=0x10;bad[1]=0x41;assert(!compatibility.accept(0x7E8,bad,8,now+60));}
   compatibility.tick(now+200);now+=300;
  }
  assert(compatibility.support(0x0C)==Support::Unknown);
  assert(compatibility.next(now,p)&&p==0x0C);
  if(variant==2)assert(!compatibility.accept(0x7E8,bitmap,8,now+1)); // late bitmap cannot complete RPM
  uint8_t reply[]={4,0x41,0x0C,0x1F,0x40,0,0,0};
  assert(compatibility.accept(0x7E8,reply,8,now+2));
  assert(compatibility.support(0x0C)==Support::Supported);
 }
 ObdSession excluded(false,true);assert(excluded.next(0,p));
 assert(!excluded.accept(0x7E8,empty,8,1));assert(!excluded.next(101,p));
 assert(excluded.next(201,p) && p==0x31); // known base-PID exclusions remain skipped
 assert(excluded.support(0x0C)==Support::Unsupported);
 // Supplied PID00 form: declared SF length 7; only four bitmap bytes are used.
 for(unsigned padding : {0U,255U}) {
  ObdSession field;assert(field.next(19750,p));
  uint8_t padded[]={7,0x41,0,0xFF,0xFF,0xFF,0xFF,static_cast<uint8_t>(padding)};
  assert(!field.accept(0x7E8,padded,7,19760));assert(!field.acceptedFrame());
  assert(!field.accept(0x7E8,padded,8,19780));assert(field.acceptedFrame());
  for(unsigned pid=1;pid<=32;++pid)assert(field.support(pid)==Support::Supported);
  assert(field.support(0x21)==Support::Unknown);
 }
 ObdSession paddedRpm;assert(paddedRpm.next(0,p));
 uint8_t base[]={6,0x41,0,0,0x10,0,0,0};paddedRpm.accept(0x7E8,base,8,1);
 assert(!paddedRpm.next(101,p));assert(paddedRpm.next(102,p) && p==0x0C);
 uint8_t rpmPad[]={7,0x41,0x0C,0x1F,0x40,0xAA,0xFF,0x55};
 rpmPad[0]=8;assert(!paddedRpm.accept(0x7E8,rpmPad,8,103));
 rpmPad[0]=0x10;assert(!paddedRpm.accept(0x7E8,rpmPad,8,104));
 rpmPad[0]=7;assert(!paddedRpm.accept(0x7E8,rpmPad,7,105));
 assert(!paddedRpm.accept(0x7E9,rpmPad,8,106));
 rpmPad[1]=0x7F;assert(!paddedRpm.accept(0x7E8,rpmPad,8,107));rpmPad[1]=0x41;
 rpmPad[2]=0x0D;assert(!paddedRpm.accept(0x7E8,rpmPad,8,108));rpmPad[2]=0x0C;
 assert(paddedRpm.accept(0x7E8,rpmPad,8,109));
 std::cout<<"OBD session regression tests passed\n";
}
