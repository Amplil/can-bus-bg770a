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
 assert(timeout.support(0x0C)==Support::Unknown);assert(!timeout.next(0x11CU,p));
 assert(!timeout.accept(0x7E8,bitmap,8,0x11D));
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
 std::cout<<"OBD session regression tests passed\n";
}
