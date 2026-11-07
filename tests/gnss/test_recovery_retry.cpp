#include <gnss-session.h>
#include <cassert>
#include <iostream>
void answer(bg770a::GnssSession& s,const char* command,uint32_t now){
 assert(s.takeCommand()==std::string(command)+"\r");assert(s.line(command,now));assert(s.line("OK",now));
}
int main(){
 for(bool late:{false,true})for(uint32_t base:{1000U,UINT32_MAX-10000U}){
  bg770a::GnssSession s;
  s.recover();assert(s.disabled());s.line("+CEREG: 1",base);assert(s.disabled() && !s.ready(base));
  s.resetIssued(base);s.line("APP RDY",base+1);s.tick(base+2201);answer(s,"AT+IFC=2,2",base+2202);
  s.tick(base+4402);answer(s,"AT+CFUN=1",base+4403);answer(s,"AT+CEREG=1",base+4404);
  assert(s.takeCommand()=="AT+CEREG?\r");s.line("AT+CEREG?",base+4405);
  s.line(late?"+CEREG: 1,0":"+CEREG: 1,1",base+4405);s.line("OK",base+4405);
  if(late) {
   assert(s.disabled() && !s.canSend());
   s.line("+CEREG: 5",base+4406);++base;
  }
  assert(s.canSend());assert(!s.disabled());
  assert(!s.ready(base+4405+s.RetryMs-1));assert(s.ready(base+4405+s.RetryMs));
  assert(s.begin(base+4405+s.RetryMs));assert(s.takeCommand()=="AT+CFUN=4\r");
  // A repeated command failure still disables GNSS until another real reset
  // and registration; a registration notification alone cannot restart it.
  s.tick(base+4405+s.RetryMs+20000);assert(s.disabled());
  s.line("+CEREG: 1",base+4405+s.RetryMs+20001);
  assert(s.disabled() && !s.ready(base+4405+2*s.RetryMs));
 }
 for(bool startFailure:{false,true}) {
  bg770a::GnssSession s;assert(s.begin(0));
  if(startFailure)answer(s,"AT+CFUN=4",1);
  const char* failed=startFailure?"AT+QGPS=1":"AT+CFUN=4";
  assert(s.takeCommand()==std::string(failed)+"\r");s.line(failed,2);s.line("ERROR",2);
  assert(s.disabled());
  if(startFailure)answer(s,"AT+QGPSEND",3);
  answer(s,"AT+CFUN=1",4);answer(s,"AT+CEREG=1",5);
  assert(s.takeCommand()=="AT+CEREG?\r");s.line("AT+CEREG?",6);
  s.line("+CEREG: 1,1",6);s.line("OK",6);
  assert(s.canSend() && !s.disabled());
  assert(!s.ready(6+s.RetryMs-1) && s.ready(6+s.RetryMs));
 }
 std::cout<<"Validated physical-reset recovery re-enables GNSS only after registration and cooldown; pre-reset URC and true wrap checked\n";
}
