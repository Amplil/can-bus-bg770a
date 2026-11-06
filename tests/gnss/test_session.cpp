#include "gnss-session.h"
#include <cassert>
#include <iostream>
using bg770a::GnssSession;
void answer(GnssSession& s, const char* command, uint32_t t, const char* result="OK") {
  assert(s.takeCommand() == std::string(command)+"\r");
  assert(s.line(command,t)); assert(s.line(result,t));
}
void start(GnssSession& s, uint32_t t) {
  assert(s.begin(t)); answer(s,"AT+CFUN=4",t); answer(s,"AT+QGPS=1",t);
  s.tick(t+1000); assert(s.takeCommand()=="AT+QGPSLOC=2\r");
  assert(s.line("AT+QGPSLOC=2",t+1000));
}
void restore(GnssSession& s, uint32_t t) {
  answer(s,"AT+QGPSEND",t); answer(s,"AT+CFUN=1",t);
  answer(s,"AT+CEREG=1",t);
  assert(s.takeCommand()=="AT+CEREG?\r"); s.line("AT+CEREG?",t);
  assert(s.line("+CEREG: 1,1",t)); s.line("OK",t); assert(s.modemIdle());
  assert(!s.line("+CEREG: 1",t)); assert(s.canSend());
}
int main() {
  const char* sample="+QGPSLOC: 123456.000,35.12345,139.12345,1.2,12.0,3,0.00,0.0,0.0,041126,08";
  GnssSession s; assert(s.canSend()); start(s,0);
  assert(!s.canSend()); assert(s.line(sample,1000)); assert(!s.fix().valid);
  assert(s.line("OK",1000)); assert(s.fix().valid); restore(s,1010);
  assert(!s.ready(301009)); assert(s.ready(301010));
  start(s,301010); assert(!s.fix().valid); s.line(sample,302010); s.line("OK",302010);
  assert(!s.fix().valid); // Same UTC from previous acquisition is not a new fix.
  s.tick(331010); restore(s,331020); assert(!s.fix().valid);
  GnssSession noAntenna; start(noAntenna,0); noAntenna.line("+CME ERROR: 516",1000);
  noAntenna.tick(30000); restore(noAntenna,30010); assert(noAntenna.canSend());
  assert(!noAntenna.fix().valid);
  GnssSession rejected; assert(rejected.begin(0)); answer(rejected,"AT+CFUN=4",0);
  answer(rejected,"AT+QGPS=1",1,"ERROR"); restore(rejected,2);
  assert(rejected.canSend()); assert(!rejected.ready(999999));
  GnssSession timeout; assert(timeout.begin(0)); timeout.takeCommand(); timeout.tick(20000);
  assert(timeout.state()==GnssSession::State::ResetNeeded); assert(!timeout.canSend());
  timeout.resetIssued(20000); timeout.line("OK",20001); assert(!timeout.canSend());
  timeout.line("APP RDY",22000); timeout.tick(24199); assert(timeout.takeCommand().empty());
  timeout.tick(24200); answer(timeout,"AT+IFC=2,2",24200);
  timeout.tick(26400); answer(timeout,"AT+CFUN=1",26400); answer(timeout,"AT+CEREG=1",26401);
  assert(timeout.takeCommand()=="AT+CEREG?\r"); timeout.line("AT+CEREG?",26402);
  assert(timeout.line("+CEREG: 1,5",26402)); timeout.line("OK",26402);
  // Boot may already be registered before CEREG URCs are enabled: no URC required.
  assert(timeout.canSend()); assert(!timeout.ready(999999));
  GnssSession boot; assert(boot.begin(0)); boot.tick(20000); boot.resetIssued(20000); boot.tick(50000);
  assert(boot.state()==GnssSession::State::ResetNeeded);
  GnssSession missingEcho; assert(missingEcho.begin(0)); missingEcho.takeCommand(); missingEcho.line("OK",1);
  assert(missingEcho.state()==GnssSession::State::ResetNeeded);
  GnssSession wrap; start(wrap,UINT32_MAX-2000); wrap.line("+CME ERROR: 516",UINT32_MAX-1000);
  wrap.tick(28000); restore(wrap,28001);
  // GNSS stop rejection must not issue CFUN=1 across an uncertain GNSS state.
  GnssSession stopRejected; start(stopRejected,0);
  stopRejected.line(sample,1000); stopRejected.line("OK",1000);
  assert(stopRejected.fix().valid);
  answer(stopRejected,"AT+QGPSEND",1001,"ERROR");
  assert(stopRejected.state()==GnssSession::State::ResetNeeded);
  assert(!stopRejected.canSend()); assert(!stopRejected.fix().valid);
  assert(stopRejected.takeCommand().empty());

  // Loss of registration is polled, without starting GNSS or allowing telemetry.
  GnssSession registration;
  assert(!registration.line("+CEREG: 0",0)); assert(!registration.canSend());
  registration.tick(4999); assert(registration.takeCommand().empty());
  registration.tick(5000); assert(registration.takeCommand()=="AT+CEREG?\r");
  assert(!registration.begin(5000)); assert(!registration.canSend());
  assert(registration.line("AT+CEREG?\r",5000));
  assert(registration.line("+CEREG: 1,2",5000)); // Mode 1 is not registration 1.
  registration.line("OK",5000); assert(registration.modemIdle());
  assert(!registration.canSend());
  registration.tick(9999); assert(registration.takeCommand().empty());
  registration.tick(10000); assert(registration.takeCommand()=="AT+CEREG?\r");
  registration.line("AT+CEREG?",10000);
  assert(registration.line("+CEREG: 1,5",10000));
  assert(!registration.canSend()); // Publish only after matching successful result.
  registration.line("OK",10000); assert(registration.canSend());
  assert(registration.registrationStatus()==5);
  assert(!registration.line("+CEREG: 3",10001)); assert(!registration.canSend());

  // Query responses and single-status URCs can be interleaved on the UART.
  GnssSession interleaved;
  interleaved.line("+CEREG: 0",0); interleaved.tick(5000);
  assert(interleaved.takeCommand()=="AT+CEREG?\r");
  interleaved.line("AT+CEREG?",5000);
  assert(interleaved.line("+CEREG: 1,1",5000));
  assert(!interleaved.line("+CEREG: 1",5001)); // WioNetwork must still observe it.
  interleaved.line("OK",5002); assert(interleaved.canSend());

  // A newer deregistration URC must supersede an earlier registered query.
  interleaved.line("+CEREG: 0",5003); interleaved.tick(10002);
  assert(interleaved.takeCommand()=="AT+CEREG?\r");
  interleaved.line("AT+CEREG?",10002);
  assert(interleaved.line("+CEREG: 1,5",10002));
  assert(!interleaved.line("+CEREG: 2",10003));
  interleaved.line("OK",10004); assert(!interleaved.canSend());
  interleaved.tick(15004); assert(interleaved.takeCommand()=="AT+CEREG?\r");
  interleaved.line("AT+CEREG?",15004);
  assert(interleaved.line("+CEREG: 1,15",15004)); // Never accept a status prefix.
  interleaved.line("OK",15005); assert(!interleaved.canSend());
  assert(interleaved.registrationStatus()==15);

  // Whitespace is allowed around complete integer tokens, including roaming.
  GnssSession spaced;
  assert(!spaced.line("+CEREG: \t0 \t",0));assert(spaced.registrationStatus()==0);
  spaced.tick(5000);assert(spaced.takeCommand()=="AT+CEREG?\r");
  spaced.line("AT+CEREG?",5000);
  assert(spaced.line("+CEREG: 1 \t, \t5 \t",5000));
  assert(spaced.registrationStatus()==0 && !spaced.canSend());
  spaced.line("OK",5001);assert(spaced.canSend() && spaced.registrationStatus()==5);
  assert(!spaced.line("+CEREG: \t15 \t",5002));
  assert(!spaced.canSend() && spaced.registrationStatus()==15);
  spaced.tick(10001);assert(spaced.takeCommand()=="AT+CEREG?\r");
  spaced.line("AT+CEREG?",10001);
  assert(spaced.line("+CEREG: 1, 1junk",10001));
  spaced.line("OK",10002);assert(!spaced.canSend() && spaced.registrationStatus()==-1);

  // During a query, extended URCs use their first integer as the status.
  // The location tokens must never be treated as the queried status.
  GnssSession extended;
  extended.line("+CEREG: 0",0);extended.tick(5000);
  assert(extended.takeCommand()=="AT+CEREG?\r");extended.line("AT+CEREG?",5000);
  assert(extended.line("+CEREG: 1, 5",5000));
  assert(!extended.line("+CEREG: 2, \"1234\",\"12345678\",7",5001));
  extended.line("OK",5002);assert(!extended.canSend() && extended.registrationStatus()==2);
  extended.tick(10002);assert(extended.takeCommand()=="AT+CEREG?\r");
  extended.line("AT+CEREG?",10002);
  assert(extended.line("+CEREG: 1,2",10002));
  assert(!extended.line("+CEREG: 5 \t, \"1234\",\"12345678\",7",10003));
  extended.line("OK",10004);assert(extended.canSend() && extended.registrationStatus()==5);

  // Failed LTE reattachment after CFUN=1 recovers within two minutes, across
  // millis rollover as well. A successful registration cancels that deadline.
  for(const uint32_t base:{0U,UINT32_MAX-60000U}) {
    GnssSession stalled;
    assert(stalled.begin(base));answer(stalled,"AT+CFUN=4",base);
    answer(stalled,"AT+QGPS=1",base,"ERROR");
    answer(stalled,"AT+QGPSEND",base);
    answer(stalled,"AT+CFUN=1",base);
    answer(stalled,"AT+CEREG=1",base);
    assert(stalled.takeCommand()=="AT+CEREG?\r");
    stalled.line("AT+CEREG?",base);stalled.line("+CEREG: 1, 2",base);stalled.line("OK",base);
    for(uint32_t elapsed=5000;elapsed<120000;elapsed+=5000) {
      stalled.tick(uint32_t(base+elapsed));assert(stalled.takeCommand()=="AT+CEREG?\r");
      stalled.line("AT+CEREG?",uint32_t(base+elapsed));
      stalled.line("+CEREG: 1, 2",uint32_t(base+elapsed));
      stalled.line("OK",uint32_t(base+elapsed));
      assert(stalled.modemIdle() && !stalled.canSend() && stalled.registrationStatus()==2);
    }
    stalled.tick(uint32_t(base+120000));
    assert(stalled.state()==GnssSession::State::ResetNeeded);
    assert(!stalled.canSend() && stalled.takeCommand().empty());
    stalled.resetIssued(uint32_t(base+120001));assert(stalled.registrationStatus()==-1);

    GnssSession restored;start(restored,base);
    restored.line("+CME ERROR: 516",uint32_t(base+1000));
    restored.tick(uint32_t(base+30000));
    restore(restored,uint32_t(base+30001));
    restored.tick(uint32_t(base+150001));
    assert(restored.canSend() && restored.state()==GnssSession::State::Idle);
  }

  GnssSession registrationError;
  registrationError.line("+CEREG: 0",0); registrationError.tick(5000);
  answer(registrationError,"AT+CEREG?",5000,"ERROR");
  assert(registrationError.state()==GnssSession::State::ResetNeeded);
  assert(!registrationError.canSend());

  // Late responses after a timeout cannot revive an abandoned transaction.
  GnssSession late; start(late,0);
  late.tick(21000); assert(late.state()==GnssSession::State::ResetNeeded);
  assert(!late.line(sample,21001)); assert(!late.line("OK",21002));
  assert(!late.fix().valid); assert(late.takeCommand().empty());
  late.resetIssued(21003);
  assert(!late.line("AT+QGPSLOC=2",21004)); assert(!late.line("OK",21005));
  assert(late.state()==GnssSession::State::BootWait);
  assert(late.line("APP RDY",22000));
  late.tick(24200); assert(late.takeCommand()=="AT+IFC=2,2\r");
  late.line("OK",24201); // No echo for the new command: cannot accept a stale OK.
  assert(late.state()==GnssSession::State::ResetNeeded);
  assert(!late.canSend()); assert(late.takeCommand().empty());

  std::cout << "GNSS session tests passed\n";
}
