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
