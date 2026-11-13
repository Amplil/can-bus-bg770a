#pragma once
#include <stdint.h>
#include <WioCellularResult.hpp>

namespace bg770a {
// WioNetwork.begin() can wait forever for a NotSearching URC after CFUN=0.
// Hold the radio off and confirm its state by query before entering begin().
// begin() then reads state 0 and never needs that unsolicited notification.
// timeout bounds the registration polling; individual AT calls retain the
// WioCellular library's own timeouts (including CFUN's 15-second timeout).
template<class Module, class Network, class Now, class Wait, class Report>
bool beginCellularNetwork(Module& module, Network& network, uint32_t timeout,
                          Now now, Wait wait, Report report) {
  report("radio-off", WioCellularResult::Ok, -1);
  auto result=module.setPhoneFunctionality(0);
  if(result!=WioCellularResult::Ok) {
    report("radio-off", result, -1);return false;
  }
  const uint32_t started=now();
  int registration=-1;
  do {
    result=module.getEpsNetworkRegistrationState(&registration);
    report("registration", result, registration);
    if(result!=WioCellularResult::Ok)return false;
    if(registration==0)break;
    const uint32_t elapsed=uint32_t(now()-started);
    if(elapsed>=timeout) {
      report("registration-timeout", WioCellularResult::Ok, registration);
      return false;
    }
    wait(timeout-elapsed<250 ? timeout-elapsed : 250);
  } while(true);

  report("network-config", WioCellularResult::Ok, registration);
  network.begin();
  // begin() preserves CFUN=0, including when all settings already match.
  report("radio-on", WioCellularResult::Ok, registration);
  result=module.setPhoneFunctionality(1);
  if(result!=WioCellularResult::Ok) {
    report("radio-on", result, registration);return false;
  }
  return true;
}
}
