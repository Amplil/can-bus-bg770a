// Exercise the installed WioCellular network implementation, including its
// missing-registration-URC wait, with deterministic modem responses.
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <WioCellularResult.hpp>
#include <module/at_client/AtParameterParser.hpp>

struct SerialStub {
  template<class T> void print(const T&) {}
  template<class T> void println(const T&) {}
  void flush() {}
} Serial;

struct MissingRegistrationUrc {};
static uint32_t nowMs=0;

struct FakeModule {
  struct PdpContext {
    int cid; std::string pdpType,apn,pdpAddr; int dataComp,headComp,ipv4AddrAlloc;
  };
  struct UrcHandler {
    std::function<bool(const std::string&)> callback;
  };

  int functionality=1,registration=2,actMode=1;
  std::string actSequence="0302",ltemBand="old-band",nbiotBand="0x0";
  std::vector<PdpContext> contexts={{1,"IP","old.apn","0.0.0.0",0,0,0}};
  std::vector<int> radioCommands;
  unsigned registrationQueries=0,configurationWrites=0,workCalls=0;
  unsigned pendingRegistrationQueries=0;
  bool registrationStuck=false;
  WioCellularResult radioOffResult=WioCellularResult::Ok;
  WioCellularResult radioOnResult=WioCellularResult::Ok;
  WioCellularResult registrationResult=WioCellularResult::Ok;

  std::unique_ptr<UrcHandler> registerUrcHandler2(
      const std::function<bool(const std::string&)>& callback) {
    return std::unique_ptr<UrcHandler>(new UrcHandler{callback});
  }
  WioCellularResult getPdpContext(std::vector<PdpContext>* out) {
    *out=contexts;return WioCellularResult::Ok;
  }
  WioCellularResult setPdpContext(const PdpContext& value) {
    ++configurationWrites;contexts={value};return WioCellularResult::Ok;
  }
  WioCellularResult getSearchAccessTechnology(int* value) {
    *value=actMode;return WioCellularResult::Ok;
  }
  WioCellularResult setSearchAccessTechnology(int value) {
    ++configurationWrites;actMode=value;return WioCellularResult::Ok;
  }
  WioCellularResult getSearchAccessTechnologySequence(std::string* value) {
    *value=actSequence;return WioCellularResult::Ok;
  }
  WioCellularResult setSearchAccessTechnologySequence(const std::string& value) {
    ++configurationWrites;actSequence=value;return WioCellularResult::Ok;
  }
  WioCellularResult getSearchFrequencyBand(std::string*,std::string* ltem,std::string* nbiot) {
    *ltem=ltemBand;*nbiot=nbiotBand;return WioCellularResult::Ok;
  }
  WioCellularResult setSearchFrequencyBand(const std::string&,const std::string& ltem,
                                          const std::string& nbiot) {
    ++configurationWrites;ltemBand=ltem;nbiotBand=nbiot;return WioCellularResult::Ok;
  }
  WioCellularResult setEpsNetworkRegistrationStatusUrc(int) {return WioCellularResult::Ok;}
  WioCellularResult getEpsNetworkRegistrationState(int* out) {
    if(functionality==0 && pendingRegistrationQueries>0 && --pendingRegistrationQueries==0)
      registration=0;
    ++registrationQueries;*out=registration;return registrationResult;
  }
  WioCellularResult getPhoneFunctionality(int* out) {
    *out=functionality;return WioCellularResult::Ok;
  }
  WioCellularResult setPhoneFunctionality(int value) {
    radioCommands.push_back(value);
    const auto result=value==0?radioOffResult:radioOnResult;
    if(result!=WioCellularResult::Ok)return result;
    functionality=value;
    // Modem changes state, but deliberately emits no +CEREG: 0 URC.
    if(value==0 && !registrationStuck && pendingRegistrationQueries==0)registration=0;
    return WioCellularResult::Ok;
  }
  void doWork(int timeout) {
    nowMs+=uint32_t(timeout);
    if(++workCalls>=8)throw MissingRegistrationUrc{};
  }
  bool doWork(int timeout,const std::function<bool()>& completion) {
    if(completion())return true;
    doWork(timeout);return completion();
  }
} WioCellular;
using WioCellularModule=FakeModule;

// This is the real dependency header, selected by WIOCELLULAR_INCLUDE.
#include <network/Bg770aNetwork.hpp>
#include <cellular-startup.h>
using Network=wiocellular::network::Bg770aNetwork;

struct Report {
  std::string stage; WioCellularResult result; int registration;
};
static std::vector<Report> reports;

static void configure(Network& network) {
  network.abortHandler=[](const char*,int){throw std::runtime_error("unexpected network abort");};
  network.config.apn="soracom.io";
  network.config.searchAccessTechnology=Network::SearchAccessTechnology::LTEM;
  network.config.ltemBand=Network::NTTDOCOMO_LTEM_BAND;
}
static void reset(uint32_t start=0) {
  WioCellular=FakeModule{};nowMs=start;reports.clear();
}
static bool begin(Network& network,uint32_t timeout=1000) {
  return bg770a::beginCellularNetwork(WioCellular,network,timeout,
    [](){return nowMs;},[](uint32_t duration){nowMs+=duration;},
    [](const char* stage,WioCellularResult result,int registration){
      reports.push_back({stage,result,registration});
    });
}
static bool reported(const char* stage,WioCellularResult result=WioCellularResult::Ok) {
  return std::any_of(reports.begin(),reports.end(),[&](const Report& report){
    return report.stage==stage && report.result==result;
  });
}

int main() {
  // Original begin() caches Searching, then waits forever for a missing URC
  // after CFUN=0; the real modem state has already become NotSearching.
  reset();
  {
    Network network;configure(network);
    bool blocked=false;
    try {network.begin();}catch(const MissingRegistrationUrc&){blocked=true;}
    assert(blocked && WioCellular.workCalls==8);
    assert(WioCellular.registration==0);
    assert(network.getNetworkState()==Network::NetworkState::Searching);
    assert(WioCellular.configurationWrites==0);
  }

  // Poll the actual state before entering upstream begin(): missing URCs
  // no longer prevent APN, technology, and band configuration.
  reset();
  {
    Network network;configure(network);
    assert(begin(network));
    assert(WioCellular.workCalls==0);
    assert(WioCellular.registrationQueries>=2);
    assert(WioCellular.contexts[0].apn=="soracom.io");
    assert(WioCellular.actMode==0);
    assert(WioCellular.ltemBand==Network::NTTDOCOMO_LTEM_BAND);
    assert((WioCellular.radioCommands==std::vector<int>{0,1}));
    assert(WioCellular.functionality==1);
    assert(reported("network-config") && reported("radio-on"));
  }

  // A delayed transition is found through queries even without a URC.
  reset();
  {
    Network network;configure(network);WioCellular.pendingRegistrationQueries=3;
    assert(begin(network));
    assert(nowMs==500 && WioCellular.registrationQueries==4);
    assert(WioCellular.workCalls==0 && WioCellular.functionality==1);
  }

  // No changes required: upstream begin() does not restore CFUN itself.
  reset();
  {
    Network network;configure(network);
    WioCellular.contexts[0].apn="soracom.io";
    WioCellular.actMode=0;WioCellular.actSequence="0203";
    WioCellular.ltemBand=Network::NTTDOCOMO_LTEM_BAND;
    assert(begin(network));
    assert(WioCellular.configurationWrites==0);
    assert((WioCellular.radioCommands==std::vector<int>{0,1}));
    assert(WioCellular.functionality==1);
  }

  // A modem that remains registered must time out before upstream begin().
  // Also cover millis() rollover, using elapsed unsigned arithmetic.
  for(const uint32_t start:{0U,UINT32_MAX-499U}) {
    reset(start);
    Network network;configure(network);WioCellular.registrationStuck=true;
    assert(!begin(network));
    assert(uint32_t(nowMs-start)>=1000 && uint32_t(nowMs-start)<=1250);
    assert(WioCellular.configurationWrites==0 && WioCellular.workCalls==0);
    assert((WioCellular.radioCommands==std::vector<int>{0}));
    assert(reported("registration-timeout"));
  }

  for(const auto failure:{WioCellularResult::CommandRejected,WioCellularResult::ReadResponseTimeout}) {
    reset();
    {
      Network network;configure(network);WioCellular.radioOffResult=failure;
      assert(!begin(network));
      assert(WioCellular.registrationQueries==0 && WioCellular.configurationWrites==0);
      assert(reported("radio-off",failure));
    }
    reset();
    {
      Network network;configure(network);WioCellular.registrationResult=failure;
      assert(!begin(network));
      assert(WioCellular.registrationQueries==1 && WioCellular.configurationWrites==0);
      assert(reported("registration",failure));
    }
    reset();
    {
      Network network;configure(network);WioCellular.radioOnResult=failure;
      assert(!begin(network));
      assert(WioCellular.configurationWrites>0 && WioCellular.functionality==0);
      assert(reported("radio-on",failure));
    }
  }
  std::cout<<"Startup: reproduced upstream missing-URC stall; guarded configuration, "
              "radio restoration, timeout, rollover, and AT failures passed\n";
}
