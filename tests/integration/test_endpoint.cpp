#include "endpoint-ack.h"
#include <cassert>
#include <iostream>
#include <string>
using bg770a::EndpointAck;
static EndpointAck::Result parse(const std::string& text){EndpointAck p;for(auto c:text)p.feed(reinterpret_cast<const uint8_t*>(&c),1);return p.finish();}
static void feed(EndpointAck& parser,const std::string& text) {
 parser.feed(reinterpret_cast<const uint8_t*>(text.data()),text.size());
}
static void classified(const std::string& response,const char* category) {
 EndpointAck parser;
 for(char byte:response)feed(parser,std::string(1,byte));
 assert(parser.result()==EndpointAck::Result::Rejected);
 assert(parser.diagnosticComplete());
 assert(std::string(parser.diagnostic())==category);
 assert(parser.receivedBytes()==response.size());
 assert(parser.finish()==EndpointAck::Result::Rejected);
}
int main(){
 using R=EndpointAck::Result;
 // Exact provided receive body; device identifiers are deliberately absent.
 assert(parse("400 {\"error\":\"Invalid payload shape\",\"details\":{\"formErrors\":[],\"fieldErrors\":{\"data\":[\"data must contain at least one record\"]}}}")==R::Rejected);
 for(const char* bad:{"", "2", "20", "200", "200 ", "2000 {}", "200 {\"ok\":", "200 {\"ok\":true", "200 {bad}", "200 {}junk", "500 {}", "207 {\"result\":\"ng\"}", "200 {\"error\":\"failed\"}", "200 {\"ok\":false}", "OK", "HTTP/1.1 200 OK\r\n"}) {assert(parse(bad)==R::Rejected);}
 assert(parse("200 {\"result\":\"ok\"}")==R::Accepted);
 assert(parse("201")==R::Accepted);assert(parse("204")==R::Accepted);
 assert(parse("201 {\"ok\":true}")==R::Accepted);
 assert(parse("200 "+std::string(2048,'x'))==R::Rejected);
 // A negative status and its JSON reason may occupy separate TCP reads. The
 // delivery decision stays negative while the bounded diagnostic completes.
 EndpointAck fragmented;
 feed(fragmented,"400");assert(fragmented.status()==400);
 assert(!fragmented.diagnosticComplete());
 feed(fragmented," {\"error\":\"Invalid payload: decode or JSON failed\"");
 assert(fragmented.result()==R::Rejected);
 assert(!fragmented.diagnosticComplete());
 feed(fragmented,"}");
 assert(fragmented.result()==R::Rejected && fragmented.diagnosticComplete());
 assert(std::string(fragmented.diagnostic())=="payload_decode");
 assert(fragmented.receivedBytes()==std::string("400 {\"error\":\"Invalid payload: decode or JSON failed\"}").size());

 classified("400 {\"error\":\"Invalid request body\"}","request_shape");
 classified("400 {\"error\":\"Invalid payload: decode or JSON failed\"}","payload_decode");
 classified("400 {\"error\":\"Invalid payload shape\",\"details\":{\"fieldErrors\":{\"data\":[\"data must contain at least one record\"]}}}","data_empty");
 classified("400 {\"error\":\"Invalid payload shape\",\"details\":{\"fieldErrors\":{\"data\":[\"time must be ISO 8601 (e.g. 2025-08-11T23:59:54+09:00)\"]}}}","time_format");
 classified("400 {\"error\":\"Invalid payload shape\",\"details\":{\"fieldErrors\":{\"imsi\":[\"private-device-value\"]}}}","payload_shape");
 classified("207 {\"result\":\"ng\",\"detail\":{\"private\":\"credential\"}}","partial_failure");
 classified("401 {\"error\":\"private API key detail\"}","authentication_failed");
 classified("403 {\"error\":\"private authorization detail\"}","forbidden");
 classified("400 {\"error\":\"identity=private-device token=private-key\",\"url\":\"https://private.example\"}","endpoint_rejected");
 classified("400 https://private.example returns a status code (400).\r\n","endpoint_rejected");
 classified("HTTP/1.1 200 OK\r\n","unrecognized_response");

 EndpointAck truncated;
 feed(truncated,"400 {\"error\":\"Invalid payload shape\"");
 assert(truncated.result()==R::Rejected && !truncated.diagnosticComplete());
 assert(std::string(truncated.diagnostic())=="incomplete_response");
 truncated.finish();assert(truncated.diagnosticComplete());
 assert(std::string(truncated.diagnostic())=="endpoint_rejected");

 EndpointAck bounded;
 feed(bounded,"400 "+std::string(1000,'x'));
 feed(bounded,std::string(2000,'x'));
 assert(bounded.result()==R::Rejected && bounded.diagnosticComplete());
 assert(bounded.receivedBytes()==3004);
 assert(std::string(bounded.diagnostic())=="response_too_large");
 feed(bounded,"{\"error\":\"Invalid request body\"}");
 assert(bounded.result()==R::Rejected && std::string(bounded.diagnostic())=="response_too_large");

 // Adding diagnostic capture must preserve the established success contract.
 EndpointAck success;feed(success,"200 {\"ok\":true}");
 assert(success.result()==R::Accepted && success.diagnosticComplete());
 assert(std::string(success.diagnostic())=="accepted");
 feed(success," \r\n");assert(success.result()==R::Accepted);
 feed(success,"private-tail");assert(success.result()==R::Rejected);
 assert(std::string(success.diagnostic())=="endpoint_rejected");
 std::cout<<"SORACOM numeric acknowledgment: rejection retained; fragmented safe diagnostics, bounded capture, privacy, and existing success/negative contracts passed\n";
}
