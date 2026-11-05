#include "endpoint-ack.h"
#include <cassert>
#include <iostream>
#include <string>
using bg770a::EndpointAck;
static EndpointAck::Result parse(const std::string& text){EndpointAck p;for(auto c:text)p.feed(reinterpret_cast<const uint8_t*>(&c),1);return p.finish();}
int main(){
 using R=EndpointAck::Result;
 // Exact provided receive body; device identifiers are deliberately absent.
 assert(parse("400 {\"error\":\"Invalid payload shape\",\"details\":{\"formErrors\":[],\"fieldErrors\":{\"data\":[\"data must contain at least one record\"]}}}")==R::Rejected);
 for(const char* bad:{"", "2", "20", "200", "200 ", "2000 {}", "200 {\"ok\":", "200 {\"ok\":true", "200 {bad}", "200 {}junk", "500 {}", "207 {\"result\":\"ng\"}", "200 {\"error\":\"failed\"}", "200 {\"ok\":false}", "OK", "HTTP/1.1 200 OK\r\n"}) {assert(parse(bad)==R::Rejected);}
 assert(parse("200 {\"result\":\"ok\"}")==R::Accepted);
 assert(parse("201")==R::Accepted);assert(parse("204")==R::Accepted);
 assert(parse("201 {\"ok\":true}")==R::Accepted);
 assert(parse("200 "+std::string(2048,'x'))==R::Rejected);
 std::cout<<"SORACOM numeric acknowledgment: exact 400, fragmentation, complete JSON, partial/unknown/truncated rejection passed\n";
}
