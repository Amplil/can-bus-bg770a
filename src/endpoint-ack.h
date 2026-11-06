#pragma once
#include <ArduinoJson.h>
#include <stddef.h>
#include <stdint.h>
#include <string>
namespace bg770a {
// SORACOM Unified Endpoint TCP: numeric status + body, not an HTTP status line.
// Accept complete JSON success bodies or documented bare 201/204 at EOF only.
class EndpointAck {
 public:
  enum class Result { Pending, Accepted, Rejected };
  Result feed(const uint8_t* bytes,size_t count) {
    received_ = count > SIZE_MAX-received_ ? SIZE_MAX : received_+count;
    if(result_==Result::Accepted) {
      for(size_t i=0;i<count;++i)if(!space(char(bytes[i])))result_=Result::Rejected;
      return result_;
    }
    // A negative status is final for delivery, but its reason may arrive in a
    // later TCP read. Keep bounded private bytes for fixed-literal diagnostics.
    const size_t available=2048-buffer_.size();
    if(count>available) {
      buffer_.append(reinterpret_cast<const char*>(bytes),available);
      oversized_=true;result_=Result::Rejected;return result_;
    }
    buffer_.append(reinterpret_cast<const char*>(bytes),count);
    evaluate(false);return result_;
  }
  Result finish(){ended_=true;evaluate(true);if(result_==Result::Pending)result_=Result::Rejected;return result_;}
  Result result()const{return result_;}
  unsigned status()const{return status_;}
  size_t receivedBytes()const{return received_;}
  bool diagnosticComplete()const {
    if(ended_ || oversized_ || result_==Result::Accepted)return true;
    if(!numericPrefix())return !buffer_.empty();
    if(buffer_.size()<4)return false;
    if(!space(buffer_[3]))return true;
    const size_t body=bodyStart();
    if(body==buffer_.size())return false;
    if(buffer_[body]=='{' || buffer_[body]=='[')return jsonEnd(body)!=0;
    return buffer_.find_first_of("\r\n",body)!=std::string::npos;
  }
  // Never expose a server-supplied string: it may contain identity, URL,
  // authentication details, or an echoed request. Only known categories leave
  // this parser. The original response body remains private and bounded.
  const char* diagnostic()const {
    if(oversized_)return "response_too_large";
    if(result_==Result::Accepted)return "accepted";
    if(!numericPrefix())return "unrecognized_response";
    if(status_==401)return "authentication_failed";
    if(status_==403)return "forbidden";
    if(status_==207)return "partial_failure";
    if(!diagnosticComplete())return "incomplete_response";
    const size_t body=bodyStart();
    if(body<buffer_.size() && (buffer_[body]=='{' || buffer_[body]=='[')) {
      JsonDocument doc;
      if(!deserializeJson(doc,buffer_.data()+body,buffer_.size()-body)) {
        if(doc["error"]=="Invalid request body")return "request_shape";
        if(doc["error"]=="Invalid payload: decode or JSON failed")return "payload_decode";
        if(doc["error"]=="Invalid payload shape") {
          for(const char* field:{"data","time"}) {
            for(JsonVariantConst message:doc["details"]["fieldErrors"][field].as<JsonArrayConst>()) {
              if(message=="data must contain at least one record")return "data_empty";
              if(message=="time must be ISO 8601 (e.g. 2025-08-11T23:59:54+09:00)")return "time_format";
            }
          }
          return "payload_shape";
        }
      }
    }
    return "endpoint_rejected";
  }
 private:
  std::string buffer_;unsigned status_=0;Result result_=Result::Pending;
  size_t received_=0;bool ended_=false,oversized_=false;
  static bool space(char c){return c==' '||c=='\r'||c=='\n'||c=='\t';}
  bool numericPrefix()const {
    for(size_t i=0;i<buffer_.size() && i<3;++i)
      if(buffer_[i]<'0' || buffer_[i]>'9')return false;
    return true;
  }
  size_t bodyStart()const {
    size_t body=buffer_.size()<3 ? buffer_.size() : 3;
    while(body<buffer_.size() && space(buffer_[body]))++body;
    return body;
  }
  size_t jsonEnd(size_t body)const {
    bool quoted=false,escaped=false;int depth=0;
    for(size_t i=body;i<buffer_.size();++i) {
      const char c=buffer_[i];
      if(quoted) {if(escaped)escaped=false;else if(c=='\\')escaped=true;else if(c=='"')quoted=false;continue;}
      if(c=='"')quoted=true;
      else if(c=='{' || c=='[')++depth;
      else if(c=='}' || c==']') {if(--depth<=0)return i+1;}
    }
    return 0;
  }
  void evaluate(bool ended) {
    if(result_!=Result::Pending)return;
    for(size_t i=0;i<buffer_.size() && i<3;++i)
      if(buffer_[i]<'0'||buffer_[i]>'9'){result_=Result::Rejected;return;}
    if(buffer_.size()<3)return;
    status_=unsigned(buffer_[0]-'0')*100+unsigned(buffer_[1]-'0')*10+unsigned(buffer_[2]-'0');
    if(buffer_.size()>3 && !space(buffer_[3])){result_=Result::Rejected;return;}
    if(buffer_.size()==3 && !ended)return;
    // 207 is a partial failure in Unified format. Other/custom status contracts
    // are unconfirmed; do not silently discard the queue for them.
    if(status_!=200 && status_!=201 && status_!=204){result_=Result::Rejected;return;}
    size_t body=3;while(body<buffer_.size() && space(buffer_[body]))++body;
    if(body==buffer_.size()) {
      if(ended && (status_==201 || status_==204))result_=Result::Accepted;
      return;
    }
    if(buffer_[body]!='{' && buffer_[body]!='['){result_=Result::Rejected;return;}
    // Find a whole JSON value before considering success, including split reads.
    bool quoted=false,escaped=false;int depth=0;size_t end=body;
    for(;end<buffer_.size();++end) {
      const char c=buffer_[end];
      if(quoted){if(escaped)escaped=false;else if(c=='\\')escaped=true;else if(c=='"')quoted=false;continue;}
      if(c=='"')quoted=true;
      else if(c=='{'||c=='[')++depth;
      else if(c=='}'||c==']'){if(--depth==0){++end;break;}}
    }
    if(depth || quoted)return;
    for(size_t i=end;i<buffer_.size();++i)if(!space(buffer_[i])){result_=Result::Rejected;return;}
    JsonDocument doc;
    if(deserializeJson(doc,buffer_.data()+body,end-body) ||
       !doc["error"].isNull() || doc["result"]=="ng" || doc["ok"]==false) {
      result_=Result::Rejected;return;
    }
    result_=Result::Accepted;
  }
};
}
