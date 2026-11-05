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
    if(result_==Result::Accepted) {
      for(size_t i=0;i<count;++i)if(!space(char(bytes[i])))result_=Result::Rejected;
      return result_;
    }
    if(result_==Result::Rejected)return result_;
    if(buffer_.size()+count>2048){result_=Result::Rejected;return result_;}
    buffer_.append(reinterpret_cast<const char*>(bytes),count);
    evaluate(false);return result_;
  }
  Result finish(){evaluate(true);if(result_==Result::Pending)result_=Result::Rejected;return result_;}
  Result result()const{return result_;}
  unsigned status()const{return status_;}
 private:
  std::string buffer_;unsigned status_=0;Result result_=Result::Pending;
  static bool space(char c){return c==' '||c=='\r'||c=='\n'||c=='\t';}
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
