#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>
namespace bg770a {
// RAM-only quarantine. No UTC is invented later from a different clock anchor.
// Not uploaded under the current endpoint contract; reboot loses this buffer.
class UnsyncedResponses {
public:
  static constexpr size_t Capacity=120;
  struct Response { uint64_t receivedMono; uint32_t id; uint8_t bytes[8]; };
private:
  Response rows_[Capacity]{};
  size_t head_=0,size_=0;
  uint32_t dropped_=0;
public:
  bool retain(uint64_t mono,uint32_t id,const uint8_t* bytes) {
    const bool evicted=size_==Capacity;
    if(evicted) {head_=(head_+1)%Capacity;--size_;++dropped_;}
    auto& row=rows_[(head_+size_++)%Capacity];
    row.receivedMono=mono;row.id=id;std::memcpy(row.bytes,bytes,8);
    return evicted;
  }
  size_t size() const {return size_;}
  uint32_t dropped() const {return dropped_;}
  const Response& at(size_t index) const {return rows_[(head_+index)%Capacity];}
};
}
