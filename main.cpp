#include <Arduino.h>
#include "src/grove-can-bus.h"

WioCAN can;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Wio BG770A CAN demo starting...");

  can.begin();
  can.setCanRate(CAN_RATE_500);
}

void loop() {
  unsigned char data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  can.send(0x123, 0, 0, 8, data);
  Serial.println("Sent CAN frame 0x123");
  delay(1000);
}
