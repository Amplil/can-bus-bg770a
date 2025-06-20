/*
 * grove-can-bus.ino
 * Grove - CAN BUS Module based on GD32E103 for Wio BG770A
 * Copyright (C) Seeed K.K.
 * MIT License
 */

#include <Adafruit_TinyUSB.h>
#include <csignal>
#include <WioCellular.h>
#include "grove-can-bus.h"

WioCAN can;

static void abortHandler(int sig) {
  Serial.printf("ABORT: Signal %d received\n", sig);
  yield();

  vTaskSuspendAll();  // FreeRTOS
  while (true) {
    ledOn(LED_BUILTIN);
    nrfx_coredep_delay_us(100000);  // Spin
    ledOff(LED_BUILTIN);
    nrfx_coredep_delay_us(100000);  // Spin
  }
}

void setup() {
  signal(SIGABRT, abortHandler);
  Serial.begin(115200);
  {
    const auto start = millis();
    while (!Serial && millis() - start < 5000) {
      delay(2);
    }
  }
  Serial.println();
  Serial.println();
  Serial.println("Grove CAN Bus Module for Wio BG770A");
  Serial.println("=====================================");

  digitalWrite(LED_BUILTIN, HIGH);
  WioCellular.begin();
  digitalWrite(PIN_VGROVE_ENABLE, VGROVE_ENABLE_ON);
  delay(1000);
  // Initialize CAN module
  can.begin();
  /*
  // Set CAN bus rate to 500kbps (commonly used)
  if(can.setCanRate(CAN_RATE_500)) {
    Serial.println("CAN bus rate set to 500kbps: OK");
  } else {
    Serial.println("CAN bus rate set to 500kbps: FAILED");
  }

  // --- set mask and filter (standard frame) ---
  // Mask0: 0x7FC, Mask1: 0x7FC
  can.setMask(0, 0, 0x000007FC); // Mask0, standard
  can.setMask(1, 0, 0x000007FC); // Mask1, standard
  // Filt0-5: 0x7E8 (ECU response ID)
  for (int i = 0; i < 6; ++i) {
    can.setFilt(i, 0, 0x000007E8); // FiltN, standard
  }
  */
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // Example: Request engine RPM every 5 seconds using OBD-II
  static unsigned long lastSend = 0;
  static int sendInterval=5000;
  unsigned char cmd=PID_ENGIN_PRM;
  
  if(millis() - lastSend > sendInterval) {
    // OBD-II request for Engine RPM (PID 0x0C)
    can.sendPid(cmd);
    Serial.print("Sent OBD-II Engine RPM request (PID: 0x");
    Serial.print(cmd, HEX);
    Serial.println(")");
    lastSend = millis();
  }
  
  // Listen for OBD-II responses (Engine RPM and other data)
  unsigned long id = 0;
  unsigned char data[8];
  if(can.receive(&id, data)) {
    Serial.print("Received CAN message - ID: 0x");
    Serial.print(id, HEX);
    Serial.print(", Data: ");
    
    for(int i = 0; i < 8; i++) {
      Serial.print("0x");
      if(data[i] < 0x10) Serial.print("0");
      Serial.print(data[i], HEX);
      if(i < 7) Serial.print(" ");
    }
    Serial.println();
    
    // Check if this is an OBD-II response for Engine RPM (PID 0x0C)
    if((id >= 0x7E8 && id <= 0x7EF) && data[1] == 0x41 && data[2] == 0x0C) {
      // Calculate RPM: (A*256 + B) / 4
      // A = data[3], B = data[4]
      unsigned int rpm = ((data[3] * 256) + data[4]) / 4;
      Serial.print("Engine RPM: ");
      Serial.print(rpm);
      Serial.println(" rpm");
    }
  }
  // Optional: Enable debug mode by sending commands through Serial monitor
  // Uncomment the following line to enable debug mode
  //can.debugMode();
  can.debugPID();

  delay( 100 );

}
