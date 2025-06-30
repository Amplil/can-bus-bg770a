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
  // Set CAN bus rate to 500kbps (commonly used)
  if(can.setCanRate(CAN_RATE_500)) {
    Serial.println("CAN bus rate set to 500kbps: OK");
  } else {
    Serial.println("CAN bus rate set to 500kbps: FAILED");
  }

  // --- set mask and filter (standard frame) ---
  // Mask設定: 0x00000000で全てのIDを通す（全通し）
  if (can.setMask(0, 0, 0x00000000)) {
    Serial.println("Mask0 set: OK (All pass)");
  } else {
    Serial.println("Mask0 set: FAILED");
  }
  if (can.setMask(1, 0, 0x00000000)) {
    Serial.println("Mask1 set: OK (All pass)");
  } else {
    Serial.println("Mask1 set: FAILED");
  }
  
  // Filter設定: 0x00000000で全てのIDを通す（全通し）
  if (can.setFilt(0, 0, 0x00000000)) {
    Serial.println("Filt0 set: OK (All pass)");
  } else {
    Serial.println("Filt0 set: FAILED");
  }
  if (can.setFilt(1, 0, 0x00000000)) {
    Serial.println("Filt1 set: OK (All pass)");
  } else {
    Serial.println("Filt1 set: FAILED");
  }
  if (can.setFilt(2, 0, 0x00000000)) {
    Serial.println("Filt2 set: OK (All pass)");
  } else {
    Serial.println("Filt2 set: FAILED");
  }
  if (can.setFilt(3, 0, 0x00000000)) {
    Serial.println("Filt3 set: OK (All pass)");
  } else {
    Serial.println("Filt3 set: FAILED");
  }
  if (can.setFilt(4, 0, 0x00000000)) {
    Serial.println("Filt4 set: OK (All pass)");
  } else {
    Serial.println("Filt4 set: FAILED");
  }
  if (can.setFilt(5, 0, 0x00000000)) {
    Serial.println("Filt5 set: OK (All pass)");
  } else {
    Serial.println("Filt5 set: FAILED");
  }
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println();
  Serial.println("=== CAN Bus Initialization Complete ===");
  Serial.println("Listening for ALL CAN messages...");
  Serial.println("Format: [timestamp] CAN ID: 0xXXX | Data: XX XX XX XX XX XX XX XX | Description");
  Serial.println("========================================");
}

void loop() {
  // 統計情報用の変数
  static unsigned long totalMessages = 0;
  static unsigned long lastStatsTime = 0;
  static unsigned long statsInterval = 10000; // 10秒ごとに統計表示
  
  // Example: Request engine RPM every 5 seconds using OBD-II
  static unsigned long lastSend = 0;
  static int sendInterval = 5000;
  unsigned char cmd = PID_ENGIN_PRM;
  if(millis() - lastSend > sendInterval) {
    // OBD-II request for Engine RPM (PID 0x0C)
    can.sendPid(cmd);
    Serial.print("Sent OBD-II Engine RPM request (PID: 0x");
    Serial.print(cmd, HEX);
    Serial.println(")");
    lastSend = millis();
  }
  /*

  // 統計情報の表示
  if(millis() - lastStatsTime > statsInterval) {
    Serial.println("=== CAN Bus Statistics ===");
    Serial.print("Total messages received: ");
    Serial.println(totalMessages);
    Serial.print("Messages per second: ");
    Serial.println(totalMessages * 1000.0 / (millis() - lastStatsTime + statsInterval));
    Serial.println("==========================");
    lastStatsTime = millis();
  }
  
  // シリアルデータの状況をチェック
  if(Serial1.available()) {
    Serial.print("Serial1 data available: ");
    Serial.print(Serial1.available());
    Serial.println(" bytes");
  }
  */
  // すべてのCAN信号を受信・表示
  unsigned long id = 0;
  unsigned char data[8];
  if(can.receive(&id, data)) {
    totalMessages++;
    
    // 受信メッセージの詳細表示
    Serial.print("[");
    Serial.print(millis());
    Serial.print("ms] CAN ID: 0x");
    if(id < 0x100) Serial.print("0");
    if(id < 0x10) Serial.print("0");
    Serial.print(id, HEX);
    Serial.print(" | Data: ");
    
    for(int i = 0; i < 8; i++) {
      if(data[i] < 0x10) Serial.print("0");
      Serial.print(data[i], HEX);
      if(i < 7) Serial.print(" ");
    }
    
    // データの解釈を試行
    Serial.print(" | ");
    
    // OBD-II応答の場合
    if((id >= 0x7E8 && id <= 0x7EF)) {
      Serial.print("OBD-II Response from ECU ");
      Serial.print(id - 0x7E8);
      if(data[0] >= 2 && data[1] == 0x41) {
        Serial.print(" | PID: 0x");
        Serial.print(data[2], HEX);
        
        // 特定のPIDの解釈
        switch(data[2]) {
          case 0x0C: // Engine RPM
            if(data[0] >= 4) {
              unsigned int rpm = ((data[3] * 256) + data[4]) / 4;
              Serial.print(" | Engine RPM: ");
              Serial.print(rpm);
              Serial.print(" rpm");
            }
            break;
          case 0x0D: // Vehicle Speed
            Serial.print(" | Vehicle Speed: ");
            Serial.print(data[3]);
            Serial.print(" km/h");
            break;
          case 0x05: // Coolant Temperature
            Serial.print(" | Coolant Temp: ");
            Serial.print(data[3] - 40);
            Serial.print(" °C");
            break;
          case 0x04: // Engine Load
            Serial.print(" | Engine Load: ");
            Serial.print(data[3] * 100.0 / 255.0);
            Serial.print(" %");
            break;
        }
      }
    }
    // その他の一般的なCAN IDの場合
    else if(id >= 0x100 && id <= 0x7FF) {
      Serial.print("Standard CAN Frame");
    }
    else if(id >= 0x18000000) {
      Serial.print("Extended CAN Frame");
    }
    else {
      Serial.print("Unknown Frame Type");
    }
    
    Serial.println();
  }
  
  // Optional: Enable debug mode by sending commands through Serial monitor
  // Uncomment the following line to enable debug mode
  //can.debugMode();
  can.debugPID();

  delay(10);
}
