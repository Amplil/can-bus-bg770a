# BG770A CAN Bus Module Library

[![Actions Status](https://github.com/arduino/arduino-cli-example/workflows/test/badge.svg)](https://github.com/arduino/arduino-cli-example/actions)
[![Spell Check](https://github.com/arduino/compile-sketches/workflows/Spell%20Check/badge.svg)](https://github.com/arduino/compile-sketches/actions?workflow=Spell+Check)

The BG770A CAN Bus module library provides your Arduino with CAN Bus capabilities through the Quectel BG770A cellular module. This library allows you to read and write messages to the CAN Bus while maintaining cellular connectivity. The BG770A is a powerful LTE Cat-M1/NB-IoT/EGPRS module that also includes integrated CAN Bus functionality.

CAN Bus is a messaging protocol system that lets various microcontrollers and sensors within a vehicle communicate with each other. CAN provides long distance, medium communication speed, and high reliability.

The BG770A module combines cellular connectivity with CAN Bus functionality, making it ideal for automotive IoT applications, fleet management, and remote vehicle monitoring.

With this library, you can:

1. Send CAN2.0 frames through BG770A
2. Receive CAN2.0 frames through BG770A
3. Get data from OBD-II ports
4. Set CAN filters and masks
5. Configure CAN Bus parameters
6. Use both cellular and CAN functionality simultaneously

## Installation

1. [Download the library](https://github.com/arduino-community/can-bus-bg770a/archive/refs/heads/main.zip)
2. Extract the zip file
3. In the Arduino IDE, navigate to Sketch > Include Library > Add .ZIP Library

## Repository Contents

* [**/examples**](./examples) - Example sketches for the library (.ino). Run these from the Arduino IDE.
* [**/src**](./src) - Source files for the library (.cpp, .h).
* [**keywords.txt**](./keywords.txt) - Keywords from this library that will be highlighted in the Arduino IDE.
* [**library.properties**](./library.properties) - General library properties for the Arduino package manager.

## Functions

- begin() - Initialize the BG770A module
- enableCAN() - Enable CAN Bus functionality
- disableCAN() - Disable CAN Bus functionality
- send() - Send CAN messages
- receive() - Receive CAN messages
- setCanRate() - Set CAN Bus baudrate
- setMask() - Set CAN filters
- setFilt() - Set CAN filters
- sendPid() - Send OBD-II PID requests
- debugMode() - Debug mode for AT commands
- debugPID() - Debug mode for PID testing
- isCANEnabled() - Check if CAN is enabled

## Hardware Setup

Connect the BG770A module to your Arduino:

- **VCC** to 3.3V (BG770A operates at 3.3V)
- **GND** to Ground
- **TX** to Arduino RX pin (e.g., Serial1 RX)
- **RX** to Arduino TX pin (e.g., Serial1 TX)
- **CAN_H** to CAN Bus High
- **CAN_L** to CAN Bus Low

## Examples

Here are examples implemented in this library. You can find more examples [here](./examples)

### Basic CAN Send Example

```cpp
// BG770A CAN SEND EXAMPLE
#include <BG770A_CAN.h>

BG770A_CAN can;

void setup() {
    Serial.begin(9600);
    can.begin();  // Initialize BG770A on Serial1
    
    // Enable CAN functionality
    if(can.enableCAN()) {
        Serial.println("CAN enabled successfully");
}

    // Set CAN baudrate to 500kbps
    can.setCanRate(CAN_RATE_500);
}

void loop() {
    unsigned char data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    // Send CAN frame: ID=0x123, Standard frame, Data frame, 8 bytes
    if(can.send(0x123, 0, 0, 8, data)) {
        Serial.println("CAN message sent successfully");
    }
    
    delay(1000);
}
```

### CAN Receive Example

```cpp
// BG770A CAN RECEIVE EXAMPLE
#include <BG770A_CAN.h>

BG770A_CAN can;

void setup() {
    Serial.begin(9600);
    can.begin();
    
    can.enableCAN();
    can.setCanRate(CAN_RATE_500);
    
    Serial.println("Waiting for CAN messages...");
}

void loop() {
    unsigned long id;
    unsigned char buf[8];
    
    if(can.receive(&id, buf)) {
        Serial.print("Received CAN ID: 0x");
        Serial.print(id, HEX);
        Serial.print(" Data: ");
        
        for(int i = 0; i < 8; i++) {
            Serial.print("0x");
            if(buf[i] < 0x10) Serial.print("0");
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    delay(10);
}
```

## BG770A Module Information

The BG770A is a series of LTE Cat-M1/NB-IoT/EGPRS modules with integrated CAN Bus functionality:

- **LTE Cat-M1/NB-IoT/EGPRS** connectivity
- **Integrated CAN Bus** controller
- **Low power consumption**
- **Wide temperature range** (-40°C to +85°C)
- **Compact size** suitable for automotive applications

## Compatible Boards

This library works with any Arduino-compatible board that has hardware serial ports, including:

- Arduino Uno (with SoftwareSerial)
- Arduino Mega
- Arduino Leonardo
- ESP32
- ESP8266
- Seeeduino XIAO
- And many others

## Troubleshooting

### Common Issues

1. **Module not responding**: Check power supply (3.3V), connections, and ensure proper grounding
2. **CAN messages not sending**: Verify CAN bus termination and that CAN is enabled
3. **No CAN messages received**: Check CAN bus connections and baudrate settings

### Debug Mode

Use the debug mode to send AT commands directly to the BG770A:

```cpp
void loop() {
    can.debugMode();  // Forward serial communication for debugging
}
```

## PC Simulation (No Hardware)

If you don't have the embedded board or a vehicle available, you can still develop and test
the parsing, validation, and packaging logic on a PC. This repo includes a lightweight
simulator that emits realistic OBD-II PID responses so you can iterate quickly without
physical hardware.

### Windows setup

1. Install **Python 3.11+** from https://www.python.org/downloads/
2. Confirm Python is on your PATH:

```bash
python --version
```

### Run the simulator

From the repo root:

```bash
python simulator/obd_simulator.py
```

This prints JSON on stdout containing:
- raw OBD-II frames (`frames`)
- decoded signal values (`signals`)
- a `timestamp`

You can pipe this output into your own tooling or use it to feed a local service.

### Logic-only parsing helpers

The `src/obd-logic.*` files provide pure parsing helpers (no Arduino dependencies) to
convert PID response bytes into human-readable values (RPM, speed, temperatures, etc.).
These helpers are intended to be reused in both the embedded firmware and your PC-side
simulation tooling.

## License

```
MIT License

Copyright (c) 2024 Arduino Community

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Support

For technical support and questions, please create an issue in the GitHub repository.
