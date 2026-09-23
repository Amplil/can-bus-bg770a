# Wio BG770A + Grove CAN BUS

このリポジトリは、Wio BG770AのnRF52840からUART経由でGrove CAN BUS Module（GD32E103）を操作し、BG770A-GLでLTE通信する例です。**BG770A-GL自体にCANコントローラはありません。**

- 構成: Wio BG770A + UART型Grove CAN BUS Module。
- プログラム: [grove-can-bus-cellular.ino](examples/grove-can-bus-cellular/grove-can-bus-cellular.ino)、[grove-can-bus-command.ino](examples/grove-can-bus-command/grove-can-bus-command.ino)。
- CANドライバ: `src/grove-can-bus.h` の `WioCAN`。Grove UARTを使う `Serial1` を占有します。
- LTEライブラリ: Seeed WioCellular。cellularの例はSORACOMのAPNと宛先を使用します。

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
