# Tabletop Relics — Firmware

Arduino firmware for the Tabletop Relics BLE props. Each prop has its own
sketch under its folder; shared BLE protocol and utilities live in `common/`.

## Props

| Folder | Hardware | Status |
|--------|----------|--------|
| `compass/` | HUZZAH32 v2, LSM9DS1, 24-LED NeoPixel ring (8 active) | In progress |
| `lantern/` | HUZZAH32 v2, NeoPixel, I2S mic | Planned |
| `fairy-stones/` | HUZZAH32 v2, NeoPixel, ESP-NOW mesh | Planned |

## Dependencies

Install these via the Arduino Library Manager before compiling:

- **NimBLE-Arduino** — BLE GATT server (lighter than the built-in ESP32 BLE)
- **Adafruit NeoPixel** — LED ring control
- **Adafruit LSM9DS1** — IMU / magnetometer (compass only)
- **Adafruit Unified Sensor** — required by LSM9DS1 library
- **ArduinoJson** — JSON encode/decode for BLE commands

## Board setup

1. In Arduino IDE, add the ESP32 board package URL:
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Install **esp32 by Espressif Systems** via Boards Manager
3. Select **Adafruit ESP32 Feather** as the target board

## BLE protocol

The GATT service layout and command/state JSON shapes are documented in the
web app repo:
[`docs/ble-protocol.md`](https://github.com/JasonSchneider/TabletopRelics/blob/main/docs/ble-protocol.md)

`common/Protocol.h` mirrors those UUIDs and types in C++.

## Repo layout

```
common/
  Protocol.h        UUIDs, command/state constants shared by all props
  BleServer.h/.cpp  NimBLE GATT server setup
  NeoRing.h/.cpp    LED ring helper (bearing → LED index)
compass/
  compass.ino       Main sketch
  CompassSensor.h   LSM9DS1 tilt-compensated heading
  CompassConfig.h   Hardware pin and LED count constants
```
