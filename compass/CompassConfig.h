#pragma once

// ------------------------------------------------- Hardware pins
#define PIN_NEOPIXEL    25   // A1 on HUZZAH32 v2 = GPIO 25
#define PIN_LSM9DS1_SDA 22   // SDA on HUZZAH32 v2 = GPIO 22
#define PIN_LSM9DS1_SCL 20   // SCL on HUZZAH32 v2 = GPIO 20

// ------------------------------------------------- LED ring
#define RING_LED_COUNT   24  // physical LEDs on the ring
#define RING_NUM_POINTS   8  // active compass points (N NE E SE S SW W NW)
                             // change to 16 when enclosure expands
#define RING_NORTH_LED    0  // physical LED index facing North in the enclosure
                             // adjust if ring is rotated during assembly

// ------------------------------------------------- Colors (RGB)
#define COLOR_NORTH      NeoRing::color(255, 200,   0)  // warm gold — commanded
#define COLOR_AMBIENT    NeoRing::color(  0, 180, 255)  // cool blue — real north
#define COLOR_CALIBRATE  NeoRing::color(120,   0, 255)  // violet sweep
#define COLOR_CONNECT    NeoRing::color(  0, 255, 140)  // green flash — BLE connected
#define COLOR_FAULT      NeoRing::color(255,   0,   0)  // red — sensor fault

// ------------------------------------------------- Ambient hysteresis
// Heading must move this many degrees past a boundary before the LED snaps
// to the next point. Prevents flicker when heading sits on a boundary.
#define HEADING_HYSTERESIS_DEG  5.0

// ------------------------------------------------- Battery / power
// On HUZZAH32 v2, GPIO 35 (A13) reads VBAT/2 via a 100k/100k divider.
// ADC is 12-bit (0–4095), 3.3 V reference.  Calibration values below are
// approximate; adjust if your board reads differently.
#define PIN_BATTERY_ADC    35        // GPIO 35 = A13 on HUZZAH32 v2
#define BATTERY_ADC_EMPTY  1860      // ~3.0 V battery → ADC ≈ 1860
#define BATTERY_ADC_FULL   2600      // ~4.2 V battery → ADC ≈ 2600

// Optional: define this to the GPIO connected to the MCP73831 STAT pin
// (active-LOW when charging).  Leave commented out if not wired.
// #define PIN_CHARGE_STAT    <gpio>

// ------------------------------------------------- Timing
#define SENSOR_UPDATE_MS    100   // how often to read LSM9DS1 in ambient mode
#define SWEEP_INTERVAL_MS    80   // calibration sweep step interval
#define BLE_NOTIFY_MS       500   // how often to push state notifications
#define BATTERY_NOTIFY_MS 30000   // how often to push battery notifications
#define FAULT_BLINK_MS      600   // fault-mode red blink half-period
#define SENSOR_RETRY_MS    5000   // how often to retry a failed sensor init
