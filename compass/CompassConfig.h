#pragma once

// ------------------------------------------------- Hardware pins
#define PIN_NEOPIXEL    25   // A1 on HUZZAH32 v2 = GPIO 25
#define PIN_LSM9DS1_SDA 23   // SDA on HUZZAH32 v2 = GPIO 23
#define PIN_LSM9DS1_SCL 22   // SCL on HUZZAH32 v2 = GPIO 22

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

// ------------------------------------------------- Ambient hysteresis
// Heading must move this many degrees past a boundary before the LED snaps
// to the next point. Prevents flicker when heading sits on a boundary.
#define HEADING_HYSTERESIS_DEG  5.0

// ------------------------------------------------- Timing
#define SENSOR_UPDATE_MS   100   // how often to read LSM9DS1 in ambient mode
#define SWEEP_INTERVAL_MS   80   // calibration sweep step interval
#define BLE_NOTIFY_MS      500   // how often to push state notifications
