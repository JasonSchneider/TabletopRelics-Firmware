#pragma once

// ------------------------------------------------- Hardware pins
#define PIN_NEOPIXEL    14   // NeoPixel ring data pin — adjust to your wiring
#define PIN_LSM9DS1_SDA 23   // I2C SDA (HUZZAH32 default)
#define PIN_LSM9DS1_SCL 22   // I2C SCL (HUZZAH32 default)

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

// ------------------------------------------------- Timing
#define SENSOR_UPDATE_MS   100   // how often to read LSM9DS1 in ambient mode
#define SWEEP_INTERVAL_MS   80   // calibration sweep step interval
#define BLE_NOTIFY_MS      500   // how often to push state notifications
