//
// Tabletop Relics — Magic Compass
//
// Hardware: Adafruit HUZZAH32 v2 (ESP32), LSM9DS1 IMU, 24-LED NeoPixel ring
// BLE:      NimBLE GATT server — mirrors the protocol in docs/ble-protocol.md
//
// Boot order:
//   1. LED hardware test
//   2. BLE advertising starts
//   3. Sensor init (graceful: if it fails, enter fault mode and keep retrying)
//
// Behaviour:
//   Power-on  → ambient mode: LED snaps to whichever of the 8 points faces
//               north. Hysteresis prevents flicker on boundaries.
//   BLE connect → green clockwise chase animation, then resume current mode.
//   Commanded → app sends a bearing, ring switches to gold and holds it.
//   Fault     → north LED pulses red, BLE still works, sensor retried every 5s.
//

#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <ArduinoJson.h>

#include "../common/Protocol.h"
#include "CompassConfig.h"
#include "CompassSensor.h"
#include "../common/NeoRing.h"

// ------------------------------------------------------------------ Objects
NeoRing        ring(PIN_NEOPIXEL, RING_LED_COUNT, RING_NUM_POINTS, RING_NORTH_LED);
CompassSensor  sensor;

// ------------------------------------------------------------------ BLE
NimBLEServer*         bleServer            = nullptr;
NimBLECharacteristic* stateChar            = nullptr;
NimBLECharacteristic* batteryChar          = nullptr;
bool                  deviceConnected      = false;
volatile bool         connectionAnimPending = false;

// ------------------------------------------------------------------ State
String   mode               = COMPASS_MODE_AMBIENT;
float    targetBearing      = 0.0;
float    currentBearing     = 0.0;
uint8_t  currentPoint       = 0;
bool     sensorAvailable    = false;
uint32_t ledColor           = COLOR_AMBIENT;
uint8_t  effectSpeed        = 50;   // 0–100; mapped to step interval / phase step
bool     randomColorEnabled = false;
uint8_t  spillCount         = 0;    // 0–4 neighbors per side lit at diminishing brightness
bool     allLedsEnabled     = false; // light all compass points at once

// ------------------------------------------------------------------ Timers
unsigned long lastSensorRead    = 0;
unsigned long lastBleNotify     = 0;
unsigned long lastBatteryNotify = 0;
unsigned long lastSweepStep     = 0;
unsigned long lastPulseStep     = 0;
unsigned long lastSensorRetry   = 0;
unsigned long lastFaultBlink    = 0;
bool          faultLedOn        = false;

// ================================================================ Helpers

uint8_t bearingToPoint(float bearing) {
  float step = 360.0 / RING_NUM_POINTS;
  return (uint8_t)(fmod(bearing + step * 0.5, 360.0) / step) % RING_NUM_POINTS;
}

uint8_t bearingToPointHysteresis(float bearing, uint8_t lastPoint) {
  uint8_t candidate = bearingToPoint(bearing);
  if (candidate == lastPoint) return lastPoint;

  float step            = 360.0 / RING_NUM_POINTS;
  float lastCenter      = lastPoint * step;
  float candidateCenter = candidate * step;
  float boundary        = fmod(lastCenter + (candidateCenter - lastCenter) * 0.5 + 360.0, 360.0);

  float dist = fmod(bearing - boundary + 360.0, 360.0);
  if (dist > 180.0) dist -= 360.0;

  return (dist >= HEADING_HYSTERESIS_DEG) ? candidate : lastPoint;
}

uint8_t readBatteryPercent() {
  int raw = analogRead(PIN_BATTERY_ADC);
  int pct = map(raw, BATTERY_ADC_EMPTY, BATTERY_ADC_FULL, 0, 100);
  return (uint8_t)constrain(pct, 0, 100);
}

bool isCharging() {
#ifdef PIN_CHARGE_STAT
  return digitalRead(PIN_CHARGE_STAT) == LOW;
#else
  return false;
#endif
}

// Map speed 0–100 to a step interval in milliseconds (100 = fast, 0 = slow).
unsigned long speedToMs(uint8_t speed) {
  uint8_t s = speed > 100 ? 100 : speed;
  return 20 + (unsigned long)(980 - s * 9.6);
}

// Map speed 0–100 to a sine-wave phase step per 20 ms tick.
// speed=0 → ~6 s/cycle; speed=50 → ~0.5 s/cycle; speed=100 → ~0.25 s/cycle.
float speedToPhaseStep(uint8_t speed) {
  uint8_t s = speed > 100 ? 100 : speed;
  return 0.02f + s * 0.0048f;
}

// Returns the active color, picking a random vivid one if randomColorEnabled.
uint32_t activeColor() {
  return randomColorEnabled ? NeoRing::randomVividColor() : ledColor;
}

void updateRing() {
  if (mode == COMPASS_MODE_AMBIENT) {
    uint8_t point = bearingToPointHysteresis(currentBearing, currentPoint);
    if (point != currentPoint) {
      currentPoint = point;
      ring.showPoint(currentPoint, COLOR_AMBIENT);
    }
  } else if (mode == COMPASS_MODE_QUEST) {
    uint8_t point = bearingToPoint(targetBearing);
    if (point != currentPoint) {
      currentPoint = point;
      ring.showPointWithSpill(currentPoint, COLOR_NORTH, spillCount);
    }
  } else if (mode == COMPASS_MODE_MANUAL) {
    uint8_t point = bearingToPoint(targetBearing);
    if (point != currentPoint) {
      currentPoint = point;
      if (allLedsEnabled) {
        ring.showAllPoints(activeColor());
      } else {
        ring.showPointWithSpill(currentPoint, activeColor(), spillCount);
      }
    }
  } else if (mode == COMPASS_MODE_OFF) {
    ring.clear();
  }
  // Spin / Pulse / Random are driven frame-by-frame in the loop, not here.
}

bool trySensorInit() {
  Serial.println("Initialising LSM9DS1...");
  if (!sensor.begin()) {
    Serial.println("LSM9DS1 not found — will retry");
    return false;
  }
  Serial.println("LSM9DS1 OK");
  currentBearing = sensor.heading();
  currentPoint   = bearingToPoint(currentBearing);
  Serial.printf("Initial heading: %.1f° → compass point %d\n", currentBearing, currentPoint);
  ring.showPoint(currentPoint, COLOR_AMBIENT);
  return true;
}

// ================================================================ BLE helpers

void notifyState() {
  if (!deviceConnected || !stateChar) return;

  // Field names must match the RelicState type in src/ble/protocol.ts
  StaticJsonDocument<192> doc;
  doc["type"]       = DEVICE_TYPE_COMPASS;
  doc["mode"]       = mode;
  doc["heading"]    = (int)round(currentBearing);
  doc["target"]     = (int)round(targetBearing);
  doc["calibrated"] = sensor.isCalibrated();
  doc["charging"]   = isCharging();

  char buf[192];
  serializeJson(doc, buf);
  stateChar->setValue(buf);
  stateChar->notify();
}

void sendDeviceInfo(NimBLECharacteristic* c) {
  // Fields must match the DeviceInfo interface in src/ble/protocol.ts
  StaticJsonDocument<128> doc;
  doc["type"]   = DEVICE_TYPE_COMPASS;
  doc["hw"]     = HW_VERSION;
  doc["fw"]     = FW_VERSION;
  doc["serial"] = String((uint32_t)ESP.getEfuseMac(), HEX);

  char buf[128];
  serializeJson(doc, buf);
  c->setValue(buf);
}

// ================================================================ BLE callbacks

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& connInfo) override {
    deviceConnected       = true;
    connectionAnimPending = true;
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    mode = COMPASS_MODE_AMBIENT;
    NimBLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    String raw = c->getValue().c_str();
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, raw) != DeserializationError::Ok) return;

    const char* op = doc[CMD_FIELD];
    if (!op) return;

    if (strcmp(op, CMD_COMPASS_SET_TARGET) == 0) {
      targetBearing = fmod(doc["bearing"].as<float>() + 360.0, 360.0);
      // Don't override manual mode — setTarget is used by both quest and manual
      if (mode != COMPASS_MODE_MANUAL) mode = COMPASS_MODE_QUEST;
      currentPoint = 255;
      updateRing();
      notifyState();

    } else if (strcmp(op, CMD_COMPASS_SET_COLOR) == 0) {
      if (doc["random"].as<bool>()) {
        randomColorEnabled = true;
      } else {
        randomColorEnabled = false;
        ledColor = NeoRing::color(
          doc["r"].as<uint8_t>(),
          doc["g"].as<uint8_t>(),
          doc["b"].as<uint8_t>()
        );
      }
      currentPoint = 255;
      updateRing();

    } else if (strcmp(op, CMD_COMPASS_SET_SPEED) == 0) {
      effectSpeed = doc["speed"].as<uint8_t>();
      // No ring update needed — loop picks up new interval/phase step automatically

    } else if (strcmp(op, CMD_COMPASS_SET_SPILL) == 0) {
      uint8_t s = doc["spill"].as<uint8_t>();
      spillCount = s > 4 ? 4 : s;
      currentPoint = 255;
      updateRing();

    } else if (strcmp(op, CMD_COMPASS_SET_ALL) == 0) {
      allLedsEnabled = doc["all"].as<bool>();
      currentPoint = 255;
      updateRing();

    } else if (strcmp(op, CMD_COMPASS_SET_MODE) == 0) {
      const char* m = doc["mode"];
      if (m) {
        mode = m;
        if (mode == COMPASS_MODE_CALIBRATE && sensorAvailable) sensor.startCalibration();
        currentPoint = 255;
        updateRing();
        notifyState();
      }

    } else if (strcmp(op, CMD_COMPASS_CALIBRATE) == 0) {
      if (sensorAvailable) {
        mode = COMPASS_MODE_CALIBRATE;
        sensor.startCalibration();
        notifyState();
      }

    } else if (strcmp(op, CMD_PING) == 0) {
      notifyState();

    } else if (strcmp(op, CMD_IDENTIFY) == 0) {
      ring.connectionAnimation(COLOR_CONNECT, 1, 30);
      currentPoint = 255;
      updateRing();
    }
  }
};

// ================================================================ Setup

void setupBle() {
  NimBLEDevice::init("Magic Compass");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  NimBLEService* svc = bleServer->createService(UUID_SERVICE);

  NimBLECharacteristic* infoChar = svc->createCharacteristic(
    UUID_DEVICE_INFO, NIMBLE_PROPERTY::READ);
  sendDeviceInfo(infoChar);

  stateChar = svc->createCharacteristic(
    UUID_STATE, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  NimBLECharacteristic* cmdChar = svc->createCharacteristic(
    UUID_COMMAND, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  cmdChar->setCallbacks(new CommandCallbacks());

  batteryChar = svc->createCharacteristic(
    UUID_BATTERY, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  uint8_t batt = readBatteryPercent();
  batteryChar->setValue(&batt, 1);

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_SERVICE);
  adv->setName("Magic Compass");
  NimBLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Magic Compass booting ===");

  // 1. LED hardware test — light each compass point white in sequence.
  //    If nothing appears the problem is wiring or PIN_NEOPIXEL.
  ring.begin();
  Serial.printf("NeoPixel ring on GPIO %d — hardware test\n", PIN_NEOPIXEL);
  for (uint8_t i = 0; i < RING_NUM_POINTS; i++) {
    ring.showPoint(i, NeoRing::color(255, 255, 255));
    delay(150);
  }
  ring.clear();
  delay(200);
  Serial.println("NeoPixel test done");

  // 2. BLE
  setupBle();
  Serial.println("BLE advertising — ready");

  // 3. Sensor — DISABLED: Wire.begin() triggers TG1WDT_SYS_RESET on this board,
  //    cause unknown. Skipping until I2C issue is resolved. sensorAvailable stays
  //    false so the device runs in fault mode (red blink) but BLE works normally.
  // TODO: re-enable once Wire/LSM9DS1 init is debugged
  // sensorAvailable = trySensorInit();
  Serial.println("Sensor disabled — running in fault mode");
}

// ================================================================ Loop

void loop() {
  unsigned long now = millis();

  // --- Connection animation
  if (connectionAnimPending) {
    connectionAnimPending = false;
    ring.connectionAnimation(COLOR_CONNECT);
    currentPoint = 255;
    updateRing();
    notifyState();
  }

  // --- Fault blink: only in sensor-dependent modes (ambient / calibrate).
  //     Quest, manual, and all effects work without the sensor.
  if (!sensorAvailable &&
      (mode == COMPASS_MODE_AMBIENT || mode == COMPASS_MODE_CALIBRATE)) {
    if (now - lastFaultBlink >= FAULT_BLINK_MS) {
      lastFaultBlink = now;
      faultLedOn = !faultLedOn;
      if (faultLedOn) ring.showPoint(0, COLOR_FAULT);
      else            ring.clear();
    }
    // Sensor retry disabled — see compass_sensor_issue memory note
    if (deviceConnected && now - lastBleNotify >= BLE_NOTIFY_MS) {
      lastBleNotify = now;
      notifyState();
    }
    return;
  }

  // --- Ambient: read sensor and snap to nearest compass point
  if (mode == COMPASS_MODE_AMBIENT && now - lastSensorRead >= SENSOR_UPDATE_MS) {
    lastSensorRead = now;
    float prevBearing = currentBearing;
    currentBearing = sensor.heading();
    uint8_t newPoint = bearingToPointHysteresis(currentBearing, currentPoint);

    static unsigned long lastDebugPrint = 0;
    if (now - lastDebugPrint >= 2000) {
      lastDebugPrint = now;
      Serial.printf("Heading: %.1f° → point %d (was %.1f° / point %d) BLE: %s\n",
        currentBearing, newPoint, prevBearing, currentPoint,
        deviceConnected ? "connected" : "advertising");
    }

    updateRing();
  }

  // --- Calibration: collect samples and sweep the ring
  if (mode == COMPASS_MODE_CALIBRATE) {
    sensor.updateCalibration();
    if (now - lastSweepStep >= SWEEP_INTERVAL_MS) {
      lastSweepStep = now;
      ring.showSweep(COLOR_CALIBRATE);
    }
  }

  // --- Spin / Random: speed controls step interval
  if (mode == COMPASS_MODE_SPIN || mode == COMPASS_MODE_RANDOM) {
    if (now - lastSweepStep >= speedToMs(effectSpeed)) {
      lastSweepStep = now;
      uint32_t c = activeColor();
      if (mode == COMPASS_MODE_SPIN)   ring.showSpinStep(c);
      else                             ring.showRandomStep(c);
    }
  }

  // --- Pulse: fixed 20 ms tick, speed controls sine-wave phase advance rate
  if (mode == COMPASS_MODE_PULSE) {
    if (now - lastPulseStep >= 20) {
      lastPulseStep = now;
      ring.showPulseStep(
        bearingToPoint(targetBearing),
        activeColor(),
        speedToPhaseStep(effectSpeed),
        allLedsEnabled,
        spillCount
      );
    }
  }

  // --- Periodic BLE state push
  if (deviceConnected && now - lastBleNotify >= BLE_NOTIFY_MS) {
    lastBleNotify = now;
    notifyState();
  }

  // --- Periodic battery notify
  if (deviceConnected && batteryChar && now - lastBatteryNotify >= BATTERY_NOTIFY_MS) {
    lastBatteryNotify = now;
    uint8_t batt = readBatteryPercent();
    batteryChar->setValue(&batt, 1);
    batteryChar->notify();
  }
}
