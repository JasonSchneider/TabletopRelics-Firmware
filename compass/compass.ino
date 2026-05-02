//
// Tabletop Relics — Magic Compass
//
// Hardware: Adafruit HUZZAH32 v2 (ESP32), LSM9DS1 IMU, 24-LED NeoPixel ring
// BLE:      NimBLE GATT server — mirrors the protocol in docs/ble-protocol.md
//
// Behaviour:
//   Power-on  → ambient mode: LED snaps to whichever of the 8 points faces
//               north. Hysteresis prevents flicker on boundaries.
//   BLE connect → green clockwise chase animation, then resume current mode.
//   Commanded → app sends a bearing, ring switches to gold and holds it.
//

#include <Arduino.h>
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
NimBLEServer*         bleServer     = nullptr;
NimBLECharacteristic* stateChar     = nullptr;
NimBLECharacteristic* batteryChar   = nullptr;
bool                  deviceConnected      = false;
volatile bool         connectionAnimPending = false; // set by BLE task, handled in loop

// ------------------------------------------------------------------ State
String  mode             = COMPASS_MODE_AMBIENT;
float   commandedBearing = 0.0;
float   currentBearing   = 0.0;
uint8_t currentPoint     = 0;    // last lit compass point, for hysteresis

// ------------------------------------------------------------------ Timers
unsigned long lastSensorRead = 0;
unsigned long lastBleNotify  = 0;
unsigned long lastSweepStep  = 0;

// ================================================================ Helpers

// Snap a bearing (0–359°) to the nearest of NUM_POINTS compass points.
uint8_t bearingToPoint(float bearing) {
  float step = 360.0 / RING_NUM_POINTS;
  return (uint8_t)(fmod(bearing + step * 0.5, 360.0) / step) % RING_NUM_POINTS;
}

// Same but applies hysteresis: only moves to a new point if the heading has
// crossed far enough past the boundary to avoid flickering.
uint8_t bearingToPointHysteresis(float bearing, uint8_t lastPoint) {
  uint8_t candidate = bearingToPoint(bearing);
  if (candidate == lastPoint) return lastPoint;

  // Angle of the boundary between lastPoint and candidate
  float step          = 360.0 / RING_NUM_POINTS;
  float lastCenter    = lastPoint * step;
  float candidateCenter = candidate * step;
  float boundary      = fmod(lastCenter + (candidateCenter - lastCenter) * 0.5 + 360.0, 360.0);

  // Angular distance from the boundary — negative means we haven't crossed far
  float dist = fmod(bearing - boundary + 360.0, 360.0);
  if (dist > 180.0) dist -= 360.0; // signed: positive = past boundary

  return (dist >= HEADING_HYSTERESIS_DEG) ? candidate : lastPoint;
}

// Refresh the LED ring to match the current mode and bearing.
void updateRing() {
  if (mode == COMPASS_MODE_AMBIENT) {
    uint8_t point = bearingToPointHysteresis(currentBearing, currentPoint);
    if (point != currentPoint) {
      currentPoint = point;
      ring.showPoint(currentPoint, COLOR_AMBIENT);
    }
  } else if (mode == COMPASS_MODE_COMMANDED) {
    uint8_t point = bearingToPoint(commandedBearing);
    if (point != currentPoint) {
      currentPoint = point;
      ring.showPoint(currentPoint, COLOR_NORTH);
    }
  }
  // Calibrate mode drives the ring directly from the loop via showSweep()
}

// ================================================================ BLE helpers

void notifyState() {
  if (!deviceConnected || !stateChar) return;

  StaticJsonDocument<128> doc;
  doc["type"]       = DEVICE_TYPE_COMPASS;
  doc["mode"]       = mode;
  doc["bearing"]    = (int)round(currentBearing);
  doc["calibrated"] = sensor.isCalibrated();

  char buf[128];
  serializeJson(doc, buf);
  stateChar->setValue(buf);
  stateChar->notify();
}

void sendDeviceInfo(NimBLECharacteristic* c) {
  StaticJsonDocument<128> doc;
  doc["type"] = DEVICE_TYPE_COMPASS;
  doc["fw"]   = FW_VERSION;
  doc["name"] = "Magic Compass";

  char buf[128];
  serializeJson(doc, buf);
  c->setValue(buf);
}

// ================================================================ BLE callbacks

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& connInfo) override {
    deviceConnected       = true;
    connectionAnimPending = true; // main loop will run the animation
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    // Return to ambient mode so the compass stays useful without the app
    mode = COMPASS_MODE_AMBIENT;
    NimBLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    String raw = c->getValue().c_str();
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, raw) != DeserializationError::Ok) return;

    const char* cmd = doc["cmd"];
    if (!cmd) return;

    if (strcmp(cmd, CMD_COMPASS_SET_BEARING) == 0) {
      commandedBearing = fmod(doc["bearing"].as<float>() + 360.0, 360.0);
      mode = COMPASS_MODE_COMMANDED;
      currentBearing = commandedBearing;
      currentPoint = 255; // force ring refresh on next updateRing()
      notifyState();

    } else if (strcmp(cmd, CMD_COMPASS_SET_MODE) == 0) {
      const char* m = doc["mode"];
      if (m) {
        mode = m;
        if (mode == COMPASS_MODE_CALIBRATE) sensor.startCalibration();
        currentPoint = 255; // force ring refresh
        notifyState();
      }

    } else if (strcmp(cmd, CMD_COMPASS_CALIBRATE) == 0) {
      mode = COMPASS_MODE_CALIBRATE;
      sensor.startCalibration();
      notifyState();

    } else if (strcmp(cmd, CMD_PING) == 0) {
      notifyState();

    } else if (strcmp(cmd, CMD_IDENTIFY) == 0) {
      // Brief identify flash — clockwise at double speed
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
  uint8_t batt = 100;
  batteryChar->setValue(&batt, 1);

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_SERVICE);
  NimBLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);
  ring.begin();

  if (!sensor.begin()) {
    Serial.println("LSM9DS1 not found — check wiring");
    // Blink north LED red until reset
    while (true) {
      ring.showPoint(0, NeoRing::color(255, 0, 0));
      delay(300);
      ring.clear();
      delay(300);
    }
  }

  // Initial heading so the ring lights immediately on boot
  currentBearing = sensor.heading();
  currentPoint   = bearingToPoint(currentBearing);
  ring.showPoint(currentPoint, COLOR_AMBIENT);

  setupBle();
  Serial.println("Magic Compass ready — advertising");
}

// ================================================================ Loop

void loop() {
  unsigned long now = millis();

  // --- Connection animation (flagged by BLE callback, run here on main task)
  if (connectionAnimPending) {
    connectionAnimPending = false;
    ring.connectionAnimation(COLOR_CONNECT);
    currentPoint = 255; // force ring refresh after animation
    updateRing();
    notifyState();
  }

  // --- Ambient: read sensor and snap to nearest north-facing point
  if (mode == COMPASS_MODE_AMBIENT && now - lastSensorRead >= SENSOR_UPDATE_MS) {
    lastSensorRead = now;
    currentBearing = sensor.heading();
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

  // --- Periodic BLE state push
  if (deviceConnected && now - lastBleNotify >= BLE_NOTIFY_MS) {
    lastBleNotify = now;
    notifyState();
  }
}
