//
// Tabletop Relics — Magic Compass
//
// Hardware: Adafruit HUZZAH32 v2 (ESP32), LSM9DS1 IMU, 24-LED NeoPixel ring
// BLE:      NimBLE GATT server — mirrors the protocol in docs/ble-protocol.md
//
// Modes:
//   ambient   — LSM9DS1 reads real north, tilt-compensated, shown in blue
//   commanded — app sends a bearing, shown in gold
//   calibrate — sweeps the ring while collecting hard-iron offsets
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
NimBLEServer*         bleServer    = nullptr;
NimBLECharacteristic* stateChar    = nullptr;
NimBLECharacteristic* telemetryChar = nullptr;
NimBLECharacteristic* batteryChar  = nullptr;
bool                  deviceConnected = false;

// ------------------------------------------------------------------ State
String mode          = COMPASS_MODE_AMBIENT;
float  commandedBearing = 0.0;
float  currentBearing   = 0.0;

// ------------------------------------------------------------------ Timers
unsigned long lastSensorRead  = 0;
unsigned long lastBleNotify   = 0;
unsigned long lastSweepStep   = 0;

// ================================================================ BLE helpers

void notifyState() {
  if (!deviceConnected || !stateChar) return;

  StaticJsonDocument<128> doc;
  doc["type"]        = DEVICE_TYPE_COMPASS;
  doc["mode"]        = mode;
  doc["bearing"]     = (int)round(currentBearing);
  doc["calibrated"]  = sensor.isCalibrated();

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
  void onConnect(NimBLEServer* s) override {
    deviceConnected = true;
    notifyState();
  }
  void onDisconnect(NimBLEServer* s) override {
    deviceConnected = false;
    // Restart advertising so the app can reconnect
    NimBLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    String raw = c->getValue().c_str();
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, raw) != DeserializationError::Ok) return;

    const char* cmd = doc["cmd"];
    if (!cmd) return;

    if (strcmp(cmd, CMD_COMPASS_SET_BEARING) == 0) {
      commandedBearing = doc["bearing"] | 0.0f;
      // Clamp to 0–359
      commandedBearing = fmod(commandedBearing + 360.0, 360.0);
      mode = COMPASS_MODE_COMMANDED;
      currentBearing = commandedBearing;
      ring.showBearing(currentBearing, COLOR_NORTH);
      notifyState();

    } else if (strcmp(cmd, CMD_COMPASS_SET_MODE) == 0) {
      const char* m = doc["mode"];
      if (m) {
        mode = m;
        if (mode == COMPASS_MODE_CALIBRATE) {
          sensor.startCalibration();
        }
        notifyState();
      }

    } else if (strcmp(cmd, CMD_COMPASS_CALIBRATE) == 0) {
      // Explicit calibrate command also starts the routine
      mode = COMPASS_MODE_CALIBRATE;
      sensor.startCalibration();
      notifyState();

    } else if (strcmp(cmd, CMD_PING) == 0) {
      notifyState(); // pong via state notification

    } else if (strcmp(cmd, CMD_IDENTIFY) == 0) {
      // Flash all active points briefly
      ring.clear();
      for (uint8_t i = 0; i < RING_NUM_POINTS; i++) {
        ring.showPoint(i, COLOR_NORTH);
        delay(60);
        ring.clear();
      }
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

  // Device info — READ
  NimBLECharacteristic* infoChar = svc->createCharacteristic(
    UUID_DEVICE_INFO, NIMBLE_PROPERTY::READ);
  sendDeviceInfo(infoChar);

  // State — READ + NOTIFY
  stateChar = svc->createCharacteristic(
    UUID_STATE, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Command — WRITE
  NimBLECharacteristic* cmdChar = svc->createCharacteristic(
    UUID_COMMAND, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  cmdChar->setCallbacks(new CommandCallbacks());

  // Battery — READ + NOTIFY (stub; wire up ADC later)
  batteryChar = svc->createCharacteristic(
    UUID_BATTERY, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  uint8_t batt = 100;
  batteryChar->setValue(&batt, 1);

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_SERVICE);
  adv->setScanResponse(true);
  NimBLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);

  ring.begin();

  if (!sensor.begin()) {
    Serial.println("LSM9DS1 not found — check wiring");
    // Flash an error pattern so it's obvious at the bench
    while (true) {
      ring.showPoint(0, NeoRing::color(255, 0, 0));
      delay(300);
      ring.clear();
      delay(300);
    }
  }

  setupBle();
  Serial.println("Magic Compass ready");
}

// ================================================================ Loop

void loop() {
  unsigned long now = millis();

  // --- Ambient mode: read sensor and update ring
  if (mode == COMPASS_MODE_AMBIENT && now - lastSensorRead >= SENSOR_UPDATE_MS) {
    lastSensorRead = now;
    currentBearing = sensor.heading();
    ring.showBearing(currentBearing, COLOR_AMBIENT);
  }

  // --- Calibration mode: sweep ring and collect samples
  if (mode == COMPASS_MODE_CALIBRATE) {
    sensor.updateCalibration();
    if (now - lastSweepStep >= SWEEP_INTERVAL_MS) {
      lastSweepStep = now;
      ring.showSweep(COLOR_CALIBRATE);
    }
  }

  // --- Periodic BLE state notification
  if (deviceConnected && now - lastBleNotify >= BLE_NOTIFY_MS) {
    lastBleNotify = now;
    notifyState();
  }
}
