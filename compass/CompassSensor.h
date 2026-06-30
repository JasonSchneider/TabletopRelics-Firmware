#pragma once
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

// Tilt-compensated compass heading using the LSM9DS1 magnetometer + accelerometer.

class CompassSensor {
public:
  CompassSensor() : _calibrated(false) {
    _hardIronOffset[0] = 0;
    _hardIronOffset[1] = 0;
    _hardIronOffset[2] = 0;
  }

  static void recoverBus(uint8_t sda, uint8_t scl) {
    pinMode(scl, OUTPUT);
    pinMode(sda, INPUT_PULLUP);
    digitalWrite(scl, HIGH);
    // Clock out up to 9 pulses until SDA floats high (device releases the bus).
    for (uint8_t i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
      digitalWrite(scl, LOW);  delayMicroseconds(5);
      digitalWrite(scl, HIGH); delayMicroseconds(5);
    }
    // Issue a STOP condition: SDA rises while SCL is high.
    pinMode(sda, OUTPUT);
    digitalWrite(sda, LOW);  delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
    digitalWrite(sda, HIGH); delayMicroseconds(5);
    // Return both pins to input so Wire.begin() can reconfigure them cleanly.
    pinMode(sda, INPUT);
    pinMode(scl, INPUT);
    delay(10);
  }

  bool begin(uint8_t sda, uint8_t scl) {
    recoverBus(sda, scl);
    Wire.begin(sda, scl);
    if (!_lsm.begin()) return false;
    _lsm.setupAccel(_lsm.LSM9DS1_ACCELRANGE_2G);
    _lsm.setupMag(_lsm.LSM9DS1_MAGGAIN_4GAUSS);
    _lsm.setupGyro(_lsm.LSM9DS1_GYROSCALE_245DPS);
    return true;
  }

  // Returns heading in degrees (0 = North, clockwise), tilt-compensated.
  float heading() {
    sensors_event_t accel, mag, gyro, temp;
    _lsm.getEvent(&accel, &mag, &gyro, &temp);

    float mx = mag.magnetic.x - _hardIronOffset[0];
    float my = mag.magnetic.y - _hardIronOffset[1];
    float mz = mag.magnetic.z - _hardIronOffset[2];

    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;

    float roll  = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    float Xh = mx * cos(pitch) + mz * sin(pitch);
    float Yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    float h = atan2(Yh, Xh) * 180.0 / PI;
    if (h < 0) h += 360.0;
    return h;
  }

  void startCalibration() {
    _magMin[0] = _magMin[1] = _magMin[2] =  1e9;
    _magMax[0] = _magMax[1] = _magMax[2] = -1e9;
    _calibrated = false;
  }

  void updateCalibration() {
    sensors_event_t accel, mag, gyro, temp;
    _lsm.getEvent(&accel, &mag, &gyro, &temp);
    float v[3] = { mag.magnetic.x, mag.magnetic.y, mag.magnetic.z };
    for (int i = 0; i < 3; i++) {
      if (v[i] < _magMin[i]) _magMin[i] = v[i];
      if (v[i] > _magMax[i]) _magMax[i] = v[i];
    }
  }

  void finishCalibration() {
    for (int i = 0; i < 3; i++) {
      _hardIronOffset[i] = (_magMax[i] + _magMin[i]) / 2.0;
    }
    _calibrated = true;
  }

  bool isCalibrated() const { return _calibrated; }

private:
  Adafruit_LSM9DS1 _lsm;
  bool  _calibrated;
  float _hardIronOffset[3];
  float _magMin[3], _magMax[3];
};
