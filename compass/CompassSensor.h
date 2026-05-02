#pragma once
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// Tilt-compensated compass heading using the LSM303 magnetometer + accelerometer.
// Uses the Adafruit Unified Sensor interface (Adafruit_LSM303_U library).

class CompassSensor {
public:
  CompassSensor()
    : _mag(12345), _accel(54321), _calibrated(false) {
    _hardIronOffset[0] = 0;
    _hardIronOffset[1] = 0;
    _hardIronOffset[2] = 0;
  }

  bool begin() {
    if (!_mag.begin())   return false;
    if (!_accel.begin()) return false;
    return true;
  }

  // Returns heading in degrees (0 = North, clockwise), tilt-compensated.
  float heading() {
    sensors_event_t magEvent, accelEvent;
    _mag.getEvent(&magEvent);
    _accel.getEvent(&accelEvent);

    float mx = magEvent.magnetic.x - _hardIronOffset[0];
    float my = magEvent.magnetic.y - _hardIronOffset[1];
    float mz = magEvent.magnetic.z - _hardIronOffset[2];

    float ax = accelEvent.acceleration.x;
    float ay = accelEvent.acceleration.y;
    float az = accelEvent.acceleration.z;

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
    sensors_event_t magEvent;
    _mag.getEvent(&magEvent);
    float v[3] = { magEvent.magnetic.x, magEvent.magnetic.y, magEvent.magnetic.z };
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
  Adafruit_LSM303_Mag_Unified   _mag;
  Adafruit_LSM303_Accel_Unified _accel;
  bool  _calibrated;
  float _hardIronOffset[3];
  float _magMin[3], _magMax[3];
};
