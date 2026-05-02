#pragma once
#include <Adafruit_LSM9DS1.h>

// Tilt-compensated compass heading from the LSM9DS1.
// Without tilt compensation the heading drifts as soon as the prop is
// not perfectly level — this uses the accelerometer to correct for roll
// and pitch before computing the magnetic bearing.

class CompassSensor {
public:
  CompassSensor() : _calibrated(false) {
    _hardIronOffset[0] = 0;
    _hardIronOffset[1] = 0;
    _hardIronOffset[2] = 0;
  }

  bool begin() {
    if (!_lsm.begin()) return false;
    _lsm.setupAccel(_lsm.LSM9DS1_ACCELRANGE_2G);
    _lsm.setupMag(_lsm.LSM9DS1_MAGGAIN_4GAUSS);
    _lsm.setupGyro(_lsm.LSM9DS1_GYROSCALE_245DPS);
    return true;
  }

  // Returns heading in degrees (0 = North, clockwise), or -1 on read error.
  float heading() {
    sensors_event_t accel, mag, gyro, temp;
    _lsm.getEvent(&accel, &mag, &gyro, &temp);

    // Apply hard-iron offset collected during calibration
    float mx = mag.magnetic.x - _hardIronOffset[0];
    float my = mag.magnetic.y - _hardIronOffset[1];

    // Tilt compensation using accelerometer
    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;

    float roll  = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    float mz = mag.magnetic.z - _hardIronOffset[2];
    float Xh = mx * cos(pitch) + mz * sin(pitch);
    float Yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    float heading = atan2(Yh, Xh) * 180.0 / PI;
    if (heading < 0) heading += 360.0;
    return heading;
  }

  // Call startCalibration(), then rotate the prop slowly through 360° in
  // all orientations, calling updateCalibration() in the loop, then
  // finishCalibration(). Hard-iron offsets are stored in RAM (not flash yet).
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
  bool   _calibrated;
  float  _hardIronOffset[3];
  float  _magMin[3], _magMax[3];
};
