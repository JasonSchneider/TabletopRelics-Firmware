#pragma once
#include <Adafruit_NeoPixel.h>

// NeoRing wraps an Adafruit_NeoPixel and provides bearing-to-LED mapping.
// Knows about the physical ring size, the number of active compass points,
// and which physical index corresponds to North.

class NeoRing {
public:
  NeoRing(uint8_t pin, uint8_t physicalCount, uint8_t numPoints, uint8_t northLed)
    : _ring(physicalCount, pin, NEO_GRB + NEO_KHZ800),
      _physicalCount(physicalCount),
      _numPoints(numPoints),
      _northLed(northLed) {}

  void begin() {
    _ring.begin();
    _ring.setBrightness(80);
    _ring.clear();
    _ring.show();
  }

  // Light the compass point nearest to bearingDeg (0–359).
  // Adjacent points fade to give a softer look.
  void showBearing(float bearingDeg, uint32_t color) {
    _ring.clear();

    float pointsPerDeg = _numPoints / 360.0;
    float exactPoint   = bearingDeg * pointsPerDeg;
    int   nearPoint    = (int)round(exactPoint) % _numPoints;
    int   prevPoint    = (nearPoint - 1 + _numPoints) % _numPoints;
    int   nextPoint    = (nearPoint + 1) % _numPoints;

    // Neighbours at 30% brightness
    uint32_t dim = dimColor(color, 0.30);
    _ring.setPixelColor(pointToLed(prevPoint), dim);
    _ring.setPixelColor(pointToLed(nextPoint), dim);
    _ring.setPixelColor(pointToLed(nearPoint), color);
    _ring.show();
  }

  // Light a single named compass point (0 = N, 1 = NE, … 7 = NW).
  void showPoint(uint8_t point, uint32_t color) {
    _ring.clear();
    _ring.setPixelColor(pointToLed(point % _numPoints), color);
    _ring.show();
  }

  // Slow rotating sweep — used during calibration.
  void showSweep(uint32_t color) {
    static uint8_t pos = 0;
    _ring.clear();
    _ring.setPixelColor(pointToLed(pos), color);
    _ring.show();
    pos = (pos + 1) % _numPoints;
  }

  void clear() {
    _ring.clear();
    _ring.show();
  }

  uint8_t numPoints() const { return _numPoints; }

  // Convenience: pack RGB into a uint32_t
  static uint32_t color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  }

private:
  Adafruit_NeoPixel _ring;
  uint8_t _physicalCount;
  uint8_t _numPoints;
  uint8_t _northLed; // physical LED index that faces North in the enclosure

  // Convert logical compass point to physical LED index.
  uint8_t pointToLed(uint8_t point) const {
    uint8_t stride = _physicalCount / _numPoints;
    return (_northLed + point * stride) % _physicalCount;
  }

  static uint32_t dimColor(uint32_t color, float factor) {
    uint8_t r = ((color >> 16) & 0xFF) * factor;
    uint8_t g = ((color >> 8)  & 0xFF) * factor;
    uint8_t b = (color         & 0xFF) * factor;
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  }
};
