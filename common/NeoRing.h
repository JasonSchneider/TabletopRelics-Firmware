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

  // Spin effect: single lit point chases around, with optional spill into neighbors.
  void showSpinStep(uint32_t color, uint8_t spillCount) {
    static uint8_t pos = 0;
    _ring.clear();
    _ring.setPixelColor(pointToLed(pos), color);
    float factor = 0.125f;
    for (uint8_t i = 1; i <= spillCount; i++) {
      uint32_t dim = dimColor(color, factor);
      _ring.setPixelColor(pointToLed((pos + _numPoints - i) % _numPoints), dim);
      _ring.setPixelColor(pointToLed((pos + i) % _numPoints), dim);
      factor *= 0.125f;
    }
    _ring.show();
    pos = (pos + 1) % _numPoints;
  }

  // Light one compass point with spill into neighbors at diminishing brightness.
  // spillCount = 0 → single LED; 1 → +1 neighbor each side at 50%; 2 → also 33%; etc.
  // Each spill ring is 12.5% the brightness of the ring inside it:
  // level 1 = 12.5%, level 2 = 1.56%, level 3 = ~0.2%, level 4 = ~0.02%
  void showPointWithSpill(uint8_t point, uint32_t color, uint8_t spillCount) {
    _ring.clear();
    _ring.setPixelColor(pointToLed(point % _numPoints), color);
    float factor = 0.125f;
    for (uint8_t i = 1; i <= spillCount; i++) {
      uint32_t dim = dimColor(color, factor);
      _ring.setPixelColor(pointToLed((point + _numPoints - i) % _numPoints), dim);
      _ring.setPixelColor(pointToLed((point + i) % _numPoints), dim);
      factor *= 0.125f;
    }
    _ring.show();
  }

  // Light all compass points at the same color.
  void showAllPoints(uint32_t color) {
    _ring.clear();
    for (uint8_t i = 0; i < _numPoints; i++) {
      _ring.setPixelColor(pointToLed(i), color);
    }
    _ring.show();
  }

  // Pulse effect: breathes via sine wave. phaseStep controls speed (call at fixed 20 ms).
  // allLeds=true pulses every point; otherwise pulses one point with optional spill.
  void showPulseStep(uint8_t point, uint32_t color, float phaseStep, bool allLeds, uint8_t spillCount) {
    static float phase = 0.0f;
    float brightness = (sinf(phase) + 1.0f) / 2.0f;
    _ring.clear();
    if (allLeds) {
      for (uint8_t i = 0; i < _numPoints; i++) {
        _ring.setPixelColor(pointToLed(i), dimColor(color, brightness));
      }
    } else {
      _ring.setPixelColor(pointToLed(point % _numPoints), dimColor(color, brightness));
      float spillFactor = brightness * 0.125f;
      for (uint8_t i = 1; i <= spillCount; i++) {
        uint32_t spillDim = dimColor(color, spillFactor);
        _ring.setPixelColor(pointToLed((point + _numPoints - i) % _numPoints), spillDim);
        _ring.setPixelColor(pointToLed((point + i) % _numPoints), spillDim);
        spillFactor *= 0.25f;
      }
    }
    _ring.show();
    phase += phaseStep;
    if (phase > 2.0f * PI) phase -= 2.0f * PI;
  }

  // Random effect: a random point lights up each step then clears.
  void showRandomStep(uint32_t color) {
    uint8_t point = random(0, _numPoints);
    _ring.clear();
    _ring.setPixelColor(pointToLed(point), color);
    _ring.show();
  }

  // Pick a vivid random color from a palette so results are never muddy.
  static uint32_t randomVividColor() {
    static const uint32_t palette[] = {
      color(255,   0,  80),  // crimson
      color(255, 140,   0),  // orange
      color(255, 200,   0),  // gold
      color(  0, 255,  80),  // green
      color(  0, 200, 255),  // cyan
      color(  0,  80, 255),  // blue
      color(140,   0, 255),  // violet
      color(255,   0, 200),  // magenta
    };
    return palette[random(0, 8)];
  }

  // Clockwise chase around all active points — call once on BLE connect.
  // Blocking: takes numPoints * stepMs * passes milliseconds total.
  void connectionAnimation(uint32_t color, uint8_t passes = 2, uint16_t stepMs = 50) {
    for (uint8_t p = 0; p < passes; p++) {
      for (uint8_t i = 0; i < _numPoints; i++) {
        _ring.clear();
        _ring.setPixelColor(pointToLed(i), color);
        _ring.show();
        delay(stepMs);
      }
    }
    // Brief full-ring flash to mark the end of the sequence
    for (uint8_t i = 0; i < _numPoints; i++) {
      _ring.setPixelColor(pointToLed(i), dimColor(color, 0.4));
    }
    _ring.show();
    delay(120);
    _ring.clear();
    _ring.show();
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
