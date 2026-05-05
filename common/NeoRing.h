#pragma once
#include <Adafruit_NeoPixel.h>

// NeoRing wraps an Adafruit_NeoPixel and provides bearing-to-LED mapping.
// Knows about the physical ring size, the number of active compass points,
// and which physical index corresponds to North.
//
// Brightness model
// ─────────────────
// NeoPixel's setBrightness() is NOT used for runtime control — it modifies
// stored pixel values in-place and causes precision loss when called repeatedly.
// Instead every render method accepts an explicit `brightness` float (0.0–1.0)
// that is applied only to the PRIMARY LED.  Spread neighbours are computed
// directly from the original full-intensity `color` using spreadIntensity^n,
// so they are NEVER attenuated by the brightness value — only by the spread
// falloff the user configured.

class NeoRing {
public:
  NeoRing(uint8_t pin, uint8_t physicalCount, uint8_t numPoints, uint8_t northLed)
    : _ring(physicalCount, pin, NEO_GRB + NEO_KHZ800),
      _physicalCount(physicalCount),
      _numPoints(numPoints),
      _northLed(northLed) {}

  void begin() {
    _ring.begin();
    _ring.setBrightness(255); // full hardware range — brightness controlled in software
    _ring.clear();
    _ring.show();
  }

  // Public dim helper so callers (compass.ino) can pre-dim colours.
  static uint32_t dim(uint32_t color, float factor) {
    return dimColor(color, factor);
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
    uint32_t dimmed = dimColor(color, 0.30);
    _ring.setPixelColor(pointToLed(prevPoint), dimmed);
    _ring.setPixelColor(pointToLed(nextPoint), dimmed);
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

  // Spin effect: single lit point chases around.
  // Primary LED drawn at `brightness`; spread neighbours drawn from full `color`
  // at spreadIntensity^n per ring — independent of brightness.
  void showSpinStep(uint32_t color, uint8_t spillCount, bool cw,
                    float brightness, float spreadIntensity) {
    static uint8_t pos = 0;
    _ring.clear();
    _ring.setPixelColor(pointToLed(pos), dimColor(color, brightness));
    float factor = spreadIntensity;
    for (uint8_t i = 1; i <= spillCount; i++) {
      uint32_t dim = dimColor(color, factor);
      _ring.setPixelColor(pointToLed((pos + _numPoints - i) % _numPoints), dim);
      _ring.setPixelColor(pointToLed((pos + i) % _numPoints), dim);
      factor *= spreadIntensity;
    }
    _ring.show();
    pos = cw ? (pos + 1) % _numPoints : (pos + _numPoints - 1) % _numPoints;
  }

  // Spin-pulse: chasing point breathes via a sine wave.
  // Call at a fixed 20 ms tick. advanceSpin=true when the spin interval has elapsed.
  // Primary LED = sineValue * brightness; spread neighbours from full color at spreadIntensity^n.
  void showSpinPulseStep(uint32_t color, float phaseStep, uint8_t spillCount,
                         bool cw, bool allLeds, bool advanceSpin,
                         float brightness, float spreadIntensity) {
    static uint8_t spinPos    = 0;
    static float   pulsePhase = 0.0f;

    if (advanceSpin) {
      spinPos = cw ? (spinPos + 1) % _numPoints : (spinPos + _numPoints - 1) % _numPoints;
    }

    float sineVal = (sinf(pulsePhase) + 1.0f) / 2.0f;
    float primaryBrightness = sineVal * brightness;
    _ring.clear();
    if (allLeds) {
      for (uint8_t i = 0; i < _numPoints; i++) {
        _ring.setPixelColor(pointToLed(i), dimColor(color, primaryBrightness));
      }
    } else {
      _ring.setPixelColor(pointToLed(spinPos), dimColor(color, primaryBrightness));
      float factor = spreadIntensity;
      for (uint8_t i = 1; i <= spillCount; i++) {
        _ring.setPixelColor(pointToLed((spinPos + _numPoints - i) % _numPoints), dimColor(color, factor));
        _ring.setPixelColor(pointToLed((spinPos + i) % _numPoints), dimColor(color, factor));
        factor *= spreadIntensity;
      }
    }
    _ring.show();

    pulsePhase += phaseStep;
    if (pulsePhase > 2.0f * PI) pulsePhase -= 2.0f * PI;
  }

  // Light one compass point with spread into neighbours.
  // Primary LED at `brightness`; spread neighbours from full `color` at spreadIntensity^n.
  void showPointWithSpill(uint8_t point, uint32_t color, uint8_t spillCount,
                          float brightness, float spreadIntensity) {
    _ring.clear();
    _ring.setPixelColor(pointToLed(point % _numPoints), dimColor(color, brightness));
    float factor = spreadIntensity;
    for (uint8_t i = 1; i <= spillCount; i++) {
      uint32_t d = dimColor(color, factor);
      _ring.setPixelColor(pointToLed((point + _numPoints - i) % _numPoints), d);
      _ring.setPixelColor(pointToLed((point + i) % _numPoints), d);
      factor *= spreadIntensity;
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
  // Primary LED = sineValue * brightness; spread neighbours from full color at spreadIntensity^n.
  void showPulseStep(uint8_t point, uint32_t color, float phaseStep, bool allLeds,
                     uint8_t spillCount, float brightness, float spreadIntensity) {
    static float phase = 0.0f;
    float sineVal = (sinf(phase) + 1.0f) / 2.0f;
    float primaryBrightness = sineVal * brightness;
    _ring.clear();
    if (allLeds) {
      for (uint8_t i = 0; i < _numPoints; i++) {
        _ring.setPixelColor(pointToLed(i), dimColor(color, primaryBrightness));
      }
    } else {
      _ring.setPixelColor(pointToLed(point % _numPoints), dimColor(color, primaryBrightness));
      float factor = spreadIntensity;
      for (uint8_t i = 1; i <= spillCount; i++) {
        uint32_t d = dimColor(color, factor);
        _ring.setPixelColor(pointToLed((point + _numPoints - i) % _numPoints), d);
        _ring.setPixelColor(pointToLed((point + i) % _numPoints), d);
        factor *= spreadIntensity;
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

  // Set hardware brightness cap (0–255). Use sparingly — prefer software brightness.
  void setBrightness(uint8_t b) {
    _ring.setBrightness(b);
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
