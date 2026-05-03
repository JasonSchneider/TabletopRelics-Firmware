#pragma once

// ------------------------------------------------------------------ UUIDs
// Must match src/ble/protocol.ts in the web app repo exactly.
// Custom 128-bit UUIDs derived from the "tabl" ASCII prefix (0x7461626c).

#define UUID_SERVICE        "7461626c-0000-1000-8000-00805f9b34fb"
#define UUID_DEVICE_INFO    "7461626c-0001-1000-8000-00805f9b34fb"  // READ
#define UUID_STATE          "7461626c-0002-1000-8000-00805f9b34fb"  // READ + NOTIFY
#define UUID_COMMAND        "7461626c-0003-1000-8000-00805f9b34fb"  // WRITE
#define UUID_TELEMETRY      "7461626c-0004-1000-8000-00805f9b34fb"  // NOTIFY (optional)
#define UUID_BATTERY        "7461626c-0005-1000-8000-00805f9b34fb"  // READ + NOTIFY

// ---------------------------------------------------------- Device types
#define DEVICE_TYPE_COMPASS      "compass"
#define DEVICE_TYPE_LANTERN      "lantern"
#define DEVICE_TYPE_FAIRY_STONES "fairy-stones"

// --------------------------------------------------------- Command field
// Web app encodes commands as { "op": "<name>", ...payload }
#define CMD_FIELD  "op"

// --------------------------------------------------------- Command names
// Universal
#define CMD_PING       "ping"
#define CMD_IDENTIFY   "identify"
#define CMD_SLEEP      "sleep"

// Compass — must match RelicCommand in protocol.ts
#define CMD_COMPASS_SET_TARGET   "compass.setTarget"   // payload: { bearing: 0–359 }
#define CMD_COMPASS_SET_MODE     "compass.setMode"      // payload: { mode: string }
#define CMD_COMPASS_SET_COLOR    "compass.setColor"     // payload: { r,g,b: 0–255 } or { random: true }
#define CMD_COMPASS_SET_SPEED    "compass.setSpeed"     // payload: { speed: 0–100 }
#define CMD_COMPASS_SET_SPILL    "compass.setSpill"     // payload: { spill: 0–4 } neighbors per side
#define CMD_COMPASS_SET_ALL      "compass.setAll"       // payload: { all: bool }
#define CMD_COMPASS_CALIBRATE    "compass.calibrate"

// -------------------------------------------------------- Compass modes
// Must match the mode union in RelicState (protocol.ts)
#define COMPASS_MODE_AMBIENT    "ambient"
#define COMPASS_MODE_QUEST      "quest"     // app-commanded bearing, gold LED
#define COMPASS_MODE_MANUAL     "manual"    // slider-driven, user-chosen color
#define COMPASS_MODE_SPIN       "spin"      // LED chases around the ring
#define COMPASS_MODE_PULSE      "pulse"     // LED breathes at target bearing
#define COMPASS_MODE_RANDOM     "random"    // random points flash and fade
#define COMPASS_MODE_CALIBRATE  "calibrate" // internal only — set via compass.calibrate command
#define COMPASS_MODE_OFF        "off"

// -------------------------------------------------------- Firmware version
#define FW_VERSION  "0.1.0"
#define HW_VERSION  "compass-r1"
