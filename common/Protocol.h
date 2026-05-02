#pragma once

// ------------------------------------------------------------------ UUIDs
// Mirrors src/ble/protocol.ts in the web app repo.
// All UUIDs are namespaced with the "tabl" ASCII prefix (0x7461626c).

#define UUID_SERVICE        "7461626c-0001-0000-0000-000000000000"
#define UUID_DEVICE_INFO    "7461626c-0002-0000-0000-000000000000"
#define UUID_STATE          "7461626c-0003-0000-0000-000000000000"
#define UUID_COMMAND        "7461626c-0004-0000-0000-000000000000"
#define UUID_TELEMETRY      "7461626c-0005-0000-0000-000000000000"
#define UUID_BATTERY        "7461626c-0006-0000-0000-000000000000"

// ---------------------------------------------------------- Device types
#define DEVICE_TYPE_COMPASS      "compass"
#define DEVICE_TYPE_LANTERN      "lantern"
#define DEVICE_TYPE_FAIRY_STONES "fairy-stones"

// --------------------------------------------------------- Command types
// Universal
#define CMD_PING       "ping"
#define CMD_IDENTIFY   "identify"
#define CMD_SLEEP      "sleep"

// Compass
#define CMD_COMPASS_SET_BEARING  "compass.setBearing"
#define CMD_COMPASS_SET_MODE     "compass.setMode"
#define CMD_COMPASS_CALIBRATE    "compass.calibrate"

// Compass modes
#define COMPASS_MODE_AMBIENT    "ambient"   // real north via magnetometer
#define COMPASS_MODE_COMMANDED  "commanded" // bearing set by app
#define COMPASS_MODE_CALIBRATE  "calibrate" // running calibration routine

// -------------------------------------------------------- Firmware version
#define FW_VERSION "0.1.0"
