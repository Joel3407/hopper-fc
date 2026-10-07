# hopper-fc

Flight-computer firmware for a planned EDF (electric ducted fan) thrust-vector
hopper, plus a host-side ground station. Teensy 4.1, C++ (Arduino framework,
PlatformIO). No airframe, vanes, servos, ESC or fan exist yet: the only hardware
is a Teensy 4.1 on a breadboard with the sensors below.

## Current code (`src/main.cpp`, ~800 lines)

- 100 Hz fixed loop: read sensors, estimate altitude, safety check, state
  machine, actuate, log (placeholder), serial commands, telemetry.
- BNO08X IMU (I2C 0x4B, fallback 0x4A): its own fused game-rotation-vector
  quaternion is converted to roll/pitch/yaw; there is no filter of our own.
- BMP580 barometer (I2C 0x47, Adafruit BMP5xx library), INA219 power monitor
  (I2C 0x40), GPS on `Serial1` at 9600 baud (GY-GPS6MV2, NEO-6M class). GPS is
  read but not used by control, telemetry or logging.
- 2-state Kalman filter (altitude AGL, vertical velocity): baro measurement,
  body-frame linear acceleration as input.
- Four PIDs (pitch, roll, yaw, altitude), control allocation to 4 vanes
  (N/E/S/W) and one ESC:
  `vane_N = -pitch + yaw`, `vane_E = roll - yaw`,
  `vane_S = pitch + yaw`, `vane_W = -roll - yaw`.
- State machine PREFLIGHT, ARMED, TAKEOFF, HOVER, LANDING, LANDED, ABORT;
  safety aborts on battery < 19.8 V, tilt > 45 deg, altitude > 20 m.
- Telemetry: 24-byte binary frame at 10 Hz on the second USB serial port
  (`SerialUSB1`), read by `ground-station/ground_station.py`. One-way only.
- Commands: single characters on the first USB serial port
  (`a d t l x r s`).
- Pins: vane servos 2/3/4/5 (N/E/S/W), ESC 6, default `Wire` for I2C.

## Not implemented

- SD logging: `log_data()` is commented out.
- LoRa telemetry: not started in `src/` (only in `archive/`).
- Servo/ESC output, gains, `HOVER_BASE_US` and `ROLL_SIGN`/`PITCH_SIGN` are
  untested; there is no hardware to test them on.

## Other directories

- `ground-station/`: Python telemetry viewer and `test_parser.py`.
- `tools/`: stand-alone sensor bring-up sketches (see `tools/README.md`).
- `archive/`: older sketch for earlier hardware, not built.

## Working notes

- Prefer clean, debuggable code over clever code.
- Ask about hardware specifics before assuming them.
- Do not upload to the board or push without being asked.
