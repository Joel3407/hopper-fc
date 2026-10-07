# hopper-fc

Flight-computer firmware for a planned electric-ducted-fan (EDF) hopper that
steers with four thrust vanes, plus a Python ground station that plots its
telemetry. The aim is a testbed for guidance, navigation and control code.

**Current state:** the airframe, vanes, servos, ESC and fan do not exist yet.
The only hardware is a Teensy 4.1 on a breadboard with the sensors listed
below. Nothing here has flown or controlled anything.

## Hardware

Only the first four rows exist today.

| Part | Role | Connection |
|---|---|---|
| Teensy 4.1 | Flight computer | n/a |
| BNO08X IMU | Orientation (its own fused quaternion) and linear acceleration | I2C, address 0x4B (0x4A tried if 0x4B fails) |
| BMP580 barometer | Altitude | I2C, address 0x47 |
| INA219 | Battery voltage and current | I2C, address 0x40 |
| GY-GPS6MV2 GPS (NEO-6M class) | Position (read but not used yet) | `Serial1`, 9600 baud |
| 4 vane servos (N/E/S/W) | Thrust-vector vanes | Planned, not built |
| EDF ESC | Fan throttle | Planned, not built |

Pin assignments in the code (`src/main.cpp`):

| Signal | Pin |
|---|---|
| Servo N | 2 |
| Servo E | 3 |
| Servo S | 4 |
| Servo W | 5 |
| ESC | 6 |
| I2C (SDA/SCL) | Teensy default `Wire` (pins 18/19 on the Teensy 4.1, per the Teensy pinout; the code does not set them) |
| GPS (RX/TX) | Teensy default `Serial1` (pins 0/1 on the Teensy 4.1, per the Teensy pinout; the code does not set them) |

Note: a device answering at I2C address 0x7E shows up on a bus scan. It is
unidentified, and no effect on the sensors or the firmware has been seen. The
firmware does not use it.

## Architecture

`src/main.cpp` is a single file with a fixed 100 Hz loop. Each tick runs, in
order:

1. Read sensors: IMU quaternion converted to roll/pitch/yaw, barometer,
   battery monitor, GPS characters fed to a parser.
2. Altitude estimate: a 2-state Kalman filter (altitude above the start point,
   vertical speed). The barometer is the measurement; IMU linear acceleration
   along its z axis is the input, with no rotation into the world frame.
3. Safety check (armed states only): abort if battery is below 19.8 V, tilt is
   above 45 degrees, or altitude is above 20 m.
4. State machine: PREFLIGHT, ARMED, TAKEOFF, HOVER, LANDING, LANDED, ABORT.
   Four PID controllers (pitch, roll, yaw hold, altitude) produce commands.
5. Actuator output: commands are mixed to four vane servo pulses and one ESC
   pulse (vane N = -pitch + yaw, E = roll - yaw, S = pitch + yaw,
   W = -roll - yaw). In PREFLIGHT, ARMED, LANDED and ABORT the vanes are
   centred and the ESC is held at idle.
6. Logging: SD logging is a commented-out placeholder; it does nothing.
7. Serial commands: single characters on the first USB serial port:
   `a` arm, `d` disarm, `t` takeoff, `l` land, `x` abort, `r` reset from
   abort, `s` status.
8. Telemetry: a 24-byte binary frame at 10 Hz on the second USB serial port
   (sync bytes 0xAA 0x55, state byte, roll, pitch, altitude, throttle and
   battery voltage as float32, XOR checksum). It is one-way; the ground station
   cannot send commands.

`ground-station/ground_station.py` reads those frames and shows live plots.
`tools/` holds two stand-alone sensor bring-up sketches (see `tools/README.md`).
`archive/` holds an older sketch for different hardware that is not built.

## Build, flash, monitor

Requires [PlatformIO](https://platformio.org/).

```
pio run                      # build
pio run -t upload            # flash the Teensy
pio device monitor           # first USB serial port, 115200 baud
```

The build enables the Teensy's dual USB serial mode (`-DUSB_DUAL_SERIAL`), so
the Teensy shows up as two serial ports.

## Ground station

```
cd ground-station
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python ground_station.py /dev/ttyACM1     # the Teensy's second port
.venv/bin/python test_parser.py                     # parser tests
```

The port name differs by system (for example `COM3` on Windows). Use the
second of the Teensy's two serial ports.

## Status

**Verified on the bench**

- Sensors initialise (BNO08X, BMP580, INA219, GPS).
- The flight computer boots.
- Ground-level barometer calibration runs.
- Tilt readings from the IMU.
- Ground-station frame parser: 6 tests in `ground-station/test_parser.py`
  (host-side only, no hardware involved).

**Written but not verified**

- Telemetry on real hardware (the firmware builds; it has not been flashed with
  the ground station connected).
- Ground-station plots.
- Control: PID gains, Kalman filter tuning and the state machine.
- Safety checks.
- Actuators: servo and ESC output, vane mixing, `HOVER_BASE_US` and the
  `ROLL_SIGN`/`PITCH_SIGN` axis signs. None of this has been run with real
  servos or an ESC.

**Not started**

- SD card logging (commented out).
- LoRa radio telemetry.
- Airframe, vanes and fan.
