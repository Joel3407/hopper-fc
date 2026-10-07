# tools/

Two stand-alone Teensy 4.1 sketches used during sensor bring-up. Neither is
built by `platformio.ini` (there is no env for them), so they do not affect the
normal firmware build.

## i2c_scanner.cpp

Scans I2C addresses 0x01-0x7F on the default `Wire` bus every 5 seconds and
prints each address that answers, plus a guess at the device name for a few
known addresses (INA219 0x40, BMP580 0x47, and some older-hardware addresses:
BNO055, MPU6050, BMP388). It does not name the BNO08X (0x4A/0x4B), so those
show as bare addresses. It sets the bus to 400 kHz, while the main firmware
uses 100 kHz for the BNO08X.

## sensor_monitor.cpp

Initialises the BNO08X, BMP5xx, INA219 and GPS (no servos, ESC or state
machine) and prints one labelled line at 10 Hz on the USB serial port:
roll/pitch/yaw, baro altitude (zeroed at start), temperature, pressure,
battery volts and current, GPS fix, satellites, and lat/lon when there is a fix.

## How to run one

PlatformIO only builds `src/main.cpp`, so copy the tool over it temporarily:

```
cp tools/sensor_monitor.cpp src/main.cpp      # or tools/i2c_scanner.cpp
pio run -t upload
pio device monitor                            # 115200 baud
git checkout src/main.cpp                     # restore the real firmware
```

Commit nothing while `src/main.cpp` is overwritten. If you have uncommitted
changes in `src/main.cpp`, commit or stash them first, because `git checkout`
discards them.
