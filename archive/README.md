# archive/

`hopper_flight_controller.ino` is an earlier single-file sketch (November 2025
era) kept for reference. It is **not built** by `platformio.ini` (PlatformIO
only compiles `src/`) and has never been flashed in its current form.

It targets older hardware that the current firmware no longer uses:

- BNO055 IMU (current firmware: BNO08X)
- BMP388 barometer (current firmware: BMP580 via the BMP5xx library)
- RFM95W LoRa telemetry via RadioHead (current firmware: USB serial telemetry)
- microSD logging (current firmware: commented-out placeholder)

It also does its own complementary filter for attitude, which the current
firmware does not (the BNO08X does the fusion). Do not treat it as a
description of the current system; see `src/main.cpp` and the root `README.md`.
