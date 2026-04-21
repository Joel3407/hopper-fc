Flight Software Context — EDF Hopper FC
I'm Joel, 18, building an EDF hopper as a GNC testbed before scaling to liquid rockets. Flight computer is Teensy 4.1. Language is C++ (Arduino/PlatformIO). Repo managed with git on Pop!_OS with zsh. I use Claude Code in terminal for actual coding — this chat is for architecture decisions, algorithm design, and debugging reasoning.
Hardware: 90mm EDF, 4x MG90S thrust vanes in + config (N/E/S/W), BNO055 IMU, BMP388 baro, INA219 battery monitor, NEO-M8N GPS, 2x RFM95W LoRa 915MHz, microSD logging. 6S LiPo (22.2V).
Current software state: ~700 line sketch exists with complementary filter (attitude), simplified Kalman (altitude/velocity), 4x PIDs, control allocation, state machine (PREFLIGHT→ARMED→TAKEOFF→HOVER→LANDING→LANDED→ABORT), LoRa telemetry, SD logging. 100Hz main loop.
Control allocation:
vane_N = -pitch + yaw
vane_E = roll - yaw
vane_S = pitch + yaw
vane_W = -roll - yaw
Long-term path: EDF hopper → GOX/propane → LOX/RP-1 → orbital. 90% of this GNC software transfers directly.
Current focus: Getting to first flight. Priority is clean, debuggable code over clever code. Ask me to clarify hardware specifics before making assumptions.
