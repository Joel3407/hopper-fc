// =============================================================================
// HOPPER-FC — Teensy 4.1 Flight Controller for EDF Hopper
// Boese Aerospace — Joel Boese
// =============================================================================

// =============================================================================
// SECTION 1 — Includes and Configuration
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_INA219.h>
#include <TinyGPSPlus.h>
#include <Servo.h>

// ---- Pin assignments --------------------------------------------------------
static constexpr uint8_t PIN_SERVO_N = 2;
static constexpr uint8_t PIN_SERVO_E = 3;
static constexpr uint8_t PIN_SERVO_S = 4;
static constexpr uint8_t PIN_SERVO_W = 5;
static constexpr uint8_t PIN_ESC     = 6;

// ---- Loop rate --------------------------------------------------------------
static constexpr uint32_t LOOP_HZ        = 100;
static constexpr uint32_t LOOP_PERIOD_US = 1000000UL / LOOP_HZ;

// ---- I2C addresses ----------------------------------------------------------
static constexpr uint8_t ADDR_BMP5XX = 0x47;
static constexpr uint8_t ADDR_INA219 = 0x40;
static constexpr uint8_t ADDR_BNO08X_PRIMARY   = 0x4B;
static constexpr uint8_t ADDR_BNO08X_FALLBACK  = 0x4A;

// ---- Safety limits ----------------------------------------------------------
static constexpr float VBAT_MIN_V     = 19.8f;  // 6S @ ~3.3V/cell
static constexpr float TILT_LIMIT_DEG = 45.0f;
static constexpr float ALT_LIMIT_M    = 20.0f;

// ---- Servo / ESC limits -----------------------------------------------------
static constexpr int SERVO_CENTER_US = 1500;
static constexpr int VANE_MAX_DELTA  = 400;   // ±µs from center
static constexpr int ESC_IDLE_US     = 1000;
static constexpr int ESC_MAX_US      = 2000;
static constexpr int HOVER_BASE_US   = 1500;  // 50% throttle — tune on bench

// ---- Control scaling --------------------------------------------------------
static constexpr float VANE_SCALE   = 5.0f;   // PID units → servo µs delta
static constexpr float ALT_SCALE    = 10.0f;  // altitude PID units → ESC µs delta

// ---- BNO08X report rate -----------------------------------------------------
static constexpr uint32_t BNO_REPORT_INTERVAL_US = 10000;  // 100 Hz

// ---- IMU axis signs ---------------------------------------------------------
// Set to -1.0f to flip an axis if the bench test shows roll/pitch reversed.
static constexpr float ROLL_SIGN  = 1.0f;
static constexpr float PITCH_SIGN = 1.0f;

// ---- Kalman filter tuning (altitude + velocity) -----------------------------
static constexpr float KF_Q_ALT  = 0.1f;  // process noise: altitude (m²)
static constexpr float KF_Q_VEL  = 1.0f;  // process noise: velocity (m²/s²)
static constexpr float KF_R_BARO = 2.0f;  // baro measurement noise (m²)

// ---- Baro ground calibration ------------------------------------------------
static constexpr int BARO_CAL_DISCARD = 10;  // settling readings thrown away after begin()
static constexpr int BARO_CAL_SAMPLES = 50;
static constexpr int BARO_BEGIN_TRIES = 5;
static constexpr int BARO_BEGIN_RETRY_MS = 100;

// ---- PID gains --------------------------------------------------------------
struct PIDGains { float kp, ki, kd; };

static constexpr PIDGains GAINS_PITCH = { 2.0f, 0.1f, 0.5f };
static constexpr PIDGains GAINS_ROLL  = { 2.0f, 0.1f, 0.5f };
static constexpr PIDGains GAINS_YAW   = { 1.0f, 0.0f, 0.3f };
static constexpr PIDGains GAINS_ALT   = { 0.5f, 0.05f, 0.2f };

static constexpr float PID_ILIMIT = 50.0f;  // integral anti-windup clamp

// ---- Flight setpoints -------------------------------------------------------
static constexpr float HOVER_ALT_M      = 3.0f;   // target hover AGL (m)
static constexpr float TAKEOFF_RAMP_S   = 3.0f;   // throttle ramp duration (s)
static constexpr float LANDING_RATE_MPS = 0.5f;   // descent rate (m/s)
static constexpr float LAND_ALT_THRESH  = 0.3f;   // AGL to declare landed (m)
static constexpr float LAND_VEL_THRESH  = 0.3f;   // vertical speed to declare landed (m/s)


// =============================================================================
// SECTION 2 — Global Objects and State Structs
// =============================================================================

// ---- Hardware objects -------------------------------------------------------
Adafruit_BNO08x bno08x;
Adafruit_BMP5xx bmp;
Adafruit_INA219 ina219(ADDR_INA219);
TinyGPSPlus     gps;
Servo           servo_N, servo_E, servo_S, servo_W, esc_servo;

bool ina219_ok = false;
sh2_SensorValue_t bno_value;

// ---- Raw sensor readings ----------------------------------------------------
struct SensorData {
    // BNO08X (latest values from the report stream)
    float   quat_w, quat_x, quat_y, quat_z;  // game rotation vector
    float   lin_accel_z_mps2;                // linear accel z, gravity removed (m/s²)
    // BMP5xx
    float   pressure_hpa;
    float   temp_c;
    float   baro_alt_m;              // MSL altitude from baro
    // INA219
    float   bus_v;
    float   current_ma;
    // GPS
    double  lat, lon;
    float   gps_alt_m;
    bool    gps_valid;
};

// ---- Filtered state estimates -----------------------------------------------
struct StateEstimate {
    float roll_deg, pitch_deg, yaw_deg;
    float alt_agl_m;    // Kalman altitude above ground level (m)
    float vel_z_mps;    // Kalman vertical velocity (m/s, up positive)
    float accel_z_g;    // linear accel z (gravity-free), used for Kalman predict
};

// ---- Control outputs --------------------------------------------------------
struct ControlCommands {
    float pitch;       // PID output — converted to servo µs in actuate()
    float roll;
    float yaw;
    int   throttle_us; // ESC pulse width
};

// ---- Per-axis PID state -----------------------------------------------------
struct PIDState {
    float integral;
    float prev_measured;  // stored measurement for derivative-on-measurement
};

// ---- Flight state machine ---------------------------------------------------
enum FlightState {
    PREFLIGHT = 0,
    ARMED,
    TAKEOFF,
    HOVER,
    LANDING,
    LANDED,
    ABORT
};

// ---- Module-level globals ---------------------------------------------------
SensorData      sensors     = {};
StateEstimate   state_est   = {};
ControlCommands cmds        = {};

PIDState pid_pitch = {}, pid_roll = {}, pid_yaw = {}, pid_alt = {};

FlightState flight_state   = PREFLIGHT;
uint32_t    state_entry_ms = 0;

float ground_alt_msl    = 0.0f;  // baro MSL at pad (m)
float yaw_hold_deg      = 0.0f;  // heading locked at arm
float landing_start_alt = 0.0f;  // AGL when landing began

// ---- Kalman filter state (2-state: altitude AGL, vertical velocity) ---------
float kf_alt = 0.0f, kf_vel = 0.0f;
float kf_P[2][2] = { { 100.0f, 0.0f }, { 0.0f, 10.0f } };

// ---- Fatal error state (non-blocking LED blink) -----------------------------
bool     fatal_err       = false;
uint32_t led_last_ms     = 0;
bool     led_state       = false;


// =============================================================================
// SECTION 3 — setup()
// Fatal sensor failures set fatal_err and return; loop() blinks LED.
// Non-fatal failures print a warning and continue.
// =============================================================================

// Enable Game Rotation Vector + Linear Acceleration at 100 Hz.
// Called from setup() and again whenever the BNO08X reports a reset.
static bool enable_bno_reports() {
    bool ok = bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
    ok &= bno08x.enableReport(SH2_LINEAR_ACCELERATION, BNO_REPORT_INTERVAL_US);
    return ok;
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    Serial1.begin(9600);  // GPS UART
    while (!Serial && millis() < 3000);

    Wire.begin();
    Wire.setClock(100000);  // BNO08X needs 100 kHz I2C

    // ---- BNO08X (fatal: no IMU = no flight) ---------------------------------
    if (bno08x.begin_I2C(ADDR_BNO08X_PRIMARY, &Wire)) {
        Serial.println("OK: BNO08X at 0x4B");
    } else if (bno08x.begin_I2C(ADDR_BNO08X_FALLBACK, &Wire)) {
        Serial.println("OK: BNO08X at 0x4A");
    } else {
        Serial.println("FATAL: BNO08X not found");
        fatal_err = true;
        return;
    }
    if (!enable_bno_reports()) {
        Serial.println("FATAL: BNO08X not found");  // reports failed: treat as no IMU
        fatal_err = true;
        return;
    }

    // ---- BMP5xx (fatal: no baro = no altitude) ------------------------------
    bool bmp_ok = false;
    for (int i = 0; i < BARO_BEGIN_TRIES && !bmp_ok; i++) {
        bmp_ok = bmp.begin(ADDR_BMP5XX);
        if (!bmp_ok) delay(BARO_BEGIN_RETRY_MS);
    }
    if (!bmp_ok) {
        Serial.println("FATAL: BMP5xx not found at 0x47");
        fatal_err = true;
        return;
    }
    Serial.println("OK: BMP5xx");

    // ---- INA219 (non-fatal: lose power monitoring only) ---------------------
    ina219_ok = ina219.begin();
    Serial.println(ina219_ok ? "OK: INA219"
                             : "WARNING: INA219 not found — power monitoring disabled");

    // ---- Vane servos: center at 1500 µs -------------------------------------
    servo_N.attach(PIN_SERVO_N);
    servo_E.attach(PIN_SERVO_E);
    servo_S.attach(PIN_SERVO_S);
    servo_W.attach(PIN_SERVO_W);
    servo_N.writeMicroseconds(SERVO_CENTER_US);
    servo_E.writeMicroseconds(SERVO_CENTER_US);
    servo_S.writeMicroseconds(SERVO_CENTER_US);
    servo_W.writeMicroseconds(SERVO_CENTER_US);

    // ---- ESC: hold minimum throttle to arm -----------------------------------
    esc_servo.attach(PIN_ESC);
    esc_servo.writeMicroseconds(ESC_IDLE_US);
    delay(1000);  // ESC arm window
    Serial.println("OK: Servos + ESC");

    // ---- Baro ground calibration for AGL reference --------------------------
    Serial.print("Calibrating baro ground level");
    // Discard first BARO_CAL_DISCARD readings after begin() (sensor settling)
    for (int i = 0; i < BARO_CAL_DISCARD; i++) {
        bmp.performReading();
        delay(20);
    }
    float alt_sum = 0.0f;
    int   samples = 0;
    for (int i = 0; i < BARO_CAL_SAMPLES; i++) {
        if (bmp.performReading()) {
            alt_sum += bmp.readAltitude(1013.25f);
            samples++;
        }
        delay(20);
        if (i % 10 == 9) Serial.print('.');
    }
    Serial.println();
    ground_alt_msl = (samples > 0) ? alt_sum / samples : 0.0f;
    kf_alt = 0.0f;
    kf_vel = 0.0f;
    Serial.printf("Ground MSL = %.2f m  (AGL will be relative to this)\n", ground_alt_msl);

    Serial.println("=== HOPPER-FC READY ===  a=arm  d=disarm  t=takeoff  l=land  x=abort  s=status");
}


// =============================================================================
// SECTION 4 — read_sensors()
// Populates the global SensorData struct each loop tick.
// =============================================================================

void read_sensors() {
    // ---- BNO08X: re-enable reports after a reset, then drain the event queue
    if (bno08x.wasReset()) {
        Serial.println("WARNING: BNO08X reset — re-enabling reports");
        enable_bno_reports();
    }
    while (bno08x.getSensorEvent(&bno_value)) {
        switch (bno_value.sensorId) {
        case SH2_GAME_ROTATION_VECTOR:
            sensors.quat_w = bno_value.un.gameRotationVector.real;
            sensors.quat_x = bno_value.un.gameRotationVector.i;
            sensors.quat_y = bno_value.un.gameRotationVector.j;
            sensors.quat_z = bno_value.un.gameRotationVector.k;
            break;
        case SH2_LINEAR_ACCELERATION:
            sensors.lin_accel_z_mps2 = bno_value.un.linearAcceleration.z;
            break;
        }
    }

    // Quaternion → roll/pitch/yaw (degrees), written straight to the estimate.
    // Game rotation vector has no magnetometer, so yaw drifts slowly.
    {
        const float w = sensors.quat_w, x = sensors.quat_x;
        const float y = sensors.quat_y, z = sensors.quat_z;
        const float RAD2DEG = 180.0f / M_PI;
        float roll  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
        float pitch = asinf(constrain(2.0f * (w * y - z * x), -1.0f, 1.0f));
        float yaw   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
        // Axis signs: flip ROLL_SIGN / PITCH_SIGN (Section 1) if bench test shows reversed.
        state_est.roll_deg  = ROLL_SIGN  * roll  * RAD2DEG;
        state_est.pitch_deg = PITCH_SIGN * pitch * RAD2DEG;
        state_est.yaw_deg   = yaw * RAD2DEG;
    }
    // Linear acceleration is already gravity-free: do not subtract 1 g.
    state_est.accel_z_g = sensors.lin_accel_z_mps2 / 9.81f;

    // ---- BMP5xx -------------------------------------------------------------
    if (bmp.performReading()) {
        sensors.pressure_hpa = bmp.pressure;  // hPa
        sensors.temp_c      = bmp.temperature;
        sensors.baro_alt_m  = bmp.readAltitude(1013.25f);
    }

    // ---- INA219 -------------------------------------------------------------
    if (ina219_ok) {
        sensors.bus_v      = ina219.getBusVoltage_V();
        sensors.current_ma = ina219.getCurrent_mA();
    }

    // ---- GPS: drain UART into TinyGPSPlus -----------------------------------
    while (Serial1.available()) {
        gps.encode(Serial1.read());
    }
    sensors.gps_valid = gps.location.isValid();
    if (sensors.gps_valid) {
        sensors.lat       = gps.location.lat();
        sensors.lon       = gps.location.lng();
        sensors.gps_alt_m = (float)gps.altitude.meters();
    }
}


// =============================================================================
// SECTION 6 — estimate_altitude(dt)
// 2-state Kalman filter: x = [altitude_AGL, vertical_velocity]
// Predict: dead-reckoning with body-frame accel_z (near-level approximation).
// Update: baro altitude converted to AGL via ground_alt_msl.
// =============================================================================

void estimate_altitude(float dt) {
    float baro_agl = sensors.baro_alt_m - ground_alt_msl;
    float az_mps2  = state_est.accel_z_g * 9.81f;

    // ---- Predict (F = [[1,dt],[0,1]], B = [0.5*dt², dt]) -------------------
    float alt_pred = kf_alt + kf_vel * dt + 0.5f * az_mps2 * dt * dt;
    float vel_pred = kf_vel + az_mps2 * dt;

    // Propagate covariance P_pred = F·P·Fᵀ + Q
    float p00p = kf_P[0][0] + dt * (kf_P[1][0] + kf_P[0][1]) + dt*dt*kf_P[1][1] + KF_Q_ALT;
    float p01p = kf_P[0][1] + dt * kf_P[1][1];
    float p10p = kf_P[1][0] + dt * kf_P[1][1];
    float p11p = kf_P[1][1] + KF_Q_VEL;

    // ---- Update (H = [1, 0]: baro measures altitude only) ------------------
    float S  = p00p + KF_R_BARO;
    float k0 = p00p / S;  // Kalman gain — altitude state
    float k1 = p10p / S;  // Kalman gain — velocity state

    float innov = baro_agl - alt_pred;
    kf_alt = alt_pred + k0 * innov;
    kf_vel = vel_pred + k1 * innov;

    // Update covariance
    kf_P[0][0] = (1.0f - k0) * p00p;
    kf_P[0][1] = (1.0f - k0) * p01p;
    kf_P[1][0] = p10p - k1 * p00p;
    kf_P[1][1] = p11p - k1 * p01p;

    state_est.alt_agl_m = kf_alt;
    state_est.vel_z_mps = kf_vel;
}


// =============================================================================
// SECTION 7 — safety_check()
// Only active when flight_state >= ARMED.
// Any limit breach triggers ABORT immediately.
// =============================================================================

void safety_check() {
    if (flight_state < ARMED) return;

    bool do_abort = false;

    // Low battery (skip check if INA219 not present or reading looks invalid)
    if (ina219_ok && sensors.bus_v > 5.0f && sensors.bus_v < VBAT_MIN_V) {
        Serial.printf("ABORT: low battery %.2fV\n", sensors.bus_v);
        do_abort = true;
    }

    // Excessive tilt (combined roll+pitch magnitude)
    float tilt = sqrtf(state_est.roll_deg  * state_est.roll_deg +
                       state_est.pitch_deg * state_est.pitch_deg);
    if (tilt > TILT_LIMIT_DEG) {
        Serial.printf("ABORT: tilt %.1f deg\n", tilt);
        do_abort = true;
    }

    // Altitude fence
    if (state_est.alt_agl_m > ALT_LIMIT_M) {
        Serial.printf("ABORT: altitude %.1f m\n", state_est.alt_agl_m);
        do_abort = true;
    }

    if (do_abort) flight_state = ABORT;
}


// =============================================================================
// SECTION 9 — PID Functions
// Placed before run_state_machine() because the state machine calls them.
//
// compute_pid():     generic PID with derivative-on-measurement + anti-windup.
// reset_pid():       clear integrator and previous measurement.
// attitude_control(): pitch/roll/yaw PIDs targeting level + yaw hold.
// altitude_control(): altitude PID returning ESC µs command.
// =============================================================================

float compute_pid(float setpoint, float measured, float dt,
                  const PIDGains& g, PIDState& ps) {
    float error = setpoint - measured;

    ps.integral = constrain(ps.integral + error * dt, -PID_ILIMIT, PID_ILIMIT);

    // Derivative on measurement: avoids derivative kick on setpoint step changes
    float deriv = (dt > 0.0f) ? -(measured - ps.prev_measured) / dt : 0.0f;
    ps.prev_measured = measured;

    return g.kp * error + g.ki * ps.integral + g.kd * deriv;
}

void reset_pid(PIDState& ps) {
    ps.integral     = 0.0f;
    ps.prev_measured = 0.0f;
}

void attitude_control(float dt) {
    cmds.pitch = compute_pid(0.0f,         state_est.pitch_deg, dt, GAINS_PITCH, pid_pitch);
    cmds.roll  = compute_pid(0.0f,         state_est.roll_deg,  dt, GAINS_ROLL,  pid_roll);
    cmds.yaw   = compute_pid(yaw_hold_deg, state_est.yaw_deg,   dt, GAINS_YAW,   pid_yaw);
}

int altitude_control(float setpoint_m, float dt) {
    float corr = compute_pid(setpoint_m, state_est.alt_agl_m, dt, GAINS_ALT, pid_alt);
    return constrain(HOVER_BASE_US + (int)(corr * ALT_SCALE), ESC_IDLE_US, ESC_MAX_US);
}


// =============================================================================
// SECTION 8 — run_state_machine(dt)
// Drives state transitions and populates cmds each loop tick.
// PIDs reset on entry to ARMED, and again on entry to HOVER and LANDING.
// Disarm is blocked during TAKEOFF, HOVER, and LANDING (enforced in handle_serial).
// =============================================================================

static void enter_state(FlightState s) {
    flight_state   = s;
    state_entry_ms = millis();
}

void run_state_machine(float dt) {
    float t = (millis() - state_entry_ms) / 1000.0f;  // seconds in current state

    switch (flight_state) {

    // ---- PREFLIGHT: sit idle, servos centered, motor off -------------------
    case PREFLIGHT:
        cmds = { 0.0f, 0.0f, 0.0f, ESC_IDLE_US };
        break;

    // ---- ARMED: motor idle, waiting for takeoff command --------------------
    case ARMED:
        cmds = { 0.0f, 0.0f, 0.0f, ESC_IDLE_US };
        break;

    // ---- TAKEOFF: ramp throttle + attitude hold, transition when at alt ----
    case TAKEOFF: {
        // Linear throttle ramp from idle to hover base over TAKEOFF_RAMP_S
        float ramp   = constrain(t / TAKEOFF_RAMP_S, 0.0f, 1.0f);
        int   base   = ESC_IDLE_US + (int)(ramp * (HOVER_BASE_US - ESC_IDLE_US));

        attitude_control(dt);
        float alt_corr = compute_pid(HOVER_ALT_M, state_est.alt_agl_m, dt, GAINS_ALT, pid_alt);
        cmds.throttle_us = constrain(base + (int)(alt_corr * ALT_SCALE), ESC_IDLE_US, ESC_MAX_US);

        // Transition when within 10% of setpoint and ramp is complete
        if (t > TAKEOFF_RAMP_S && state_est.alt_agl_m >= HOVER_ALT_M * 0.9f) {
            reset_pid(pid_pitch); reset_pid(pid_roll);
            reset_pid(pid_yaw);   reset_pid(pid_alt);
            enter_state(HOVER);
        }
        break;
    }

    // ---- HOVER: hold altitude setpoint, maintain level ---------------------
    case HOVER:
        attitude_control(dt);
        cmds.throttle_us = altitude_control(HOVER_ALT_M, dt);
        break;

    // ---- LANDING: walk setpoint down at constant rate ----------------------
    case LANDING: {
        float setpoint = landing_start_alt - LANDING_RATE_MPS * t;
        if (setpoint < 0.0f) setpoint = 0.0f;

        attitude_control(dt);
        cmds.throttle_us = altitude_control(setpoint, dt);

        if (state_est.alt_agl_m < LAND_ALT_THRESH &&
            fabsf(state_est.vel_z_mps) < LAND_VEL_THRESH) {
            enter_state(LANDED);
        }
        break;
    }

    // ---- LANDED: motor off, wait for operator disarm -----------------------
    case LANDED:
        cmds = { 0.0f, 0.0f, 0.0f, ESC_IDLE_US };
        break;

    // ---- ABORT: cut motor immediately --------------------------------------
    case ABORT:
        cmds = { 0.0f, 0.0f, 0.0f, ESC_IDLE_US };
        break;
    }
}


// =============================================================================
// SECTION 10 — actuate()
// Control allocation → servo microseconds and ESC output.
//
// Allocation (CLAUDE.md convention):
//   vane_N = -pitch + yaw
//   vane_E =  roll  - yaw
//   vane_S =  pitch + yaw
//   vane_W = -roll  - yaw
// =============================================================================

static int clamp_vane(int us) {
    return constrain(us, SERVO_CENTER_US - VANE_MAX_DELTA,
                         SERVO_CENTER_US + VANE_MAX_DELTA);
}

void actuate() {
    // Safe states: center all vanes, kill throttle
    if (flight_state == PREFLIGHT || flight_state == ARMED ||
        flight_state == LANDED    || flight_state == ABORT) {
        servo_N.writeMicroseconds(SERVO_CENTER_US);
        servo_E.writeMicroseconds(SERVO_CENTER_US);
        servo_S.writeMicroseconds(SERVO_CENTER_US);
        servo_W.writeMicroseconds(SERVO_CENTER_US);
        esc_servo.writeMicroseconds(ESC_IDLE_US);
        return;
    }

    // Scale PID float outputs to servo delta-microseconds
    int p = (int)(cmds.pitch * VANE_SCALE);
    int r = (int)(cmds.roll  * VANE_SCALE);
    int y = (int)(cmds.yaw   * VANE_SCALE);

    servo_N.writeMicroseconds(clamp_vane(SERVO_CENTER_US - p + y));
    servo_E.writeMicroseconds(clamp_vane(SERVO_CENTER_US + r - y));
    servo_S.writeMicroseconds(clamp_vane(SERVO_CENTER_US + p + y));
    servo_W.writeMicroseconds(clamp_vane(SERVO_CENTER_US - r - y));

    esc_servo.writeMicroseconds(constrain(cmds.throttle_us, ESC_IDLE_US, ESC_MAX_US));
}


// =============================================================================
// SECTION 11 — log_data()
// SD card CSV logging placeholder. Uses Teensy 4.1 built-in SD slot.
// Uncomment everything (and add #include <SD.h> above) when SD card arrives.
// =============================================================================

void log_data() {
    // static bool sd_ready  = false;
    // static bool sd_header = false;
    // if (!sd_ready) {
    //     sd_ready = SD.begin(BUILTIN_SDCARD);
    //     if (!sd_ready) return;
    // }
    // File f = SD.open("log.csv", FILE_WRITE);
    // if (!f) return;
    // if (!sd_header) {
    //     f.println("time_ms,state,roll_deg,pitch_deg,yaw_deg,"
    //               "alt_agl_m,vel_z_mps,baro_alt_m,vbat_v,current_ma,"
    //               "lat,lon,throttle_us");
    //     sd_header = true;
    // }
    // f.printf("%lu,%d,%.2f,%.2f,%.2f,%.3f,%.3f,%.2f,%.3f,%.1f,%.6f,%.6f,%d\n",
    //          millis(), (int)flight_state,
    //          state_est.roll_deg, state_est.pitch_deg, state_est.yaw_deg,
    //          state_est.alt_agl_m, state_est.vel_z_mps,
    //          sensors.baro_alt_m, sensors.bus_v, sensors.current_ma,
    //          sensors.lat, sensors.lon, cmds.throttle_us);
    // f.close();
}

// =============================================================================
// send_telemetry() — binary frame on SerialUSB1 at 10 Hz (ground station).
// Frame: 0xAA 0x55 | state u8 | roll, pitch, alt_agl, throttle(0..1), vbat
//        (float32 LE each) | XOR checksum of payload. 24 bytes total.
// Teensy 4.1 is little-endian, so memcpy of floats is already LE.
// =============================================================================
static constexpr uint32_t TELEM_PERIOD_MS = 100;  // 10 Hz

void send_telemetry() {
    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if (now - last_ms < TELEM_PERIOD_MS) return;
    last_ms = now;

    uint8_t frame[2 + 1 + 5 * 4 + 1];
    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = (uint8_t)flight_state;

    float vals[5] = {
        state_est.roll_deg,
        state_est.pitch_deg,
        state_est.alt_agl_m,
        (cmds.throttle_us - 1000) / 1000.0f,
        sensors.bus_v,
    };
    memcpy(&frame[3], vals, sizeof(vals));

    uint8_t cs = 0;
    for (size_t i = 2; i < sizeof(frame) - 1; i++) cs ^= frame[i];
    frame[sizeof(frame) - 1] = cs;

    SerialUSB1.write(frame, sizeof(frame));
}


// =============================================================================
// SECTION 12 — handle_serial()
// Single-char command interface over USB Serial.
// Disarm ('d') is blocked during TAKEOFF, HOVER, and LANDING.
// =============================================================================

static const char* state_name(FlightState s) {
    switch (s) {
        case PREFLIGHT: return "PREFLIGHT";
        case ARMED:     return "ARMED";
        case TAKEOFF:   return "TAKEOFF";
        case HOVER:     return "HOVER";
        case LANDING:   return "LANDING";
        case LANDED:    return "LANDED";
        case ABORT:     return "ABORT";
        default:        return "UNKNOWN";
    }
}

void handle_serial() {
    if (!Serial.available()) return;
    char cmd = (char)Serial.read();

    switch (cmd) {

    case 'a':  // Arm
        if (flight_state == PREFLIGHT || flight_state == LANDED) {
            reset_pid(pid_pitch); reset_pid(pid_roll);
            reset_pid(pid_yaw);   reset_pid(pid_alt);
            yaw_hold_deg = state_est.yaw_deg;
            enter_state(ARMED);
            Serial.println("ARMED");
        } else {
            Serial.printf("Cannot arm from %s\n", state_name(flight_state));
        }
        break;

    case 'd':  // Disarm — blocked during active flight
        if (flight_state == TAKEOFF || flight_state == HOVER ||
            flight_state == LANDING) {
            Serial.printf("DISARM BLOCKED in %s — use 'x' to abort\n",
                          state_name(flight_state));
        } else {
            enter_state(PREFLIGHT);
            Serial.println("DISARMED → PREFLIGHT");
        }
        break;

    case 't':  // Takeoff
        if (flight_state == ARMED) {
            reset_pid(pid_pitch); reset_pid(pid_roll);
            reset_pid(pid_yaw);   reset_pid(pid_alt);
            enter_state(TAKEOFF);
            Serial.printf("TAKEOFF — target %.1f m AGL\n", HOVER_ALT_M);
        } else {
            Serial.printf("Cannot takeoff from %s\n", state_name(flight_state));
        }
        break;

    case 'l':  // Land
        if (flight_state == HOVER) {
            landing_start_alt = state_est.alt_agl_m;  // descend from current altitude
            reset_pid(pid_pitch); reset_pid(pid_roll);
            reset_pid(pid_yaw);   reset_pid(pid_alt);
            enter_state(LANDING);
            Serial.printf("LANDING from %.1f m AGL\n", landing_start_alt);
        } else {
            Serial.printf("Cannot land from %s\n", state_name(flight_state));
        }
        break;

    case 'x':  // Abort — always available
        enter_state(ABORT);
        Serial.println("ABORT — motor cut");
        break;

    case 'r':  // Reset — ABORT → PREFLIGHT
        if (flight_state == ABORT) {
            enter_state(PREFLIGHT);
            Serial.println("RESET — back to PREFLIGHT");
        } else {
            Serial.printf("Cannot reset from %s\n", state_name(flight_state));
        }
        break;

    case 's':  // Status
        Serial.printf("[%s] roll=%.1f° pitch=%.1f° yaw=%.1f° | "
                      "alt=%.2fm vel=%.2fm/s | "
                      "baro=%.2fm baro_raw=%.2fm | "
                      "vbat=%.2fV I=%.0fmA | "
                      "gps=%s thr=%d µs\n",
                      state_name(flight_state),
                      state_est.roll_deg, state_est.pitch_deg, state_est.yaw_deg,
                      state_est.alt_agl_m, state_est.vel_z_mps,
                      state_est.alt_agl_m, sensors.baro_alt_m - ground_alt_msl,
                      sensors.bus_v, sensors.current_ma,
                      sensors.gps_valid ? "fix" : "no fix",
                      cmds.throttle_us);
        break;

    default:
        break;
    }
}


// =============================================================================
// SECTION 13 — loop()
// Fixed 100 Hz loop using micros(). Non-blocking throughout.
// Fatal error path: blink LED at 2.5 Hz and skip all flight code.
// =============================================================================

void loop() {
    // ---- Fatal error: blink LED, skip flight code --------------------------
    if (fatal_err) {
        uint32_t now = millis();
        if (now - led_last_ms >= 200) {
            led_last_ms = now;
            led_state   = !led_state;
            digitalWrite(LED_BUILTIN, led_state);
        }
        return;
    }

    // ---- Fixed-rate gate: enforce LOOP_PERIOD_US ---------------------------
    static uint32_t prev_us = 0;
    uint32_t now_us = micros();
    if (now_us - prev_us < LOOP_PERIOD_US) return;

    float dt = (now_us - prev_us) / 1.0e6f;
    prev_us  = now_us;

    // Guard against first-iteration spike or micros() rollover (~71 min)
    if (dt > 0.05f) dt = 0.01f;

    // ---- Flight pipeline ---------------------------------------------------
    read_sensors();       // Section 4: sensor reads + BNO08X attitude
    estimate_altitude(dt); // Section 6: Kalman filter
    safety_check();        // Section 7: limit enforcement
    run_state_machine(dt); // Section 8: state logic + PID compute
    actuate();             // Section 10: servo + ESC writes
    log_data();            // Section 11: SD card (placeholder)
    handle_serial();       // Section 12: command parsing
    send_telemetry();      // 10 Hz binary frame on SerialUSB1
}
