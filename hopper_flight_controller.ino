/*
 * EDF HOPPER FLIGHT CONTROLLER
 * 
 * Hardware: Teensy 4.1 (600 MHz ARM Cortex-M7)
 * Sensors: BNO055 IMU, BMP388 Baro, INA219 Battery, NEO-M8N GPS
 * Actuators: 4× Servos (thrust vanes), 1× ESC (EDF motor)
 * Telemetry: RFM95W LoRa 915MHz
 * 
 * Control Strategy:
 * - 100 Hz main loop
 * - Complementary filter for attitude (gyro + accel)
 * - Kalman filter for altitude/velocity (baro + accel)
 * - PID control for attitude and altitude
 * - Control allocation for 4 thrust vanes
 * 
 * Author: Joel's Rocket Project
 * Date: November 2025
 */

#include <Wire.h>
#include <Servo.h>
#include <SD.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_INA219.h>
#include <TinyGPS++.h>
#include <RH_RF95.h>

// ==================== CONFIGURATION ====================

// Pin definitions
#define SERVO_NORTH_PIN     2   // PWM capable pins
#define SERVO_EAST_PIN      3
#define SERVO_SOUTH_PIN     4
#define SERVO_WEST_PIN      5
#define ESC_PIN             6
#define LED_PIN            13
#define BUZZER_PIN         14
#define LORA_CS_PIN        10
#define LORA_RST_PIN       9
#define LORA_INT_PIN       8
#define SD_CS_PIN          BUILTIN_SDCARD

// Control parameters
#define LOOP_FREQUENCY     100   // Hz
#define LOOP_PERIOD_MS     10    // 1000/100 = 10ms

// Servo limits (microseconds)
#define SERVO_CENTER       1500
#define SERVO_MIN          1300  // -20° deflection
#define SERVO_MAX          1700  // +20° deflection

// ESC limits (microseconds)
#define ESC_MIN            1000  // Motor off
#define ESC_ARMED          1100  // Armed but not spinning
#define ESC_MAX            2000  // Full throttle

// Battery safety
#define BATTERY_CELLS      6
#define CELL_MIN_VOLTAGE   3.3   // Cutoff voltage per cell
#define BATTERY_MIN_VOLTAGE (CELL_MIN_VOLTAGE * BATTERY_CELLS)  // 19.8V

// Flight limits
#define MAX_ALTITUDE_M     20.0  // Software altitude limit
#define MAX_TILT_DEG       45.0  // Abort if tilt exceeds this
#define LANDING_VELOCITY_MS 0.5  // Target landing speed

// PID gains (will need tuning!)
struct PIDGains {
    float Kp, Ki, Kd;
};

PIDGains pitch_gains   = {2.0, 0.1, 0.5};
PIDGains roll_gains    = {2.0, 0.1, 0.5};
PIDGains yaw_gains     = {1.0, 0.0, 0.3};
PIDGains altitude_gains = {0.5, 0.05, 0.2};

// Complementary filter alpha (0.98 = trust gyro 98%, accel 2%)
#define COMP_FILTER_ALPHA  0.98

// ==================== GLOBAL OBJECTS ====================

// Sensors
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX bmp;
Adafruit_INA219 ina219;
TinyGPSPlus gps;
RH_RF95 radio(LORA_CS_PIN, LORA_INT_PIN);

// Actuators
Servo servo_north, servo_east, servo_south, servo_west;
Servo esc;

// SD card
File logfile;

// ==================== STATE VARIABLES ====================

// Flight state machine
enum FlightState {
    PREFLIGHT,    // On ground, not armed
    ARMED,        // Armed, ready to fly
    TAKEOFF,      // Climbing to hover altitude
    HOVER,        // Maintaining altitude
    LANDING,      // Descending to ground
    LANDED,       // On ground after flight
    ABORT         // Emergency shutdown
};
FlightState flight_state = PREFLIGHT;

// Sensor data
struct SensorData {
    // IMU
    float roll, pitch, yaw;           // degrees
    float gyro_x, gyro_y, gyro_z;     // deg/s
    float accel_x, accel_y, accel_z;  // m/s²
    
    // Barometer
    float pressure;                    // hPa
    float temperature;                 // °C
    float altitude_raw;                // meters
    
    // Battery
    float voltage;                     // volts
    float current;                     // amps
    
    // GPS
    float gps_lat, gps_lon;           // degrees
    float gps_altitude;                // meters
    int gps_satellites;
    bool gps_valid;
};
SensorData sensor;

// State estimates (filtered)
struct StateEstimate {
    float roll, pitch, yaw;            // degrees (complementary filter)
    float altitude;                    // meters (Kalman filter)
    float velocity_z;                  // m/s vertical (Kalman filter)
    float altitude_ground;             // ground reference altitude
};
StateEstimate state;

// Control commands
struct ControlCommands {
    float pitch_cmd;   // degrees
    float roll_cmd;    // degrees
    float yaw_cmd;     // deg/s
    float throttle;    // 0-1
};
ControlCommands control;

// PID controller state
struct PIDState {
    float integral;
    float last_error;
};
PIDState pid_pitch, pid_roll, pid_yaw, pid_altitude;

// Timing
unsigned long last_loop_time = 0;
unsigned long loop_count = 0;

// Telemetry
unsigned long last_telemetry_time = 0;
#define TELEMETRY_INTERVAL_MS 100  // 10 Hz

// ==================== SETUP ====================

void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial monitor
    
    Serial.println("=== EDF HOPPER FLIGHT CONTROLLER ===");
    Serial.println("Initializing...");
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400kHz fast mode
    
    // Initialize pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Initialize sensors
    if (!init_sensors()) {
        Serial.println("ERROR: Sensor initialization failed!");
        error_blink();
    }
    
    // Initialize actuators
    init_actuators();
    
    // Initialize radio
    if (!init_radio()) {
        Serial.println("WARNING: Radio initialization failed!");
    }
    
    // Initialize SD card
    if (!init_sd_card()) {
        Serial.println("WARNING: SD card initialization failed!");
    }
    
    // Calibration
    Serial.println("Calibrating sensors (hold still for 3 seconds)...");
    calibrate_sensors();
    
    // Ready
    Serial.println("Initialization complete!");
    Serial.println("Commands: 'a' = arm, 'd' = disarm, 't' = takeoff, 'l' = land");
    beep(3);  // 3 short beeps = ready
    
    last_loop_time = millis();
}

// ==================== MAIN LOOP ====================

void loop() {
    unsigned long current_time = millis();
    
    // Enforce 100 Hz loop rate
    if (current_time - last_loop_time < LOOP_PERIOD_MS) {
        return;  // Not time yet
    }
    
    float dt = (current_time - last_loop_time) / 1000.0;  // seconds
    last_loop_time = current_time;
    loop_count++;
    
    // Blink LED heartbeat
    if (loop_count % 50 == 0) {  // Every 0.5 seconds
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    
    // 1. READ SENSORS
    read_sensors();
    
    // 2. STATE ESTIMATION
    estimate_attitude(dt);
    estimate_altitude_velocity(dt);
    
    // 3. SAFETY CHECKS
    if (!safety_check()) {
        flight_state = ABORT;
    }
    
    // 4. STATE MACHINE & CONTROL
    run_state_machine(dt);
    
    // 5. ACTUATE
    actuate_servos();
    actuate_esc();
    
    // 6. TELEMETRY
    if (current_time - last_telemetry_time > TELEMETRY_INTERVAL_MS) {
        send_telemetry();
        last_telemetry_time = current_time;
    }
    
    // 7. DATA LOGGING
    log_data();
    
    // 8. HANDLE SERIAL COMMANDS
    handle_serial_commands();
}

// ==================== SENSOR INITIALIZATION ====================

bool init_sensors() {
    bool success = true;
    
    // BNO055 IMU
    if (!bno.begin()) {
        Serial.println("ERROR: BNO055 not found!");
        success = false;
    } else {
        bno.setExtCrystalUse(true);
        Serial.println("BNO055 OK");
    }
    
    // BMP388 Barometer
    if (!bmp.begin_I2C()) {
        Serial.println("ERROR: BMP388 not found!");
        success = false;
    } else {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_100_HZ);
        Serial.println("BMP388 OK");
    }
    
    // INA219 Battery Monitor
    if (!ina219.begin()) {
        Serial.println("ERROR: INA219 not found!");
        success = false;
    } else {
        Serial.println("INA219 OK");
    }
    
    // GPS (just start serial, will validate later)
    Serial1.begin(9600);  // NEO-M8N default baud rate
    Serial.println("GPS serial started");
    
    return success;
}

// ==================== ACTUATOR INITIALIZATION ====================

void init_actuators() {
    // Attach servos
    servo_north.attach(SERVO_NORTH_PIN);
    servo_east.attach(SERVO_EAST_PIN);
    servo_south.attach(SERVO_SOUTH_PIN);
    servo_west.attach(SERVO_WEST_PIN);
    
    // Center all servos
    servo_north.writeMicroseconds(SERVO_CENTER);
    servo_east.writeMicroseconds(SERVO_CENTER);
    servo_south.writeMicroseconds(SERVO_CENTER);
    servo_west.writeMicroseconds(SERVO_CENTER);
    
    // Attach and arm ESC
    esc.attach(ESC_PIN);
    esc.writeMicroseconds(ESC_MIN);  // Send min signal
    delay(2000);  // ESC arming sequence
    esc.writeMicroseconds(ESC_ARMED);
    
    Serial.println("Actuators initialized");
}

// ==================== RADIO INITIALIZATION ====================

bool init_radio() {
    pinMode(LORA_RST_PIN, OUTPUT);
    digitalWrite(LORA_RST_PIN, HIGH);
    delay(10);
    digitalWrite(LORA_RST_PIN, LOW);
    delay(10);
    digitalWrite(LORA_RST_PIN, HIGH);
    delay(10);
    
    if (!radio.init()) {
        return false;
    }
    
    if (!radio.setFrequency(915.0)) {  // 915 MHz for North America
        return false;
    }
    
    radio.setTxPower(23, false);  // Max power
    radio.setSignalBandwidth(125000);
    radio.setSpreadingFactor(7);
    
    Serial.println("LoRa radio OK");
    return true;
}

// ==================== SD CARD INITIALIZATION ====================

bool init_sd_card() {
    if (!SD.begin(SD_CS_PIN)) {
        return false;
    }
    
    // Create log file with timestamp
    char filename[32];
    sprintf(filename, "flight_%lu.csv", millis());
    logfile = SD.open(filename, FILE_WRITE);
    
    if (logfile) {
        // Write CSV header
        logfile.println("time,state,roll,pitch,yaw,altitude,velocity,throttle,voltage,current");
        Serial.print("Logging to: ");
        Serial.println(filename);
        return true;
    }
    
    return false;
}

// ==================== SENSOR CALIBRATION ====================

void calibrate_sensors() {
    const int samples = 300;  // 3 seconds at 100 Hz
    float pressure_sum = 0;
    
    for (int i = 0; i < samples; i++) {
        if (bmp.performReading()) {
            pressure_sum += bmp.pressure / 100.0;  // Convert to hPa
        }
        delay(10);
    }
    
    // Set ground reference altitude
    float ground_pressure = pressure_sum / samples;
    state.altitude_ground = bmp.readAltitude(ground_pressure);
    
    Serial.print("Ground altitude reference: ");
    Serial.print(state.altitude_ground);
    Serial.println(" m");
}

// ==================== SENSOR READING ====================

void read_sensors() {
    // BNO055 IMU
    sensors_event_t orientation, gyro, accel;
    bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&accel, Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    sensor.roll = orientation.orientation.z;   // BNO055 axis mapping
    sensor.pitch = orientation.orientation.y;
    sensor.yaw = orientation.orientation.x;
    
    sensor.gyro_x = gyro.gyro.x * 57.2958;  // rad/s to deg/s
    sensor.gyro_y = gyro.gyro.y * 57.2958;
    sensor.gyro_z = gyro.gyro.z * 57.2958;
    
    sensor.accel_x = accel.acceleration.x;
    sensor.accel_y = accel.acceleration.y;
    sensor.accel_z = accel.acceleration.z;
    
    // BMP388 Barometer
    if (bmp.performReading()) {
        sensor.pressure = bmp.pressure / 100.0;  // Pa to hPa
        sensor.temperature = bmp.temperature;
        sensor.altitude_raw = bmp.readAltitude(1013.25) - state.altitude_ground;
    }
    
    // INA219 Battery Monitor
    sensor.voltage = ina219.getBusVoltage_V();
    sensor.current = ina219.getCurrent_mA() / 1000.0;  // mA to A
    
    // GPS
    while (Serial1.available() > 0) {
        if (gps.encode(Serial1.read())) {
            if (gps.location.isValid()) {
                sensor.gps_lat = gps.location.lat();
                sensor.gps_lon = gps.location.lng();
                sensor.gps_altitude = gps.altitude.meters();
                sensor.gps_satellites = gps.satellites.value();
                sensor.gps_valid = true;
            }
        }
    }
}

// ==================== STATE ESTIMATION ====================

void estimate_attitude(float dt) {
    // COMPLEMENTARY FILTER: Fuse gyro (short-term) + accel (long-term)
    // attitude = alpha * (attitude + gyro*dt) + (1-alpha) * accel
    
    // Integrate gyro
    float roll_gyro = state.roll + sensor.gyro_x * dt;
    float pitch_gyro = state.pitch + sensor.gyro_y * dt;
    
    // Use accelerometer to correct drift (assumes no acceleration)
    float roll_accel = atan2(sensor.accel_y, sensor.accel_z) * 57.2958;  // rad to deg
    float pitch_accel = atan2(-sensor.accel_x, sqrt(sensor.accel_y*sensor.accel_y + sensor.accel_z*sensor.accel_z)) * 57.2958;
    
    // Complementary filter
    state.roll = COMP_FILTER_ALPHA * roll_gyro + (1 - COMP_FILTER_ALPHA) * roll_accel;
    state.pitch = COMP_FILTER_ALPHA * pitch_gyro + (1 - COMP_FILTER_ALPHA) * pitch_accel;
    
    // Yaw: use magnetometer from BNO055 (already fused in sensor.yaw)
    state.yaw = sensor.yaw;
}

void estimate_altitude_velocity(float dt) {
    // KALMAN FILTER: Fuse barometer (position) + accelerometer (acceleration)
    // Simplified 1D Kalman filter for altitude and vertical velocity
    
    // Prediction step
    float altitude_pred = state.altitude + state.velocity_z * dt;
    float velocity_pred = state.velocity_z + sensor.accel_z * dt;
    
    // Measurement update (baro altitude)
    float innovation = sensor.altitude_raw - altitude_pred;
    float K_altitude = 0.5;  // Kalman gain (simplified, should be derived)
    float K_velocity = 0.1;
    
    state.altitude = altitude_pred + K_altitude * innovation;
    state.velocity_z = velocity_pred + K_velocity * innovation;
}

// ==================== SAFETY CHECKS ====================

bool safety_check() {
    // Battery voltage check
    if (sensor.voltage < BATTERY_MIN_VOLTAGE) {
        Serial.println("ABORT: Low battery voltage!");
        return false;
    }
    
    // Tilt check
    if (abs(state.roll) > MAX_TILT_DEG || abs(state.pitch) > MAX_TILT_DEG) {
        Serial.println("ABORT: Excessive tilt!");
        return false;
    }
    
    // Altitude check
    if (state.altitude > MAX_ALTITUDE_M) {
        Serial.println("ABORT: Altitude limit exceeded!");
        return false;
    }
    
    return true;  // All checks passed
}

// ==================== STATE MACHINE ====================

void run_state_machine(float dt) {
    switch (flight_state) {
        case PREFLIGHT:
            control.throttle = 0;
            control.pitch_cmd = 0;
            control.roll_cmd = 0;
            control.yaw_cmd = 0;
            reset_pid_controllers();
            break;
            
        case ARMED:
            control.throttle = 0;
            control.pitch_cmd = 0;
            control.roll_cmd = 0;
            control.yaw_cmd = 0;
            break;
            
        case TAKEOFF:
            // Simple open-loop takeoff: full throttle until 2m altitude
            control.throttle = 0.7;
            attitude_control(dt);  // Stabilize attitude
            
            if (state.altitude > 2.0) {
                flight_state = HOVER;
                Serial.println("Entering HOVER mode");
            }
            break;
            
        case HOVER:
            // Closed-loop altitude hold at 3m
            altitude_control(3.0, dt);
            attitude_control(dt);
            break;
            
        case LANDING:
            // Descend slowly to ground
            altitude_control(0.5, dt);  // Target 0.5m above ground
            attitude_control(dt);
            
            // Detect landing: low altitude + low velocity
            if (state.altitude < 0.2 && abs(state.velocity_z) < 0.2) {
                flight_state = LANDED;
                control.throttle = 0;
                Serial.println("Landed!");
                beep(2);
            }
            break;
            
        case LANDED:
            control.throttle = 0;
            control.pitch_cmd = 0;
            control.roll_cmd = 0;
            control.yaw_cmd = 0;
            break;
            
        case ABORT:
            // Emergency shutdown
            control.throttle = 0;
            control.pitch_cmd = 0;
            control.roll_cmd = 0;
            control.yaw_cmd = 0;
            beep_continuous();
            break;
    }
}

// ==================== CONTROL ALGORITHMS ====================

void attitude_control(float dt) {
    // PID control for attitude stabilization
    // Target: level (0° roll, 0° pitch)
    
    // Pitch PID
    float pitch_error = 0 - state.pitch;
    control.pitch_cmd = compute_pid(&pid_pitch, pitch_gains, pitch_error, dt);
    
    // Roll PID
    float roll_error = 0 - state.roll;
    control.roll_cmd = compute_pid(&pid_roll, roll_gains, roll_error, dt);
    
    // Yaw rate PID (target 0 deg/s for now)
    float yaw_error = 0 - sensor.gyro_z;
    control.yaw_cmd = compute_pid(&pid_yaw, yaw_gains, yaw_error, dt);
}

void altitude_control(float target_altitude, float dt) {
    // PID control for altitude hold
    float altitude_error = target_altitude - state.altitude;
    float throttle_cmd = compute_pid(&pid_altitude, altitude_gains, altitude_error, dt);
    
    // Add feedforward for gravity compensation
    control.throttle = 0.6 + throttle_cmd;  // 0.6 = hover throttle estimate
    
    // Clamp throttle
    control.throttle = constrain(control.throttle, 0.0, 1.0);
}

float compute_pid(PIDState* pid, PIDGains gains, float error, float dt) {
    // Proportional
    float P = gains.Kp * error;
    
    // Integral (with anti-windup)
    pid->integral += error * dt;
    pid->integral = constrain(pid->integral, -10.0, 10.0);
    float I = gains.Ki * pid->integral;
    
    // Derivative
    float D = gains.Kd * (error - pid->last_error) / dt;
    pid->last_error = error;
    
    return P + I + D;
}

void reset_pid_controllers() {
    pid_pitch.integral = 0;
    pid_pitch.last_error = 0;
    
    pid_roll.integral = 0;
    pid_roll.last_error = 0;
    
    pid_yaw.integral = 0;
    pid_yaw.last_error = 0;
    
    pid_altitude.integral = 0;
    pid_altitude.last_error = 0;
}

// ==================== CONTROL ALLOCATION ====================

void actuate_servos() {
    // CONTROL ALLOCATION: Convert control commands to vane deflections
    // Vanes in + configuration: North (0°), East (90°), South (180°), West (270°)
    
    // Math:
    // Pitch control: North/South vanes (North = negative pitch)
    // Roll control: East/West vanes
    // Yaw control: All 4 vanes differentially
    
    float vane_N = -control.pitch_cmd + control.yaw_cmd;
    float vane_E = control.roll_cmd - control.yaw_cmd;
    float vane_S = control.pitch_cmd + control.yaw_cmd;
    float vane_W = -control.roll_cmd - control.yaw_cmd;
    
    // Map to servo microseconds (1 degree command = 10 microseconds deflection)
    int servo_N = SERVO_CENTER + (int)(vane_N * 10);
    int servo_E = SERVO_CENTER + (int)(vane_E * 10);
    int servo_S = SERVO_CENTER + (int)(vane_S * 10);
    int servo_W = SERVO_CENTER + (int)(vane_W * 10);
    
    // Clamp to limits
    servo_N = constrain(servo_N, SERVO_MIN, SERVO_MAX);
    servo_E = constrain(servo_E, SERVO_MIN, SERVO_MAX);
    servo_S = constrain(servo_S, SERVO_MIN, SERVO_MAX);
    servo_W = constrain(servo_W, SERVO_MIN, SERVO_MAX);
    
    // Write to servos
    servo_north.writeMicroseconds(servo_N);
    servo_east.writeMicroseconds(servo_E);
    servo_south.writeMicroseconds(servo_S);
    servo_west.writeMicroseconds(servo_W);
}

void actuate_esc() {
    // Map throttle (0-1) to ESC microseconds (1100-2000)
    int esc_value = ESC_ARMED + (int)(control.throttle * (ESC_MAX - ESC_ARMED));
    esc_value = constrain(esc_value, ESC_ARMED, ESC_MAX);
    esc.writeMicroseconds(esc_value);
}

// ==================== TELEMETRY ====================

void send_telemetry() {
    // Pack telemetry into bytes for LoRa transmission
    uint8_t packet[32];
    
    // Simple telemetry format (you'll want to optimize this)
    packet[0] = (uint8_t)flight_state;
    memcpy(&packet[1], &state.roll, 4);       // bytes 1-4
    memcpy(&packet[5], &state.pitch, 4);      // bytes 5-8
    memcpy(&packet[9], &state.altitude, 4);   // bytes 9-12
    memcpy(&packet[13], &control.throttle, 4); // bytes 13-16
    memcpy(&packet[17], &sensor.voltage, 4);  // bytes 17-20
    
    radio.send(packet, 21);
    radio.waitPacketSent();
}

// ==================== DATA LOGGING ====================

void log_data() {
    if (!logfile) return;
    
    // Log every 10 loops (10 Hz logging)
    if (loop_count % 10 != 0) return;
    
    // CSV format: time,state,roll,pitch,yaw,altitude,velocity,throttle,voltage,current
    logfile.print(millis());
    logfile.print(",");
    logfile.print((int)flight_state);
    logfile.print(",");
    logfile.print(state.roll, 2);
    logfile.print(",");
    logfile.print(state.pitch, 2);
    logfile.print(",");
    logfile.print(state.yaw, 2);
    logfile.print(",");
    logfile.print(state.altitude, 2);
    logfile.print(",");
    logfile.print(state.velocity_z, 2);
    logfile.print(",");
    logfile.print(control.throttle, 3);
    logfile.print(",");
    logfile.print(sensor.voltage, 2);
    logfile.print(",");
    logfile.println(sensor.current, 2);
    
    // Flush every second to prevent data loss
    if (loop_count % 100 == 0) {
        logfile.flush();
    }
}

// ==================== SERIAL COMMANDS ====================

void handle_serial_commands() {
    if (!Serial.available()) return;
    
    char cmd = Serial.read();
    
    switch (cmd) {
        case 'a':  // Arm
            if (flight_state == PREFLIGHT) {
                flight_state = ARMED;
                Serial.println("ARMED");
                beep(1);
            }
            break;
            
        case 'd':  // Disarm
            flight_state = PREFLIGHT;
            Serial.println("DISARMED");
            beep(2);
            break;
            
        case 't':  // Takeoff
            if (flight_state == ARMED) {
                flight_state = TAKEOFF;
                Serial.println("TAKEOFF");
                beep(1);
            }
            break;
            
        case 'l':  // Land
            if (flight_state == HOVER || flight_state == TAKEOFF) {
                flight_state = LANDING;
                Serial.println("LANDING");
            }
            break;
            
        case 'x':  // Emergency abort
            flight_state = ABORT;
            Serial.println("ABORT!");
            break;
            
        case 's':  // Status
            print_status();
            break;
    }
}

void print_status() {
    Serial.println("\n=== STATUS ===");
    Serial.print("State: ");
    print_state_name();
    Serial.print("  Loop: ");
    Serial.print(loop_count);
    Serial.println(" Hz: ~100");
    
    Serial.print("Attitude: R=");
    Serial.print(state.roll, 1);
    Serial.print("° P=");
    Serial.print(state.pitch, 1);
    Serial.print("° Y=");
    Serial.print(state.yaw, 1);
    Serial.println("°");
    
    Serial.print("Altitude: ");
    Serial.print(state.altitude, 2);
    Serial.print(" m  Velocity: ");
    Serial.print(state.velocity_z, 2);
    Serial.println(" m/s");
    
    Serial.print("Battery: ");
    Serial.print(sensor.voltage, 2);
    Serial.print(" V  ");
    Serial.print(sensor.current, 2);
    Serial.println(" A");
    
    Serial.print("GPS: ");
    if (sensor.gps_valid) {
        Serial.print(sensor.gps_satellites);
        Serial.println(" sats");
    } else {
        Serial.println("NO FIX");
    }
    
    Serial.println("==============\n");
}

void print_state_name() {
    switch (flight_state) {
        case PREFLIGHT: Serial.print("PREFLIGHT"); break;
        case ARMED: Serial.print("ARMED"); break;
        case TAKEOFF: Serial.print("TAKEOFF"); break;
        case HOVER: Serial.print("HOVER"); break;
        case LANDING: Serial.print("LANDING"); break;
        case LANDED: Serial.print("LANDED"); break;
        case ABORT: Serial.print("ABORT"); break;
    }
}

// ==================== UTILITY FUNCTIONS ====================

void beep(int count) {
    for (int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }
}

void beep_continuous() {
    static unsigned long last_beep = 0;
    if (millis() - last_beep > 500) {
        digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
        last_beep = millis();
    }
}

void error_blink() {
    // Fast blink = error
    while (true) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}
