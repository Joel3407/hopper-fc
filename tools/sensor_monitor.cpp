// Sensor-only monitor for Teensy 4.1.
// No servos, no ESC, no state machine. Prints one labeled line at 10 Hz.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_INA219.h>
#include <TinyGPSPlus.h>

static const uint32_t BNO_REPORT_US = 10000;   // 100 Hz
static const uint32_t PRINT_MS      = 100;     // 10 Hz
static const float    SEA_LEVEL_HPA = 1013.25f;

Adafruit_BNO08x bno08x(-1);
Adafruit_BMP5xx bmp;
Adafruit_INA219 ina(0x40);
TinyGPSPlus     gps;
sh2_SensorValue_t sensorValue;

bool bnoOk = false, bmpOk = false, inaOk = false;

// Latest values
float roll = NAN, pitch = NAN, yaw = NAN;
float alt = NAN, temp = NAN;
float altZero = 0.0f;
bool  altZeroed = false;

static bool enableGameRotation() {
    return bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, BNO_REPORT_US);
}

static void quatToEuler(float w, float x, float y, float z) {
    float sinr = 2.0f * (w * x + y * z);
    float cosr = 1.0f - 2.0f * (x * x + y * y);
    roll = atan2f(sinr, cosr) * RAD_TO_DEG;

    float sinp = 2.0f * (w * y - z * x);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    pitch = asinf(sinp) * RAD_TO_DEG;

    float siny = 2.0f * (w * z + x * y);
    float cosy = 1.0f - 2.0f * (y * y + z * z);
    yaw = atan2f(siny, cosy) * RAD_TO_DEG;
}

static void initBNO() {
    Serial.print("BNO08x ... ");
    bool found = bno08x.begin_I2C(0x4B, &Wire);
    if (!found) found = bno08x.begin_I2C(0x4A, &Wire);
    if (found && enableGameRotation()) {
        bnoOk = true;
        Serial.println("OK");
    } else {
        Serial.println("FAILED");
    }
}

static void initBMP() {
    Serial.print("BMP5xx ... ");
    for (int i = 0; i < 5 && !bmpOk; i++) {
        bmpOk = bmp.begin((uint8_t)0x47);
        if (!bmpOk) delay(100);
    }
    Serial.println(bmpOk ? "OK" : "FAILED");
    if (!bmpOk) return;

    // First samples after begin() are unsettled: discard some, then average for the zero.
    for (int i = 0; i < 10; i++) { bmp.performReading(); delay(20); }
    float sum = 0; int n = 0;
    for (int i = 0; i < 10; i++) {
        if (bmp.performReading()) { sum += 44330.0f * (1.0f - powf(bmp.pressure / SEA_LEVEL_HPA, 0.1903f)); n++; }
        delay(20);
    }
    if (n > 0) { altZero = sum / n; altZeroed = true; }
}

static void initINA() {
    Serial.print("INA219 ... ");
    inaOk = ina.begin(&Wire);
    Serial.println(inaOk ? "OK" : "FAILED");
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    Wire.begin();
    Wire.setClock(100000);

    initBNO();
    initBMP();
    initINA();

    Serial1.begin(9600);
    Serial.println("GPS    ... OK (waiting for NMEA)");
}

void loop() {
    // GPS: feed parser continuously
    while (Serial1.available()) gps.encode(Serial1.read());

    // BNO08x: drain all pending events
    if (bnoOk) {
        if (bno08x.wasReset()) {
            Serial.println("BNO08x reset - re-enabling report");
            enableGameRotation();
        }
        while (bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
                auto &q = sensorValue.un.gameRotationVector;
                quatToEuler(q.real, q.i, q.j, q.k);
            }
        }
    }

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint < PRINT_MS) return;
    lastPrint = millis();

    if (bmpOk && bmp.performReading()) {
        float a = 44330.0f * (1.0f - powf(bmp.pressure / SEA_LEVEL_HPA, 0.1903f));
        if (!altZeroed) { altZero = a; altZeroed = true; }
        alt  = a - altZero;
        temp = bmp.temperature;
    }

    float vbat = NAN, cur = NAN;
    if (inaOk) {
        vbat = ina.getBusVoltage_V();
        cur  = ina.getCurrent_mA();
    }

    Serial.print("roll=");  Serial.print(roll, 1);
    Serial.print(" pitch="); Serial.print(pitch, 1);
    Serial.print(" yaw=");   Serial.print(yaw, 1);
    Serial.print(" | alt="); Serial.print(alt, 2);
    Serial.print(" temp=");  Serial.print(temp, 1);
    Serial.print(" p=");     Serial.print(bmp.pressure, 2);
    Serial.print(" | vbat="); Serial.print(vbat, 2);
    Serial.print(" cur=");   Serial.print(cur, 0);
    Serial.print(" | gps="); Serial.print(gps.location.isValid() ? "fix" : "no fix");
    Serial.print(" sats=");  Serial.print(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
    if (gps.location.isValid()) {
        Serial.print(" lat="); Serial.print(gps.location.lat(), 6);
        Serial.print(" lon="); Serial.print(gps.location.lng(), 6);
    }
    Serial.println();
}
