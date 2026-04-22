#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_INA219.h>
#include <TinyGPSPlus.h>

MPU6050 mpu;
Adafruit_BMP5xx bmp;
Adafruit_INA219 ina219(0x40);
TinyGPSPlus gps;

static uint32_t lastPrint = 0;

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);
    while (!Serial && millis() < 3000);

    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();

    bmp.begin((uint8_t)0x47);

    ina219.begin();
}

void loop() {
    while (Serial1.available())
        gps.encode(Serial1.read());

    if (millis() - lastPrint < 100)
        return;
    lastPrint = millis();

    // MPU6050 roll/pitch from raw accel
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float axf = ax / 16384.0f;
    float ayf = ay / 16384.0f;
    float azf = az / 16384.0f;
    float roll  = atan2f(ayf, azf) * 180.0f / M_PI;
    float pitch = atan2f(-axf, sqrtf(ayf * ayf + azf * azf)) * 180.0f / M_PI;

    // BMP580
    bmp.performReading();
    float temp_c   = bmp.temperature;
    float pressure = bmp.pressure;
    float altitude = bmp.readAltitude(1013.25);

    // INA219
    float busV = ina219.getBusVoltage_V();

    // GPS
    float lat = gps.location.isValid() ? (float)gps.location.lat() : 0.0f;
    float lon = gps.location.isValid() ? (float)gps.location.lng() : 0.0f;

    Serial.printf(
        "roll=%.2f pitch=%.2f | alt=%.2fm temp=%.1fC | bus=%.2fV | lat=%.6f lon=%.6f\n",
        roll, pitch, altitude, temp_c, busV, lat, lon
    );
}
