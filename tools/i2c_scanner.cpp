#include <Arduino.h>
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    Wire.begin();
    Wire.setClock(400000);
    Serial.println("I2C Scanner ready");
}

void loop() {
    Serial.println("\n--- I2C Scan ---");
    int found = 0;
    for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X", addr);
            if (addr == 0x28 || addr == 0x29) Serial.print("  BNO055");
            if (addr == 0x40)                 Serial.print("  INA219");
            if (addr == 0x47)                 Serial.print("  BMP580");
            if (addr == 0x68)                 Serial.print("  MPU6050");
            if (addr == 0x76 || addr == 0x77) Serial.print("  BMP388");
            Serial.println();
            found++;
        }
    }
    Serial.printf("Found %d device(s)\n", found);
    delay(5000);
}
