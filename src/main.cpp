#include <Arduino.h>
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    pinMode(15, OUTPUT);
    digitalWrite(15, LOW);
    delay(200);
    digitalWrite(15, HIGH);
    delay(700);
    Wire.begin();
    Serial.println("I2C Scanner ready");
}

void loop() {
    Serial.println("\n--- I2C Scan ---");
    int found = 0;
    for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X\n", addr);
            found++;
        }
    }
    Serial.printf("Found %d device(s)\n", found);
    delay(5000);
}
