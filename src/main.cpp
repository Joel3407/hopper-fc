#include <Arduino.h>
#include <Wire.h>

struct KnownDevice {
    uint8_t addr;
    const char* name;
};

static constexpr KnownDevice KNOWN_DEVICES[] = {
    { 0x28, "BNO055" },
    { 0x40, "INA219" },
    { 0x46, "BMP580" },
    { 0x76, "BMP388" },
};

static const char* identify(uint8_t addr) {
    for (const auto& d : KNOWN_DEVICES) {
        if (d.addr == addr) return d.name;
    }
    return nullptr;
}

static void scan() {
    Serial.println("--- I2C Scan ---");
    uint8_t found = 0;

    for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();

        if (err == 0) {
            const char* name = identify(addr);
            if (name) {
                Serial.printf("  0x%02X  %s\n", addr, name);
            } else {
                Serial.printf("  0x%02X  (unknown)\n", addr);
            }
            found++;
        }
    }

    if (found == 0) {
        Serial.println("  No devices found");
    } else {
        Serial.printf("  %u device(s) found\n", found);
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("Boese Aerospace — I2C Scanner");
    Serial.printf("Build: %s %s\n\n", __DATE__, __TIME__);

    Wire.begin();
    Wire.setClock(400000);

    scan();
}

void loop() {
    static uint32_t last = 0;
    if (millis() - last >= 5000) {
        last = millis();
        scan();
    }
}
