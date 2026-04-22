#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP5xx.h>

static Adafruit_BMP5xx bmp;

// Sea-level pressure for altitude reference — adjust to local QNH
static constexpr float SEA_LEVEL_HPA = 1013.25f;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("Boese Aerospace — BMP581 Test");
    Serial.printf("Build: %s %s\n\n", __DATE__, __TIME__);

    Wire.begin();
    Wire.setClock(400000);

    if (!bmp.begin((uint8_t)0x47)) {
        Serial.println("ERROR: BMP581 not found at 0x47 — check wiring");
        while (1) delay(100);
    }

    Serial.println("  Press(hPa)   Temp(C)   Alt(m)");
    Serial.println("  ----------   -------   ------");
}

void loop() {
    static uint32_t last = 0;
    if (millis() - last < 200) return;
    last = millis();

    if (!bmp.performReading()) {
        Serial.println("ERROR: BMP581 read failed");
        return;
    }

    float pressure    = bmp.pressure / 100.0f;
    float temperature = bmp.temperature;
    float altitude    = bmp.readAltitude(SEA_LEVEL_HPA);

    Serial.printf("  %10.2f   %7.2f   %6.1f\n", pressure, temperature, altitude);
}
