#include <Arduino.h>
#include <TinyGPSPlus.h>

TinyGPSPlus gps;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial1.begin(9600);
    Serial.println("TinyGPSPlus GPS test — Serial1 @ 9600 baud");
    Serial.println("-------------------------------------------");
}

void loop() {
    while (Serial1.available()) {
        char c = Serial1.read();

        // Print raw NMEA character-by-character; newline on '\n'
        if (c == '\r') {
            // skip bare CR
        } else if (c == '\n') {
            Serial.println();
        } else {
            Serial.write(c);
        }

        // Feed to TinyGPSPlus; print parsed fix whenever a sentence completes
        if (gps.encode(c) && gps.location.isUpdated()) {
            Serial.println();
            Serial.println("=== Parsed Fix ===");

            Serial.print("  Lat:  ");
            if (gps.location.isValid())
                Serial.println(gps.location.lat(), 6);
            else
                Serial.println("INVALID");

            Serial.print("  Lon:  ");
            if (gps.location.isValid())
                Serial.println(gps.location.lng(), 6);
            else
                Serial.println("INVALID");

            Serial.print("  Alt:  ");
            if (gps.altitude.isValid()) {
                Serial.print(gps.altitude.meters(), 1);
                Serial.println(" m");
            } else {
                Serial.println("INVALID");
            }

            Serial.print("  Sats: ");
            if (gps.satellites.isValid())
                Serial.println(gps.satellites.value());
            else
                Serial.println("INVALID");

            Serial.println("==================");
            Serial.println();
        }
    }
}
