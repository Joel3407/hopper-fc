#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219(0x40);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Wire.begin();

    if (!ina219.begin()) {
        Serial.println("ERROR: INA219 not found at 0x40");
        while (1);
    }

    Serial.println("INA219 ready");
    Serial.println("Bus(V)\tCurrent(mA)\tPower(mW)");
}

void loop() {
    float busV   = ina219.getBusVoltage_V();
    float currentMA = ina219.getCurrent_mA();
    float powerMW   = ina219.getPower_mW();

    Serial.print(busV, 3);     Serial.print(" V\t");
    Serial.print(currentMA, 2); Serial.print(" mA\t");
    Serial.print(powerMW, 2);   Serial.println(" mW");

    delay(500);
}
