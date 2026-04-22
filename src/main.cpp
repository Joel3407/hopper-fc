#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu(0x68);

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
VectorInt16 accel;
VectorInt16 gyro;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Wire.begin();
    Wire.setClock(400000);

    Serial.println("Initializing MPU-6050...");
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU-6050 connection FAILED");
        while (1);
    }
    Serial.println("MPU-6050 connected");

    devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
        Serial.print("DMP init failed: ");
        Serial.println(devStatus);
        while (1);
    }

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("DMP ready\n");
}

void loop() {
    static uint32_t lastPrint = 0;
    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetAccel(&accel, fifoBuffer);
        mpu.dmpGetGyro(&gyro, fifoBuffer);
    }

    if (millis() - lastPrint >= 100) {
        lastPrint = millis();

        Serial.print("YPR  ");
        Serial.print(ypr[0] * 180.0f / M_PI, 2);
        Serial.print("  ");
        Serial.print(ypr[1] * 180.0f / M_PI, 2);
        Serial.print("  ");
        Serial.print(ypr[2] * 180.0f / M_PI, 2);

        Serial.print("  |  A ");
        Serial.print(accel.x); Serial.print(" ");
        Serial.print(accel.y); Serial.print(" ");
        Serial.print(accel.z);

        Serial.print("  |  G ");
        Serial.print(gyro.x); Serial.print(" ");
        Serial.print(gyro.y); Serial.print(" ");
        Serial.println(gyro.z);
    }
}
