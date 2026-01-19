
// Calculations based on MPU-6050-Fusion by jremington
// Original source: https://github.com/jremington/MPU-6050-Fusion

#include "FastIMU.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#define IMU_ADDRESS 0x68
#define I2C_SDA 9
#define I2C_SCL 10

// Network connection
const char* ssid = "YourSSID";
const char* password = "YourPassword";
const char* pcIP = "192.168.1.XX"; // local IP adress of the PCM to send packets to
const int port = 5555; // Port that will listen to UDP - 5555 by default

calData calib = { 0 }; // Calibration data
AccelData accelData; // Sensor data
GyroData gyroData;

MPU6050 IMU;
WiFiUDP udp;

// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion
float q[4] = { 1.0, 0.0, 0.0, 0.0 };

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 20.0;
float Ki = 0.0;

// globals for AHRS loop timing
unsigned long last_ms = 0; // millis() timers
float yaw, pitch, roll, yaw_rad, pitch_rad, roll_rad; // Euler angle output
float yaw_offset = 0.0; // Drift correction variable

void setup()
{

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // 400khz clock
    Serial.begin(115200);

    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0) {
        Serial.print("Error initializing IMU: ");
        Serial.println(err);
        while (true) {
            ;
        }
    }

    Serial.println("Keep IMU level.");
    IMU.calibrateAccelGyro(&calib);
    Serial.println("Calibration done!");
    Serial.println("Accel biases X/Y/Z: ");
    Serial.print(calib.accelBias[0]);
    Serial.print(", ");
    Serial.print(calib.accelBias[1]);
    Serial.print(", ");
    Serial.println(calib.accelBias[2]);
    Serial.println("Gyro biases X/Y/Z: ");
    Serial.print(calib.gyroBias[0]);
    Serial.print(", ");
    Serial.print(calib.gyroBias[1]);
    Serial.print(", ");
    Serial.println(calib.gyroBias[2]);

    IMU.init(calib, IMU_ADDRESS);

    err = IMU.setGyroRange(500); // USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
    err = IMU.setAccelRange(2); // THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g

    if (err != 0) {
        Serial.print("Error Setting range: ");
        Serial.println(err);
        while (true) {
            ;
        }
    }

    delay(3000);
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Standby - connecting");
        Serial.println(WiFi.status());
    }
    udp.begin(port);
    Serial.println("Connected");
}

void loop()
{

    static float deltat = 0; // loop time in seconds
    static unsigned long now = 0, last = 0; // micros() timers

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost! Reconnecting...");
        WiFi.disconnect();
        WiFi.begin(ssid, password);
        return;
    }

    // raw data
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t Tmp; // temperature

    // scaled data as vector
    float Axyz[3];
    float Gxyz[3];

    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);

    Axyz[0] = accelData.accelX;
    Axyz[1] = accelData.accelY;
    Axyz[2] = accelData.accelZ;

    Gxyz[0] = (gyroData.gyroX) * 3.14 / 180.0; // 250 LSB(d/s) default to radians/s
    Gxyz[1] = (gyroData.gyroY) * 3.14 / 180.0;
    Gxyz[2] = (gyroData.gyroZ) * 3.14 / 180.0;

    now = micros();
    deltat = (now - last) * 1.0e-6; // seconds since last update
    last = now;

    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

    roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    // conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
    yaw = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
    // to degrees
    yaw *= 180.0 / PI;
    if (yaw < 0)
        yaw += 360.0; // compass circle
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    // DRIFT COMPENSATION - I did not use magnetometer, but since you will be mostly facing monitor head on, I've created a brackets for dfift compensation based on that. Works good for me.
    float current_view_yaw = yaw - yaw_offset;
    // normalisation
    if (current_view_yaw > 180.0)
        current_view_yaw -= 360.0;
    if (current_view_yaw < -180.0)
        current_view_yaw += 360.0;

    float drift_step = 0.0;

    if (current_view_yaw <= 7 && current_view_yaw >= -7) {
        drift_step = 1 * deltat; /// factor of drift compensation
    } else if (current_view_yaw > 7 && current_view_yaw <= 15 || current_view_yaw < -7 && current_view_yaw >= -15) {
        drift_step = 0.7 * deltat;
    } else if (current_view_yaw > 15 && current_view_yaw < 30 || current_view_yaw < -15 && current_view_yaw > -30) {
        drift_step = 0.5 * deltat;
    }

    if (current_view_yaw > drift_step) {
        yaw_offset += drift_step;
    } else if (current_view_yaw < -drift_step) {
        yaw_offset -= drift_step;
    } else {
        yaw_offset = yaw;
    }

    yaw = current_view_yaw;

    if (yaw < 0)
        yaw += 360.0;

    // print angles for serial plotter...
    Serial.print(yaw, 0);
    Serial.print(", ");
    Serial.print(pitch, 0);
    Serial.print(", ");
    Serial.println(roll, 0);

    yaw_rad = yaw * 3.14 / 180.0; // Angle to radians for OpenTrack
    pitch_rad = pitch * 3.14 / 180.0;
    roll_rad = roll * 3.14 / 180.0;

    uint8_t tinyPacket[14]; // Header of UDP packet
    tinyPacket[0] = 0x00;
    tinyPacket[1] = 0x02;

    packFloat(tinyPacket, 2, yaw_rad); // UDP data packet
    packFloat(tinyPacket, 6, pitch_rad);
    packFloat(tinyPacket, 10, roll_rad);

    if (udp.beginPacket(pcIP, port)) {
        udp.write(tinyPacket, 14);
        if (!udp.endPacket()) {
            Serial.println("UDP Send Error");
        }
    } else {
        Serial.println("UDP Begin Error");
    }

    delay(5);
}

// Based on MPU-6050-Fusion by jremington
// Original source: https://github.com/jremington/MPU-6050-Fusion

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat)
{
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez; // error terms
    float qa, qb, qc;
    static float ix = 0.0, iy = 0.0, iz = 0.0; // integral feedback terms
    float tmp;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    tmp = ax * ax + ay * ay + az * az;
    if (tmp > 0.0) {

        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recipNorm = 1.0 / sqrt(tmp);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity in the body frame (factor of two divided out)
        vx = q[1] * q[3] - q[0] * q[2]; // to normalize these terms, multiply each by 2.0
        vy = q[0] * q[1] + q[2] * q[3];
        vz = q[0] * q[0] - 0.5f + q[3] * q[3];

        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f) {
            ix += Ki * ex * deltat; // integral error a pomocnicza do pakowania floatów (scaled by Ki
            iy += Ki * ey * deltat;
            iz += Ki * ez * deltat;
            gx += ix; // apply integral feedback
            gy += iy;
            gz += iz;
        }

        // Apply proportional feedback to gyro term
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    // Integrate rate of change of quaternion, q cross gyro term
    deltat = 0.5 * deltat;
    gx *= deltat; // pre-multiply common factors
    gy *= deltat;
    gz *= deltat;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);

    // renormalise quaternion
    recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] = q[0] * recipNorm;
    q[1] = q[1] * recipNorm;
    q[2] = q[2] * recipNorm;
    q[3] = q[3] * recipNorm;
}

// Function for Little-Endian
void packFloat(uint8_t* buf, int offset, float value)
{
    union {
        float f;
        uint8_t b[4];
    } data;
    data.f = value;
    // Format Little-Endian
    buf[offset] = data.b[0];
    buf[offset + 1] = data.b[1];
    buf[offset + 2] = data.b[2];
    buf[offset + 3] = data.b[3];
}
