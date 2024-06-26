#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

// MPU9250 객체 생성
MPU9250 accelgyro;
I2Cdev I2C_M;

uint8_t buffer_m[6];
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];

// 상태를 저장할 변수
enum DogState { WALKING, RUNNING, SHAKING, STILL };
DogState currentState;

void setup()
{
    Wire.begin();
    Serial.begin(38400);
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
    delay(1000);
}

void loop()
{
    getAccel_Data();
    getGyro_Data();

    determineState();

    switch(currentState) {
        case WALKING:
            Serial.println("Dog is Walking");
            break;
        case RUNNING:
            Serial.println("Dog is Running");
            break;
        case SHAKING:
            Serial.println("Dog is Shaking");
            break;
        case STILL:
            Serial.println("Dog is Still");
            break;
    }

    delay(1000);
}

void determineState() {
    // 가속도 값의 크기를 계산
    float accelMagnitude = sqrt(Axyz[0]*Axyz[0] + Axyz[1]*Axyz[1] + Axyz[2]*Axyz[2]);
    // 자이로 값의 크기를 계산
    float gyroMagnitude = sqrt(Gxyz[0]*Gxyz[0] + Gxyz[1]*Gxyz[1] + Gxyz[2]*Gxyz[2]);

    // 상태 결정 로직 (범위와 조건은 상황에 따라 조정 필요)
    if (accelMagnitude < 1.05 && gyroMagnitude < 10) {
        currentState = STILL;
    } else if (gyroMagnitude > 100) {
        currentState = SHAKING;
    } else if (accelMagnitude < 1.2) {
        currentState = WALKING;
    } else {
        currentState = RUNNING;
    }
}

void getAccel_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (float)ax / 16384;
    Axyz[1] = (float)ay / 16384;
    Axyz[2] = (float)az / 16384;
}

void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (float)gx * 250 / 32768;
    Gxyz[1] = (float)gy * 250 / 32768;
    Gxyz[2] = (float)gz * 250 / 32768;
}
