#include <Wire.h>
#include <MPU9250.h>

// MPU9250 객체 생성
MPU9250 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.setup(0x68)) {
    Serial.println("MPU 연결 실패!");
    while (1);
  }
}

void loop() {
  // 센서 데이터 업데이트
  mpu.update();

  // 가속도 데이터
  float accelX = mpu.getAccX();
  float accelY = mpu.getAccY();
  float accelZ = mpu.getAccZ();

  // 자이로 데이터
  float gyroX = mpu.getGyroX();
  float gyroY = mpu.getGyroY();
  float gyroZ = mpu.getGyroZ();

  // 가속도 크기 계산
  float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  // 자이로 크기 계산
  float gyroMagnitude = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  // 강아지 상태 판단
  String dogState;
  if (accelMagnitude < 0.5 && gyroMagnitude < 0.5) {
    dogState = "가만히 있음";
  } else if (accelMagnitude < 1.5 && gyroMagnitude < 1.5) {
    dogState = "걷는 중";
  } else if (accelMagnitude < 3.0 && gyroMagnitude < 3.0) {
    dogState = "뛰는 중";
  } else {
    dogState = "몸을 터는 중";
  }

  // 상태 출력
  Serial.println("강아지 상태: " + dogState);
  
  delay(1000);
}
