#include <ArduinoBLE.h>       // BLE 라이브러리
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MLX90614.h>
#include <Arduino_LSM6DSOX.h> // IMU 센서 라이브러리

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 particleSensor;

// 심박수 및 온도 관련 변수
long lastBeat = 0; // 마지막 박동이 발생한 시간
float beatsPerMinute;
float objectTemp;

// BLE 서비스 및 characteristic 정의
BLEService sensorDataService("180D"); // 서비스 UUID

// 결합된 데이터를 전송할 characteristic 정의 (가속도 및 자이로 데이터 포함)
BLEStringCharacteristic combinedDataCharacteristic("2A57", BLERead | BLENotify, 20); // 결합된 데이터 전송용 characteristic

// IMU 관련 변수
float ax, ay, az; // 가속도계 데이터를 저장할 변수
float gx, gy, gz; // 자이로스코프 데이터를 저장할 변수

//전체 데이터를 저장할 변수
char ptArrCompleteData[4][21];

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // 심박수 및 온도 센서 초기화
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { // 기본 I2C 포트, 400kHz 속도 사용
    Serial.println("MAX30105를 찾을 수 없습니다. 배선/전원을 확인하십시오.");
    while (1);
  }
  if (!mlx.begin()) {
    Serial.println("MLX 센서에 연결하는 중 오류 발생. 배선을 확인하십시오.");
  }

  // IMU 센서 초기화
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // BLE 초기화
  if (!BLE.begin()) {
    Serial.println("BLE 시작 실패!");
    while (1); // 무한 루프
  }

  BLE.setLocalName("DogMonitor"); // BLE 장치 이름 설정
  BLE.setAdvertisedService(sensorDataService); 

  // BLE 서비스에 characteristic 추가
  sensorDataService.addCharacteristic(combinedDataCharacteristic); // 결합된 데이터 전송용 characteristic 추가

  BLE.addService(sensorDataService); 
  BLE.advertise(); // BLE 시작

  Serial.println("센서에 일정한 압력으로 손가락을 올려 놓으세요.");

  particleSensor.setup(); // 센서 기본 설정으로 구성
  particleSensor.setPulseAmplitudeRed(0x0A); // 센서가 작동 중임을 표시하기 위해 적색 LED를 낮게 설정
  particleSensor.setPulseAmplitudeGreen(0); // 녹색 LED 끄기
}

void loop() {
    long irValue = particleSensor.getIR();
    BLEDevice central = BLE.central(); // 장치 연결 확인

    if (central) {
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        // 체온, 심박수, 가속도계, 자이로스코프 값 결합
        objectTemp = mlx.readObjectTempC();
        IMU.readAcceleration(ax, ay, az); // 가속도계 값 읽기
        IMU.readGyroscope(gx, gy, gz); // 자이로스코프 값 읽기

        // ptArrCompleteData 배열 초기화
        char ptArrCompleteData[5][21]; // 5개의 요소, 각 요소의 최대 길이는 20

        // 각 데이터를 ptArrCompleteData에 할당
        snprintf(ptArrCompleteData[0], sizeof(ptArrCompleteData[0]), "%.2f | ", objectTemp); // 0번째 요소: objectTemp
        snprintf(ptArrCompleteData[1], sizeof(ptArrCompleteData[1]), "%.1f | ", beatsPerMinute); // 1번째 요소: beatsPerMinute
        snprintf(ptArrCompleteData[2], sizeof(ptArrCompleteData[2]), "%.2f,%.2f,%.2f | ", ax, ay, az); // 2번째 요소: ax, ay, az
        snprintf(ptArrCompleteData[3], sizeof(ptArrCompleteData[3]), "%.2f,%.2f,%.2f!", gx, gy, gz); // 3번째 요소: gx, gy, gz
        
        // BLE로 데이터 송신
        for (int i = 0; i < 4; i++) {
            combinedDataCharacteristic.writeValue(ptArrCompleteData[i]); // i번째 요소 송신
            Serial.println("전송된 데이터:");
            Serial.println(ptArrCompleteData[i]); // 전송된 데이터 출력
            delay(200); // 0.2초 대기
        }

        delay(1000); // 1초마다 데이터 갱신
    }
}



