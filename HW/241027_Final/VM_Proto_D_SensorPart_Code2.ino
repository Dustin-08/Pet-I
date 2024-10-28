// 초기 코드
#include <ArduinoBLE.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MLX90614.h>
#include <Arduino_LSM6DSOX.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 particleSensor;

long lastBeat = 0;
float beatsPerMinute;
float objectTemp;

BLEService sensorDataService("180D");

BLEStringCharacteristic combinedDataCharacteristic("2A57", BLERead | BLENotify, 20);

float ax, ay, az;
float gx, gy, gz;

char ptArrCompleteData[4][21];

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105를 찾을 수 없습니다. 배선/전원을 확인하십시오.");
    while (1);
  }
  if (!mlx.begin()) {
    Serial.println("MLX 센서에 연결하는 중 오류 발생. 배선을 확인하십시오.");
  }
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  if (!BLE.begin()) {
    Serial.println("BLE 시작 실패!");
    while (1);
  }

  BLE.setLocalName("Pet-I");
  BLE.setAdvertisedService(sensorDataService);

  sensorDataService.addCharacteristic(combinedDataCharacteristic);
  BLE.addService(sensorDataService);
  BLE.advertise();

  Serial.println("센서에 일정한 압력으로 손가락을 올려 놓으세요.");

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("BLE 연결됨: ");
    Serial.println(central.address());

    // 하늘색으로 LED 켜기 (파랑 + 초록)
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, LOW);

    while (central.connected()) {
      long irValue = particleSensor.getIR();
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);
      objectTemp = mlx.readObjectTempC();
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);

      char ptArrCompleteData[5][21];
      snprintf(ptArrCompleteData[0], sizeof(ptArrCompleteData[0]), "%.2f | ", objectTemp);
      snprintf(ptArrCompleteData[1], sizeof(ptArrCompleteData[1]), "%.1f | ", beatsPerMinute);
      snprintf(ptArrCompleteData[2], sizeof(ptArrCompleteData[2]), "%.2f,%.2f,%.2f | ", ax, ay, az);
      snprintf(ptArrCompleteData[3], sizeof(ptArrCompleteData[3]), "%.2f,%.2f,%.2f!", gx, gy, gz);

      for (int i = 0; i < 4; i++) {
          combinedDataCharacteristic.writeValue(ptArrCompleteData[i]);
          Serial.println("전송된 데이터:");
          Serial.println(ptArrCompleteData[i]);
          delay(200);
      }

      delay(1000);
    }

    Serial.print("BLE 연결 끊김: ");
    Serial.println(central.address());
  } else {
    // 주황색 깜빡임 (빨강 + 초록)
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);
    delay(500);
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    delay(500);
  }
}
