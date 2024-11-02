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
    Serial.println("MAX30105를 찾을 수 없습니다.");
    while (1);
  }
  if (!mlx.begin()) {
    Serial.println("MLX 센서에 연결 오류.");
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("BLE 시작 실패!");
    while (1);
  }

  BLE.setLocalName("PET-I Sensor");
  BLE.setAdvertisedService(sensorDataService);
  sensorDataService.addCharacteristic(combinedDataCharacteristic);

  BLE.addService(sensorDataService);
  BLE.advertise();

  Serial.println("센서 준비 완료.");

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
}

void loop() {
  long irValue = particleSensor.getIR();
  BLEDevice central = BLE.central();

  if (central) {
    // BLE 연결 상태: 하늘색 LED (파랑+초록)
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, LOW);

    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    objectTemp = mlx.readObjectTempC();
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    snprintf(ptArrCompleteData[0], sizeof(ptArrCompleteData[0]), " %.2f| %.1f V|", objectTemp, beatsPerMinute);
    snprintf(ptArrCompleteData[1], sizeof(ptArrCompleteData[1]), " %.2f| %.2f| %.2fA", ax, ay, az);
    snprintf(ptArrCompleteData[2], sizeof(ptArrCompleteData[2]), "%.2f| %.2f| %.2f!", gx, gy, gz);
    
    for (int i = 0; i < 3; i++) {
      combinedDataCharacteristic.writeValue(ptArrCompleteData[i]);
      Serial.println("전송된 데이터:");
      Serial.println(ptArrCompleteData[i]);
      delay(200);
    }

    delay(1000);

  } else {
    // BLE 연결되지 않은 상태: 주황색 LED (빨강+초록)
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);

    BLE.disconnect();  // 강제 연결 초기화
    BLE.advertise();   // 광고 재개
    Serial.println("BLE 광고 재개 중...");
    delay(1000);       // 안정성 확보를 위해 잠시 대기
  }
}
