#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// BLE 서비스 및 특성 정의
BLEService testingService("180A"); // BLE 서비스 UUID 설정
BLEByteCharacteristic switchCharacteristic("2A57", BLERead | BLEWrite); // LED 제어를 위한 특성, 읽기/쓰기 가능
BLECharacteristic textCharacteristic("2A58", BLERead | BLENotify, 20); // 텍스트 전송 특성, 읽기/알림 가능, 최대 20바이트

// IMU 값 전송을 위한 특성 (최대 50바이트로 데이터 크기 지정)
BLECharacteristic imuCharacteristic("2A59", BLERead | BLENotify, 50);

// 가속도계 값을 저장할 변수
float x, y, z;

void setup() {
  Serial.begin(9600); // 시리얼 통신 시작
  pinMode(LEDR, OUTPUT); // 빨간 LED 핀 설정
  pinMode(LEDG, OUTPUT); // 녹색 LED 핀 설정
  pinMode(LEDB, OUTPUT); // 파란 LED 핀 설정

  // BLE 초기화
  if (!BLE.begin()) { // BLE 시작 실패 시 에러 메시지 출력
    Serial.println("BLE 시작 실패!");
    while (1); // 무한 루프에 빠져 프로그램 종료
  }

  // IMU 센서 초기화
  if (!IMU.begin()) { // IMU 시작 실패 시 에러 메시지 출력
    Serial.println("IMU 초기화 실패!");
    while (1); // 무한 루프에 빠져 프로그램 종료
  }

  // 가속도계 샘플 레이트 출력 (IMU가 얼마나 자주 데이터를 업데이트하는지)
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  // BLE 설정
  BLE.setLocalName("Pet-I_BLE_Testing"); // BLE 로컬 이름 설정
  BLE.setAdvertisedService(testingService); // 서비스 광고

  // 서비스에 특성 추가
  testingService.addCharacteristic(switchCharacteristic); // LED 제어 특성 추가
  testingService.addCharacteristic(textCharacteristic); // 텍스트 전송 특성 추가
  testingService.addCharacteristic(imuCharacteristic); // IMU 데이터 전송 특성 추가

  // BLE에 서비스 추가
  BLE.addService(testingService);

  // 특성 초기값 설정 (LED 제어 초기값 0)
  switchCharacteristic.writeValue(0);

  // BLE 광고 시작
  BLE.advertise();

  Serial.println("BLE 서비스 시작");
}

// IMU 데이터를 읽고 BLE로 전송하는 함수
void sendIMUData() {
  // IMU 가속도계 값이 사용 가능한 경우
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z); // 가속도계 값 읽기
    char imuData[50]; // IMU 데이터를 저장할 배열
    // 가속도계 값 형식에 맞게 데이터를 문자열로 변환 (gyro = x, y, z | accel = x, y, z)
    snprintf(imuData, sizeof(imuData), "gyro = x: %.2f, y: %.2f, z: %.2f | accel = x: %.2f, y: %.2f, z: %.2f", x, y, z, x * 10, y * 10, z * 10);
    Serial.println(imuData); // 시리얼 모니터에 IMU 값 출력
    imuCharacteristic.writeValue(imuData); // BLE로 IMU 데이터 전송
  }
}

// BLE 연결 및 LED 제어 함수
void led_testing() {
  // BLE 중앙 기기가 연결될 때까지 대기
  BLEDevice central = BLE.central();

  // BLE 중앙 기기가 연결된 경우
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address()); // 연결된 중앙 기기의 MAC 주소 출력

    textCharacteristic.writeValue("Pet-I is Running Now"); // BLE를 통해 연결 성공 메시지 전송
    digitalWrite(LEDR, HIGH); // 빨간 LED 켜기
    digitalWrite(LEDG, LOW); // 녹색 LED 끄기
    digitalWrite(LEDB, LOW); // 파란 LED 끄기

    // 중앙 기기가 연결된 동안 계속 실행
    while (central.connected()) {
      // 중앙 기기에서 LED 제어 값을 변경한 경우
      if (switchCharacteristic.written()) {
        // 변경된 값에 따라 LED 제어
        switch (switchCharacteristic.value()) {
          case 1: // 1 입력 시 빨간 LED 켜기
            Serial.println("LEDR LED On");
            textCharacteristic.writeValue("LEDR LED On"); // BLE를 통해 상태 전송
            digitalWrite(LEDR, LOW); // 빨간 LED 켜기
            digitalWrite(LEDG, HIGH); // 녹색 LED 끄기
            digitalWrite(LEDB, HIGH); // 파란 LED 끄기
            break;
          case 2: // 2 입력 시 녹색 LED 켜기
            Serial.println("LEDG LED On");
            textCharacteristic.writeValue("LEDG LED On");
            digitalWrite(LEDR, HIGH); // 빨간 LED 끄기
            digitalWrite(LEDG, HIGH); // 녹색 LED 켜기
            digitalWrite(LEDB, LOW); // 파란 LED 끄기
            break;
          case 3: // 3 입력 시 파란 LED 켜기
            Serial.println("LEDB LED On");
            textCharacteristic.writeValue("LEDB LED On");
            digitalWrite(LEDR, HIGH); // 빨간 LED 끄기
            digitalWrite(LEDG, LOW); // 녹색 LED 끄기
            digitalWrite(LEDB, HIGH); // 파란 LED 켜기
            break;
          default: // 그 외 값 입력 시 모든 LED 끄기
            Serial.println("LED Off");
            textCharacteristic.writeValue("LED Off");
            digitalWrite(LEDR, HIGH); // 빨간 LED 끄기
            digitalWrite(LEDG, HIGH); // 녹색 LED 끄기
            digitalWrite(LEDB, HIGH); // 파란 LED 끄기
            break;
        }
      }

      // IMU 데이터 전송 (1초에 한 번)
      sendIMUData();
      delay(1000); // 1초 대기
    }

    // 연결이 끊긴 경우
    Serial.print("Disconnected from central: ");
    Serial.println(central.address()); // 연결이 끊긴 중앙 기기의 MAC 주소 출력
    digitalWrite(LEDR, HIGH); // 빨간 LED 끄기
    digitalWrite(LEDG, HIGH); // 녹색 LED 끄기
    digitalWrite(LEDB, HIGH); // 파란 LED 끄기
  }
}

void loop() {
  // 메인 루프에서 BLE 연결 및 IMU 데이터 전송 처리
  led_testing();
}
