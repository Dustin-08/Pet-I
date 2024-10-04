#include <ArduinoBLE.h>

long previousMillis = 0;
int interval = 0;
int ledState = LOW;

BLEService rgbledService("180A"); // BLE LED 서비스

// BLE LED 스위치 특성 - 사용자 정의 128비트 UUID, 중앙에서 읽기 및 쓰기 가능
BLEByteCharacteristic switchCharacteristic("2A57", BLERead | BLEWrite);

// BLE 텍스트 전송 특성 - 사용자 정의 128비트 UUID, 중앙에서 읽기 가능
BLECharacteristic textCharacteristic("2A58", BLERead | BLENotify, 20);

// 3색 LED의 각 핀을 3, 4, 5번으로 설정합니다.(빨강 = 3, 초록 = 4, 파랑 = 5)
int red = 3;
int green = 4;
int blue = 5;

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  // 내장 LED 핀을 출력 모드로 설정
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);

  // 초기화 시작
  if (!BLE.begin()) {
    Serial.println("Bluetooth® Low Energy 시작 실패!");

    while (1);
  }

  // 광고용 로컬 이름과 서비스 UUID 설정:
  BLE.setLocalName("Pet-I_BLE_Testing");
  BLE.setAdvertisedService(rgbledService);

  // 특성을 서비스에 추가
  rgbledService.addCharacteristic(switchCharacteristic);
  rgbledService.addCharacteristic(textCharacteristic);

  // 서비스 추가
  BLE.addService(rgbledService);

  // 특성의 초기 값 설정:
  switchCharacteristic.writeValue(0);

  // 광고 시작
  BLE.advertise();

  Serial.println("BLE LED 서비스 시작");
}

void loop() {
  // BLE 페리페럴 연결 대기:
  BLEDevice central = BLE.central();

  // 중앙이 페리페럴에 연결되었을 경우:
  if (central) {
    Serial.print("Connected to central: ");
    // 중앙의 MAC 주소 출력:
    Serial.println(central.address());

    // 연결이 되었음을 알리는 텍스트 전송
    textCharacteristic.writeValue("Pet-I is Running Now");

    // 중앙이 페리페럴에 연결된 동안:
    while (central.connected()) {
      // 원격 장치가 특성에 쓴 경우,
      // 그 값을 사용하여 LED 제어:
      if (switchCharacteristic.written()) {
        switch (switchCharacteristic.value()) {   // 0이 아닌 모든 값
          case 1:
            Serial.println("Red LED On");
            digitalWrite(red, HIGH);            // LED를 켬
            digitalWrite(green, LOW);
            digitalWrite(blue, LOW);
            break;
          case 2:
            Serial.println("Green LED On");
            digitalWrite(red, LOW);
            digitalWrite(green, HIGH);          // LED를 켬
            digitalWrite(blue, LOW);
            break;
          case 3:
            Serial.println("Blue LED On");
            digitalWrite(red, LOW);
            digitalWrite(green, LOW);
            digitalWrite(blue, HIGH);           // LED를 켬
            break;
          default:
            Serial.println("LED Off");
            digitalWrite(red, LOW);
            digitalWrite(green, LOW);
            digitalWrite(blue, LOW);
            break;
        }
      }
    }

    // 중앙이 연결을 끊었을 때 출력:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    digitalWrite(red, LOW);         // LED를 끔
    digitalWrite(green, LOW);       // LED를 끔
    digitalWrite(blue, LOW);        // LED를 끔
  }
}
