/*
This program developed for KMU_Capstone_Design_Competition.
Developed by Dustin Choi.
Used Sensor: Arduino Nano 33 IoT, MLX-90614, MAX30102 or SZH-hws008, Li-ion_Battery, Charging Module, RGB Led, SD Card Module
*/

// 최종 ToDo: SD 카드 백업, 자이로로 state 파악

// ToDo: SD 카드 복구 기능 추가
// ToDo: 리튬폴리머 배터리의 전압 및 잔량을 측정하면서 동시에 작동 시키는 방법은? -> A1 핀을 VCC 라인에 연결
// ToDo: 심박은 delay(60000)으로, 온도와 배터리는 delay(600000)으로 -> 현재 이와 같이 동작
// ToDo: 앱에서 RGB int 값 받으면 RGB Led에 값 적용
// ToDo: 강아지 나이별로 주기 설정 후 적용 -> 추후 집중학기제 때 도입 예정

// 0. Libraries==================================================================================================================================
#include "Wire.h"
//#include "I2Cdev.h" <- 왜인지 컴파일 에러 냄
// MPU-------------------------------------------------------------------
// MLX-------------------------------------------------------------------
#include <Adafruit_MLX90614.h>
// SZH-------------------------------------------------------------------
//#include <PulseSensorPlayground.h> <- 왜인지 컴파일 에러 냄
// Battery---------------------------------------------------------------
// RGB_Led------------------------------------------------------------
// SD--------------------------------------------------------------------
#include <SdFat.h>
// BT--------------------------------------------------------------------
#include <ArduinoBLE.h>

// 1. 변수 선언==================================================================================================================================
// MPU--------------------------------------------------------------------
// 상태를 저장할 변수
enum DogState { WALKING, RUNNING, SHAKING, STILL }; // 총 4개의 상태로 분류
DogState currentState; // 강아지 현재 상태
// MLX--------------------------------------------------------------------
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

float objectTotal; // 물체 측정 온도 총합
float objectAvg; // 물체 측정 온도 평균
// SZH---------------------------------------------------------------------
// Battery-----------------------------------------------------------------
// 아날로그 핀 정의
const int batteryPin = A1;
// 전압 레퍼런스 값 (아두이노는 5V를 기준으로 함)
const float referenceVoltage = 5.0;
// 아날로그 읽기 최대 값 (아두이노의 10비트 ADC는 0-1023 값을 가짐)
const int maxADCValue = 1023;
// 배터리 전압 최대 값 (예: 12V 배터리 사용 시)
const float maxBatteryVoltage = 4.7;

int analogValue;
float batteryVoltage;
float batteryPercentage;
// RGB_Led--------------------------------------------------------------
// 3색 LED의 각 핀을 9, 10, 11번으로 설정합니다.(빨강 = 9, 초록 = 10, 파랑 = 11)
int red = 9;
int green = 7;
int blue = 8;
// SD----------------------------------------------------------------------
// BT-----------------------------------------------------------------------
bool isConnected = false;
long previousMillis = 0;  // last time the battery level was checked, in ms

// 2. setup() 함수 모음==============================================================================================================================
// MPU--------------------------------------------------------------------
void MPU_Init(){
}
// MLX--------------------------------------------------------------------
void MLX_Init(){
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };
}
// SZH---------------------------------------------------------------------
void SZH_Init(){
}
// Battery-----------------------------------------------------------------
// Battery는 Init이 필요 없음
// RGB_Led--------------------------------------------------------------
void RGB_Init(){
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
}
// SD----------------------------------------------------------------------
//void SD_Init(){
//  dd
//}
// BT-----------------------------------------------------------------------
void BT_Init(){
  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }
  BLE.setLocalName("Pet-I_Proto_B_2");
  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

// 3. setup() 부분==================================================================================================================================
void setup(){
  Serial.begin(9600);    // initialize serial communication
//  MPU_Init();
  MLX_Init();
//  SZH_Init();
  RGB_Init();
//  SD_Init();
  BT_Init();
}

// 4. loop() 함수 전방 선언==============================================================================================================================
// MPU--------------------------------------------------------------------
void MPU_Loop();
// MLX--------------------------------------------------------------------
void MLX_Loop();
// SZH---------------------------------------------------------------------
void SZH_Loop();
// Battery-----------------------------------------------------------------
void Battery_Loop();
// RGB_Led--------------------------------------------------------------
void Blink_RGB_Loop();
void Static_RGB_Loop();
// SD----------------------------------------------------------------------
//void SD_Loop();
// BT-----------------------------------------------------------------------
void BT_Loop();

// 5. loop() 함수 모음==================================================================================================================================
// MPU--------------------------------------------------------------------
void MPU_Loop(){
}
// MLX--------------------------------------------------------------------
void MLX_Loop(){
  objectTotal = 0;
  for (int i = 0; i < 10; i++) {
    objectTotal += mlx.readObjectTempC();
    delay(100); // 임의의 딜레이 추가
  }
  objectAvg = objectTotal / 10.0;
  //Serial.print("1. MLXObject (Avg): ");
  Serial.print("[1] Temp: ");
  Serial.print(objectAvg);
  //Serial.println("*C");
  Serial.print(" *C, ");
  //delay(500);
}
// SZH---------------------------------------------------------------------
void SZH_Loop(){
}
// Battery-----------------------------------------------------------------
void Battery_Loop(){
  // 배터리 핀에서 아날로그 값 읽기
  analogValue = analogRead(batteryPin);

  // 읽은 값을 실제 전압으로 변환
  batteryVoltage = (analogValue * referenceVoltage) / maxADCValue;

  // 배터리 전압 비율 계산 (0-1)
  batteryPercentage = (batteryVoltage / maxBatteryVoltage) * 100;

  // 배터리 전압과 잔량을 시리얼 모니터에 출력
  Serial.print("[3]. Battery Voltage: ");
  Serial.print(batteryVoltage);
  Serial.print(" V, ");

  Serial.print("[4]. Battery Percentage: ");
  Serial.print(batteryPercentage);
  Serial.print(" %, ");
}
// RGB_Led--------------------------------------------------------------
void Blink_RGB_Loop(){
  digitalWrite(blue, HIGH);
  digitalWrite(green, HIGH);
  // 0.5초 동안 대기합니다.
  delay(500);
  digitalWrite(blue, LOW);
  digitalWrite(green, LOW);
  // 0.5초 동안 대기합니다.
  delay(500);
}

void Static_RGB_Loop(){
  digitalWrite(blue, HIGH);
  digitalWrite(green, HIGH);
}
// SD----------------------------------------------------------------------
//void SD_Loop(){
//  dd
//}
// BT-----------------------------------------------------------------------
void BT_Loop(){
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Turn on  ");
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    //digitalWrite(LED_BUILTIN, HIGH);

    // while the central is connected:
    // 동작 수행
    while (central.connected()) {
      Static_RGB_Loop();
      MLX_Loop();
      SZH_Loop();
      Battery_Loop();
      //SD_Loop();
      MPU_Loop();
    }
    // when the central disconnects, turn off the LED:
    Serial.print("No Running... ");
    Blink_RGB_Loop();
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

// 6. loop() 부분==================================================================================================================================
void loop(){
   BT_Loop();
   Serial.println();
   delay(500);
}

// 7. Extra()======================================================================================================================================
