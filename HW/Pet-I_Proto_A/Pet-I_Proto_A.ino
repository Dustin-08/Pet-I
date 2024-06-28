/*
This program developed for KMU_Capstone_Design_Competition.
Developed by Dustin Choi.
Used Sensor: Arduino Pro Mini, MLX-90614, MPU9250, MAX30102 or SZH-hws008, Li-ion_Battery, Charging Module, RGB Led, SD Card Module
*/

// 최종 ToDo: SD 카드 백업, 자이로로 state 파악, 블루투스 값 나눠서 앱인벤터에 띄우기 

// 0. Libraries==================================================================================================================================
#include "Wire.h"
#include "I2Cdev.h"
// MPU-------------------------------------------------------------------
#include "MPU9250.h"
// MLX--------------------------------------------------------------------
#include <Adafruit_MLX90614.h>
// SZH---------------------------------------------------------------------
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>
// SD-----------------------------------------------------------------------
#include <SdFat.h>
// BT-----------------------------------------------------------------------
#include <SoftwareSerial.h> // 블루투스를 활용하기 위한 라이브러리 선언

// 1. 변수 선언==================================================================================================================================
// MPU--------------------------------------------------------------------
MPU9250 accelgyro; // MPU9250 객체 생성
I2Cdev I2C_M;

uint8_t buffer_m[6];
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];

// 상태를 저장할 변수
enum DogState { WALKING, RUNNING, SHAKING, STILL }; // 총 4개의 상태로 분류
DogState currentState; // 강아지 현재 상태
// MLX--------------------------------------------------------------------
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

float objectTotal; // 물체 측정 온도 총합
float objectAvg; // 물체 측정 온도 평균
// SZH---------------------------------------------------------------------
PulseSensorPlayground pulseSensor;

const int PulseWire = 0; // A0핀
int Threshold = 550;
int myBPM;
// Battery-----------------------------------------------------------------
// 아날로그 핀 정의
const int batteryPin = A1;
// 전압 레퍼런스 값 (아두이노는 5V를 기준으로 함)
const float referenceVoltage = 5.0;
// 아날로그 읽기 최대 값 (아두이노의 10비트 ADC는 0-1023 값을 가짐)
const int maxADCValue = 1023;
// 배터리 전압 최대 값 (예: 12V 배터리 사용 시)
const float maxBatteryVoltage = 4.2;

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
#define BTtx 3 // 블루투스 모듈의 tx를 D3으로 설정
#define BTrx 2 // 블루투스 모듈의 rx를 D2으로 설정

SoftwareSerial BT(BTtx, BTrx); // tx, rx

char data = 0; // 앱을 통해 0 또는 1이라는 문자열을 받을건데 0이 꺼지는 default 값이므로 0으로 설정

// 2. setup() 함수 모음==================================================================================================================================
// MPU--------------------------------------------------------------------
void MPU_Init(){
  Wire.begin();
  accelgyro.initialize();
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
  pulseSensor.analogInput(PulseWire);
  pulseSensor.setThreshold(Threshold);
  
  if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");
  }
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
  BT.begin(9600); // BT를 보드레이트 9600으로 설정
}

// 3. setup() 부분==================================================================================================================================
void setup(){
  Serial.begin(9600);    // initialize serial communication
  MPU_Init();
  MLX_Init();
  SZH_Init();
  RGB_Init();
  //SD_Init();
  BT_Init();
}

// 4. loop() 함수 전방 선언============================================================================================================================
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

// 5. loop() 함수 모음================================================================================================================================
// MPU--------------------------------------------------------------------
void MPU_Loop(){
  getAccel_Data();
    getGyro_Data();

    determineState();

    switch(currentState) {
        case WALKING:
            Serial.print("[4]. State: 걷기");
            BT.print("걷기"); // for Debuging
            break;
        case RUNNING:
            Serial.print("[4]. State: 뛰기");
            BT.print("뛰기");  // for Debuging
            break;
        case SHAKING:
            Serial.print("[4]. State: 털기");
            BT.print("털기");  // for Debuging
            break;
        case STILL:
            Serial.print("[4]. State: Idle");
            BT.print("Idle");  // for Debuging
            break;
    }
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
  //Serial.print("27.47");  // for Debuging
  //Serial.println("*C");
  Serial.print(" *C, ");
  //BT.print(objectAvg);  // for Debuging
  //delay(500);
}
// SZH---------------------------------------------------------------------
void SZH_Loop(){
  myBPM = pulseSensor.getBeatsPerMinute();

//  if (pulseSensor.sawStartOfBeat()) {
//    Serial.print("[2] BPM: ");
//    //Serial.println(myBPM);
//    Serial.print(myBPM);
//    Serial.print(", ");
//  }

  //delay(500);
  Serial.print("[2] BPM: ");
    Serial.println(myBPM);
    //Serial.print("96");  // for Debuging
    Serial.print(", ");
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
//  Serial.print("[3]. Battery Voltage: ");
//  Serial.print(batteryVoltage);
//  Serial.print(" V, ");

  Serial.print("[3]. Battery Percentage: ");
  //Serial.print("98.8");  // for Debuging
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
  // 블루투스 모듈 수신 확인용
  if(BT.available() > 0) { // 블루투스가 연결되면
    data = BT.read(); // 데이터를 수신 받아서 읽음
  }
  // 실제 문자열에 따라 작동하는 구문
  if(data == '0') { // data라는 문자열이 0이라면
    Serial.println("Turn OFF"); // 시리얼 모니터에 Turn OFF라는 문구 출력
    Blink_RGB_Loop();
    isConnected = false;
  }
  else if(data == '1') { // data라는 문자열이 1이라면
    isConnected = true;
    Serial.println("Turn on"); // 시리얼 모니터에 Turn OFF라는 문구 출력
  }
  if(isConnected){
    Static_RGB_Loop();
    MLX_Loop();
    SZH_Loop();
    Battery_Loop();
    //SD_Loop();
    MPU_Loop();
  }else if (!isConnected) {
    // 연결되지 않았을 때 빨강색과 초록색 LED가 0.5초 간격으로 깜빡임
    Serial.println("No Running...");
    Blink_RGB_Loop();
  }
  // 문자열 초기화
  data = 0; // data라는 문자열을 초기화
}

// 6. loop() 부분==================================================================================================================================
void loop(){
   //MLX_Loop();
   //SZH_Loop();
   //Blink_RGB_Loop();
   //Static_RGB_Loop();
   BT_Loop();
   Serial.println();
   delay(500);
}

// 7. Extra()==================================================================================================================================
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