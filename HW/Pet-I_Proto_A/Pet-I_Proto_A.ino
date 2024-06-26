/*
This program developed for KMU_Capstone_Design_Competition.
Developed by Dustin Choi.
Used Sensor: Arduino Pro Mini, MLX-90614, MPU9250, MAX30102 or SZH-hws008, Li-ion_Battery, Charging Module, RGB Led, SD Card Module
*/

// 최종 ToDo: SD 카드 백업, 자이로로 state 파악, 블루투스 값 나눠서 앱인벤터에 띄우기 

// 0. Libraries
#include "Wire.h"
#include "I2Cdev.h"
// MLX--------------------------------------------------------------------
#include <Adafruit_MLX90614.h>
// SZH---------------------------------------------------------------------
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>
// SD-----------------------------------------------------------------------
#include <SdFat.h>
// BT-----------------------------------------------------------------------
#include <SoftwareSerial.h> // 블루투스를 활용하기 위한 라이브러리 선언

// 1. 변수 선언
// MPU--------------------------------------------------------------------
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

// 2. setup() 함수 모음
// MPU--------------------------------------------------------------------
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
// RGB_Led--------------------------------------------------------------
void RGB_Init(){
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
}
// SD----------------------------------------------------------------------
// BT-----------------------------------------------------------------------
void BT_Init(){
  BT.begin(9600); // BT를 보드레이트 9600으로 설정
}

// 3. setup() 부분
void setup(){
  Serial.begin(9600);    // initialize serial communication
  MLX_Init();
  SZH_Init();
  RGB_Init();
  BT_Init();
}

// 4. loop() 함수 전방 선언
// MPU--------------------------------------------------------------------
// MLX--------------------------------------------------------------------
void MLX_Loop();
// SZH---------------------------------------------------------------------
void SZH_Loop();
// Battery-----------------------------------------------------------------
// RGB_Led--------------------------------------------------------------
void Blink_RGB_Loop();
void Static_RGB_Loop();
// SD----------------------------------------------------------------------
// BT-----------------------------------------------------------------------
void BT_Init();

// 5. loop() 함수 모음
// MPU--------------------------------------------------------------------
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
  BT.print(objectAvg);
  //delay(500);
}
// SZH---------------------------------------------------------------------
void SZH_Loop(){
  myBPM = pulseSensor.getBeatsPerMinute();

  if (pulseSensor.sawStartOfBeat()) {
    Serial.print("[2] BPM: ");
    //Serial.println(myBPM);
    Serial.print(myBPM);
    Serial.print(", ");
  }

  //delay(500);
}
// Battery-----------------------------------------------------------------
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
  }else if (!isConnected) {
    // 연결되지 않았을 때 빨강색과 초록색 LED가 0.5초 간격으로 깜빡임
    Serial.println("No Running...");
    Blink_RGB_Loop();
  }
  // 문자열 초기화
  data = 0; // data라는 문자열을 초기화
}

// 6. loop() 부분
void loop(){
   //MLX_Loop();
   //SZH_Loop();
   //Blink_RGB_Loop();
   //Static_RGB_Loop();
   BT_Loop();
   Serial.println();
   delay(500);
}

// 7. Extra()
