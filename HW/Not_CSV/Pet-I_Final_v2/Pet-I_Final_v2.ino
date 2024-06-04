/*
This program developed for KMU_Capstone_Design_Competiion.
Developed by Dustin Choi.
Used Sensor: Arduino Pro Mini, MLX-90614, HC-06, SZH-, MPU 9250, Li-ion_Battery, Charging Module, RGB Led, SD Card Module
*/

// ToDo: SD 카드 복구 기능 추가, Li-ion Battery
// ToDo: 리튬폴리머 배터리의 전압 및 잔량을 측정하면서 동시에 작동 시키는 방법은?
// ToDo: 심박은 delay(60000)으로, 온도와 배터리는 delay(600000)으로
// ToDo: 자이로로 state 파악
// ToDo: 앱에서 id값 받으면 블루투스 기기명 변경
// ToDo: 앱에서 RGB int 값 받으면 RGB Led에 값 적용
// ToDo: Led 페어링 상태 표시
// ToDo: 강아지 나이별로 주기 설정 후 적용
// ToDo: 케이스에 회로도 적용

// 0. Libraries

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include <SoftwareSerial.h>
#include <Adafruit_MLX90614.h>
#include <PulseSensorPlayground.h>
#include <SdFat.h>

// 1. 변수 선언
// MPU--------------------------------------------------------------------------
MPU9250 mpu;

// BT----------------------------------------------------------------------------
#define BTtx       8 // 블루투스 모듈의 tx를 D8으로 설정
#define BTrx       9 // 블루투스 모듈의 rx를 D9로 설정

SoftwareSerial BT(BTtx, BTrx); // tx, rx
char data = 0; // 앱을 통해 0 또는 1이라는 문자열을 받을건데 0이 꺼지는 default 값이므로 0으로 설정

// MLX-90614---------------------------------------------------------------------
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// SZH-----------------------------------------------------------------------------
#define USE_ARDUINO_INTERRUPTS true
PulseSensorPlayground pulseSensor;

const int PulseWire = 0;
const int LED13 = 13;
int Threshold = 550;

// Voltage and Battery
// 아날로그 핀 정의
const int batteryPin = A1;

// 전압 레퍼런스 값 (아두이노는 5V를 기준으로 함)
const float referenceVoltage = 5.0;

// 아날로그 읽기 최대 값 (아두이노의 10비트 ADC는 0-1023 값을 가짐)
const int maxADCValue = 1023;

// 배터리 전압 최대 값 (5V 배터리 사용 시)
const float maxBatteryVoltage = 5.0;

// 3. void setup()
void setup()
{
    Serial.begin(9600);  // 시리얼 모니터 보드레이트 설정
    MPU_Init();
    BT_Init();
    MLX_Init();
    SZH_Init();
    //Battery_Init();
    SdCard_Init();
}

// 4. setup() 모음
void MPU_Init(){
    Wire.begin();
    mpu.initialize(); // Correct method to initialize MPU9250
//    if (!mpu.testConnection()) {
//        Serial.println("MPU 연결 실패!");
//        while (1);
//    }
    Serial.println("MPU 연결 성공!");
}


void BT_Init(){
    BT.begin(9600); // BT를 보드레이트 9600으로 설정
}

void MLX_Init(){
    if (!mlx.begin()) {
        Serial.println("Error connecting to MLX sensor. Check wiring.");
        while (1);
    };
    Serial.print("Emissivity = "); Serial.println(mlx.readEmissivity());
    Serial.println("================================================");
}

void SZH_Init(){
    pulseSensor.analogInput(PulseWire);
    pulseSensor.blinkOnPulse(LED13);
    pulseSensor.setThreshold(Threshold);

    if (pulseSensor.begin()) {
        Serial.println("We created a pulseSensor Object !");
    }
}

void SdCard_Init(){
    // test
}

// 5. function 선언
void MPU_Loop();
void BT_Loop();
void MLX_Lopp();
void SZH_Loop();
void Battery_Loop();
void SdCard_Loop();

// 6. function-----------------------------------------------------------------------------------------
void MPU_Loop(){
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
    delay(60000);
}

void BT_Loop(){
    if (BT.available()) { // 블루투스가 연결되면
        data = BT.read(); // 데이터를 수신 받아서 읽음
        if (data == '1') { // 블루투스 연결 상태 확인
            // 연결된 경우: Pet-I 작동 On4
            while(1){
                BT.println("Pet-I Turn On");
                delay(1000);
            }
            //BT.println("Pet-I Turn On");
            // delay(1000);
            MPU_Loop();
            MLX_Loop();
            SZH_Loop();
            Battery_Loop();
            SdCard_Loop();
        } else if (data == '0') {
            // 연결되지 않은 경우: Pet-I 작동 OFF
            BT.println("Pet-I Turn Off");
            delay(1000);
        }//else if (data == ''){
            // 특정값 수신시: 카메라 영상 보이기
        //}
    }
}

void MLX_Loop(){
    float ambientTotal = 0;
    float objectTotal = 0;

    for (int i = 0; i < 10; i++) {
        ambientTotal += mlx.readAmbientTempC();
        objectTotal += mlx.readObjectTempC();
        delay(100); // 임의의 딜레이 추가
        }

    float ambientAvg = ambientTotal / 10.0;
    float objectAvg = objectTotal / 10.0;

    //Serial.print("Ambient (Avg) = "); 
    //Serial.print(ambientAvg);
    Serial.print("Object AVG = "); 
    Serial.println(objectAvg); // 실질적인 객체의 온도
    //Serial.println("*C");
    // Serial.print("Ambient (Avg) = "); Serial.print(mlx.readAmbientTempF());
    // Serial.print("*F\tObject (Avg) = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");

    //Serial.println();
    delay(60000); // 1분
}

void SZH_Loop(){
    int myBPM = pulseSensor.getBeatsPerMinute();

    if (pulseSensor.sawStartOfBeat()) {
        //Serial.println("♥  A HeartBeat Happened ! ");
        Serial.print("BPM: ");
        Serial.println(myBPM);
    }

    delay(60000);
}

void Battery_Loop(){
    // 배터리 핀에서 아날로그 값 읽기
    int analogValue = analogRead(batteryPin);
  
    // 읽은 값을 실제 전압으로 변환
    float batteryVoltage = (analogValue * referenceVoltage) / maxADCValue;
  
    // 배터리 전압 비율 계산 (0-1)
    float batteryPercentage = (batteryVoltage / maxBatteryVoltage) * 100;
  
    // 배터리 전압과 잔량을 시리얼 모니터에 출력
    Serial.print("Battery Voltage: ");
    Serial.print(batteryVoltage);
    Serial.println(" V");
  
    Serial.print("Battery Percentage: ");
    Serial.print(batteryPercentage);
    Serial.println(" %");
  
    // 1분 대기
    delay(60000);
}

void SdCard_Loop(){
    //test
}

// 7. void loop()----------------------------------------------------------------------------
void loop()
{
    //MPU_Loop();
    BT_Loop();
    //MLX_Loop();
    //SZH_Loop();
    // Battery_Loop();
    // SdCard_Loop();
}
