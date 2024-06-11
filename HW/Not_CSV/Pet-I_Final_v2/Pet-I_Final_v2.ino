/*
This program developed for KMU_Capstone_Design_Competition.
Developed by Dustin Choi.
Used Sensor: Arduino Pro Mini, MLX-90614, HC-06, SZH-, MPU 9250, Li-ion_Battery, Charging Module, RGB Led, SD Card Module
*/

// 최종 ToDo: SD 카드, (RGB 코드 추가), 자이로로 state 파악

// ToDo: SD 카드 복구 기능 추가
// ToDo: 리튬폴리머 배터리의 전압 및 잔량을 측정하면서 동시에 작동 시키는 방법은? -> A1 핀을 VCC 라인에 연결
// ToDo: 심박은 delay(60000)으로, 온도와 배터리는 delay(600000)으로 -> 현재 이와 같이 동작
// ToDo: 자이로로 state 파악
// ToDo: 앱에서 id값 받으면 블루투스 기기명 변경 -> AT 모드 사용해야함
// ToDo: 앱에서 RGB int 값 받으면 RGB Led에 값 적용
// ToDo: Led 페어링 상태 표시
// ToDo: 강아지 나이별로 주기 설정 후 적용 -> 추후 집중학기제 때 도입 예정
// ToDo: 케이스에 회로도 적용 -> 제작 완료

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
MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

float heading;
float tiltheading;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];
#define sample_num_mdate  5000
volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;

// BT----------------------------------------------------------------------------
#define BTtx       8 // 블루투스 모듈의 tx를 D8으로 설정
#define BTrx       9 // 블루투스 모듈의 rx를 D9으로 설정

SoftwareSerial BT(BTtx, BTrx); // tx, rx
char data = 0; // 앱을 통해 0 또는 1이라는 문자열을 받을건데 0이 꺼지는 default 값이므로 0으로 설정

// 주기적 데이터 전송을 위한 타이머 변수
unsigned long previousMillisHeartRate = 0;
unsigned long previousMillisTempBattery = 0;
const long intervalHeartRate = 60000; // 1분
const long intervalTempBattery = 600000; // 10분

// MLX-90614---------------------------------------------------------------------
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// SZH-----------------------------------------------------------------------------
#define USE_ARDUINO_INTERRUPTS true
PulseSensorPlayground pulseSensor;

const int PulseWire = 0;
const int LED13 = 13;
int Threshold = 550;

// Voltage and Battery--------------------------------------------------------------
// 아날로그 핀 정의
const int batteryPin = A1;

// 전압 레퍼런스 값 (아두이노는 5V를 기준으로 함)
const float referenceVoltage = 5.0;

// 아날로그 읽기 최대 값 (아두이노의 10비트 ADC는 0-1023 값을 가짐)
const int maxADCValue = 1023;

// 배터리 전압 최대 값 (5V 배터리 사용 시)
const float maxBatteryVoltage = 5.0;

// SD Card---------------------------------------------------------------------------
const int chipSelect = 10; // Arduino Pro Mini의 CS는 D10
SdFat sd;
SdFile myFile;

// RGB Led---------------------------------------------------------------------------
int red = 9;
int green = 7;
int blue = 8;

// 3. void setup()
void setup()
{
    Serial.begin(9600);  // 시리얼 모니터 보드레이트 설정
    MPU_Init();
    BT_Init();
    MLX_Init();
    SZH_Init();
    //Battery_Init(); // setup() 부분 존재 x
    SdCard_Init();
    RGB_Init();
}

// 4. setup() 모음
void MPU_Init(){
    Wire.begin();
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
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

void RGB_Init(){
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
}

// 5. function 선언
void MPU_Loop();
void BT_Loop();
void MLX_Loop();
void SZH_Loop();
void Battery_Loop();
void SdCard_Loop();
void RGB_Loop();
void blinkRGB_Loop();
void sendDataOverBluetooth();

// 6. function-----------------------------------------------------------------------------------------
void MPU_Loop(){
    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated(); 
    getHeading();               
    getTiltHeading();

    // Serial.println("calibration parameter: ");
    // Serial.print(mx_centre);
    // Serial.print("         ");
    // Serial.print(my_centre);
    // Serial.print("         ");
    // Serial.println(mz_centre);
    // Serial.println("     ");


    Serial.println("Acceleration(g) of X,Y,Z:");
    Serial.print(Axyz[0]);
    Serial.print(",");
    Serial.print(Axyz[1]);
    Serial.print(",");
    Serial.println(Axyz[2]);
    Serial.println("Gyro(degress/s) of X,Y,Z:");
    Serial.print(Gxyz[0]);
    Serial.print(",");
    Serial.print(Gxyz[1]);
    Serial.print(",");
    Serial.println(Gxyz[2]);
    //delay(60000);
    // 강아지 상태 판단
    String dogState;
//    if (accelMagnitude < 0.5 && gyroMagnitude < 0.5) {
//      dogState = "가만히 있음";
//    } else if (accelMagnitude < 1.5 && gyroMagnitude < 1.5) {
//      dogState = "걷는 중";
//    } else if (accelMagnitude < 3.0 && gyroMagnitude < 3.0) {
//      dogState = "뛰는 중";
//    } else {
//      dogState = "몸을 터는 중";
//    }
  
    // 상태 출력
    Serial.println("강아지 상태: " + dogState);
    delay(60000);
}

void BT_Loop(){
    unsigned long currentMillis = millis();

    if (BT.available()) { // 블루투스가 연결되면
        data = BT.read(); // 데이터를 수신 받아서 읽음
        if (data == '1') { // 블루투스 연결 상태 확인
            // 연결된 경우: Pet-I 작동 On
            BT.println("Pet-I Turn On");
            RGB_Loop();
            
            // 1분마다 심박 전송
            if (currentMillis - previousMillisHeartRate >= intervalHeartRate) {
                previousMillisHeartRate = currentMillis;
                SZH_Loop();
            }

            // 10분마다 온도 및 배터리 전송
            if (currentMillis - previousMillisTempBattery >= intervalTempBattery) {
                previousMillisTempBattery = currentMillis;
                MLX_Loop();
                Battery_Loop();
            }
        } else if (data == '0') {
            // 연결되지 않은 경우: Pet-I 작동 OFF
            BT.println("Pet-I Turn Off");
            blinkRGB_Loop();
            delay(1000);
        }
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

    Serial.print("Object AVG = "); 
    Serial.println(objectAvg); // 실질적인 객체의 온도
    sendDataOverBluetooth("Temperature", objectAvg);
}

void SZH_Loop(){
    int myBPM = pulseSensor.getBeatsPerMinute();

    if (pulseSensor.sawStartOfBeat()) {
        Serial.print("BPM: ");
        Serial.println(myBPM);
        sendDataOverBluetooth("HeartRate", myBPM);
    }
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

    sendDataOverBluetooth("Battery", batteryPercentage);
}

void SdCard_Loop(){
    //test
}

// RGB Led 루프
void RGB_Loop(){
   digitalWrite(green, HIGH);
}

void blinkRGB_Loop(){
  digitalWrite(green, HIGH);
  // 0.5초 동안 대기합니다.
  delay(500);
  // 3색 LED의 초록색 LED가 꺼지도록 합니다.
  digitalWrite(green, LOW);
  // 0.5초 동안 대기합니다.
  delay(500);
}

// 블루투스 처리 위한 데이터 타입 정리 함수
void sendDataOverBluetooth(String dataType, float dataValue) {
    BT.print(dataType);
    BT.print(": ");
    BT.println(dataValue);
}

// MPU 처리 위한 함수 모음
void getHeading(void)
{
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0)    tiltheading += 360;
}

void Mxyz_init_calibrated ()
{

    Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
    Serial.print("  ");
    Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
    Serial.print("  ");
    Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
    while (!Serial.find("ready"));
    Serial.println("  ");
    Serial.println("ready");
    Serial.println("Sample starting......");
    Serial.println("waiting ......");

    get_calibration_Data ();

    Serial.println("     ");
    Serial.println("compass calibration parameter ");
    Serial.print(mx_centre);
    Serial.print("     ");
    Serial.print(my_centre);
    Serial.print("     ");
    Serial.println(mz_centre);
    Serial.println("    ");
}


void get_calibration_Data ()
{
    for (int i = 0; i < sample_num_mdate; i++)
    {
        get_one_sample_date_mxyz();

        if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
        if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

    }

    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];

    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;
}

void get_one_sample_date_mxyz()
{
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}

void getAccel_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}


// 7. void loop()----------------------------------------------------------------------------
void loop()
{
    BT_Loop();
}
