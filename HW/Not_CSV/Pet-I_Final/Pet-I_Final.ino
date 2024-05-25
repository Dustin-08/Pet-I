/*
This program developed for KMU_CapstoneDesign_Competiion.
Developed by Dustin Choi.
Use Sensor: Arduino Pro Micro, MLX-90614, HC-06, SZH-, MPU 9250, Li-ion_Battery, Charging Module, RGB Led, SD Card Module
*/

// 0. Libraries
// #pragma region include 사용 용도?
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include <SoftwareSerial.h>
#include <Adafruit_MLX90614.h>
#include <PulseSensorPlayground.h>
#include <SdFat.h>


#include "csv.h"
#include <math.h>

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
#define BTrx       9 // 블루투스 모듈의 rx를 D9로 설정

SoftwareSerial BT(BTtx, BTrx); // tx, rx
char data = 0; // 앱을 통해 0 또는 1이라는 문자열을 받을건데 0이 꺼지는 default 값이므로 0으로 설정

// MLX-90614---------------------------------------------------------------------
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// SZH-----------------------------------------------------------------------------
#define USE_ARDUINO_INTERRUPTS true

const int PulseWire = 0;
const int LED13 = 13;
int Threshold = 550;

// 3. void setup()
void setup()
{
    Serial.begin(9600);  // 시리얼 모니터 보드레이트 설정
    MPU_Init();
    BT_Init();
    MLX_Init();
    SZH_Init();
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

// 5. function 선언
MPU_Loop();
BT_Loop();
MLX_Lopp();
SZH_Loop();

// 6. function-----------------------------------------------------------------------------------------
void MPU_Loop(){
    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated(); 
    getHeading();               
    getTiltHeading();

    Serial.println("calibration parameter: ");
    Serial.print(mx_centre);
    Serial.print("         ");
    Serial.print(my_centre);
    Serial.print("         ");
    Serial.println(mz_centre);
    Serial.println("     ");


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
    Serial.println("Compass Value of X,Y,Z:");
    Serial.print(Mxyz[0]);
    Serial.print(",");
    Serial.print(Mxyz[1]);
    Serial.print(",");
    Serial.println(Mxyz[2]);
    Serial.println("The clockwise angle between the magnetic north and X-Axis:");
    Serial.print(heading);
    Serial.println(" ");
    Serial.println("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
    Serial.println(tiltheading);
    Serial.println("   ");
    Serial.println();
    delay(1000);
}

void BT_Loop(){
    if (BT.available()) { // 블루투스가 연결되면
        data = BT.read(); // 데이터를 수신 받아서 읽음
        if (data == '1') { // 블루투스 연결 상태 확인
            // 연결된 경우: Pet-I 작동 On
            // BT.print();
            // delay(1000);
        } else if (data == '0') {
            // 연결되지 않은 경우: Pet-I 작동 OFF
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

    Serial.print("Ambient (Avg) = "); Serial.print(ambientAvg);
    Serial.print("*C\tObject (Avg) = "); Serial.print(objectAvg); Serial.println("*C");
    Serial.print("Ambient (Avg) = "); Serial.print(mlx.readAmbientTempF());
    Serial.print("*F\tObject (Avg) = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");

    Serial.println();
    delay(500);
}

void SZH_Loop(){
    int myBPM = pulseSensor.getBeatsPerMinute();

    if (pulseSensor.sawStartOfBeat()) {
        Serial.println("♥  A HeartBeat Happened ! ");
        Serial.print("BPM: ");
        Serial.println(myBPM);
    }

    delay(20);
}

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
    MPU_Loop();
    BT_Loop();
    MLX_Loop();
    SZH_Loop();
}