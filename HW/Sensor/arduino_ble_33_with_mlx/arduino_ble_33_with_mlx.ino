// 16진수로 전송될 경우 온도 단위를 뺼 것

#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
BLEService sensorService("180C");  // 예제 서비스 UUID
BLEStringCharacteristic temperatureCharacteristic("2A6E", BLERead | BLENotify, 32);  // 온도
BLEStringCharacteristic accelCharacteristic("AC10", BLERead | BLENotify, 32);  // 가속도
BLEStringCharacteristic magCharacteristic("AC11", BLERead | BLENotify, 32);  // 자계
void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!mlx.begin()) {
    Serial.println("Failed to initialize MLX90614 sensor!");
    while (1);
  }
  if (!IMU.begin()) {
    Serial.println("Failed to initialize LSM9DS1 sensor!");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Nano33BLE");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(temperatureCharacteristic);
  sensorService.addCharacteristic(accelCharacteristic);
  sensorService.addCharacteristic(magCharacteristic);
  BLE.addService(sensorService);
  BLE.advertise();

  Serial.println("BLE and sensors initialized successfully!");
}
void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.println("Connected to central");
    while (central.connected()) {
      float temperature = mlx.readObjectTempC();
      float ax, ay, az;
      float mx, my, mz;

      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
      }
      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(mx, my, mz);
      }

      // Data formatting
      char tempBuffer[32];
      char accelBuffer[32];
      char magBuffer[32];

      snprintf(tempBuffer, sizeof(tempBuffer), "%.2f", temperature);
      snprintf(accelBuffer, sizeof(accelBuffer), "Accel: %.2f, %.2f, %.2f", ax, ay, az);
      snprintf(magBuffer, sizeof(magBuffer), "Mag: %.2f, %.2f, %.2f", mx, my, mz);

      // Data sending
      temperatureCharacteristic.writeValue(tempBuffer);
      accelCharacteristic.writeValue(accelBuffer);
      magCharacteristic.writeValue(magBuffer);

      // Data to Serial Monitor
      Serial.println(tempBuffer);
      Serial.println(accelBuffer);
      Serial.println(magBuffer);

      delay(500); // Update interval
    }
    Serial.println("Disconnected from central");
  }
}
