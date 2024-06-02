#include <ArduinoBLE.h>

BLEService ledService("180C");  // 예제 서비스 UUID
BLECharacteristic sendCharacteristic("2A6E", BLERead | BLENotify, 1);  // 예제 특성 UUID

void setup() {
  // initialize digital pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  // initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("LED_Control");
  BLE.setAdvertisedService(ledService);

  // add characteristic to the service
  ledService.addCharacteristic(sendCharacteristic);

  // add service
  BLE.addService(ledService);

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");
}

void loop() {
  // listen for BLE central device to connect:
  BLEDevice central = BLE.central();
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      delay(900);                      // wait for a second
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
      delay(100);                      // wait for a second
  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    while (central.connected()) {
      // LED blink code
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      delay(1000);                      // wait for a second
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
      delay(1000);                      // wait for a second

      // send 'H' over BLE
      Serial.println("Sending 'H' over BLE");
      sendCharacteristic.writeValue("H");
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
