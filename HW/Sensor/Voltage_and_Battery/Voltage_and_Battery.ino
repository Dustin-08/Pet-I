// 아날로그 핀 정의
const int batteryPin = A1;

// 전압 레퍼런스 값 (아두이노는 5V를 기준으로 함)
const float referenceVoltage = 5.0;

// 아날로그 읽기 최대 값 (아두이노의 10비트 ADC는 0-1023 값을 가짐)
const int maxADCValue = 1023;

// 배터리 전압 최대 값 (예: 12V 배터리 사용 시)
const float maxBatteryVoltage = 5.0;

void setup() {
  // 시리얼 통신 시작
  Serial.begin(9600);
}

void loop() {
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

  // 1초 대기
  delay(1000);
}
