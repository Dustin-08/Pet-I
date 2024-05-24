/**
*시간, 분, 초를 받아 00:00:00 포맷으로 전환합니다.
*/
String timeFormat(int h, int m, int s){
  String time_format;
  if (h < 10) time_format += '0';
  time_format += String(h, DEC);
  time_format += String(':');
  
  if (m < 10) time_format += '0';
  time_format += String(m, DEC);
  time_format += String(':');
  
  if (s < 10) time_format += '0';
  time_format += String(s, DEC);

  return time_format;
}

String millisToTime(uint32_t millis){ //milli seconds convert to hh:mm:ss
  uint32_t seconds = millis / 1000;
  int h = seconds / 3600;
  int m = (seconds % 3600) / 60;
  int s = seconds % 60;
  return timeFormat(h, m, s);
}

uint32_t timeToMillis(String time){

  String time_buf[3];
  str_split(time,time_buf, 3, ':');
  uint32_t millis = 0;
  millis = 1000 * (time_buf[0].toInt() * 3600 + time_buf[1].toInt() * 60 + time_buf[2].toInt());
  return millis;
}

void str_split(String resource, String* trimmedStrPtr, byte trimmed_len, char delimiter){
  byte colIdx = 0;
  int idx_before = 0;

  resource.trim();
  resource.replace(" ", "");
  
  int idxToTrim = resource.indexOf(delimiter, 0);
  if (idxToTrim == -1) { return; }

  while(colIdx < trimmed_len){
    *trimmedStrPtr = resource.substring(idx_before, idxToTrim);
    trimmedStrPtr++;

    if (idxToTrim == -1) { break; }

    idx_before = idxToTrim + 1;
    idxToTrim = resource.indexOf(delimiter, idx_before);
    colIdx++;
  }
}

String state_to_str(State st){
  switch (st){
    case IDLE:
    return String("IDLE");
    case ASCENT:
    return String("ASCENT");
    case ROCKET_SEPERATION:
    return String("ROCKET_SEPERATION");
    case PC_RELEASE:
    return String("PC_RELEASE");
    case HS_RELEASE:
    return String("HS_RELEASE");
    case LANDED:
    return String("LANDED");
  }
}

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc//Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}