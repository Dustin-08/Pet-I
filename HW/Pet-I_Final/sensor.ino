
//MPU
Kalman kalmanX;
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

//GPS
HardwareSerial mySerial = Serial1;

String _gps_time = "00:00:00";
long _gps_latitude = 0;
long _gps_latitude_dec = 0;
long _gps_longitude = 0;
long _gps_lontitude_dec = 0;

float _gps_altitude = 0.0;
byte _gps_satellites = 0;
String _gps_save_data = "";

// BMP
float altitude_offset = 0.0;

// MPU
// float tiltX_offset = 0.0;
// float tiltY_offset = 0.0;

//-----------------------init-----------------------
void XBeeInit(){
  
}

void set_zero_altitude(float value){
  altitude_offset = value;
}

void BMPInit(){
  Serial.println("");
  Serial.println("BMP setting...");
  uint32_t start_time = millis();
  if (!bmp.begin()) { //BMP 모듈 인식이 안되었다면
    Serial.println("  error : BMP 모듈을 찾을 수 없습니다.");
  }else{
    set_zero_altitude(bmp.readAltitude());
  }
  
  Serial.print("BMP setting done (");
  Serial.print(millis() - start_time);
  Serial.println("ms)");

}

void SDInit(){  
  Serial.println("SD Card setting...");  
  uint32_t start_time = millis();
  ////////////////sdcard initializing////////////////
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    Serial.println("  error : Card failed, or not present");
    // // don't do anything more:
    // while (1);
    return;
  }
  Recover();
  
  Serial.print("SD Card setting done (");
  Serial.print(millis() - start_time);
  Serial.println("ms)");
  
}

void MPUInit() {
  Serial.println("MPU setting...");
  uint32_t start_time = millis();
  Wire.begin();
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print("  error : sensor read failed");
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  offsetX = roll; // 23.03.07
  offsetY = pitch; // 23.03.07
  timer = micros();


  Serial.print("MPU setting done (");
  Serial.print(millis() - start_time);
  Serial.println("ms)");

}

uint32_t gps_timer = millis();
void GPSInit(){
  Serial.println("GPS setting...");
  uint32_t start_time = millis();

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  getGPSData();
  launch_time = timeToMillis(csv.gps_time);
  Serial.print("launch_time : ");
  Serial.println(launch_time);

  Serial.print("GPS setting done (");
  Serial.print(millis() - start_time);
  Serial.println("ms)");
}

void getGPSData(){
  char c = GPS.read();
  if (false)
    if (c) Serial.print(c);
  csv.gps_time = timeFormat(GPS.hour, GPS.minute, GPS.seconds);
  
  if (GPS.newNMEAreceived()){
    gps_timer = millis();
    if (!GPS.parse(GPS.lastNMEA()))
      return;
    
    #if DEBUG | GPS_DEBUG
    Serial.print("\nlastNMEA(");
    Serial.print(millis() - gps_timer);
    Serial.print(")s : ");
    Serial.print(GPS.lastNMEA());
    Serial.println(GPS.altitude);
    Serial.println(GPS.fix);
    #endif
    

    csv.gps_altitude = GPS.altitude;
    csv.gps_latitude = (int)(GPS.latitude / 100) + fmod(GPS.latitude, 100) / 60.0;
    csv.gps_longitude = (int)(GPS.longitude / 100) + fmod(GPS.longitude, 100) / 60.0;
    csv.gps_sats = GPS.satellites;
  }
}
//-----------------------loop-----------------------
void BMPLoop(){
  csv.temperature = bmp.readTemperature();
  csv.pressure = bmp.readPressure() / 1000;
  csv.altitude = bmp.readAltitude() - altitude_offset;
  
  // temperature = bmp.readTemperature(); //BMP 온도값
  // pressure = bmp.readPressure()/1000;  //raw 데이터가 (pa) 이기때문에 /1000.
  // altitude = bmp.readAltitude(/*기준고도기압값*/);
}

void MPULoop() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */

  csv.tilt_X = kalAngleX - offsetX - tiltX_offset;
  csv.tilt_Y = kalAngleY - offsetY - tiltY_offset;

  #if 0 // Set to 1 to print the temperature
    Serial.print("\t");

    double temperature = (double)tempRaw / 340.0 + 36.53;
    Serial.print(temperature); Serial.print("\t");
  #endif
}


void SDLoop(){
  bool too_long = false;
  if (myFile.exists("recovery.csv")){
    if (myFile.open("recovery.csv", FILE_READ)){
      size_t n;
      int ln = 1;
      while ((n = myFile.fgets(recv_line, sizeof(recv_line))) > 0) {
        if(ln >= 5){
          too_long = true;
          break;
        }
        #if DEBUG | SD_DEBUG
        // Print line number.
        Serial.print(ln++);
        Serial.print(": ");
        Serial.print(recv_line);
        #endif
        strncpy(recv_last, recv_line, RECV_LEN);
      }
      myFile.close();
    }else{
      Serial.println("  error : recovery.csv open(READ) error");
    }
  }
  
  if (too_long) {
    myFile.remove("recovery.csv");
  }

  if(myFile.open("recovery.csv", O_WRITE | O_CREAT)){
    String rcv_data = millisToTime(csv.mission_time) + ", "
    // + String(csv.mission_time) + ", "
    + String(csv.packet_count) + ", "
    + String(altitude_offset) + ", "
    + String(csv.state) + ", "
    + String(csv.sim_mode) + ", "
    + String(tiltX_offset) + ", "
    + String(tiltY_offset) + ", "
    + String(csv.b1) + ", "
    + String(csv.b2) + ", "
    + String(csv.b3) + "\n";

    if (too_long) {
      myFile.println("MISSION_TIME, PACKET_COUNT, REFERENCE_ALTITUDE, STATE, SIM_MODE, TX_OFFSET, TY_OFFSET, BW1, BW2, BW3");      
    }
    
    myFile.print(rcv_data);
    myFile.close();
  }else{
    Serial.println("  error: recovery open");
  }
}

void GPSLoop(){
  getGPSData();
}
