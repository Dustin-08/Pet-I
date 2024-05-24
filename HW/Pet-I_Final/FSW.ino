// 발사 도중에 전원이 끊겨 초기화가 되면 recovery.csv 파일을 SD로 부터 읽어
// mission_time ( launch_time ), packet_count,
//
void Recover()
{
  Serial.println("  recovery start..");

  if (!sd.exists("recovery.csv"))
  {
    Serial.println("  file not exists. device could not recover");
    if (sd.exists("log.csv"))
    {
      sd.remove("log.csv");
      Serial.println("  log.csv deleted!");
    }
    return;
  }

  // recovery 파일은 launch 이후에만 저장되므로 파일이 존재함은 발사 이후로 간주 합니다.
  launched = true;       // 고도를 기준으로 (>5m) 판별됨
  ready = true;          // CMD,1064,INIT,READY
  send_telemetry = true; // CMD,1064,CX,ON

  // recovery.csv 파일을 읽어 마지막 줄 recv_last 문자열 변수에 저장합니다.
  if (!myFile.open("recovery.csv", FILE_READ))
  {
    Serial.println("  error : recovery file(READ) open err");
  }
  else
  {
    size_t n;
    int ln = 1;

    while ((n = myFile.fgets(recv_line, sizeof(recv_line))) > 0)
    {
// Print line number.
#if DEBUG | SD_DEBUG
      Serial.print("  recv->");
      Serial.print(ln++);
      Serial.print(": ");
      Serial.print(recv_line);
#endif
      strncpy(recv_last, recv_line, RECV_LEN);
    }

#if DEBUG | SD_DEBUG
    Serial.println("  read Done");
    Serial.print("  last line : ");
    Serial.println(recv_last);
#endif

    String text[10];
    str_split(recv_last, text, 10, ',');

    String r_mt = text[0]; // recovery mission_time
    String r_pc = text[1]; // recovery packet count
    String r_ra = text[2]; // recovery reference altitude
    String r_st = text[3]; // recovery state
    String r_sm = text[4]; // recovery sim_mode

    myFile.close();
    csv.mission_time = timeToMillis(r_mt);
    csv.packet_count = r_pc.toInt();
    ref_altitude = r_ra.toFloat();
    csv.state = r_st.toInt();
    csv.sim_mode = text[4].toInt();
    tiltX_offset = text[5].toFloat();
    tiltY_offset = text[6].toFloat();
    csv.b1 = text[7].toInt();
    csv.b2 = text[8].toInt();
    csv.b3 = text[9].toInt();
    Serial.println("  device recovered!");
  }
}

/**
 * GCS로부터 명령을 받고 csv 멤버 변수 혹은 전역 변수를 변경합니다.
 *
 */
void CommandReceiver()
{
  if (Serial2.available())
  {
    String commands[4];
    String CMD_buf = "";
    CMD_buf = Serial2.readStringUntil('\0');
    Serial.println(CMD_buf);
    // CMD_buf.toCharArray(csv.CMD_ECHO, CMD_buf.length()); // csv.CMD_ECHO에 CMD_buf를 String -> char* 로 변환하여 저장
    str_split(CMD_buf, commands, 4, ',');
    if(CMD_buf != ""){
      csv.CMD_ECHO = commands[0] + commands[1] + commands[2] + commands[3];
    }

    if (commands[0] == "CMD")
    {
      if (commands[2] == "CX")
      {
        if (commands[3] == "ON")
        { // CMD,1064,CX,ON
          send_telemetry = true;
          Serial.println("  CX_ON received. telemetry started.");
        }
        else if (commands[3] == "OFF")
        { // CMD,1064,CX,OFF
          send_telemetry = false;
          Serial.println("  CX_OFF received. telemetry ended.");
        }
      }
      else if (commands[2] == "ST")
      {
        if (commands[3] == "GPS")
        { // CMD,1064,ST,GPS
          launch_time = timeToMillis(csv.gps_time);
          csv.mission_time =
              Serial.print("  launch time set to GPS time (");
          Serial.print(csv.gps_time);
          Serial.println(")");
          // Serial.println("ST_GPS");
        }
        else
        { // CMD,1064,ST,<UTC_TIME>
          // launch_time = timeToMillis(commands[3]);
          csv.mission_time = timeToMillis(commands[3]);
          Serial.print("  launch time set to UTC time (");
          Serial.print(launch_time); // 수정필요
          Serial.println(")");
        }
      }
      else if (commands[2] == "SIM")
      {
        if (sim_enable)
        {
          if (commands[3] == "ENABLE")
          { // CMD,1064,SIM,ENABLE
            csv.sim_mode = true;
            Serial.println("  simulation mode enabled. simulation mode start");
          }
          else if (commands[3] == "ACTIVATE")
          { // CMD,1064,SIM,AICTIVATE and sim_enable received before.
            csv.sim_mode = true;
            Serial.println("  simulation mode activated. simulation mode start");
          }
          else if (commands[3] == "DISABLE"){
            if (csv.sim_mode == false)
            {
              Serial.println("  simulation mode already disabled.");
            }
            else
            {
              csv.sim_mode = false;
              sim_enable = false;
              Serial.println("  simulation mode disabled. simulation mode end");
            }
          }
        } 
        else 
        {
          if (commands[3] == "ENABLE")
          { // CMD,1064,SIM,ENABLE
            sim_enable = true;
            Serial.println("  simulation mode enabled");
          }
          else if (commands[3] == "ACTIVATE")
          { // CMD,1064,SIM,AICTIVATE and sim_enable received before.
            sim_enable = true;
            Serial.println("  simulation mode activated");
          }
          else if (commands[3] == "DISABLE")
          { 
            if (csv.sim_mode == false)
            {
              Serial.println("  simulation mode already disabled.");
            }
            else
            {
              csv.sim_mode = false;
              sim_enable = false;
              Serial.println("  simulation mode disabled. simulation mode end");
            }
          }
        }
        
       
      }
      else if (commands[2] == "SIMP")
      { // CMD,1064,SIP,<VALUE>
        if (csv.sim_mode)
        {                                       // == true
          double pres = commands[3].toDouble(); // Pa
          csv.pressure = (float)pres / 1000;    // kPa

          csv.altitude = 44330 * (1 - pow((pres / 101325), 1.0 / 5.255)); // 수정필요
          // Serial.print("  set pressure value to ");
          // Serial.println(commands[3]);
        }
        else
        {
          Serial.println("  simulation mode not activated.");
        }
      }
      else if (commands[2] == "CAL")
      { // CMD,1064,CAL
        float currentAlt = bmp.readAltitude();
        set_zero_altitude(currentAlt);
        tiltX_offset = csv.tilt_X;
        tiltY_offset = csv.tilt_Y;

        Serial.print("zero altitude ref set to ");
        Serial.println(currentAlt);
      }
      else if (commands[2] == "INIT")
      {
        if (commands[3] == "READY")
        {
          csv.state = IDLE;
          csv.packet_count = 0;
          csv.mission_time = 0;
          ready = true;
          csv.sim_mode = false;
          launch_time = timeToMillis(csv.gps_time);
          launched = false;
          csv.b1 = false;
          csv.b2 = false;
          csv.b3 = false;
          
          if (sd.exists("recovery.csv"))
          {
            sd.remove("recovery.csv");
            Serial.println("  recovery.csv deleted!");
          }
          else
          {
            Serial.println("  recovery.csv not exist.");
            Serial.println("  launched variable to false");
          }

          if (sd.exists("log.csv"))
          {
            sd.remove("log.csv");
            Serial.println("  log.csv deleted!");
          }
          else
          {
            Serial.println("  log.csv not exist.");
            Serial.println("  launched variable to false");
          }

          Serial.println("device initialized.");
        }
        else if (commands[3] == "RECOVER")
        {
          if (launched)
          {
            Recover();
          }
          else
          {
            Serial.println("  rocket not launched.");
          }
        }
      }
      else if (commands[2] == "BW")
      {
        if (commands[3] == "1")
        {
          // Burnwire 1
          digitalWrite(bw_2, LOW);
          digitalWrite(bw_3, LOW);
          digitalWrite(bw_1, HIGH);
          bw_enable = true;
          bw_timer = millis();
          bw_num = 2;
          descent = true;
        }
        else if (commands[3] == "2")
        {
          // Burnwire 2
          digitalWrite(bw_1, LOW);
          digitalWrite(bw_3, LOW);
          digitalWrite(bw_2, HIGH);
          bw_enable = true;
          bw_timer = millis();
          bw_num = 3;
          descent = true;
        }
        else if (commands[3] == "3")
        {
          // Burnwire 3
          digitalWrite(bw_1, LOW);
          digitalWrite(bw_2, LOW);
          digitalWrite(bw_3, HIGH);
          bw_enable = true;
          bw_timer = millis();
          bw_num = 4;
          descent = true;
        }
      }

      CMD_buf = "";
      for (int i = 0; i < 4; i++)
      {
        commands[i] = "";
      }
      Serial2.flush();
    }
  }
}
int dct_i = 0;
void DetectDescent()
{
  altitude_stack[dct_i] = csv.altitude;
  // ---DEBUG print---
  // Serial.print("altitude stack : [");
  // for(int dct_j=0; dct_j < DESCENT_STACK_SIZE; dct_j++){
  //   Serial.print(altitude_stack[dct_j]);
  //   Serial.print(" ");
  // }
  // Serial.print("] ");
  // Serial.print("differences : ");
  float diff = altitude_stack[dct_i] - altitude_stack[(dct_i + 1) % DESCENT_STACK_SIZE];
  // Serial.print(diff);
  // Serial.print("  ");
  if (diff <= DESCENT_MARGIN_ERR)
  { // APOGREE_MARGIN_ERR <- 음수값
    descent = true;
    // Serial.println("DESCENT!");
  }

  dct_i = (dct_i + 1) % DESCENT_STACK_SIZE;
}

// csv를 GCS로 보냅니다.
void SendCSV()
{
  Serial2.print(csv.to_string());
}

// launch_time : 발사 시작 UTC 시간(ms), csv.mission_time : 발사 후 지난 시간(ms)
// gps 시간과 비교하여 발사 후 지난 시간이 1초(1000ms) 이상 차이가나면
// 발사 시간을 조절합니다. // deprecated
void TimeSync()
{
  Serial.println(GPS.fix);
  if (abs(launch_time + csv.mission_time - timeToMillis(csv.gps_time)) >= 1 && GPS.fix == 1)
  {
    Serial.print(String(GPS.fix + " : "));
    Serial.println("mission_time synchronized!");
    csv.mission_time = timeToMillis(csv.gps_time) - launch_time;
  }
}

void LogCSV()
{
  if (!myFile.open("log.csv", O_WRITE | O_CREAT | O_APPEND))
  {
    Serial.println("  error : log file(WRITE) open err");
  }
  else
  {
    myFile.print(csv.to_string());
    myFile.close();
  }
}

void BurnwireControl()
{
  if (bw_num == 6)
  { // bw 루프는 단 한번만 실행됩니다.
    return;
  }

  // 최초 실행
  if (!bw_enable)
  {
    bw_timer = millis();
    bw_enable = true;
    bw_num = 1;
  }
  else
  {

    // 로직 파악을 위해 구조 유지
    if (bw_num == 1)
    {                         // BW_TIMING : millis
      digitalWrite(bw_1, HIGH); // 풀파워
      Serial.println("bw1 start");
      csv.b1 = true;
      bw_num = 2;
    }
    else if (bw_num == 2 && millis() - bw_timer >= 20000) // 1번 번와이어 20초 지속
    {
      digitalWrite(bw_1, LOW);
      digitalWrite(bw_2, HIGH); // 풀파워
      csv.b2 = true;
      Serial.println("bw1 end");
      Serial.println("bw2 start");
      bw_num = 3;
      bw_timer = millis();
    }
    else if (bw_num == 3 && millis() - bw_timer >= 30000){ // 2번 번와이어 30초 지속
      digitalWrite(bw_2, LOW);
      Serial.println("bw2 end");
      // bw_timer = millis();
      bw_num = 4;
    }
    else if (bw_num == 4 && csv.state == LANDED && millis() - bw_timer >= 40000) // 2번 ~ 3번 사이 10초 휴식
    {
      digitalWrite(bw_2, LOW);
      digitalWrite(bw_3, HIGH); // 풀파워
      csv.b3 = true;
      bw_timer = millis();
      Serial.println("bw3 start");
      bw_num = 5;
    }else if (bw_num == 5 && millis() - bw_timer >= 30000){ // 3번 번와이어 30초 지속 후 종료
      digitalWrite(bw_3, LOW);
      bw_num = 6;
      Serial.println("bw3 end");
      Serial.println("burnwire mechanism ended.");
    }
  }
}

void StateControl()
{
  // Serial.print("altitude : ");
  // Serial.println(csv.altitude);
  if (csv.state == IDLE && csv.altitude > 5)
  {
    csv.state = ASCENT;
    launched = true;
  }
  else if (csv.state == ASCENT && descent)
  { // && descent?
    csv.state = ROCKET_SEPERATION;

    // 카메라 작동
    if (!recorded)
    {
      trig_time = millis();
      digitalWrite(camera, LOW);
      record_video = true;
      recorded = true;
    }

    Serial.println("  Start to record video!");
    // 
  }
  else if (csv.state == ROCKET_SEPERATION && csv.altitude <= 505)
  {
    csv.state = HS_RELEASE;
  }
  else if (csv.state == HS_RELEASE && csv.altitude <= 200)
  {
    csv.state = PC_RELEASE;
  }
  else if (csv.state == PC_RELEASE && csv.altitude <= 5)
  {
    csv.state = LANDED;
    // 카메라 종료(저장)
    if (recorded)
    {
      trig_time = millis();
      digitalWrite(camera, LOW);
      end_video = true;
    }
    land_packet = csv.packet_count;
    Serial.println("  End to record video!");
  }
}
