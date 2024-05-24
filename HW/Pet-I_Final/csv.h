#include <Arduino.h>

enum State{
  IDLE = 0,
  WALKING,
  RUNNING,
  SHAKING
};

String state_to_str(State st);
String millisToTime(uint32_t millis);

class csv_data{
  public:
    csv_data();

    const int pet_id = 1064; // iot 기기별 id 부여
    /*uint32_t mission_time;
    int packet_count;
    bool sim_mode;*/
    State state; // 현재 상태를 분류된 4가지에서 select
    //float altitude;
    float temperature; // 체온
    //float pressure;
    float voltage; // 전압
    float bpm; // 심박
    //String gps_time;
    float tilt_x;
    float tilt_y;
    float tilt_z;
    /*int gps_sats;
    float tilt_X;
    float tilt_Y;
    String CMD_ECHO;
    bool b1; //burnwire;
    bool b2;
    bool b3;*/
  String to_string();
  
};