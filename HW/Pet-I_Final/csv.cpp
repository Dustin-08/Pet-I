#include "csv.h"

csv_data::csv_data(){
  state = IDLE; // enum State (int) (IDLE | WALKING | RUNNING | SHAKING )
  temperature = 0.0; // float
  bpm = 0.0; // float
  voltage = 4.1; // float (V)
  tilt_x = 0.0; // float
  tilt_y = 0.0; // float
  tilt_z = 0.0; // float
}

String csv_data::to_string(){
//   String walked = (int)state >= (int) ? String("W") : String("N");
//   String ran = (int)state >= (int)PC_RELEASE ? String("C") : String("N");
//   String shaked = (int)state >= (int)LANDED ? String("M") : String("N");

  String csv_buf = String(team_id, DEC) + ", "
    + state_to_str(state) + ", "
    + hs_deployed + ", " // HS_DEPLOYED
    + pc_deployed + ", " // PC_DEPLOYED
    + mast_raised + ", " // MAST_RAISED
    + String(temperature, 1) + ", "
    + String(bpm, 1) + ", "
    + String(voltage, 1) + ", "
    + String(tilt_x, 2) + ", "
    + String(tilt_y, 2) + ", "
    + String(tilt_z, 2) + ", "
    + cmd_echo + ", " // 쉼표 제거됩니다
    + '\r'; 

    return csv_buf;
}