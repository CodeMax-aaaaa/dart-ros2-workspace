std_msgs/Header header
bool motor_ls_online 0
bool motor_y_online 0
bool motor_dm_online 0
bool judge_online 0
bool[4] motor_fw_online [0,0,0,0]
bool rc_online 0
bool dart_launcher_online 0

uint8 dart_state 255 # boot = 100, protect = 101, remote = 102, match = 103-106 Enter Wait Launch Reload, undefined=255

bool motor_y_resetting 0
bool motor_dm_resetting 0
bool motor_ls_resetting 0

uint8 dart_launch_process 0

int32 motor_y_angle 0
int32[4] motor_fw_velocity [0,0,0,0]
float64[4] motor_fw_current [0,0,0,0]

float64[4] bus_voltage [0,0,0,0]