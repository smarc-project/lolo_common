#ifndef captainscientistmessages
#define captainscientistmessages

//V1.0
//2019 03 25

//CAPTAIN -> SCIENTIST
#define CS_STATUS           8
#define CS_CONTROL          9
#define CS_RUDDER_PORT      10
#define CS_RUDDER_STRB      11
#define CS_ELEVATOR         12
#define CS_THRUSTER_PORT    13
#define CS_THRUSTER_STRB    14
#define CS_BATTERY          15
#define CS_DVL              16
#define CS_GPS              17
#define CS_IMU              18
#define CS_MAG              19
#define CS_PRESSURE         20
#define CS_TEXT             40

//SCIENTIST -> CAPTAIN
#define SC_HEARTBEAT             150
#define SC_ABORT                 151
#define SC_DONE                  152
#define SC_SET_RUDDER_PORT       160
#define SC_SET_RUDDER_STRB       161
#define SC_SET_ELEVATOR          162
#define SC_SET_THRUSTER_PORT     163
#define SC_SET_THRUSTER_STRB     164
#define SC_SET_TARGET_PITCH      165
#define SC_SET_TARGET_YAW        166
#define SC_SET_TARGET_YAW_RATE   167
#define SC_SET_TARGET_SPEED      168
#define SC_SET_TARGET_RPM        169
#define SC_SET_TARGET_DEPTH      170
#define SC_SET_TARGET_ALTITUDE   171
#define SC_SET_TARGET_WAYPOINT   172

#endif
