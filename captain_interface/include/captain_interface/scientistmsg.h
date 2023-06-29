/*! \file scientistmsg.h
    \brief Constants for scientist interface
    
    This file contains constants for the scientist interface.
    The constants are used to identify the messages between the scientist and the captain
*/

#ifndef captainscientistmessages
#define captainscientistmessages

//V1.1
//2021 11 06

//CAPTAIN -> SCIENTIST
#define CS_LEAK             7
#define CS_STATUS           8
#define CS_CONTROL          9
#define CS_RUDDER           10
//#define CS_ELEVATOR         12
#define CS_THRUSTER_PORT    13
#define CS_THRUSTER_STRB    14
#define CS_BATTERY          15
//#define CS_DVL              16
//#define CS_GPS              17
//#define CS_IMU              18
//#define CS_MAG              19
//#define CS_PRESSURE         20
#define CS_ELEVON_STRB      21
#define CS_ELEVON_PORT      22
//#define CS_VBS              23
//#define CS_POSITION         24
//#define CS_FLS              25
#define CS_CTRL_STATUS      26
#define CS_SENSOR_STATUS    27
#define CS_TEXT             40
#define CS_USBL_RECEIVED    41
#define CS_MENUSTREAM       50
#define CS_MISSIONLOG       51
#define CS_DATALOG          52
#define CS_REQUEST_OUT      100
#define CS_DVL_PD0_FIXED         28
#define CS_DVL_PD0_VARIABLE      29
#define CS_DVL_PD0_BOTTOMTRACK   30


//SCIENTIST -> CAPTAIN
#define SC_REQUEST_IN            101
#define SC_HEARTBEAT             150
#define SC_ABORT                 151
#define SC_DONE                  152
#define SC_SET_RUDDER            161
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
#define SC_MENUSTREAM            200

#define SC_NAV_DATA             110
#define SC_ALTITUDE_DATA        111

#define SC_USBL_TRANSMIT        142

//Service ID:s
#define SERVICE_CONTROLLER_WAYPOINT  210 
#define SERVICE_CONTROLLER_PITCH     211
#define SERVICE_CONTROLLER_ROLL      212
#define SERVICE_CONTROLLER_YAW       213
#define SERVICE_CONTROLLER_YAWRATE   214
#define SERVICE_CONTROLLER_DEPTH     215
#define SERVICE_CONTROLLER_ALTITUDE  216
#define SERVICE_CONTROLLER_SPEED     217

#define SERVICE_ACTION_FAIL     0
#define SERVICE_ACTION_SUCCESS  1
#define SERVICE_ACTION_DISABLE  0
#define SERVICE_ACTION_ENABLE   1

#endif