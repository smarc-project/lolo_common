# Message containing information about the status of the captain
Header header

# Possible control inputs
uint8 RC           = 0
uint8 RF           = 1
uint8 CAPTAIN      = 2
uint8 SCIENTIST    = 3
uint8 EMERGENCY    = 4

uint8 active_control_input

float32 targetWaypoint_lat
float32 targetWaypoint_lon
float32 targetYaw
float32 targetPitch
float32 targetSpeed
float32 targetRPM
float32 targetDepth
float32 targetAltitude
