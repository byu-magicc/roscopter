# roscopter commands
# can be used for any level of command:
# high level: n,e,d,psi
# mid level: xvel, yvel, zvel, yaw_rate
# low level: xacc, yacc, zacc, yaw_acc

# uint8 MODE_NORTH_EAST_DOWN_YAW = 0
uint8 MODE_NPOS_EPOS_DPOS_YAW = 0
uint8 MODE_NVEL_EVEL_DPOS_YAWRATE = 1
uint8 MODE_NACC_EACC_DACC_YAWRATE = 2

# ros2
# builtin_interfaces/Time stamp
time stamp

uint8 mode

float32 cmd1
float32 cmd2
float32 cmd3
float32 cmd4
