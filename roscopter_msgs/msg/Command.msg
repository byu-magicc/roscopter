# roscopter commands
# can be used for any level of command:
# high level: n_pos, e_pos, d_pos, yaw
# mid level: n_pos, e_pos, d_vel, yaw
#        or: n_vel, e_vel, d_pos, yaw_rate
#        or: n_vel, e_vel, d_vel, yaw_rate
# low level: n_acc, e_acc, d_acc, yaw_acc

# uint8 MODE_NORTH_EAST_DOWN_HEADING
uint8 MODE_NPOS_EPOS_DPOS_YAW = 0
uint8 MODE_NVEL_EVEL_DPOS_YAWRATE = 1
uint8 MODE_NACC_EACC_DACC_YAWRATE = 2
uint8 MODE_NVEL_EVEL_DVEL_YAWRATE = 3
uint8 MODE_NPOS_EPOS_DVEL_YAW = 4

# ros2
# builtin_interfaces/Time stamp
time stamp

uint8 mode

float32 cmd1
float32 cmd2
float32 cmd3
float32 cmd4
