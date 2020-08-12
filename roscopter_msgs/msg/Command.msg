# roscopter commands
# can be used for any level of command:
# high level: n_pos, e_pos, d_pos, yaw
#           : n_pos, e_pos, d_vel, yaw
#           : n_vel, e_vel, d_pos, yaw_rate
#           : n_vel, e_vel, d_vel, yaw_rate
#  mid level: n_acc, e_acc, d_acc, yaw_acc
#  low level: roll , pitch, yaw_rate , throttle
#           : roll_rate, pitch_rate, yaw_rate, throttle

# uint8 MODE_NORTH_EAST_DOWN_HEADING
uint8 MODE_NPOS_EPOS_DPOS_YAW = 0
uint8 MODE_NPOS_EPOS_DVEL_YAW = 1
uint8 MODE_NVEL_EVEL_DPOS_YAWRATE = 2
uint8 MODE_NVEL_EVEL_DVEL_YAWRATE = 3
uint8 MODE_NACC_EACC_DACC_YAWRATE = 4
uint8 MODE_ROLL_PITCH_YAWRATE_THROTTLE = 5
uint8 MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 6


# ros2
# builtin_interfaces/Time stamp
time stamp

uint8 mode

float32 cmd1
float32 cmd2
float32 cmd3
float32 cmd4
