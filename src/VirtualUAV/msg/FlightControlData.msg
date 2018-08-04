# The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).

Header header
int32 ts
# Quaternion component
float32 q0
float32 q1
float32 q2
float32 q3

# Acceleration
float32 ax
float32 ay
float32 az

#Velocity
float32 vx
float32 vy
float32 vz
# if from guidance
uint8 health_flag
uint8 feedback_sensor_id

# Angular speed (rad/s)
float32 wx
float32 wy
float32 wz

#latitude is in angle
float64 latitude
#longitude is in angle
float64 longitude
float32 altitude
float32 height
#reliablity [0,5]
int8 health 

#    North(x)
#   /
#  /
# /______East(y)
# |
# |
# Donw (-z)
float32 x
float32 y
float32 z 

int8 cur_ctrl_dev_in_navi_mode
int8 serial_req_status
