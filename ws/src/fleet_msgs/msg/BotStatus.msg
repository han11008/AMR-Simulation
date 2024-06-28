int8 STATUS_ACTIVE=0
int8 STATUS_INACTIVE=1
int8 STATUS_EXECUTING=2
int8 STATUS_SUCCEEDED=3
int8 STATUS_CANCELLED=4
int8 STATUS_REJECTED=5

string bot_name
float64 keepout_radius
int8 status
geometry_msgs/Vector3 current_pos
float64 distance_remain
