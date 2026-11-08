# hopcopter's waypoint decision at a steering apex (JumpLib/waypoint_tracker.py), on
# /hopcopter/apex_decision. The planner reads it to know which hops of its plan remain
# and when a held robot hops in place (docs/lidar_mapping_plan.md, "Replanning").

std_msgs/Header header        # stamp: the apex, in sim time (Gazebo's odometry stamp)
float64 path_id               # the path being flown: its marker's header stamp [s]; 0 for none
string mode                   # track, retry, goal, hold or idle
int32 land_index              # the waypoint the next hop aims at; -1 for none
int32 retries                 # missed landings in a row on that waypoint
geometry_msgs/Point aim       # where the next hop aims (world); z = its target apex [m]
geometry_msgs/Point foot      # where this flight's foot is predicted to land (world); z = ground
uint32 hop_number             # touchdowns since Play
