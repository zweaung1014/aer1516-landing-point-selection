# One plan from ballistic_planner_node: the takeoff of every hop, and the stance
# that redirects the robot into it (stage 3 of docs/follow_planner_plan.md).
# Published just before the /ballistic_trajectory marker pair, with the same
# header.stamp. Vectors are in the world frame.

std_msgs/Header header

geometry_msgs/Point start         # start cell centre; z = ground [m]
float64 start_speed               # arrival speed the plan assumes there: the last
                                  # warm-up landing, as the search binned it [m/s]
geometry_msgs/Vector3 start_axis  # body z axis at that landing, which launches
                                  # hops[0] (unit)
float64 start_tilt                # angle of start_axis from vertical [rad]

PlannedHop[] hops                 # hops[i] lands on the marker's point i + 1

# The model the plan was made with
bool momentum_aware               # false: the reference search, whose leg
                                  # launches along the arc (alpha_leg = arc angle)
float64 g                         # [m/s^2]
float64 eta                       # fraction of landing kinetic energy the stance keeps
float64 k2                        # stance model: the landing axis sits theta_v / k2
                                  # from -v_in toward the takeoff direction
