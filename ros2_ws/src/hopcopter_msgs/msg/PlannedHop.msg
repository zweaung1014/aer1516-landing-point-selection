# One planned hop, from the previous landing (or the start) to `landing`.

geometry_msgs/Point landing             # landing cell centre; z = ground [m]
float64 heading                         # takeoff heading [rad, atan2(dy, dx)]
float64 alpha_leg                       # leg launch angle above horizontal [rad]
float64 v_leg                           # leg launch speed [m/s]: sqrt(eta) x the arrival speed
float64 v_h                             # horizontal speed of the arc [m/s]
float64 target_apex                     # apex above the takeoff ground [m]: the marker's z
float64 e_inject_per_kg                 # push energy, as an instant vertical kick [J/kg]
geometry_msgs/Vector3 arrival_velocity  # CoM velocity at touchdown on `landing` [m/s]
geometry_msgs/Vector3 landing_axis      # body z axis at that touchdown, which redirects the
                                        # robot into the next hop, or stops it at the goal (unit)
float64 landing_tilt                    # angle of landing_axis from vertical [rad]
