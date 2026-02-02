## ROS 2, Gazebo, Motion Planning & Crazyflie Instructions

- Default ROS 2 distribution: **Humble**
- Default simulator: **Gazebo**
- Default hardware platform: **Crazyflie-based hopping robot**

### ROS 2 & Gazebo
- For ROS 2 or Gazebo questions and code:
  - Consult **Context7** for official documentation
  - Verify APIs, QoS defaults, callback behavior, executor and callback group semantics
  - Verify simulation timing, sensors, frames, and physics assumptions in Gazebo
  - Cite the documentation page or section title for non-trivial claims

### Motion Planning (OMPL / planners)
- For motion planning or path planning code (e.g., OMPL, planners, state spaces, validity checkers):
  - Consult **OMPL documentation via Context7**
  - Verify planner assumptions and guarantees (e.g., probabilistic completeness, asymptotic optimality)
  - Verify parameter meanings (step size, range, goal bias, rewiring radius, etc.)
  - Cite the relevant OMPL documentation page or section

### Visualization (RViz2)
- For RViz2-related visualization code or debugging (e.g., point clouds, paths, markers, TF):
  - Consult **RViz2 documentation via Context7**
  - Verify message types (`sensor_msgs/PointCloud2`, `nav_msgs/Path`, `visualization_msgs/Marker`)
  - Verify frame conventions, `frame_id` usage, and TF availability
  - Verify marker namespaces, lifetimes, and update behavior
  - Cite the relevant RViz2 documentation page or section when behavior is non-obvious

### Crazyflie Platform & Control
- For Crazyflie-related code (firmware, control, dynamics, messaging):
  - Base assumptions on the **Crazyflie platform constraints** (mass, thrust limits, onboard compute, communication latency)
  - Verify control loops, estimator usage (IMU, onboard EKF), and update rates against **official Crazyflie documentation or firmware references via Context7**
  - Do not assume high-bandwidth sensing, large payloads, or desktop-class compute
  - Explicitly state when code is intended for **simulation only** versus **onboard execution**

### Hopping / Hybrid Locomotion
- For hopping, jumping, or hybrid aerial–legged behavior:
  - Make actuator, impact, and timing assumptions explicit
  - Avoid implicit assumptions of continuous ground contact
  - Verify feasibility with respect to Crazyflie thrust, mass, and structural limits
  - Clearly separate planning, control, and state-estimation responsibilities

- If official documentation cannot be found or is ambiguous, state this explicitly and avoid guessing
