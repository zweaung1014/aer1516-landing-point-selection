# Perception-Informed Autonomous Navigation for a Monopedal Hopping Robot 
(Instructions are for simulation)

This project simulates **Hopcopter** — a Crazyflie-based monopedal hopping robot — in Gazebo Harmonic. A ROS2 pipeline integrates LiDAR point-cloud processing, OMPL RRT* global path planning, and a multi-criteria local planner that scores and refines candidate landing zones in real time, enabling fully autonomous obstacle-aware navigation. An optional natural-language goal interface (powered by Claude) lets users command the robot in plain English.

## Demo

**Simulation** — [Local planner explanation (YouTube)](https://youtu.be/Yvn6zSlEApQ)

![Hopcopter simulation](docs/simulation.gif)

**Hardware** — [Full hardware demo (YouTube)](https://youtu.be/_PuOxqvgZJw)

![Hopcopter hardware](docs/hardware.gif)

## Authors

**Zwe Min Htet Aung** — zweminhtetaung@gmail.com

---

## Architecture Overview

```
┌─────────────┐       ┌──────────────────┐       ┌───────────────┐       ┌────────────┐
│  LiDAR      │──────▶│  rrt_star node   │──────▶│ local_planner │──────▶│  hopcopter │
│ Point Cloud │       │  (Global Planner) │       │  (Adjustment)  │       │ (Control)  │
└─────────────┘       └──────────────────┘       └───────────────┘       └────────────┘
                             ▲
                             │
                      ┌──────┴───────┐
                      │  /goal_pose  │
                      │              │
                      ├──────────────┤
                      │ Option A:    │
                      │  ros2 topic  │
                      │  pub --once  │
                      ├──────────────┤
                      │ Option B:    │
                      │  NLP Goal    │
                      │  Interface   │
                      └──────────────┘
```

**Pipeline:**
1. **rrt_star** — Reads LiDAR point cloud, constructs a 2D occupancy grid, listens for `/goal_pose`, and uses OMPL to compute a global path. Publishes a set of waypoints.
2. **local_planner** — Subscribes to waypoints and adjusts them based on obstacle proximity, slope, and edge criteria for safe landing zones.
3. **hopcopter** — Controls the hopping robot's attitude to follow the adjusted waypoints.

---

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Harmonic
- System packages:
  ```bash
  sudo apt-get install libboost-program-options-dev libusb-1.0-0-dev python3-colcon-common-extensions
  sudo apt-get install ros-humble-motion-capture-tracking ros-humble-tf-transformations
  sudo apt-get install ros-humble-ros-gzharmonic
  ```
- OMPL (Open Motion Planning Library)
- PCL (Point Cloud Library): `sudo apt-get install libpcl-all-dev`
- Python packages: `pip3 install cflib scipy`

---

## Installation

```bash
# Clone the repository
git clone <repo-url> ~/CrazySim
cd ~/CrazySim

# Build the ROS2 workspace
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Running the Simulation

Each step runs in a **separate terminal**. In every terminal, source the workspace first:

```bash
cd ~/CrazySim/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Terminal 1 — Start Gazebo

```bash
cd ~/CrazySim/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -m crazyflie -x 0 -y 0
```

> **NVIDIA GPU:** If using a discrete NVIDIA GPU, prefix with:
> ```bash
> __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -m crazyflie -x 0 -y 0
> ```

### Terminal 2 — Launch cflib Server

Connects to the simulated Crazyflie via cflib:

```bash
cd ~/CrazySim/ros2_ws
ros2 launch crazyflie launch.py backend:=cflib
```

### Terminal 3 — LiDAR Bridge

Bridges LiDAR point cloud from Gazebo to ROS2:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /cf_0/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
```

### Terminal 4 — IMU Bridge

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /cf_0/imu@sensor_msgs/msg/Imu@gz.msgs.IMU
```

### Terminal 5 — Odometry Bridge

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/cf_0/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
```

### Terminal 6 — TF Broadcaster

```bash
ros2 run tf_transform_broadcast tf_transform_broadcaster
```

### Terminal 7 — RRT* Global Planner

```bash
ros2 run rt_star_planner rrt_star
```

### Terminal 8 — Local Planner

```bash
ros2 run local_planning local_planner
```

### Terminal 9 — Hopcopter Controller

```bash
ros2 run hopping_robot hopcopter
```

### Start the Simulation

Click **Play** in the Gazebo GUI. The robot will begin hopping.

### Send a Goal

Choose one of the following methods:

**Option A — Manual goal command:**

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 2.0, y: 0.5, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

**Option B — NLP Goal Interface (natural language):**

```bash
export PYTHONPATH=$HOME/CrazySim/ros2_ws/.venv/lib/python3.10/site-packages:$PYTHONPATH
ros2 run nlp_goal_interface nlp_goal_node
```

Then type a goal in natural language (e.g., "go to position 2, 0.5, 0.3").
## Running the Simulation — Ballistic 2.5D Pipeline

Alternative to the RRT* pipeline above: the 2.5D ballistic hopping A* planner
(C++ port of `hopcopter-ballistic-planning/`). It plans **once** per goal on a
known elevation map published by `elevation_map_publisher` (LiDAR-based map
building comes later), and does not use the LiDAR bridge, TF broadcaster,
RRT*, or the local planner. **7 terminals** instead of 9; source every
terminal as above.

### Terminal 1 — Start Gazebo (low-wall world)

```bash
cd ~/CrazySim/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -m crazyflie -x 0 -y 0 -w crazysim_low_wall
```

The `crazysim_low_wall` world is the default world without the chair/prius,
plus a 0.4 m wall at world x ∈ [1.2, 1.4] matching the published elevation map
(`maps/low_wall.py` shifted by the map origin (−0.5, −2.5)).

### Terminal 2 — Launch cflib Server

```bash
cd ~/CrazySim/ros2_ws
ros2 launch crazyflie launch.py backend:=cflib
```

### Terminal 3 — IMU Bridge

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /cf_0/imu@sensor_msgs/msg/Imu@gz.msgs.IMU
```

### Terminal 4 — Odometry Bridge

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/cf_0/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
```

### Terminal 5 — Elevation Map Publisher

Publishes the known 2.5D map (`grid_map_msgs/GridMap` on `/elevation_map`,
5×5 m @ 0.1 m, origin (−0.5, −2.5); scenario selectable via the `scenario`
parameter: `low_wall` or `flat`):

```bash
ros2 run elevation_map_publisher elevation_map_publisher_node
```

### Terminal 6 — Ballistic Motion Planner

Plans once per `/goal_pose` and publishes the waypoints on
`/ballistic_trajectory` (Marker id=400, per-waypoint z = terrain + 0.8):

```bash
ros2 run ballistic_motion_planner ballistic_planner_node
```

### Terminal 7 — Hopcopter Controller

```bash
ros2 run hopping_robot hopcopter
```

### Start and Send a Goal

Click **Play** in the Gazebo GUI; the robot begins hopping in place. Then send
the low-wall scenario goal (Python GOAL (4.5, 2.5) → world (4.0, 0.0)):

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

The planner logs the hop table (takeoff angle, speeds, injected energy per
hop) and writes `ballistic_path_*.csv` / `ballistic_hops_*.csv` to
`data/data_ballistic_planner/` for comparison against
`data/data_hopcopter/landing_points_*.csv`.

### Offline Parity Check

Verifies the C++ port against the Python reference planner (no ROS graph
needed):

```bash
ros2 run ballistic_motion_planner ballistic_parity_test
```

Compare with the Python side (`cd ~/CrazySim/hopcopter-ballistic-planning &&
python main.py low_wall`): waypoints, per-hop physics, and search counters
should match.
