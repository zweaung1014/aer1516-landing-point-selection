f# Setup of Crazywarm2 with CrazySim
This guide details the setup and configuration of Crazyswarm2 with the CrazySim simulator, as well as the required code modifications for compatibility and example usage.

## Crazyswarm2
### Installation of Crazyswarm2
Install the Crazyswarm2 following the instructions https://imrclab.github.io/crazyswarm2/installation.html

### Configuration
Find the crazyflie.yaml file in ros2_ws/src/crazyswarm2/crazyflie/config/
Replace it to our own configure file crazyflie.yaml

## Code issues
### tf_transformation
Open the crazyflie_server.py in cfros2_ws/install/crazyflie/lib/crazyflie
Find 2 places where it do the transformation from euler angle to quaternion: q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
Replace them by another function which doesn't have numpy version conflicts: q = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

Delect the libarary: import tf_transformations
Replace it by: from scipy.spatial.transform import Rotation as R

### crazyflie-python-lib
In crazyflie_server.py, comment or delete the relevent code with link_statistics
```
self.swarm._cfs[link_uri].status = {}
self.swarm._cfs[link_uri].status["latency"] = 0.0
self.swarm._cfs[link_uri].cf.link_statistics.latency_updated.add_callback(partial(self._latency_callback, uri=link_uri))
self.swarm._cfs[link_uri].status["num_rx_unicast"] = 0.0
self.swarm._cfs[link_uri].cf.link_statistics.uplink_rate_updated.add_callback(partial(self._uplink_rate_callback, uri=link_uri))
self.swarm._cfs[link_uri].status["num_tx_unicast"] = 0.0
self.swarm._cfs[link_uri].cf.link_statistics.downlink_rate_updated.add_callback(partial(self._downlink_rate_callback, uri=link_uri))
```

## Example code
Find the set_param.py in ~/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples
Replace it by our own code set_param.py

## test ramp
### Run the Crazysim first
```
cd ~/CrazySim/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -m crazyflie -x 0 -y 0

To run with NVIDIA graphics card, run this: 
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -m crazyflie -x 0 -y 0

### Use this if you wanna spawn with a roll angle
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh \
     -m crazyflie -x 0 -y 0 -z 1 -R 25
```
### Launch the server
```
ros2 launch crazyflie launch.py backend:=cflib
```
If launch the crazyflie_server successfully, you will see all the topics like /cf_1/cmd_vel

### Run the example code
```
ros2 run crazyflie_examples set_param
```
You will see the motors ramped and the crazyflie goes up and fall down.
by echo the /tf topic the position of the crazyflie could be seen.
```
ros2 topic echo /tf
```

source install/setup.bash
colcon build

### When you recompile after tuning the internal PID gains, do this from crazyflie-firmware
rm -rf sitl_make/build
cd sitl_make
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -B build
cmake --build build

# or do this (from https://github.com/gtfactslab/CrazySim):
cd crazyflie-firmware
mkdir -p sitl_make/build && cd $_
cmake ..
make all

Debug print:
rm -rf sitl_make/build
cd sitl_make
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_C_FLAGS="-DDEBUG_PRINT_ENABLED" -B build
cmake --build build

### Command to bridge gazebo to ros2
# This is for gpu_lidar:
ros2 run ros_gz_bridge parameter_bridge \
  /cf_0/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan

# This is for PointCloud2:
ros2 run ros_gz_bridge parameter_bridge \
  /cf_0/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
  Frame ID: crazyflie_0/lidar/lidar_sensor

ros2 run ros_gz_bridge parameter_bridge \
  /cf_0/mid360/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked

# This is for camera:
ros2 run ros_gz_bridge parameter_bridge /cf_0/camera@sensor_msgs/msg/Image@gz.msgs.Image
Frame ID: crazyflie_0/camera_link/camera

# Just testing crazyflie_mapping_demo:
gz.msgs.PointCloudPacked
ros2 run ros_gz_bridge parameter_bridge \
	/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked

ros2 run ros_gz_bridge parameter_bridge \
	/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan

# 1) Bidirectional (simple):
# This is for acceleration:
ros2 run ros_gz_bridge parameter_bridge \
	/cf_0/imu@sensor_msgs/msg/Imu@gz.msgs.IMU
# This is for velocity:
ros2 run ros_gz_bridge parameter_bridge \
  "/cf_0/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
(or)
ros2 run ros_gz_bridge parameter_bridge /cf_0/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry

# 2) One‑way  GZ → ROS  (sensor‑only):
ros2 run ros_gz_bridge parameter_bridge \
     /cf_0/imu@sensor_msgs/msg/Imu[gz.msgs.IMU

This is the structure of /cf_0/imu
zweminhtetaung@zweminhtetaung-Surface-Book-2:~/CrazySim/ros2_ws$ ros2 interface show sensor_msgs/msg/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

geometry_msgs/Quaternion orientation
	float64 x 0
	float64 y 0
	float64 z 0
	float64 w 1
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
	float64 x
	float64 y
	float64 z
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
	float64 x
	float64 y
	float64 z
float64[9] linear_acceleration_covariance # Row major x, y z

### Starting up rviz2
# Reference: https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2

# To start recording rosbag:
ros2 bag record <topic name>

# When you're done capturing:
ros2 bag info <bag_name>

# To check or access the rosbag file you just saved:
ros2 bag info rosbag2_2025_10_15-14_28_18
# or
ros2 bag play rosbag2_2025_10_15-14_28_18

# To check message structure
ros2 interface show tutorial_interfaces/msg/Num

# ROS2 command for goal
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 2.0, y: 2.0, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
or
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 2.0, y: 0.5, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# Transform the lidar point cloud from lidar frame to world frame, include 0.8 offset:
ros2 run tf2_ros static_transform_publisher 0 0 0.8 0 0 0 world crazyflie_0/lidar/lidar_sensor

# Launch the fast planner for ros2:
ros2 launch  plan_manage  kino_replan_launch.xml