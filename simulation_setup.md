# Crazyflie Simulation Installation

## System
#### Ubuntu 22.04.5 LTS

## Installation

Install ROS2 Desktop: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
Install Gazebo: https://gazebosim.org/docs/harmonic/install_ubuntu/
Please install ROS2 and Gazebo first before following command.

## Cazyflie Simulation Workspace
#### Create a project workspace folder
```
mkdir ~/crazyflie_mapping_demo
cd crazyflie_mapping_demo
mkdir simulation_ws
mkdir ros2_ws
cd ros2_ws
mkdir src

```
#### Clone code
```
cd ~/crazyflie_mapping_demo/simulation_ws
git clone https://github.com/bitcraze/crazyflie-simulation.git
```

```
cd ~/crazyflie_mapping_demo/ros2_ws/src
git clone https://github.com/knmcguire/crazyflie_ros2_multiranger.git
git clone https://github.com/knmcguire/ros_gz_crazyflie
git clone https://github.com/IMRCLab/crazyswarm2 --recursive
```

```
sudo apt-get install libboost-program-options-dev libusb-1.0-0-dev python3-colcon-common-extensions
sudo apt-get install ros-humble-motion-capture-tracking ros-humble-tf-transformations
sudo apt-get install ros-humble-ros-gzharmonic ros-humble-teleop-twist-keyboard
pip3 install cflib transform3D 
```

```
cd  ~/crazyflie_mapping_demo/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
```

<ins>_If there are some error with regards to dependences, install use_</ins>
```
 sudo apt-get install ros-<distro_name>-<package_name>
```

#### Envorimental Setup
```
echo ". ~/ros2_humble/install/local_setup.bash" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### Run Simulation inside Workspace
```
cd  ~/crazyflie_mapping_demo/ros2_ws/
source ~/crazyflie_mapping_demo/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="/home/$USER/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
```

#### Launch Gazebo
```
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py
```
#### Launch keyboard teleop
```
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```