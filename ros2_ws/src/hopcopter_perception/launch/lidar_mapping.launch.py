"""The LiDAR mapping pipeline (docs/lidar_mapping_plan.md, stage 5).

    ros2 launch hopcopter_perception lidar_mapping.launch.py scenario:=low_wall

Starts, in place of the README's Terminals 3-5:
  - ros_gz_bridge (config/bridge_sim.yaml): /clock, /cf_0/imu, /cf_0/odom, the
    LiDAR and its IMU, and the release of the hold;
  - lidar_sim_adapter: finite points outside the blind radius -> /lidar/points
    (the mapper's) and every 3rd of them -> /lio/points (FAST-LIO's); the 1 kHz
    LiDAR IMU averaged to 200 Hz -> /lio/imu (optionally clipped at the real
    sensor's range);
  - gz_truth_tf: Gazebo's pose as TF (world -> cf_0/base_link -> cf_0/lidar);
  - FAST-LIO (fastlio_mapping, config/fast_lio_sim.yaml), from the ~/fastlio_ws overlay;
  - lio_anchor: anchors FAST-LIO's camera_init to the world during the hold,
    releases the robot, and logs FAST-LIO against Gazebo (data/data_lio/);
  - lidar_elevation_mapper: /elevation_map from the scans;
  - elevation_map_publisher on /elevation_map_known: the scenario's known map,
    which the LiDAR map is scored against (data/data_elevation_map/).
Gazebo must be started with -H (sitl_singleagent.sh), which holds the robot
until lio_anchor releases it.

Arguments:
  scenario:=<world>        the known map's scenario (elevation_map_publisher)
  pose_source:=lio         the mapper's pose: lio (FAST-LIO, stage 3b) or gazebo
                           (Gazebo's truth, stage 3a; no FAST-LIO, released after 1 s)
  mapper_input:=raw        lio only: raw (/lidar/points read in FAST-LIO's body frame) or
                           registered (FAST-LIO's dense /cloud_registered)
  mapping:=true            false: no LiDAR map; the known map goes on /elevation_map
                           (the known-map mode, for controller work); a held robot is
                           released after 1.5 s
  imu_acc_limit_g:=0       clip the LiDAR IMU's accelerometer at this range (0: off)
  imu_gyro_limit_dps:=0    clip its gyro at this range (0: off)
  fast_lio_config:=<file>  FAST-LIO's parameter file
  bridge:=true             false: no ros_gz_bridge, to replay a bag of the bridged
                           topics (/clock, /cf_0/*) instead of running Gazebo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _pipeline(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    share = get_package_share_directory('hopcopter_perception')
    scenario = arg('scenario')
    pose_source = arg('pose_source')
    mapping = arg('mapping').lower() in ('1', 'true', 'yes')
    sim = {'use_sim_time': True}
    actions = []

    if arg('bridge').lower() in ('1', 'true', 'yes'):
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
            parameters=[{'config_file': os.path.join(share, 'config', 'bridge_sim.yaml')}],
            output='screen'))
    actions.append(Node(
        package='hopcopter_perception', executable='gz_truth_tf', parameters=[sim],
        output='screen'))

    if not mapping:
        actions.append(Node(
            package='elevation_map_publisher', executable='elevation_map_publisher_node',
            parameters=[{'scenario': scenario, 'topic': '/elevation_map'}], output='screen'))
        actions.append(Node(
            package='hopcopter_perception', executable='lio_anchor',
            parameters=[sim, {'mode': 'gazebo', 'settle_time': 1.5}], output='screen'))
        actions.append(LogInfo(msg=f'Known-map mode: the {scenario} map on /elevation_map'))
        return actions

    actions.append(Node(
        package='hopcopter_perception', executable='lidar_sim_adapter',
        parameters=[sim, {'imu.acc_limit_g': float(arg('imu_acc_limit_g')),
                          'imu.gyro_limit_dps': float(arg('imu_gyro_limit_dps'))}],
        output='screen'))
    actions.append(Node(
        package='elevation_map_publisher', executable='elevation_map_publisher_node',
        parameters=[{'scenario': scenario, 'topic': '/elevation_map_known'}], output='screen'))

    mapper = {'use_sim_time': True}
    if pose_source == 'lio':
        actions.append(Node(
            package='fast_lio', executable='fastlio_mapping', name='laser_mapping',
            parameters=[arg('fast_lio_config'), sim], output='screen'))
        actions.append(Node(
            package='hopcopter_perception', executable='lio_anchor',
            parameters=[sim, {'mode': 'lio'}], output='screen'))
        if arg('mapper_input') == 'registered':
            mapper.update({'cloud_topic': '/cloud_registered', 'sensor_frame': 'body'})
        else:
            mapper.update({'cloud_topic': '/lidar/points', 'cloud_frame': 'body'})
        mapper['height.attitude_sigma_deg'] = 0.3
    elif pose_source == 'gazebo':
        actions.append(Node(
            package='hopcopter_perception', executable='lio_anchor',
            parameters=[sim, {'mode': 'gazebo'}], output='screen'))
        mapper.update({'cloud_topic': '/lidar/points', 'height.attitude_sigma_deg': 0.1})
    else:
        raise RuntimeError(f"pose_source must be 'lio' or 'gazebo', not '{pose_source}'")
    actions.append(Node(
        package='hopcopter_perception', executable='lidar_elevation_mapper',
        parameters=[mapper], output='screen'))
    actions.append(LogInfo(msg=f'LiDAR mapping on {scenario}: pose from {pose_source}'))
    return actions


def generate_launch_description():
    share = get_package_share_directory('hopcopter_perception')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='low_wall'),
        DeclareLaunchArgument('pose_source', default_value='lio'),
        DeclareLaunchArgument('mapper_input', default_value='raw'),
        DeclareLaunchArgument('mapping', default_value='true'),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('imu_acc_limit_g', default_value='0'),
        DeclareLaunchArgument('imu_gyro_limit_dps', default_value='0'),
        DeclareLaunchArgument(
            'fast_lio_config', default_value=os.path.join(share, 'config', 'fast_lio_sim.yaml')),
        OpaqueFunction(function=_pipeline),
    ])
