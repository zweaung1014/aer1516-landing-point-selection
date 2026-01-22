#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import math
import numpy as np

from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, Twist, AccelStamped
from crazyflie_interfaces.msg import LogDataGeneric, FullState
from nav_msgs.msg import Odometry
import sys
from scipy.spatial.transform import Rotation
from hopping_robot.RisLib.standard_pid import PidControlRaw
from hopping_robot.RisLib import IIR2Filter
from hopping_robot.RisLib.cflog import LoggingCore
from functools import partial
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from queue import Queue

import logging
import time
from threading import Thread
from datetime import datetime

import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

# the libraries for hopping
from hopping_robot.JumpLib.jumping_tools import JumpingStateTrackerOnboard
from hopping_robot.JumpLib.jumping_model import LandingStateEstimator
from hopping_robot.JumpLib.jumping_tools import JumpingHeightController
from hopping_robot.JumpLib.jumping_model import LinearJumpingController
from hopping_robot.JumpLib.jumping_model import InPlaneJumpingModel

uri = uri_helper.uri_from_env(default='udp://0.0.0.0:19850') # radio://0/80/2M/E7E7E7E7E7

# Log commands into a file
timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_filename = f"standard_pid_output_{timestamp_str}.txt"
logging.basicConfig(
    filename=log_filename,
    level=logging.INFO,
    format="%(asctime)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)


# class RealTimeSleeper:
#     def __init__(self, sample_time):
#         self._sample_time = sample_time
#         self.loop_start_time = time.time()
#         self.loop_flag = 0

#     def init(self):
#         self.loop_start_time = time.time()

#     def sleep(self):
#         self.loop_flag = self.loop_flag + 1
#         current_time = time.time()

#         loop_end_time = (self.loop_start_time + self._sample_time)
#         sleep_time = loop_end_time - current_time
#         if sleep_time > 0:
#             time.sleep(sleep_time)
#         else:
#             print('warning: loop frequency lower than expected!')
#         self.loop_start_time = time.time()

class Differentiator:
    def __init__(self, diff_steps):
        self.t_queue = deque([0] * diff_steps, maxlen=diff_steps)
        self.data_queue = deque([0] * diff_steps, maxlen=diff_steps)
        self.data_rate = 0

    def step(self, data_now, abstime):
        dt = abstime - self.t_queue[0]
        #dt = 0.1
        if dt == 0:
            self.t_queue.append(abstime)
        else:
            self.data_rate = (data_now - self.data_queue[0]) / dt
            self.t_queue.append(abstime)
            self.data_queue.append(data_now)

        return self.data_rate

class trajectory(object):
    def __init__(self,reverse_R=0.75):
        self.origin_need = False
        self.origin_x = 0
        self.origin_y = 0
        self.reverse_R = reverse_R
        self.traj_enable = False

    def set_origin(self,x,y,jumping_counter):
        if self.origin_need:
            self.origin_x = x
            self.origin_y = y
            self.jumping_counter_begin = jumping_counter
            self.origin_need = False

    def trajectory_begin(self):
        self.traj_enable = True
        self.origin_need = True

    def traj_generate(self,jumping_counter):
        N_cycle = 80
        trans_step = 8
        self.index = jumping_counter - self.jumping_counter_begin
        if self.index < trans_step:
            desired_x = self.origin_x + self.reverse_R / trans_step * self.index
            desired_y = self.origin_y
        else:
            desired_x = self.origin_x + self.reverse_R * math.cos(2 * math.pi * (self.index - trans_step) / N_cycle)
            desired_y = self.origin_y + self.reverse_R  * math.sin(4 * math.pi *(self.index - trans_step) / N_cycle)
        return desired_x, desired_y

class hopcopter(Node):
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        # Initiate ROS2 node
        super().__init__('hopcopter')

        print('Connecting to %s' % link_uri)

        # Define important variables
        self.first_loop = True
        self.flight_enable = False
        self.controller_start_flag = True
        self.G = 9.6
        self.G_flight_time_mocap = 9.8
        self.leg_length = 0.4
        self.jumping_counter = 0
        self.jumping_height_record = 0.5  # CoM vertical movement distance
        self.powered_climbing_thrust = 14300 # might need to tune this, was 15000
        self.ready_to_drop = True

        # Initiate ROS2 publisher
        self.rpyt = self.create_publisher(Twist, '/cf_1/cmd_vel_legacy', 50)

        # Initiate ROS2 subscribers
        ## This is used to obtain pose from the firmware. Calculated by integrating acceleration twice.
        self.pose = self.create_subscription(PoseStamped, '/cf_1/pose', self.robot_pose, 50)
        self.current_pos = self.current_ori = None
        
        ## This is used to obtain acceleration data from the firmware. Not accurate enough.
        self.imu = self.create_subscription(LogDataGeneric, '/cf_1/imu', self.motorPower, 50)
        self.motor_m1, self.motor_m2, self.motor_m3, self.motor_m4 = 0.0, 0.0, 0.0, 0.0

        # This is used to obtain acceleration from Gazebo ground truth
        self.gz_imu = self.create_subscription(Imu, '/cf_0/imu', self.gz_imu_acceleration, 50)
        self.ax = self.ay = self.az = 0.0

        # This is used to obtain velocity and position/orientation from Gazebo ground truth
        self.gz_odom = self.create_subscription(Odometry, '/cf_0/odom', self.gz_odom_velocity, 50)
        self.vel_x, self.vel_y, self.vel_z = 0.0, 0.0, 0.0
        self.pos_x, self.pos_y, self.pos_z = 0.0, 0.0, 0.0
        self.ori_x, self.ori_y, self.ori_z, self.ori_w = 0.0, 0.0, 0.0, 0.0

        # Trajectory following variables
        self.trajectory_queue = Queue()
        self.current_goal_tolerance = 0.4  # meters - configurable tolerance
        self.has_active_goal = False
        self.goal_reached_count = 0
        self.last_waypoint_position = None  # Store last completed waypoint
        self.pending_new_traj = False       # True between DELETE and ADD to avoid jumps
        self.trajectory_received = False    # Only allow control after first trajectory received
        
        # Initialize desired positions as None until trajectory is received
        self.desired_x = None
        self.desired_y = None
        self.desired_z = 0.8  # Always keep altitude command at 0.8 m
        self.desired_yaw = 0.0
        
        # Subscribe to trajectory from OMPL RRT* planner
        self.trajectory_sub = self.create_subscription(
            Marker,
            '/ompl_rrt_star_trajectory',
            self.trajectory_callback,
            10
        )

        # Create a timer for publisher
        self.timer = self.create_timer(0.01, self.RPYT_commands) # 0.01 is 100 Hz, 0.002 is 500 Hz

        # Initialize the IIR2Filter object
        self.sample_time = 0.01  # loop sample time
        self.sample_rate = 1 / self.sample_time
        self.Filter_x = IIR2Filter.IIR2Filter(4, [10], 'lowpass', design='cheby2', rs=5, fs=self.sample_rate)
        self.Filter_y = IIR2Filter.IIR2Filter(4, [10], 'lowpass', design='cheby2', rs=5, fs=self.sample_rate)
        self.Filter_z = IIR2Filter.IIR2Filter(4, [10], 'lowpass', design='cheby2', rs=5, fs=self.sample_rate)
        
        # Initialize the Differentiator object for xyz_error_dot
        self.Diff_X = Differentiator(diff_steps=2)
        self.Diff_Y = Differentiator(diff_steps=2)
        self.Diff_Z = Differentiator(diff_steps=2)
        
        # Initialize the Differentiator object for velocity
        self.Diff_X_vel = Differentiator(diff_steps=20)
        self.Diff_Y_vel = Differentiator(diff_steps=20)
        self.Diff_Z_vel = Differentiator(diff_steps=20)

        # Initialize the important objects for hopping
        self.JSTO = JumpingStateTrackerOnboard(acc_z_up_limit=2.0, z_dot_limit=0.2 ,controller_engage_time_auto=True,
                            controller_engage_time=0.16, powered_climbing_end_timer=0.2, powered_failing_begin_timer=0.15) # was powered_climbing_end_timer=0.12
        self.LSE = LandingStateEstimator()
        self.JHC = JumpingHeightController(leg_efficiency=0.7, g=9.6, t_p_low=0.04, t_p_high=0.7, t_i = 0.06, thrust_gain=0.7) # was thrust_gain=0.55
        self.IPJM = InPlaneJumpingModel(1.688, 2.469)
        self.LJC = LinearJumpingController(high_ballistic_enable = True, model=self.IPJM, g = self.G, normal_gain=1)
        self.TJ = trajectory(reverse_R=1.1)

        self.t0 = self.get_clock().now()
        
    def gz_odom_velocity(self, msg: Odometry) -> None:
        """
        Ground-truth pose & velocity from Gazebo (bridged /cf_0/odom).
        Linear velocities are in m/s. Frame semantics:
        - msg.header.frame_id  : reference frame (often 'world' or 'odom')
        - msg.child_frame_id   : moving body frame (e.g., 'cf_0/base_link')
        - twist.twist.linear.* : velocity of child w.r.t. header frame, expressed in child_frame_id
            (this is the standard nav_msgs/Odometry convention; confirm by inspecting frames).
        """
        # Extract position data
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z

        # Extract orientation data
        self.ori_x = msg.pose.pose.orientation.x
        self.ori_y = msg.pose.pose.orientation.y
        self.ori_z = msg.pose.pose.orientation.z
        self.ori_w = msg.pose.pose.orientation.w

        # Extract velocity data
        self.vel_x = msg.twist.twist.linear.x
        self.vel_y = msg.twist.twist.linear.y
        self.vel_z = msg.twist.twist.linear.z

        # FOR DEBUGGING: track jumping state if conditions
        self.condition_log = []

        # # Optionally keep angular velocities too
        # self.wx = msg.twist.twist.angular.x
        # self.wy = msg.twist.twist.angular.y
        # self.wz = msg.twist.twist.angular.z

    def gz_imu_acceleration(self, msg: Imu) -> None:
        """
        Extract linear acceleration (m/s²) from sensor_msgs/msg/Imu.
        """
        acc = msg.linear_acceleration

        # Convert acceleration from SI unit to G
        self.ax = acc.x/self.G
        self.ay = acc.y/self.G
        self.az = acc.z/self.G

        # optional: timestamp in seconds (float)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def trajectory_callback(self, msg: Marker) -> None:
        """Handle incoming trajectory messages from /ompl_rrt_star_trajectory"""
        try:
            # Only accept trajectories labeled with id==400
            if msg.action == 0:
                # Ignore any ADD that is not id==400
                if msg.id != 400:
                    self.get_logger().info(f"Ignoring ADD for id={msg.id} (only id=400 used)")
                    return

                # Require more than 1 point to treat it as a trajectory
                if len(msg.points) > 1:
                    # Clear existing trajectory
                    while not self.trajectory_queue.empty():
                        self.trajectory_queue.get()

                    # Build full trajectory list
                    traj_points = [(float(p.x), float(p.y), float(p.z)) for p in msg.points]

                    # Determine robot's current position for selecting a starting waypoint
                    # Prefer Gazebo ground-truth if available; otherwise fall back to firmware pose
                    if self.current_pos is not None:
                        cur_x_fw = float(self.current_pos.x)
                        cur_y_fw = float(self.current_pos.y)
                    else:
                        cur_x_fw = None
                        cur_y_fw = None

                    cur_x = float(self.pos_x) if self.pos_x is not None else cur_x_fw
                    cur_y = float(self.pos_y) if self.pos_y is not None else cur_y_fw

                    # If we have a usable current position, choose the closest waypoint
                    start_index = 0
                    if cur_x is not None and cur_y is not None:
                        min_d = float('inf')
                        for i, (wx, wy, wz) in enumerate(traj_points):
                            d = math.sqrt((wx - cur_x)**2 + (wy - cur_y)**2)
                            if d < min_d:
                                min_d = d
                                start_index = i

                    # Queue trajectory from the chosen start_index onward
                    for waypoint in traj_points[start_index:]:
                        self.trajectory_queue.put(waypoint)

                    # Mark that we've received our first trajectory
                    self.trajectory_received = True

                    # New trajectory is ready; clear pending flag and load first waypoint
                    self.pending_new_traj = False
                    if not self.trajectory_queue.empty():
                        self._load_next_waypoint()
                else:
                    # Ignore single-point goal messages on id==400 as well
                    self.get_logger().info(f"Ignoring single-point goal message (id={msg.id})")

            elif msg.action == 2:  # DELETE action
                # Only clear when id==400
                if msg.id != 400:
                    self.get_logger().info(f"Ignoring DELETE for id={msg.id} (only id=400 used)")
                    return

                # Clear trajectory
                while not self.trajectory_queue.empty():
                    self.trajectory_queue.get()
                # Mark that an updated trajectory is incoming to suppress fallback to last goal
                self.has_active_goal = False
                self.pending_new_traj = True
                self.get_logger().info("Trajectory (id=400) cleared (awaiting new path)")
                
        except Exception as e:
            self.get_logger().error(f"Error in trajectory_callback: {e}")
    
    def _load_next_waypoint(self):
        """Load the next waypoint from the queue as the current goal"""
        if not self.trajectory_queue.empty():
            waypoint = self.trajectory_queue.get()
            # Ignore the z component from trajectory; enforce desired_z=0.8
            wx, wy, _wz = waypoint
            self.desired_x = wx
            self.desired_y = wy
            self.desired_z = 0.8
            self.desired_yaw = 0.0  # Keep yaw at 0 as requested
            self.has_active_goal = True
            # Store this as the last waypoint position
            self.last_waypoint_position = (self.desired_x, self.desired_y, 0.8)
            self.get_logger().info(f"New goal: x={self.desired_x:.3f}, y={self.desired_y:.3f}, z={self.desired_z:.3f}")
        else:
            # No more waypoints - hold the last position if we had one
            if self.last_waypoint_position is not None:
                self.desired_x, self.desired_y, _z = self.last_waypoint_position
                self.desired_z = 0.8
                self.desired_yaw = 0.0
                self.get_logger().info(f"All waypoints completed - holding last position: x={self.desired_x:.3f}, y={self.desired_y:.3f}, z={self.desired_z:.3f}")
            self.has_active_goal = False
            self.get_logger().info("All waypoints completed")

        # # debug print (disable with ROS log level if too chatty)
        # self.get_logger().debug(
        #     f"[{t:.3f}] IMU acc: x={self.ax:.3f}, y={self.ay:.3f}, z={self.az:.3f}"
        # )
        
    def motorPower(self, msg):
        """
        Store the four PWM values that arrive in the same order you
        declared them in crazyflies.yaml:
            - motor.m1
            - motor.m2
            - motor.m3
            - motor.m4

        LogDataGeneric has only three fields:
            * header            (std_msgs/Header)
            * timestamp (uint32)  ─ raw STM32 tick [ms]
            * values    (float32[])

        See message spec: https://docs.ros.org/.../LogDataGeneric.html :contentReference[oaicite:0]{index=0}
        """
        # 1. Guard‑check – you asked for 4 vars, so we expect 4 floats
        if len(msg.values) < 4:
            self.get_logger().warn(
                f"motorPower(): expected 4 values, got {len(msg.values)}"
            )
            return

        # 2. Unpack in the declared order
        self.motor_m1, self.motor_m2, self.motor_m3, self.motor_m4 = msg.values[:4]
    
    def robot_pose(self, msg):
        print("At robot_pose()")

        self.current_pos = msg.pose.position
        self.current_ori = msg.pose.orientation
        # self.get_logger().info(
        #     f"///LOGGING INFO FROM robot_pose()/// Position: x={self.current_pos.x}, y={self.current_pos.y}, z={self.current_pos.z} | "
        #     f"Orientation (quaternion): x={self.current_ori.x}, y={self.current_ori.y}, z={self.current_ori.z}, w={self.current_ori.w}"
        # )
    
    def saturation(self, x, max_x, min_x):
        if x > max_x:
            return max_x
        elif x < min_x:
            return min_x
        else:
            return x

    def RPYT_commands(self):
        print("At RPYT_commands")
        
        # Check if Pose information is available
        if self.current_pos is None or self.current_ori is None:
            self.get_logger().warn("Pose not received yet.")
            return

        # Protect against a 0‑norm quaternion that appears in the very first ground‑truth message
        gzgt_quat = [self.ori_x, self.ori_y, self.ori_z, self.ori_w]
        if np.allclose(gzgt_quat, 0):
            self.get_logger().warn(
                "Received zero‑norm quaternion from Gazebo; skipping frame")
            return
        # -------------------------------------------------------------
        
        # control loop freq
        Abs_time = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        # Trajectory-based goal management - only respond to trajectories, ignore other goals
        if not self.trajectory_received:
            # No trajectory received yet - keep robot stationary at current position
            if self.desired_x is None:  # First time initialization
                self.desired_x, self.desired_y = self.pos_x, self.pos_y
                self.desired_z = 0.8
                self.desired_yaw = 0.0
        else:
            if self.last_waypoint_position is not None:
                self.desired_x, self.desired_y, _z = self.last_waypoint_position
                self.desired_z = 0.8
                self.desired_yaw = 0.0
        
        self.desired_x_dot, self.desired_y_dot, self.desired_z_dot = 0, 0, 0

        # Flight controller
        kp_x, ki_x, kd_x, constant_x = 5, 0, 10, 0 # 5, 0, 15, 0
        kp_y, ki_y, kd_y, constant_y = 5, 0, 10, 0 # 5, 0, 15, 0
        kp_z, ki_z, kd_z, constant_z = 1000, 0, 2000, 35000 # 1000, 0, 3000, 38790
        PID_x_flight = PidControlRaw(kp_x, ki_x, kd_x, constant_x, 0, 0) #before: 20, 0, 5, 0, 0, 0
        PID_y_flight = PidControlRaw(kp_y, ki_y, kd_y, constant_y, 0, 0)
        PID_z_flight = PidControlRaw(kp_z, ki_z, kd_z, constant_z, 0, 0) #32200 is hover thrust for 580.81 g
        roll_flight, pitch_flight, yaw_flight = 0, 0, 0,
        pitch_bias_flight, roll_bias_flight = 0, 0
        thrust_flight = 0

        ## Send goals to PID controller, update_reference(desired_x, desired_x_dot)
        PID_x_flight.update_reference(self.desired_x, self.desired_x_dot)
        PID_y_flight.update_reference(self.desired_y, self.desired_y_dot)
        PID_z_flight.update_reference(self.desired_z, self.desired_z_dot)

        # Firmware data, converting quat to euler
        quat = [self.current_ori.x, self.current_ori.y, self.current_ori.z, self.current_ori.w]
        robot_euler = Rotation.from_quat(quat).as_euler('zyx', degrees=False) # in radians

        # Gazebo ground truth data, converting quat to euler
        gzgt_quat = [self.ori_x, self.ori_y, self.ori_z, self.ori_w]
        gzgt_robot_euler = Rotation.from_quat(gzgt_quat).as_euler('zyx', degrees=False) # in radians

        ## Filter and differentiator
        X_f = self.Filter_x.filter(self.pos_x) # was self.current_pos.x
        Y_f = self.Filter_y.filter(self.pos_y)
        Z_f = self.Filter_z.filter(self.pos_z)

        # Check if current goal is reached and load next waypoint
        if self.has_active_goal:
            # Calculate distance to current goal
            distance_to_goal = math.sqrt(
                (X_f - self.desired_x)**2 + 
                (Y_f - self.desired_y)**2 + 
                (Z_f - self.desired_z)**2
            )
            
            # Check if goal is reached within tolerance
            if distance_to_goal < self.current_goal_tolerance:
                self.goal_reached_count += 1
                self.get_logger().info(f"Goal reached! Distance: {distance_to_goal:.3f}m")
                self._load_next_waypoint()

        self.Diff_X.step(self.pos_x, time.time()) # was self.current_pos.x
        self.Diff_Y.step(self.pos_y, time.time())
        self.Diff_Z.step(self.pos_z, time.time())

        # Extract velocities
        self.Diff_X_vel.step(self.pos_x, time.time()) # was self.current_pos.x
        self.Diff_Y_vel.step(self.pos_y, time.time())
        self.Diff_Z_vel.step(self.pos_z, time.time())
        vel_x = self.vel_x
        vel_y = self.vel_y
        vel_z = self.vel_z

        ## Obtain the control outputs, update_error(x, x_dot, abstime), Diff_Z.data_rate stores estimated vertical speed
        u_x = PID_x_flight.update_error(X_f, self.Diff_X.data_rate, Abs_time)
        u_y = PID_y_flight.update_error(Y_f, self.Diff_Y.data_rate, Abs_time)
        u_z = PID_z_flight.update_error(Z_f, self.Diff_Z.data_rate, Abs_time)
        angle_yaw = gzgt_robot_euler[0]

        ## Define the commands
        pitch_flight = (u_x * math.cos(angle_yaw) + u_y * math.sin(angle_yaw))
        roll_flight = -(u_y * math.cos(angle_yaw) - u_x * math.sin(angle_yaw))

        pitch_flight = self.saturation(pitch_flight, 25, -25) + pitch_bias_flight
        roll_flight = self.saturation(roll_flight, 25, -25) + roll_bias_flight
        thrust_flight = round(self.saturation(u_z, 60000, 10000)) # CFLIB thrust scaled between 10K to 60K
        Yaw_error = self.desired_yaw - angle_yaw
        if Yaw_error > math.pi:
            Yaw_error = Yaw_error - 2 * math.pi
        elif Yaw_error < -math.pi:
            Yaw_error = Yaw_error + 2 * math.pi
        yaw_flight = - Yaw_error * 30

        # jummping controller
        if not self.flight_enable and self.controller_start_flag:
            # jumping state tracking
            self.JSTO.step(self.az, self.Diff_Z.data_rate) # pass acceleration in the z direction

            if self.ready_to_drop:
                self.jumping_counter = 0
                self.JSTO.init()
                if self.JSTO.jumping_state_old == 1 and self.JSTO.jumping_state == 2: # first time landing
                    self.ready_to_drop = False
                    self.leg_length = Z_f
                    self.JHC.estimate_height(Abs_time)

            if self.JSTO.jumping_state_old == 2 and self.JSTO.jumping_state == 3:  # takeoff
                self.jumping_counter = self.jumping_counter + 1

            if self.JSTO.jumping_state_old == 1 and self.JSTO.jumping_state == 2:  # landing
                self.LSE.update_landing_location(X_f, Y_f, Abs_time)
                self.JHC.estimate_height(Abs_time)

            # run jumping controller after the apex
            if self.JSTO.jumping_state_old == 3 and self.JSTO.jumping_state == 1:
                jumping_height_record = Z_f - self.leg_length
                if True:
                    falling_time = math.sqrt(2 * jumping_height_record / self.G_flight_time_mocap)
                    landing_speed_z = - self.G_flight_time_mocap * falling_time

                    self.JHC.update_apex_state(Abs_time, self.Diff_X.data_rate, self.Diff_Y.data_rate)
                    self.JSTO.powered_climbing_end_timer = self.JHC.step(self.desired_z)

                    self.LSE.estimation_now(self.vel_x, self.vel_y, falling_time,
                                       self.pos_x, self.pos_y, Abs_time)
                    self.LJC.set_reference(self.desired_x, self.desired_y, jumping_height_record, )
                    self.LJC.update_landing_state(self.vel_x, self.vel_y, landing_speed_z, self.LSE.landing_x, self.LSE.landing_y, )
                    
                    # Plan ballistic trajectory
                    self.LJC.jumping_planning()

                    # Stance phase model - calculate landing attitude and roll/pitch commands
                    self.LJC.inverse_jumping_model(gzgt_robot_euler[0], Abs_time) # calculate landing attitude

        # Determine state and assign the appropriate commands
        if self.controller_start_flag:
            if self.flight_enable:
                self.condition_log.append("flight_enable")
                roll_flight, pitch_flight, self.desired_yaw, thrust_flight = roll_flight, pitch_flight, self.desired_yaw, thrust_flight
            else:
                if self.JSTO.jumping_state == 1:
                    self.condition_log.append("1")
                    roll_flight, pitch_flight, self.desired_yaw, thrust_flight = self.LJC.roll, self.LJC.pitch, self.desired_yaw, 1000 #- power_falling_thrust  # unpowered falling
                elif self.JSTO.jumping_state == 2:
                    self.condition_log.append("2")
                    roll_flight, pitch_flight, self.desired_yaw, thrust_flight = 0, 0, self.desired_yaw,  1000 # stance, unpowered swing
                elif self.JSTO.jumping_state == 3 and self.JSTO.powered_climbing_end_flag:
                    self.condition_log.append("3_true")
                    roll_flight, pitch_flight, self.desired_yaw, thrust_flight = 0, 0, self.desired_yaw,  self.powered_climbing_thrust  # powered climbing
                elif self.JSTO.jumping_state == 3:
                    self.condition_log.append("3_false")
                    roll_flight, pitch_flight, self.desired_yaw, thrust_flight = 0, 0, self.desired_yaw, 1000  # climbing
                else:
                    self.condition_log.append("Not_123")
                    roll_flight, pitch_flight, self.desired_yaw, thrust_flight = 0, 0, self.desired_yaw,  1000  # this cannot be reached
                if self.ready_to_drop:
                    self.condition_log.append("ready_to_drop")
                    roll_flight, pitch_flight, self.desired_yaw, thrust_flight = 0, 0, self.desired_yaw,  1000
        else:
            self.condition_log.append("false")
            roll_flight, pitch_flight, self.desired_yaw, thrust_flight = 0, 0, 0, 0

        # Send those commands
        if self.first_loop == True:
            # Initiate by sending 0,0,0,0 first
            self.msg = Twist()
            self.msg.linear.y = self.msg.linear.x = self.msg.angular.z = self.msg.linear.z = 0.0
            self.rpyt.publish(self.msg)
            self.first_loop = False
        else:
            # Put RPYT in Twist
            self.msg.linear.y = float(roll_flight) # was roll_flight
            self.msg.linear.x = float(-pitch_flight) # was pitch_flight
            self.msg.angular.z = float(yaw_flight) # was desired_yaw, Yaw_error
            self.msg.linear.z = float(thrust_flight) # was thrust_flight
        
        # Save acceleration data (units are in G)
        acc_x, acc_y, acc_z = self.ax, self.ay, self.az

        # Log them into a text file
        error_x = self.desired_x - X_f
        error_y = self.desired_y - Y_f
        error_z = self.desired_z - Z_f
        error_x_dot = self.desired_x_dot - self.Diff_X.data_rate
        error_y_dot = self.desired_y_dot - self.Diff_Y.data_rate
        error_z_dot = self.desired_z_dot - self.Diff_Z.data_rate        
        logging.info(
            f"UNIX Time: {time.time()}, "
            f"motor_m1: {self.motor_m1}, motor_m2: {self.motor_m2}, motor_m3: {self.motor_m3}, motor_m4: {self.motor_m4}, "
            f"gzgt_pos_x: {self.pos_x}, gzgt_pos_y: {self.pos_y}, gzgt_pos_z: {self.pos_z}, "
            f"gzgt_current_roll: {gzgt_robot_euler[2]*(180/math.pi)}, gzgt_current_pitch: {-gzgt_robot_euler[1]*(180/math.pi)}, gzgt_current_yaw: {gzgt_robot_euler[0]}, "
            f"self.current_pos.x: {self.current_pos.x}, self.current_pos.y: {self.current_pos.y}, self.current_pos.z: {self.current_pos.z}, "
            f"self.current_ori.x: {self.current_ori.x}, self.current_ori.y: {self.current_ori.y}, self.current_ori.z: {self.current_ori.z}, self.current_ori.w: {self.current_ori.w}, "
            f"current_x: {X_f}, current_y: {Y_f}, current_z: {Z_f}, "
            f"current_roll: {gzgt_robot_euler[2]*(180/math.pi)}, current_pitch: {-gzgt_robot_euler[1]*(180/math.pi)}, current_yaw: {angle_yaw}, " # leave angle_yaw as radian since command (yaw_flight) is also in radians
            f"desired_x: {self.desired_x}, desired_y: {self.desired_y}, desired_z: {self.desired_z}, desired_yaw: {self.desired_yaw}, "
            f"has_active_goal: {self.has_active_goal}, goals_reached: {self.goal_reached_count}, queue_size: {self.trajectory_queue.qsize()}, queue_contents: {list(self.trajectory_queue.queue)}, "
            f"u_x: {u_x}, u_y: {u_y}, u_z: {u_z}, "
            f"kp_x: {kp_y}, error_x: {error_x}, ki_x: {ki_x}, error_x_int: {None}, kd_x: {kd_x}, error_x_dot: {error_x_dot}, constant_x: {constant_x}, "
            f"kp_y: {kp_y}, error_y: {error_y}, ki_y: {ki_y}, error_y_int: {None}, kd_y: {kd_y}, error_y_dot: {error_y_dot}, constant_y: {constant_y}, "
            f"kp_z: {kp_z}, error_z: {error_z}, ki_z: {ki_z}, error_z_int: {None}, kd_z: {kd_z}, error_z_dot: {error_z_dot}, constant_z: {constant_z}, "
            f"vel_x: {vel_x}, vel_y: {vel_y}, vel_z: {vel_z}, "
            f"acc_x: {acc_x}, acc_y: {acc_y}, acc_z: {acc_z}, "
            f"roll_flight: {roll_flight}, pitch_flight: {pitch_flight}, thrust_flight: {thrust_flight}, yaw_error: {Yaw_error}, "
            f"Roll: {self.msg.linear.y}, Pitch: {self.msg.linear.x}, Desired_yaw: {-self.msg.angular.z}, Thrust: {self.msg.linear.z}, "
            f"jumping_state: {self.JSTO.jumping_state}, JSTO.powered_climbing_end_flag: {self.JSTO.powered_climbing_end_flag}, "
            f"condition_log: {self.condition_log[-1]}, "
        )
        # Publish the RPYT commands
        self.rpyt.publish(self.msg)
    
def main():
    print("Running node")
    rclpy.init(args=sys.argv)
    node = hopcopter(uri)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    node.destroy_node()
    rclpy.shutdown()

    # === Run CSV and plotting script after shutdown ===
    # e_to_csv_standardPID.main()


if __name__ == '__main__':
    main()


