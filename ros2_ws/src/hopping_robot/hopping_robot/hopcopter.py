#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import math
import numpy as np

from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, PointStamped, PoseArray
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, Twist, AccelStamped
from crazyflie_interfaces.msg import LogDataGeneric, FullState
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
import sys
from scipy.spatial.transform import Rotation
from hopping_robot.RisLib.standard_pid import PidControlRaw
from hopping_robot.RisLib import IIR2Filter
from hopping_robot.RisLib.cflog import LoggingCore
from functools import partial
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from grid_map_msgs.msg import GridMap
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import csv
import os
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
        self.powered_climbing_thrust = 21000 # might need to tune this, was 15000
        self.ready_to_drop = True

        # Per-hop energy tracking (configurable)
        self.apex_deduction = 0.34128  # subtracted from raw apex z to get hop height
        self.hop_mass = 0.3609         # kg, for PE/KE bookkeeping
        self.hop_g = 9.8               # m/s^2, for PE/KE bookkeeping
        self.hop_number = 0
        self._arc_max_z = None            # running max base z since last touchdown (apex it fell from)
        self._prev_speed = 0.0            # 3D speed one tick ago (pre-impact landing speed)
        self._climb_energy_injection = 0.0  # propeller work injected in the climb after a touchdown
        self._climb_peak_speed = 0.0        # peak 3D speed during that climb (takeoff velocity)
        self._pending_hop_row = None        # row opened at touchdown, finalized at next apex

        # Initiate ROS2 publisher
        self.rpyt = self.create_publisher(Twist, '/cf_1/cmd_vel_legacy', 50)

        # Initiate ROS2 subscribers
        ## This is used to obtain pose from the firmware. Calculated by integrating acceleration twice.
        self.pose = self.create_subscription(PoseStamped, '/cf_1/pose', self.robot_pose, 50)
        self.current_pos = self.current_ori = None
        
        ## This is used to obtain acceleration data from the firmware. Not accurate enough.
        self.imu = self.create_subscription(LogDataGeneric, '/cf_1/imu', self.motorPower, 50)
        self.motor_m1, self.motor_m2, self.motor_m3, self.motor_m4 = 0.0, 0.0, 0.0, 0.0

        ## Motor PWM feedback (motor.m1..m4), logged by crazyswarm2 as 'motorpower'.
        self.motorpower = self.create_subscription(LogDataGeneric, '/cf_1/motorpower', self.motorPower, 50)

        # This is used to obtain acceleration from Gazebo ground truth
        self.gz_imu = self.create_subscription(Imu, '/cf_0/imu', self.gz_imu_acceleration, 50)
        self.ax = self.ay = self.az = 0.0

        # This is used to obtain velocity and position/orientation from Gazebo ground truth
        self.gz_odom = self.create_subscription(Odometry, '/cf_0/odom', self.gz_odom_velocity, 50)
        self.vel_x, self.vel_y, self.vel_z = 0.0, 0.0, 0.0
        self.pos_x, self.pos_y, self.pos_z = 0.0, 0.0, 0.0
        self.ori_x, self.ori_y, self.ori_z, self.ori_w = 0.0, 0.0, 0.0, 0.0

        # Trajectory following variables - using list for O(1) index access (local planner needs this)
        self.waypoint_list = []  # List of (x, y, z) tuples
        self.current_goal_tolerance = 0.4  # meters - configurable tolerance
        self.has_active_goal = False
        self.goal_reached_count = 0
        self.last_waypoint_position = None  # Store last completed waypoint
        self.pending_new_traj = False       # True between DELETE and ADD to avoid jumps
        self.trajectory_received = False    # Only allow control after first trajectory received
        
        # Local planner integration - jumping state tracking for external nodes
        self.prev_jumping_state = 0  # Track state transitions
        self.queue_state_published_this_cycle = False  # Ensure single publish per jump cycle
        
        # Initialize desired positions as None until trajectory is received
        self.desired_x = None
        self.desired_y = None
        # desired_rel_z is the constant hop height above the current ground;
        # desired_z is that height plus the ground elevation under the robot.
        self.desired_rel_z = 0.8
        self.desired_z = 0.8
        self.desired_yaw = 0.0

        # Ground elevation under the robot, read from /elevation_map at its XY;
        # 0.0 until a map arrives (flat-ground fallback keeps desired_z at 0.8).
        self.current_ground_z = 0.0
        self._emap_data = None       # flat float array of the "elevation" layer
        self._emap_res = 0.0
        self._emap_cx = 0.0          # map center (world) x
        self._emap_cy = 0.0          # map center (world) y
        self._emap_nrows = 0         # cells along x
        self._emap_ncols = 0         # cells along y
        self._emap_outer_start = 0
        self._emap_inner_start = 0
        
        # Trajectory start position tracking for follower-gating
        self.trajectory_start_x = 0.0
        self.trajectory_start_y = 0.0
        self.trajectory_start_received = False
        
        # Subscribe to trajectory from OMPL RRT* planner
        self.trajectory_sub = self.create_subscription(
            Marker,
            '/ompl_rrt_star_trajectory',
            self.trajectory_callback,
            10
        )

        # Subscribe to trajectory from the ballistic motion planner (same
        # Marker id=400 contract; only one planner runs at a time)
        self.ballistic_trajectory_sub = self.create_subscription(
            Marker,
            '/ballistic_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Subscribe to trajectory start position from RRT* planner for follower-gating
        self.traj_start_sub = self.create_subscription(
            PoseStamped,
            '/trajectory_start_position',
            self.trajectory_start_callback,
            10
        )
        
        # === Local Planner Integration ===
        # Publisher for jumping state (Int8: 1=falling, 2=stance, 3=climbing)
        self.jumping_state_pub = self.create_publisher(Int8, '/jumping_state', 10)
        
        # Publisher for trajectory queue state (PoseArray: [0]=current_goal, [1]=next_waypoint if exists)
        self.queue_state_pub = self.create_publisher(PoseArray, '/trajectory_queue_state', 10)
        
        # Publisher for visited waypoints (PointStamped: every waypoint the robot targets)
        self.visited_waypoint_pub = self.create_publisher(PointStamped, '/visited_waypoint', 10)
        
        # Subscriber for adjusted waypoint from local planner
        self.adjusted_waypoint_sub = self.create_subscription(
            PointStamped,
            '/local_planner/adjusted_waypoint',
            self.adjusted_waypoint_callback,
            10
        )

        # Subscribe to the 2.5D elevation map (transient_local to catch the
        # latched map published before this node starts)
        emap_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.elevation_map_sub = self.create_subscription(
            GridMap,
            '/elevation_map',
            self.elevation_map_callback,
            emap_qos
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

        # === Landing-point CSV logger ===
        csv_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),  # package dir
            '..', '..', '..', '..', '..', 'data', 'data_hopcopter'
        )
        csv_dir = os.path.normpath(csv_dir)
        # Fallback to a known absolute path if relative resolution fails
        if not os.path.isdir(csv_dir):
            csv_dir = '/home/zweminhtetaung/CrazySim/data/data_hopcopter'
        os.makedirs(csv_dir, exist_ok=True)
        csv_filename = os.path.join(
            csv_dir,
            f"landing_points_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        )
        self.csv_file = open(csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'jump_number',
            'base_x', 'base_y', 'base_z',
            'roll_deg', 'pitch_deg', 'yaw_deg',
            'leg_length',
            'foot_x', 'foot_y', 'foot_z',
        ])
        self.get_logger().info(f'Landing-point CSV: {csv_filename}')

        # === Per-hop energy-tracking CSV logger ===
        hop_dir = '/home/zweminhtetaung/CrazySim/data/data_ballistic_planner/hop_tracking'
        os.makedirs(hop_dir, exist_ok=True)
        hop_csv_filename = os.path.join(
            hop_dir,
            f"hopcopter_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        )
        self.hop_csv_file = open(hop_csv_filename, 'w', newline='')
        self.hop_csv_writer = csv.writer(self.hop_csv_file)
        self.hop_csv_writer.writerow([
            'timestamp', 'hop_number',
            'land_x', 'land_y', 'land_z',
            'roll_deg', 'pitch_deg', 'yaw_deg',
            'previous_apex_h', 'potential_energy_J',
            'landing_velocity_mps', 'landing_ke_J',
            'takeoff_velocity_mps', 'takeoff_ke_J',
            'energy_injection_J',
        ])
        self.get_logger().info(f'Hop-tracking CSV: {hop_csv_filename}')
        
    # ------------------------------------------------------------------
    #  Compute foot contact position in world frame
    # ------------------------------------------------------------------
    def _compute_foot_position(self, pos_x, pos_y, pos_z,
                               ori_x, ori_y, ori_z, ori_w,
                               leg_length):
        """
        Transform the body-frame leg vector (0, 0, -leg_length) into world
        frame using the current orientation quaternion and add to the
        base_link position.

        Returns (foot_x, foot_y, foot_z) in world frame.
        """
        R = Rotation.from_quat([ori_x, ori_y, ori_z, ori_w])
        leg_body = np.array([0.0, 0.0, -leg_length])
        leg_world = R.apply(leg_body)
        foot_x = pos_x + leg_world[0]
        foot_y = pos_y + leg_world[1]
        foot_z = pos_z + leg_world[2]
        return foot_x, foot_y, foot_z

    def _total_prop_force(self):
        """Total upward propeller force [N] from the four motor PWMs, using the
        Gazebo MulticopterMotorModel: omega=0.04076521*pwm+380.8359, F=k*omega^2."""
        k = 28e-8
        total = 0.0
        for pwm in (self.motor_m1, self.motor_m2, self.motor_m3, self.motor_m4):
            if pwm >= 1000:
                omega = 0.04076521 * pwm + 380.8359
                total += k * omega * omega
        return total

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

    def elevation_map_callback(self, msg: GridMap) -> None:
        """Cache the 'elevation' layer of the 2.5D map for ground lookups."""
        try:
            if 'elevation' not in msg.layers:
                return
            layer_idx = list(msg.layers).index('elevation')
            arr = msg.data[layer_idx]
            # grid_map stores column-major: outer dim = columns (y), inner = rows (x)
            if len(arr.layout.dim) >= 2:
                ncols = arr.layout.dim[0].size
                nrows = arr.layout.dim[1].size
            else:
                nrows = int(round(msg.info.length_x / msg.info.resolution))
                ncols = int(round(msg.info.length_y / msg.info.resolution))
            self._emap_res = msg.info.resolution
            self._emap_cx = msg.info.pose.position.x
            self._emap_cy = msg.info.pose.position.y
            self._emap_nrows = nrows
            self._emap_ncols = ncols
            self._emap_outer_start = msg.outer_start_index
            self._emap_inner_start = msg.inner_start_index
            self._emap_data = np.asarray(arr.data, dtype=np.float64)
        except Exception as e:
            self.get_logger().error(f"Error in elevation_map_callback: {e}")

    def _ground_elevation_at(self, x, y):
        """Nearest-cell ground elevation at world (x, y) from the 2.5D map.

        Returns the last known ground z when there is no map, the query falls
        outside it, or the cell is an obstacle (NaN on the wire)."""
        if self._emap_data is None or self._emap_res <= 0.0:
            return self.current_ground_z
        res = self._emap_res
        nrows, ncols = self._emap_nrows, self._emap_ncols
        # grid_map index convention: index 0 sits at the max-x/max-y corner, so
        # increasing row/col index moves toward decreasing world x/y.
        ru = int(round(0.5 * (nrows - 1) - (x - self._emap_cx) / res))
        cu = int(round(0.5 * (ncols - 1) - (y - self._emap_cy) / res))
        if ru < 0 or ru >= nrows or cu < 0 or cu >= ncols:
            return self.current_ground_z
        rb = (ru + self._emap_inner_start) % nrows
        cb = (cu + self._emap_outer_start) % ncols
        z = self._emap_data[cb * nrows + rb]
        if not math.isfinite(z):
            return self.current_ground_z
        return float(z)

    def trajectory_start_callback(self, msg: PoseStamped) -> None:
        """Handle trajectory start position from RRT* planner for follower-gating"""
        try:
            self.trajectory_start_x = msg.pose.position.x
            self.trajectory_start_y = msg.pose.position.y
            self.trajectory_start_received = True
            self.get_logger().info(f"Received trajectory start position: ({self.trajectory_start_x:.3f}, {self.trajectory_start_y:.3f})")
        except Exception as e:
            self.get_logger().error(f"Error in trajectory_start_callback: {e}")

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
                    # Apply follower-gating to filter out "behind" points
                    filtered_points = self.apply_follower_gating(msg.points)
                    
                    # Clear existing trajectory and populate with new waypoints
                    self.waypoint_list.clear()
                    for point in filtered_points:
                        waypoint = (float(point.x), float(point.y), float(point.z))
                        self.waypoint_list.append(waypoint)

                    # Mark that we've received our first trajectory
                    self.trajectory_received = True

                    # New trajectory is ready; clear pending flag and load first waypoint
                    self.pending_new_traj = False
                    if len(self.waypoint_list) > 0:
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
                self.waypoint_list.clear()
                # Mark that an updated trajectory is incoming to suppress fallback to last goal
                self.has_active_goal = False
                self.pending_new_traj = True
                self.get_logger().info("Trajectory (id=400) cleared (awaiting new path)")
                
        except Exception as e:
            self.get_logger().error(f"Error in trajectory_callback: {e}")
    
    def apply_follower_gating(self, trajectory_points):
        """Filter trajectory points to remove those behind current robot position"""
        if not self.trajectory_start_received or len(trajectory_points) < 2:
            return trajectory_points
            
        # Calculate trajectory direction vector (from start toward goal)
        start_point = trajectory_points[0] 
        end_point = trajectory_points[-1]
        traj_direction_x = end_point.x - start_point.x
        traj_direction_y = end_point.y - start_point.y
        
        # Normalize direction vector
        traj_length = math.sqrt(traj_direction_x**2 + traj_direction_y**2)
        if traj_length < 1e-6:  # Avoid division by zero
            return trajectory_points
            
        traj_dir_norm_x = traj_direction_x / traj_length
        traj_dir_norm_y = traj_direction_y / traj_length
        
        # Current robot position when trajectory is ready
        current_robot_x = self.pos_x
        current_robot_y = self.pos_y
        
        filtered_points = []
        for point in trajectory_points:
            # Vector from current robot position to this trajectory point
            to_point_x = point.x - current_robot_x
            to_point_y = point.y - current_robot_y
            
            # Dot product with trajectory direction (positive = ahead, negative = behind)
            dot_product = to_point_x * traj_dir_norm_x + to_point_y * traj_dir_norm_y
            
            # Keep points that are ahead of current robot position (with small tolerance)
            if dot_product >= -0.1:  # Small negative tolerance for numerical stability
                filtered_points.append(point)
        
        # Ensure we always have at least one point
        if not filtered_points and trajectory_points:
            filtered_points = [trajectory_points[0]]
            
        self.get_logger().info(f"Follower-gating: filtered {len(trajectory_points)} -> {len(filtered_points)} points")
        return filtered_points
    
    def _load_next_waypoint(self):
        """Load the next waypoint from the list as the current goal (pops index 0)"""
        if len(self.waypoint_list) > 0:
            waypoint = self.waypoint_list.pop(0)  # Remove and return first element
            # Goals are XY-only; the waypoint's z is ignored (desired_z tracks
            # the ground under the robot, computed each control tick).
            wx, wy, wz = waypoint
            self.desired_x = wx
            self.desired_y = wy
            self.desired_yaw = 0.0  # Keep yaw at 0 as requested
            self.has_active_goal = True
            # Store this as the last waypoint position (XY only)
            self.last_waypoint_position = (self.desired_x, self.desired_y)
            self.get_logger().info(f"New goal: x={self.desired_x:.3f}, y={self.desired_y:.3f}")
            
            # Publish visited waypoint so local planner can log it to CSV
            visited_msg = PointStamped()
            visited_msg.header.stamp = self.get_clock().now().to_msg()
            visited_msg.header.frame_id = 'world'
            visited_msg.point.x = float(wx)
            visited_msg.point.y = float(wy)
            visited_msg.point.z = float(wz)
            self.visited_waypoint_pub.publish(visited_msg)
        else:
            # No more waypoints - hold the last position if we had one
            if self.last_waypoint_position is not None:
                self.desired_x, self.desired_y = self.last_waypoint_position
                self.desired_yaw = 0.0
                self.get_logger().info(f"All waypoints completed - holding last position: x={self.desired_x:.3f}, y={self.desired_y:.3f}")
            self.has_active_goal = False
            self.get_logger().info("All waypoints completed")
    
    def adjusted_waypoint_callback(self, msg: PointStamped) -> None:
        """
        Handle adjusted waypoint from local planner.
        The waypoint index to modify is encoded in header.stamp.sec (index 0, 1, etc.)
        """
        try:
            receive_time = time.time()
            waypoint_index = msg.header.stamp.sec  # Index of waypoint to adjust
            
            if waypoint_index < len(self.waypoint_list):
                old_wp = self.waypoint_list[waypoint_index]
                new_wp = (msg.point.x, msg.point.y, old_wp[2])  # Keep original z
                self.waypoint_list[waypoint_index] = new_wp
                
                # DEBUG: timing info
                if hasattr(self, 'state3_start_time'):
                    time_since_state3 = (receive_time - self.state3_start_time) * 1000
                    self.get_logger().info(
                        f"[TIMING] Adjusted waypoint received at {receive_time:.6f} ({time_since_state3:.1f}ms after state3 start) | "
                        f"waypoint[{waypoint_index}]: ({old_wp[0]:.3f}, {old_wp[1]:.3f}) -> ({new_wp[0]:.3f}, {new_wp[1]:.3f})"
                    )
                else:
                    self.get_logger().info(
                        f"Local planner adjusted waypoint[{waypoint_index}]: "
                        f"({old_wp[0]:.3f}, {old_wp[1]:.3f}) -> ({new_wp[0]:.3f}, {new_wp[1]:.3f})"
                    )
            else:
                self.get_logger().warn(
                    f"Local planner tried to adjust waypoint[{waypoint_index}] but list only has {len(self.waypoint_list)} items"
                )
        except Exception as e:
            self.get_logger().error(f"Error in adjusted_waypoint_callback: {e}")
    
    def _publish_queue_state(self):
        """
        Publish current trajectory queue state for local planner.
        PoseArray format:
          - poses[0]: Current goal (desired_x, desired_y, desired_z) - the waypoint robot is heading toward
          - poses[1]: Next waypoint (waypoint_list[0]) if exists - the one local planner should adjust
        Position encodes (x, y, z), orientation.w encodes 1.0 if valid, 0.0 if not.
        """
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Pose 0: Current goal (what robot is heading toward now)
        current_goal = Pose()
        if self.desired_x is not None and self.desired_y is not None:
            current_goal.position.x = float(self.desired_x)
            current_goal.position.y = float(self.desired_y)
            current_goal.position.z = float(self.desired_z)
            current_goal.orientation.w = 1.0  # Valid flag
        else:
            current_goal.orientation.w = 0.0  # Invalid flag
        msg.poses.append(current_goal)
        
        # Pose 1: Next waypoint (the one local planner should adjust)
        next_waypoint = Pose()
        if len(self.waypoint_list) > 0:
            wp = self.waypoint_list[0]  # First item in list is next waypoint
            next_waypoint.position.x = float(wp[0])
            next_waypoint.position.y = float(wp[1])
            next_waypoint.position.z = float(wp[2])
            next_waypoint.orientation.w = 1.0  # Valid flag
        else:
            next_waypoint.orientation.w = 0.0  # Invalid flag - no next waypoint
        msg.poses.append(next_waypoint)
        
        self.queue_state_pub.publish(msg)
        self.get_logger().debug(
            f"Published queue state: current_goal=({current_goal.position.x:.2f}, {current_goal.position.y:.2f}), "
            f"next_wp_valid={next_waypoint.orientation.w > 0.5}"
        )

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
                self.desired_yaw = 0.0
        else:
            if self.last_waypoint_position is not None:
                self.desired_x, self.desired_y = self.last_waypoint_position
                self.desired_yaw = 0.0

        # Altitude target = constant hop height above the ground the robot is
        # currently standing on (2.5D elevation map lookup at its XY).
        self.current_ground_z = self._ground_elevation_at(self.pos_x, self.pos_y)
        self.desired_z = self.current_ground_z + self.desired_rel_z

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
                (Y_f - self.desired_y)**2
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

            # Per-hop tracking: apex is the running max base z between touchdowns
            if self._arc_max_z is None or self.pos_z > self._arc_max_z:
                self._arc_max_z = self.pos_z
            # Accumulate propeller energy + peak speed during the powered climb
            if self.JSTO.jumping_state == 3:
                self._climb_energy_injection += self._total_prop_force() * self.vel_z * self.sample_time
                speed_3d = math.sqrt(self.vel_x**2 + self.vel_y**2 + self.vel_z**2)
                if speed_3d > self._climb_peak_speed:
                    self._climb_peak_speed = speed_3d
            
            # === Local Planner Integration: Publish jumping state and queue state ===
            # Publish current jumping state for local planner
            jumping_state_msg = Int8()
            jumping_state_msg.data = self.JSTO.jumping_state
            self.jumping_state_pub.publish(jumping_state_msg)
            
            # Detect transition TO state 3 (takeoff) - publish queue state once
            if self.prev_jumping_state != 3 and self.JSTO.jumping_state == 3:
                self.queue_state_published_this_cycle = False  # Reset flag for new jump cycle
                self.state3_start_time = time.time()  # DEBUG: record state 3 start time
                self.get_logger().info(f"[TIMING] State 3 START at {self.state3_start_time:.6f}")
            
            # Detect transition FROM state 3 TO state 1 (apex reached, start falling)
            if self.prev_jumping_state == 3 and self.JSTO.jumping_state == 1:
                state1_start_time = time.time()
                if hasattr(self, 'state3_start_time'):
                    state3_duration = state1_start_time - self.state3_start_time
                    self.get_logger().info(f"[TIMING] State 1 START at {state1_start_time:.6f} (state3 lasted {state3_duration*1000:.1f}ms)")
            
            # Publish queue state once at the START of state 3
            if self.JSTO.jumping_state == 3 and not self.queue_state_published_this_cycle:
                queue_pub_time = time.time()
                self._publish_queue_state()
                self.queue_state_published_this_cycle = True
                if hasattr(self, 'state3_start_time'):
                    delay_from_state3 = (queue_pub_time - self.state3_start_time) * 1000
                    self.get_logger().info(f"[TIMING] Queue state published at {queue_pub_time:.6f} ({delay_from_state3:.1f}ms after state3 start)")
            
            # Track state for transition detection
            self.prev_jumping_state = self.JSTO.jumping_state
            # === End Local Planner Integration ===

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

                # --- Log landing foot position to CSV ---
                foot_x, foot_y, foot_z = self._compute_foot_position(
                    self.pos_x, self.pos_y, self.pos_z,
                    self.ori_x, self.ori_y, self.ori_z, self.ori_w,
                    self.leg_length,
                )
                euler = Rotation.from_quat(
                    [self.ori_x, self.ori_y, self.ori_z, self.ori_w]
                ).as_euler('zyx', degrees=True)  # [yaw, pitch, roll]
                self.csv_writer.writerow([
                    f'{time.time():.6f}',
                    self.jumping_counter,
                    f'{self.pos_x:.6f}', f'{self.pos_y:.6f}', f'{self.pos_z:.6f}',
                    f'{euler[2]:.4f}', f'{-euler[1]:.4f}', f'{euler[0]:.4f}',
                    f'{self.leg_length:.6f}',
                    f'{foot_x:.6f}', f'{foot_y:.6f}', f'{foot_z:.6f}',
                ])
                self.csv_file.flush()

                # --- Open per-hop tracking row (finalized at the next apex) ---
                self.hop_number += 1
                apex_z = self._arc_max_z if self._arc_max_z is not None else self.pos_z
                previous_apex_h = apex_z - self.apex_deduction
                pe = self.hop_mass * self.hop_g * previous_apex_h
                landing_v = self._prev_speed
                landing_ke = 0.5 * self.hop_mass * landing_v ** 2
                self._pending_hop_row = [
                    f'{time.time():.6f}', self.hop_number,
                    f'{foot_x:.6f}', f'{foot_y:.6f}', f'{foot_z:.6f}',
                    f'{euler[2]:.4f}', f'{-euler[1]:.4f}', f'{euler[0]:.4f}',
                    f'{previous_apex_h:.6f}', f'{pe:.6f}',
                    f'{landing_v:.6f}', f'{landing_ke:.6f}',
                ]
                # Reset accumulators for the arc that starts at this touchdown
                self._arc_max_z = self.pos_z
                self._climb_energy_injection = 0.0
                self._climb_peak_speed = 0.0

            # run jumping controller after the apex
            if self.JSTO.jumping_state_old == 3 and self.JSTO.jumping_state == 1:
                jumping_height_record = Z_f - self.leg_length
                if True:
                    falling_time = math.sqrt(2 * jumping_height_record / self.G_flight_time_mocap)
                    landing_speed_z = - self.G_flight_time_mocap * falling_time

                    self.JHC.update_apex_state(Abs_time, self.Diff_X.data_rate, self.Diff_Y.data_rate)
                    self.JSTO.powered_climbing_end_timer = self.JHC.step(self.desired_rel_z)

                    self.LSE.estimation_now(self.vel_x, self.vel_y, falling_time,
                                       self.pos_x, self.pos_y, Abs_time) # predict landing state (x,y)
                    self.LJC.set_reference(self.desired_x, self.desired_y, jumping_height_record, ) # set desired x and y
                    self.LJC.update_landing_state(self.vel_x, self.vel_y, landing_speed_z, self.LSE.landing_x, self.LSE.landing_y, ) #keep track of current state in real time
                    
                    # Plan ballistic trajectory
                    self.LJC.jumping_planning()

                    # Stance phase model - calculate landing attitude and roll/pitch commands
                    self.LJC.inverse_jumping_model(gzgt_robot_euler[0], Abs_time) # calculate landing attitude

                # --- Finalize per-hop row with this arc's takeoff velocity + injection ---
                if self._pending_hop_row is not None:
                    takeoff_v = self._climb_peak_speed
                    takeoff_ke = 0.5 * self.hop_mass * takeoff_v ** 2
                    self._pending_hop_row.extend([
                        f'{takeoff_v:.6f}', f'{takeoff_ke:.6f}',
                        f'{self._climb_energy_injection:.6f}',
                    ])
                    self.hop_csv_writer.writerow(self._pending_hop_row)
                    self.hop_csv_file.flush()
                    self._pending_hop_row = None

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
        
        # Publish the RPYT commands
        self.rpyt.publish(self.msg)

        # Remember this tick's 3D speed for next tick's pre-impact landing velocity
        self._prev_speed = math.sqrt(self.vel_x**2 + self.vel_y**2 + self.vel_z**2)
    
def main():
    print("Running node")
    rclpy.init(args=sys.argv)
    node = hopcopter(uri)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        # Close the landing-point CSV file
        if hasattr(node, 'csv_file') and not node.csv_file.closed:
            node.csv_file.close()
        # Close the hop-tracking CSV file
        if hasattr(node, 'hop_csv_file') and not node.hop_csv_file.closed:
            node.hop_csv_file.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



