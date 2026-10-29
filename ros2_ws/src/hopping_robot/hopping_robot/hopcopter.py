#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import math
import numpy as np

from collections import deque

import rclpy
from rclpy.node import Node
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
from grid_map_msgs.msg import GridMap
from rcl_interfaces.msg import SetParametersResult
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
from hopping_robot.JumpLib.jumping_model import LinearJumpingController
from hopping_robot.JumpLib.jumping_model import InPlaneJumpingModel
from hopping_robot.JumpLib.waypoint_tracker import WaypointTracker
from hopping_robot.JumpLib.push_controller import PushController, push_accel

uri = uri_helper.uri_from_env(default='udp://0.0.0.0:19850') # radio://0/80/2M/E7E7E7E7E7

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
        # Effective gravity in an unpowered fall at idle thrust (stage 0 measured 9.55 level,
        # 9.0-9.3 steered); predicts the fall time and the landing speed at each apex
        self.G_flight_time_mocap = 9.55
        # CoM height at touchdown with the leg fully extended (stage 0 measured 0.367). It used
        # to be the filtered z at the first touchdown, 0.420, which made LSE predict landings short
        self.leg_length = 0.366
        self.jumping_counter = 0
        self.jumping_height_record = 0.5  # CoM vertical movement distance
        # PWM of the powered climb (D1): 0.81 g, below hover (34265), so a push ends by the apex on
        # its own. It was 21000 (0.48 g), too weak to reach the planned apexes.
        self.powered_climbing_thrust = 30000
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
        self._apex_target_set = None        # target apex set at the latest apex (the next push aims at it)
        self._arc_target_apex = None        # target apex that sized the arc now in flight

        # Warm-up: hop level in place for this many touchdowns after Play (counted by
        # hop_number) before steering toward the goal, so the first steered hop falls
        # from a steady apex
        self.warmup_touchdowns = 2
        self.steering_enabled = False

        # Initiate ROS2 publisher
        self.rpyt = self.create_publisher(Twist, '/cf_1/cmd_vel_legacy', 50)

        # Initiate ROS2 subscribers
        ## This is used to obtain pose from the firmware. Calculated by integrating acceleration twice.
        self.pose = self.create_subscription(PoseStamped, '/cf_1/pose', self.robot_pose, 50)
        self.current_pos = self.current_ori = None
        self.cf_pose_stamp = 0.0  # header stamp of the latest /cf_1/pose (published at 10 Hz)

        ## This is used to obtain acceleration data from the firmware. Not accurate enough.
        self.imu = self.create_subscription(LogDataGeneric, '/cf_1/imu', self.motorPower, 50)
        self.motor_m1, self.motor_m2, self.motor_m3, self.motor_m4 = 0.0, 0.0, 0.0, 0.0
        self.motor_fw_ms = 0  # firmware tick [ms] of the latest motor PWM log packet

        ## Motor PWM feedback (motor.m1..m4), logged by crazyswarm2 as 'motorpower'.
        self.motorpower = self.create_subscription(LogDataGeneric, '/cf_1/motorpower', self.motorPower, 50)

        # This is used to obtain acceleration from Gazebo ground truth
        self.gz_imu = self.create_subscription(Imu, '/cf_0/imu', self.gz_imu_acceleration, 50)
        self.ax = self.ay = self.az = 0.0
        self.imu_az = 0.0      # raw z specific force [m/s^2] (self.az is the same divided by G)
        self.imu_stamp = 0.0   # sim time [s]

        # This is used to obtain velocity and position/orientation from Gazebo ground truth
        self.gz_odom = self.create_subscription(Odometry, '/cf_0/odom', self.gz_odom_velocity, 50)
        self.vel_x, self.vel_y, self.vel_z = 0.0, 0.0, 0.0
        self.pos_x, self.pos_y, self.pos_z = 0.0, 0.0, 0.0
        self.ori_x, self.ori_y, self.ori_z, self.ori_w = 0.0, 0.0, 0.0, 0.0
        # World-frame velocity from differentiating consecutive odom poses on their
        # stamps, logged next to the odom twist to calibrate the twist's frame and lag
        self.odom_stamp = 0.0  # sim time [s]
        self.dvel_x, self.dvel_y, self.dvel_z = 0.0, 0.0, 0.0
        self._odom_prev = None  # (stamp, x, y, z) of the previous odom message

        # Trajectory following: the tracker picks each hop's waypoint at the steering apex,
        # from where that flight will land (one decision per real touchdown)
        self.tracker = WaypointTracker(reach=0.8, max_retries=2)
        self.path_id = None                 # header stamp [s] of the accepted id=400 marker
        self._delete_time = None            # wall time of a DELETE not yet followed by an ADD
        self.delete_timeout = 0.5           # [s] drop the path if no ADD follows a DELETE
        self.min_fall_time = 0.15           # [s] a touchdown sooner after the apex is a false trigger
        self._apex_abs_time = -math.inf     # Abs_time of the latest apex
        self._touchdown_since_decision = True
        self._decision = None               # latest committed Decision
        # Braking tilt guard (D4): never command a landing tilt above this
        self.landing_tilt_max = math.radians(20.0)

        # Initialize desired positions as None until the first tick
        self.desired_x = None
        self.desired_y = None
        # desired_z is the hop apex height above the takeoff ground, taken from
        # the planner's waypoint z (target_apex); desired_rel_z is the default
        # used before a path arrives, during warm-up, and when hopping in place.
        # desired_rel_z is a ROS parameter so an in-place apex sweep can change it
        # at runtime: ros2 param set /hopcopter desired_rel_z 1.0
        self.desired_rel_z = self.declare_parameter('desired_rel_z', 0.8).value
        self.desired_z = self.desired_rel_z
        self.desired_yaw = 0.0
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Ground elevation lookup from /elevation_map (_ground_elevation_at).
        # Currently unused: hop height comes from the planner's target_apex.
        self.current_ground_z = 0.0
        self._emap_data = None       # flat float array of the "elevation" layer
        self._emap_res = 0.0
        self._emap_cx = 0.0          # map center (world) x
        self._emap_cy = 0.0          # map center (world) y
        self._emap_nrows = 0         # cells along x
        self._emap_ncols = 0         # cells along y
        self._emap_outer_start = 0
        self._emap_inner_start = 0

        # Subscribe to trajectory from the ballistic motion planner
        # (Marker id=400 contract)
        self.ballistic_trajectory_sub = self.create_subscription(
            Marker,
            '/ballistic_trajectory',
            self.trajectory_callback,
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
        # JSTO's push timer is unused (self.push times the push); 0 so it can never add one
        self.JSTO = JumpingStateTrackerOnboard(acc_z_up_limit=2.0, z_dot_limit=0.2 ,controller_engage_time_auto=True,
                            controller_engage_time=0.16, powered_climbing_end_timer=0.0, powered_failing_begin_timer=0.15) # was powered_climbing_end_timer=0.12
        self.LSE = LandingStateEstimator()
        # Powered climb: one push per real stance, cut off as soon as the coast will reach the target
        # apex (stage 2). Apex heights are above the liftoff CoM height, leg_length over flat ground.
        # t_lead: thrust still arriving after the cut (21 ms measured), plus half a tick; t_max caps one push.
        self.push = PushController(push_accel(self.powered_climbing_thrust, self.hop_mass), z_lo=self.leg_length,
                                   g_coast=self.G_flight_time_mocap, t_lead=0.026, t_max=0.4, v_min=1.0)
        self._push_on = False
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
            'previous_apex_h', 'target_apex', 'potential_energy_J',
            'landing_velocity_mps', 'landing_ke_J',
            'takeoff_velocity_mps', 'takeoff_ke_J',
            'energy_injection_J',
        ])
        self.get_logger().info(f'Hop-tracking CSV: {hop_csv_filename}')

        # === Per-tick commanded-vs-actual attitude CSV logger ===
        att_dir = '/home/zweminhtetaung/CrazySim/data/data_ballistic_planner/position_attitude_tracking'
        os.makedirs(att_dir, exist_ok=True)
        att_csv_filename = os.path.join(
            att_dir,
            f"attitude_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        )
        self.att_csv_file = open(att_csv_filename, 'w', newline='')
        self.att_csv_writer = csv.writer(self.att_csv_file)
        self.att_csv_writer.writerow([
            'timestamp', 'jumping_state', 'hop_number',
            'des_roll_deg', 'des_pitch_deg', 'des_yaw_deg',
            'act_roll_deg', 'act_pitch_deg', 'act_yaw_deg',
            # Gazebo ground truth: odom pose, velocity differentiated from it, odom twist
            'odom_stamp', 'pos_x', 'pos_y', 'pos_z',
            'diff_vx', 'diff_vy', 'diff_vz',
            'odom_vx', 'odom_vy', 'odom_vz',
            'imu_stamp', 'imu_az',
            # what was commanded and what the motors did
            'thrust_cmd', 'push',
            'motor_m1', 'motor_m2', 'motor_m3', 'motor_m4', 'motor_fw_ms',
            # the firmware's own estimate (/cf_1/pose), same angle conventions as act_*
            'cf_pose_stamp', 'cf_x', 'cf_y', 'cf_z',
            'cf_roll_deg', 'cf_pitch_deg', 'cf_yaw_deg',
        ])
        self._att_rows = 0
        self.get_logger().info(f'Attitude-tracking CSV: {att_csv_filename}')

        # === Per-apex decision CSV logger (one row per 3 -> 1 transition) ===
        dec_csv_filename = os.path.join(
            hop_dir,
            f"apex_decisions_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        )
        self.dec_csv_file = open(dec_csv_filename, 'w', newline='')
        self.dec_csv_writer = csv.writer(self.dec_csv_file)
        self.dec_csv_writer.writerow([
            'timestamp', 'odom_stamp', 'hop_number', 'steering',
            # decided: 1 if this apex made a decision (0: no real touchdown since the last one);
            # land_idx: waypoint this flight was aimed at; target_idx: what the next hop aims at
            'path_id', 'decided', 'mode', 'land_idx', 'target_idx', 'retries',
            'target_x', 'target_y', 'target_apex',
            'apex_x', 'apex_y', 'apex_z',
            'odom_vx', 'odom_vy', 'odom_vz',
            'diff_vx', 'diff_vy', 'diff_vz',
            'fall_h', 'fall_time', 'land_vz',
            'pred_land_x', 'pred_land_y', 'foot_land_x', 'foot_land_y',
            'takeoff_heading_deg', 'takeoff_elev_deg', 'unreachable',
            'cmd_roll_deg', 'cmd_pitch_deg', 'cmd_tilt_deg', 'planned_tilt_deg', 'tilt_capped',
            'theta_v_deg',
            # the climb that just ended: its target and the apex reached, both above the liftoff
            # CoM height; how its push ended and lasted; the state at the tick that ended it
            'climb_target', 'climb_apex', 'push_end', 'push_s', 'cut_h', 'cut_vz', 'cut_cos_tilt',
        ])
        self.get_logger().info(f'Apex-decision CSV: {dec_csv_filename}')

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
            (measured 2026-10-29 with calibrate_hops.py: body frame, ~10 ms window).
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

        # Differentiate the pose on the message stamps (sim time)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._odom_prev is not None:
            dt = stamp - self._odom_prev[0]
            if dt > 0.0:
                self.dvel_x = (self.pos_x - self._odom_prev[1]) / dt
                self.dvel_y = (self.pos_y - self._odom_prev[2]) / dt
                self.dvel_z = (self.pos_z - self._odom_prev[3]) / dt
        self._odom_prev = (stamp, self.pos_x, self.pos_y, self.pos_z)
        self.odom_stamp = stamp

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
        self.imu_az = acc.z

        # timestamp in seconds (float), sim time
        self.imu_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Every 1 kHz sample, so the push controller can tell a real stance from a blip
        self.push.imu(acc.z, self.imu_stamp)

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

    def trajectory_callback(self, msg: Marker) -> None:
        """Handle incoming trajectory messages from /ballistic_trajectory"""
        try:
            # Only accept trajectories labeled with id==400
            if msg.action == 0:
                # Ignore any ADD that is not id==400
                if msg.id != 400:
                    self.get_logger().info(f"Ignoring ADD for id={msg.id} (only id=400 used)")
                    return

                # Require more than 1 point to treat it as a trajectory
                if len(msg.points) > 1:
                    # Identify the path in the decision log by the marker's stamp
                    self.path_id = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    # Each point's z is the planner's target apex for the hop that lands on it.
                    # The tracker syncs to the path at the next steering apex, from where that
                    # flight lands, so points behind the robot need no filtering here.
                    self.tracker.set_path([(p.x, p.y, p.z) for p in msg.points])
                    self._delete_time = None
                    self.get_logger().info(
                        "New path: " + " -> ".join(f"({p.x:.2f}, {p.y:.2f})" for p in msg.points))
                else:
                    # Ignore single-point goal messages on id==400 as well
                    self.get_logger().info(f"Ignoring single-point goal message (id={msg.id})")

            elif msg.action == 2:  # DELETE action
                # Only clear when id==400
                if msg.id != 400:
                    self.get_logger().info(f"Ignoring DELETE for id={msg.id} (only id=400 used)")
                    return

                # The planner sends DELETE right before every new path, so keep flying the
                # current one; RPYT_commands drops it if no ADD follows within delete_timeout
                self._delete_time = time.time()

        except Exception as e:
            self.get_logger().error(f"Error in trajectory_callback: {e}")

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
        self.motor_fw_ms = msg.timestamp

    def robot_pose(self, msg):
        print("At robot_pose()")

        self.current_pos = msg.pose.position
        self.current_ori = msg.pose.orientation
        self.cf_pose_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # self.get_logger().info(
        #     f"///LOGGING INFO FROM robot_pose()/// Position: x={self.current_pos.x}, y={self.current_pos.y}, z={self.current_pos.z} | "
        #     f"Orientation (quaternion): x={self.current_ori.x}, y={self.current_ori.y}, z={self.current_ori.z}, w={self.current_ori.w}"
        # )
    
    def _on_set_parameters(self, params):
        """Apply a runtime change of desired_rel_z (in-place apex sweeps). Hops in place read it
        at every apex, so the new value applies from the next one."""
        for p in params:
            if p.name == 'desired_rel_z':
                if p.value <= 0.0:
                    return SetParametersResult(successful=False, reason='desired_rel_z must be > 0')
                self.desired_rel_z = p.value
                self.get_logger().info(f'desired_rel_z set to {p.value:.3f} m')
        return SetParametersResult(successful=True)

    def _steer(self, aim_x, aim_y, fall_h, landing_speed_z, yaw, abs_time):
        """Plan this fall's landing attitude so that the next takeoff heads for (aim_x, aim_y),
        under the braking tilt guard. Returns the landing attitude (body z axis, world frame)
        and the tilt the unguarded plan needed."""
        self.LJC.set_reference(aim_x, aim_y, fall_h)
        self.LJC.update_landing_state(self.vel_x, self.vel_y, landing_speed_z,
                                      self.LSE.landing_x, self.LSE.landing_y)
        self.LJC.jumping_planning()
        tilt_planned = self.LJC.limit_landing_tilt(self.landing_tilt_max)
        return self.LJC.inverse_jumping_model(yaw, abs_time), tilt_planned

    def _decide_and_steer(self, fall_h, landing_speed_z, yaw, abs_time):
        """At a steering apex: decide what the next hop aims at from where this flight will
        land, then plan the landing attitude toward it. Returns the decision, the landing
        attitude and what the decision log needs."""
        land_before = self.tracker.land
        p_com = (self.LSE.landing_x, self.LSE.landing_y)
        decided = self._touchdown_since_decision or self._decision is None
        if decided:
            d = self.tracker.decide(*p_com)
            z_b, tilt_planned = self._steer(d.aim_x, d.aim_y, fall_h, landing_speed_z, yaw, abs_time)
            # Waypoints are foot positions, and the foot lands away from the CoM along the
            # landing tilt: decide again from the foot, and re-plan if the aim changes
            foot = (p_com[0] - self.leg_length * z_b[0], p_com[1] - self.leg_length * z_b[1])
            d_foot = self.tracker.decide(*foot)
            if (d_foot.mode, d_foot.land, d_foot.aim_x, d_foot.aim_y) != (d.mode, d.land, d.aim_x, d.aim_y):
                z_b, tilt_planned = self._steer(d_foot.aim_x, d_foot.aim_y, fall_h, landing_speed_z,
                                                yaw, abs_time)
            d = d_foot
            prev = self._decision
            self.tracker.apply(d)
            self._decision = d
            self._touchdown_since_decision = False
            if prev is None or (prev.mode, prev.land) != (d.mode, d.land):
                msg = (f"Apex: landing at ({foot[0]:.2f}, {foot[1]:.2f}), {d.mode} -> aim at "
                       f"({d.aim_x:.2f}, {d.aim_y:.2f})"
                       + ("" if d.land is None else f", waypoint {d.land}")
                       + ("" if d.apex is None else f", apex {d.apex:.3f}")
                       + (f" ({d.reason})" if d.reason else ""))
                if d.mode in ('retry', 'hold'):
                    self.get_logger().warn(msg)
                else:
                    self.get_logger().info(msg)
        else:
            # No real touchdown since the last decision (a false trigger): keep its aim
            d = self._decision
            z_b, tilt_planned = self._steer(d.aim_x, d.aim_y, fall_h, landing_speed_z, yaw, abs_time)
            foot = (p_com[0] - self.leg_length * z_b[0], p_com[1] - self.leg_length * z_b[1])
        self.desired_x, self.desired_y = d.aim_x, d.aim_y
        return d, z_b, {'decided': decided, 'land_before': land_before, 'foot': foot,
                        'tilt_planned': tilt_planned}

    def _log_apex_decision(self, target_apex, fall_h, fall_time, land_vz, landing_attitude_z_b,
                           steer_log, climb, climb_apex):
        """One row per apex: the state the controller acted on and what it commanded.
        landing_attitude_z_b and steer_log are None on warm-up apexes, which do not steer;
        climb is None when no push was armed in the climb that just ended."""
        if landing_attitude_z_b is not None:
            d = self._decision
            tv = self.LJC.takeoff_velocity  # unit takeoff direction handed to the stance model
            idx = lambda k: '' if k is None else k  # noqa: E731
            decision = [int(steer_log['decided']), d.mode, idx(steer_log['land_before']),
                        idx(d.land), d.retries]
            foot = [f'{steer_log["foot"][0]:.4f}', f'{steer_log["foot"][1]:.4f}']
            steer = [
                f'{math.degrees(math.atan2(tv[1], tv[0])):.3f}',
                f'{math.degrees(math.asin(self.saturation(tv[2], 1.0, -1.0))):.3f}',
                int(self.LJC.norm_d > self.LJC.norm_d_max),
                f'{self.LJC.roll:.3f}', f'{self.LJC.pitch:.3f}',
                f'{math.degrees(math.acos(self.saturation(landing_attitude_z_b[2], 1.0, -1.0))):.3f}',
                f'{math.degrees(steer_log["tilt_planned"]):.3f}',
                int(steer_log['tilt_planned'] > self.landing_tilt_max),
                f'{math.degrees(self.IPJM.theta_v):.3f}',
            ]
        else:
            decision = ['', 'warmup', '', '', '']
            foot = ['', '']
            steer = ['', '', '', f'{self.LJC.roll:.3f}', f'{self.LJC.pitch:.3f}', '', '', '', '']
        if climb is None:
            push = ['', f'{climb_apex:.4f}', 'unarmed', '', '', '', '']
        else:
            push = [f'{climb.target:.4f}', f'{climb_apex:.4f}', climb.end, f'{climb.duration:.4f}',
                    f'{climb.cut_h:.4f}', f'{climb.cut_v:.4f}', f'{climb.cut_cos:.4f}']
        self.dec_csv_writer.writerow([
            f'{time.time():.6f}', f'{self.odom_stamp:.6f}', self.hop_number, int(self.steering_enabled),
            '' if self.path_id is None else f'{self.path_id:.6f}',
            *decision,
            f'{self.desired_x:.4f}', f'{self.desired_y:.4f}', f'{target_apex:.4f}',
            f'{self.pos_x:.4f}', f'{self.pos_y:.4f}', f'{self.pos_z:.4f}',
            f'{self.vel_x:.4f}', f'{self.vel_y:.4f}', f'{self.vel_z:.4f}',
            f'{self.dvel_x:.4f}', f'{self.dvel_y:.4f}', f'{self.dvel_z:.4f}',
            f'{fall_h:.4f}', f'{fall_time:.4f}', f'{land_vz:.4f}',
            f'{self.LSE.landing_x:.4f}', f'{self.LSE.landing_y:.4f}', *foot,
            *steer,
            *push,
        ])
        self.dec_csv_file.flush()

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

        # Until a path arrives, hop in place where the robot starts
        if self.desired_x is None:  # First time initialization
            self.desired_x, self.desired_y = self.pos_x, self.pos_y
            self.desired_yaw = 0.0
            self.tracker.hold_xy = (self.pos_x, self.pos_y)

        # A DELETE with no new path after it: drop the path and hop in place
        if self._delete_time is not None and time.time() - self._delete_time > self.delete_timeout:
            self._delete_time = None
            self.tracker.clear()
            self.get_logger().warn("Path deleted and no new path followed; hopping in place")

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

            # Powered climb toward this climb's target apex. The differentiated pose is world frame
            # and right from the first tick after liftoff (the odom twist is body frame and reads
            # low there); the tilt only scales the thrust that still arrives after the cut.
            cos_tilt = 1.0 - 2.0 * (self.ori_x ** 2 + self.ori_y ** 2)
            self._push_on = self.push.step(self.odom_stamp, self.JSTO.jumping_state, self.pos_z,
                                           self.dvel_z, cos_tilt, self.desired_z)

            # Per-hop tracking: apex is the running max base z between touchdowns
            if self._arc_max_z is None or self.pos_z > self._arc_max_z:
                self._arc_max_z = self.pos_z
            # Accumulate propeller energy + peak speed during the powered climb
            if self.JSTO.jumping_state == 3:
                self._climb_energy_injection += self._total_prop_force() * self.vel_z * self.sample_time
                speed_3d = math.sqrt(self.vel_x**2 + self.vel_y**2 + self.vel_z**2)
                if speed_3d > self._climb_peak_speed:
                    self._climb_peak_speed = speed_3d

            if self.ready_to_drop:
                self.jumping_counter = 0
                self.JSTO.init()
                if self.JSTO.jumping_state_old == 1 and self.JSTO.jumping_state == 2: # first time landing
                    self.ready_to_drop = False

            if self.JSTO.jumping_state_old == 2 and self.JSTO.jumping_state == 3:  # takeoff
                self.jumping_counter = self.jumping_counter + 1

            # One waypoint decision per real touchdown. A "touchdown" sooner than min_fall_time
            # after the apex is a false trigger; 3 -> 2 is a landing whose apex was missed.
            if self.JSTO.jumping_state == 2 and self.JSTO.jumping_state_old in (1, 3):
                if Abs_time - self._apex_abs_time >= self.min_fall_time:
                    self._touchdown_since_decision = True

            if self.JSTO.jumping_state_old == 1 and self.JSTO.jumping_state == 2:  # landing
                self.LSE.update_landing_location(X_f, Y_f, Abs_time)

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
                    f'{previous_apex_h:.6f}',
                    '' if self._arc_target_apex is None else f'{self._arc_target_apex:.6f}',
                    f'{pe:.6f}',
                    f'{landing_v:.6f}', f'{landing_ke:.6f}',
                ]
                # The arc about to take off was sized at the latest apex
                self._arc_target_apex = self._apex_target_set
                # Reset accumulators for the arc that starts at this touchdown
                self._arc_max_z = self.pos_z
                self._climb_energy_injection = 0.0
                self._climb_peak_speed = 0.0

            # run jumping controller after the apex
            if self.JSTO.jumping_state_old == 3 and self.JSTO.jumping_state == 1:
                if not self.steering_enabled and self.hop_number >= self.warmup_touchdowns:
                    self.steering_enabled = True
                    self.get_logger().info(
                        f"Warm-up done ({self.hop_number} touchdowns); steering toward goal from this apex")

                self._apex_abs_time = Abs_time
                jumping_height_record = Z_f - self.leg_length
                # The climb that just ended: its push, and the apex it reached
                climb = self.push.pop_climb()
                climb_apex = self._arc_max_z - self.push.z_lo
                if True:
                    falling_time = math.sqrt(2 * max(jumping_height_record, 0.0) / self.G_flight_time_mocap)
                    landing_speed_z = - self.G_flight_time_mocap * falling_time

                    self.LSE.estimation_now(self.vel_x, self.vel_y, falling_time,
                                       self.pos_x, self.pos_y, Abs_time) # predict landing state (x,y)
                    landing_attitude_z_b = steer_log = None
                    if self.steering_enabled:
                        # Decide what the next hop aims at from where this flight lands, and plan
                        # the landing attitude (stance model) that turns the fall toward it
                        decision, landing_attitude_z_b, steer_log = self._decide_and_steer(
                            jumping_height_record, landing_speed_z, gzgt_robot_euler[0], Abs_time)
                        # The planned apex for that hop; hops in place use the default height
                        target_apex = decision.apex if decision.apex is not None else self.desired_rel_z
                    else:
                        # Warm-up hop: fall level, in place, at the default height
                        self.LJC.roll = 0.0
                        self.LJC.pitch = 0.0
                        target_apex = self.desired_rel_z

                    # The next climb's push aims at that apex
                    self.desired_z = target_apex
                    self._apex_target_set = target_apex

                    self._log_apex_decision(target_apex, jumping_height_record, falling_time,
                                            landing_speed_z, landing_attitude_z_b, steer_log,
                                            climb, climb_apex)

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
        push_active = False  # powered climb commanded this tick
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
                elif self.JSTO.jumping_state == 3 and self._push_on:
                    self.condition_log.append("3_true")
                    roll_flight, pitch_flight, self.desired_yaw, thrust_flight = 0, 0, self.desired_yaw,  self.powered_climbing_thrust  # powered climbing
                    push_active = True
                elif self.JSTO.jumping_state == 3:
                    self.condition_log.append("3_false")
                    roll_flight, pitch_flight, self.desired_yaw, thrust_flight = 0, 0, self.desired_yaw, 1000  # climbing
                else:
                    self.condition_log.append("Not_123")
                    roll_flight, pitch_flight, self.desired_yaw, thrust_flight = 0, 0, self.desired_yaw,  1000  # this cannot be reached
                if self.ready_to_drop:
                    self.condition_log.append("ready_to_drop")
                    roll_flight, pitch_flight, self.desired_yaw, thrust_flight = 0, 0, self.desired_yaw,  1000
                    push_active = False
        else:
            self.condition_log.append("false")
            roll_flight, pitch_flight, self.desired_yaw, thrust_flight = 0, 0, 0, 0

        # --- Log commanded vs actual attitude (yaw_flight is a rate, so log the yaw angle target) ---
        self.att_csv_writer.writerow([
            f'{time.time():.6f}', self.JSTO.jumping_state, self.hop_number,
            f'{roll_flight:.4f}', f'{pitch_flight:.4f}', f'{math.degrees(self.desired_yaw):.4f}',
            f'{math.degrees(gzgt_robot_euler[2]):.4f}', f'{-math.degrees(gzgt_robot_euler[1]):.4f}',
            f'{math.degrees(gzgt_robot_euler[0]):.4f}',
            f'{self.odom_stamp:.6f}', f'{self.pos_x:.6f}', f'{self.pos_y:.6f}', f'{self.pos_z:.6f}',
            f'{self.dvel_x:.5f}', f'{self.dvel_y:.5f}', f'{self.dvel_z:.5f}',
            f'{self.vel_x:.5f}', f'{self.vel_y:.5f}', f'{self.vel_z:.5f}',
            f'{self.imu_stamp:.6f}', f'{self.imu_az:.4f}',
            f'{thrust_flight:.0f}', int(push_active),
            f'{self.motor_m1:.0f}', f'{self.motor_m2:.0f}', f'{self.motor_m3:.0f}', f'{self.motor_m4:.0f}',
            self.motor_fw_ms,
            f'{self.cf_pose_stamp:.6f}',
            f'{self.current_pos.x:.6f}', f'{self.current_pos.y:.6f}', f'{self.current_pos.z:.6f}',
            f'{math.degrees(robot_euler[2]):.4f}', f'{-math.degrees(robot_euler[1]):.4f}',
            f'{math.degrees(robot_euler[0]):.4f}',
        ])
        self._att_rows += 1
        if self._att_rows % 100 == 0:
            self.att_csv_file.flush()

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
        # Close the attitude-tracking CSV file
        if hasattr(node, 'att_csv_file') and not node.att_csv_file.closed:
            node.att_csv_file.close()
        # Close the apex-decision CSV file
        if hasattr(node, 'dec_csv_file') and not node.dec_csv_file.closed:
            node.dec_csv_file.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



