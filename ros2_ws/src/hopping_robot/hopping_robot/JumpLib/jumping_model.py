import numpy as np
import math
import time
from scipy.spatial.transform import Rotation
from collections import deque

class InPlaneJumpingModel:
    def __init__(self, k1, k2):
        # simplest linear jumping model, the landing velocity does not affect the model,
        # see input_theta_l() for details
        self._k1 = k1
        self._k2 = k2

        # theta_l -> from - landing velocity to landing attitude
        # theta_t -> from - landing velocity to takeoff attitude (do not use for now)
        # theta_v -> from - landing velocity to takeoff velocity
        # beta    -> from   landing attitude to takeoff attitude
        self.theta_l = 0.0
        self.theta_t = 0.0
        self.theta_v = 0.0
        self.beta = 0.0
        self.p_dot_ld = 2.0
        self.p_dot_tk = 2.0

    def input_theta_l(self, theta_l, p_dot_ld):
        self.p_dot_ld = p_dot_ld
        self.p_dot_tk = self.p_dot_ld

        self.theta_l = theta_l
        self.theta_t = self._k1 * self.theta_l  # main model
        self.theta_v = self._k2 * self.theta_l
        self.beta = self.theta_t - self.theta_l

    def input_beta(self, beta, p_dot_ld):
        self.beta = beta
        self.input_theta_l(self.beta / (self._k1 - 1), p_dot_ld)

    def input_theta_v(self, theta_v, p_dot_ld):
        self.theta_v = theta_v
        self.input_theta_l(self.theta_v / self._k2, p_dot_ld)

    def input_theta_t(self, theta_t, p_dot_ld):
        self.theta_t = theta_t
        self.input_theta_l(self.theta_t / self._k1, p_dot_ld)


class JumpingModel3D:
    def __init__(self, model):
        self.model = model
        self.tol = 0.0001

    def fcn1(self, v_landing, v_takeoff_dir):
        # V_landing = np.array([x_dot, y_dot, z_dot])
        # V_takeoff_dir = np.array([x_dot, y_dot, z_dot])

        p_dot_ld = np.linalg.norm(v_landing)

        v_surface = np.cross(- v_landing, v_takeoff_dir)
        v_surface_norm = np.linalg.norm(v_surface)
        if v_surface_norm < self.tol:
            v_surface = np.array([1, 0, 0])
        else:
            v_surface = v_surface / v_surface_norm
        theta_v = math.atan2(v_surface_norm, np.dot(- v_landing, v_takeoff_dir))  # theta_v

        self.model.input_theta_v(theta_v, p_dot_ld)
        r = Rotation.from_rotvec(self.model.theta_l * v_surface)

        landing_attitude_z_b = r.apply(-v_landing) / np.linalg.norm(p_dot_ld)
        v_takeoff = v_takeoff_dir * self.model.p_dot_tk

        return landing_attitude_z_b, v_takeoff

# new jumping planner with large jump ability
class LinearJumpingController:
    def __init__(self, high_ballistic_enable = True, model=None, g=9.81, velocity_gain = 0.85, normal_gain = 0.9):
        # parameters
        self.high_ballistic = high_ballistic_enable
        self.takeoff_angle = math.pi/4
        self.remove_velocity_gain_flag = False
        self.velocity_gain = velocity_gain
        self.normal_gain = normal_gain

        self.g = g

        if model is None:
            self.model_2d = InPlaneJumpingModel(1.5, 2)
        else:
            self.model_2d = model
        self.JM3D = JumpingModel3D(self.model_2d)

        # variables
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.jumping_altitude = 0.5
        self.landing_velocity = np.array([0, 0, -2])
        self.takeoff_velocity = np.array([0, 0, 2])
        self.landing_x = 0.0
        self.landing_y = 0.0

        self.roll = 0.0
        self.pitch = 0.0

        # logging
        self.logging_list = [
            'LJC_takeoff_angle', 'LJC_desired_x', 'LJC_desired_y',
            'LJC_landing_x_dot_estimated', 'LJC_landing_y_dot_estimated', 'LJC_landing_z_dot_estimated', ]
        self.logging_data = [0.0] * len(self.logging_list)
        self.run_time = 0

    def set_reference(self, desired_x, desired_y, jumping_altitude, ):
        self.desired_x = desired_x
        self.desired_y = desired_y
        self.jumping_altitude = jumping_altitude

    def update_landing_state(self, landing_x_dot, landing_y_dot, landing_z_dot,
                             landing_x, landing_y, ):

        self.landing_x_dot_estimated = landing_x_dot
        self.landing_y_dot_estimated = landing_y_dot
        self.landing_z_dot_estimated = landing_z_dot
        self.landing_velocity = np.array([landing_x_dot, landing_y_dot, landing_z_dot])
        self.landing_x = landing_x
        self.landing_y = landing_y

    def remove_velocity_gain(self,):
        self.remove_velocity_gain_flag = True

    def jumping_planning(self, ):
        # norm_d_max = self.jumping_altitude * 2
        norm_d_max = self.jumping_altitude * 2 + (
                    self.landing_x_dot_estimated ** 2 + self.landing_y_dot_estimated ** 2) / self.g

        if self.remove_velocity_gain_flag:
            d_x = self.normal_gain * (self.desired_x - self.landing_x)
            d_y = self.normal_gain * (self.desired_y - self.landing_y)
            self.remove_velocity_gain_flag = False
        else:
            d_x = self.velocity_gain * (self.desired_x - self.landing_x)
            d_y = self.velocity_gain * (self.desired_y - self.landing_y)
        norm_d = math.sqrt(d_x * d_x + d_y * d_y)

        if norm_d > norm_d_max:
            norm_v = math.cos(math.pi / 4)
            vertical_v = norm_v
        else:
            if not self.high_ballistic:
                self.takeoff_angle = 0.5 * math.asin(norm_d / norm_d_max)
            else:
                self.takeoff_angle = 0.5 * (math.pi - math.asin(norm_d / norm_d_max))
            norm_v = math.cos(self.takeoff_angle)
            vertical_v = math.sin(self.takeoff_angle)

        u_x = norm_v * d_x / norm_d
        u_y = norm_v * d_y / norm_d

        self.takeoff_velocity = np.array([u_x, u_y, vertical_v])

        self.takeoff_velocity = self.takeoff_velocity / np.linalg.norm(self.takeoff_velocity)


    def inverse_jumping_model(self, yaw, Abs_time):
        # landing attitude vector
        landing_attitude_z_b, _ = self.JM3D.fcn1(self.landing_velocity, self.takeoff_velocity)

        # Adjust for yaw. Transform the landing attitude into the body frame.
        r = Rotation.from_rotvec(yaw * np.array([0, 0, -1])) # was -1
        v_landing_desired_body = r.apply(landing_attitude_z_b)
        self.roll = - math.asin(v_landing_desired_body[1]) # was -
        self.pitch = math.asin(v_landing_desired_body[0] / math.cos(self.roll)) * 180 / math.pi
        self.roll = self.roll * 180 / math.pi

        self.logging_data = [
            self.takeoff_angle, self.desired_x, self.desired_y,
            self.landing_x_dot_estimated, self.landing_y_dot_estimated, self.landing_z_dot_estimated]
        return landing_attitude_z_b


class LandingStateEstimator:
    def __init__(self, ):
        self.previous_landing_x = 0.0
        self.previous_landing_y = 0.0
        self.previous_landing_dx = 0
        self.previous_landing_dy = 0


        self.landing_x = 0.0
        self.landing_y = 0.0
        self.previous_landing_time = 0.0
        self.landing_time_estimated = self.previous_landing_time

        # logging
        self.logging_list = ['LSE_landing_x', 'LSE_landing_y', 'LSE_landing_time_estimated', 'LSE_previous_landing_time']
        self.logging_data = [0.0] * len(self.logging_list)

    def update_landing_location(self, x, y, Abs_time):
        self.previous_landing_x = x
        self.previous_landing_y = y
        self.previous_landing_time = Abs_time

    def update_landing_location_self(self, Abs_time):
        self.previous_landing_x = self.landing_x
        self.previous_landing_y = self.landing_y
        self.previous_landing_time = Abs_time

    def estimation(self, x_dot, y_dot, flight_time):
        landing_x_dot = x_dot
        landing_y_dot = y_dot
        self.landing_x = self.previous_landing_x + landing_x_dot * flight_time
        self.landing_y = self.previous_landing_y + landing_y_dot * flight_time
        self.landing_time_estimated = self.previous_landing_time + flight_time
        # logging
        self.logging_data = [self.landing_x, self.landing_y, self.landing_time_estimated, self.previous_landing_time]

    def estimation_now(self, x_dot, y_dot, falling_time, x, y, t):
        landing_x_dot = x_dot
        landing_y_dot = y_dot
        self.landing_x = x + landing_x_dot * falling_time
        self.landing_y = y + landing_y_dot * falling_time
        self.landing_time_estimated = t + falling_time
        # logging
        self.logging_data = [self.landing_x, self.landing_y, self.landing_time_estimated, self.previous_landing_time]

