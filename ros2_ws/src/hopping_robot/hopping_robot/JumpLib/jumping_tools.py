# bsn functions and classes
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation



def sqrt_safe(a, a_sqrt):
    if a >= 0:
        return math.sqrt(a)
    else:
        return a_sqrt

def saturation(x, max_x, min_x):
    if x > max_x:
        return max_x
    elif x < min_x:
        return min_x
    else:
        return x
    
class JumpingStateTrackerOnboard:
    def __init__(self, acc_z_up_limit=0.6, z_dot_limit=0.1, controller_engage_time=0.1, controller_engage_ratio=0.2,
                 controller_engage_time_auto=True, powered_climbing_end_timer=0.2,  powered_failing_begin_timer=0.1):
        self.acc_z_up_limit = acc_z_up_limit
        self.acc_z_delay = 1
        self.jumping_state = 1
        self.jumping_state_old = 1
        self.takeoff_time = time.time()
        self.landing_time = time.time()
        self.predicted_max_altitude_abs_time = time.time()
        self._predicted_max_altitude_time_flag = False

        # modified by ding, self.aerial_time = 1.0
        self.aerial_time = 1

        # modified by song, use height and dz
        self.z_dot_limit = z_dot_limit
        self.z_dot_delay = 0
        self.apex_time = time.time()

        self.init_flag = False

        # change the controller engage time depending on the jumping period
        self.controller_engage_time_auto = controller_engage_time_auto
        self.controller_engage_ratio = controller_engage_ratio
        self.controller_engage_abs_time = controller_engage_time
        self.controller_engage_timer = time.time()
        self.controller_engage_flag = False

        self.powered_climbing_end_flag = False
        self.powered_climbing_end_abs_time = time.time()
        self.powered_climbing_end_timer = powered_climbing_end_timer

        self.powered_failing_begin_flag = False
        self.powered_failing_begin_abs_time = time.time()
        self.powered_failing_begin_timer = powered_failing_begin_timer

        self.logging_list = [
            'JSTO_jumping_state', 'JSTO_aerial_time', 'JSTO_controller_engage_flag',
            'JSTO_powered_climbing_end_timer', ]
        self.logging_data = [0.0] * len(self.logging_list)

    def step(self, acc_z, z_dot):
        self.jumping_state_old = self.jumping_state

        # falling to stance
        # if self.acc_z_delay < self.acc_z_up_limit < acc_z
        if self.acc_z_delay < self.acc_z_up_limit < acc_z or self.acc_z_delay > - self.acc_z_up_limit > acc_z:
            self.jumping_state = 2
            self.landing_time = time.time()
            self.aerial_time = self.landing_time - self.takeoff_time

            self.aerial_time = 0.6  # modified by ding

        # stance to climbing
        # if acc_z < self.acc_z_up_limit < self.acc_z_delay
        if acc_z < self.acc_z_up_limit < self.acc_z_delay or acc_z > - self.acc_z_up_limit > self.acc_z_delay:
            self.jumping_state = 3
            self.takeoff_time = time.time()
            self.predicted_max_altitude_abs_time = self.takeoff_time + self.aerial_time * 0.5
            self._predicted_max_altitude_time_flag = True

            if self.controller_engage_time_auto:
                self.controller_engage_abs_time = self.aerial_time * self.controller_engage_ratio
            self.controller_engage_timer = self.takeoff_time + self.controller_engage_abs_time
            self.controller_engage_flag = True

            self.powered_climbing_end_abs_time = self.takeoff_time + self.powered_climbing_end_timer
            self.powered_climbing_end_flag = True # This needs to be true for the powered_climbing_thrust to kick in. Turns off after a certain duration

        # climbing to falling (predicted)
        # if time.time() > self.predicted_max_altitude_abs_time and self._predicted_max_altitude_time_flag:
        #     self.jumping_state = 1
        #     self._predicted_max_altitude_time_flag = False
        if self._predicted_max_altitude_time_flag and - self.z_dot_limit ** 2 < self.z_dot_delay * z_dot <= 0 and z_dot < 0 <= self.z_dot_delay:
            self.jumping_state = 1
            self.apex_time = time.time()
            self._predicted_max_altitude_time_flag = False

            self.powered_failing_begin_abs_time = self.apex_time + self.powered_failing_begin_timer
            self.powered_failing_begin_flag = True

        # controller engage
        if time.time() > self.controller_engage_timer and self.controller_engage_flag:
            self.controller_engage_flag = False

        if time.time() > self.powered_climbing_end_abs_time and self.powered_climbing_end_flag:
            self.powered_climbing_end_flag = False

        # if time.time() > self.powered_failing_begin_abs_time and self.powered_failing_begin_flag:
        #     self.powered_failing_begin_flag = False

        self.acc_z_delay = acc_z
        self.z_dot_delay = z_dot
        self.logging_data = [self.jumping_state, self.aerial_time, self.controller_engage_flag,
                             self.powered_climbing_end_timer, ]

    def init(self, ):
        if not self.init_flag:
            self.jumping_state = 1
            self.jumping_state_old = 1
            self.aerial_time = 0.5
            self.init_flag = True


class JumpingHeightController:
    def __init__(self, leg_efficiency=0.8, g=9.81, t_p_low=0.04, t_p_high=0.3, t_i = 0.04, thrust_gain=1):
        self.thrust_gain = thrust_gain
        self.t_p_high = t_p_high
        self.t_p_low = t_p_low
        self.t_i = t_i
        self.g = g
        self.leg_efficiency = leg_efficiency

        self.nominal_height = 0.5
        self.falling_time = 0.5

        self.apex_time = time.time()
        self.apex_x_dot = 0
        self.apex_y_dot = 0
        self.landing_time = time.time()

        self.powered_climbing_time = 0.12

        self.logging_list = ['JHC_nominal_h','JHC_leg_efficiency','JHC_desired_h','JHC_falling_time']
        self.logging_data = [0.0] * len(self.logging_list)
        

    def update_apex_state(self,apex_timestamp,x_dot, y_dot):
        self.apex_time = apex_timestamp
        self.apex_x_dot = x_dot
        self.apex_y_dot = y_dot
        
    def estimate_height(self, landing_timestamp):
        self.landing_time = landing_timestamp
        if self.landing_time <= self.apex_time:
            print("Error: landing time is smaller than apex time")
            self.nominal_height = 0.5
        else:
            self.falling_time = self.landing_time - self.apex_time
            self.nominal_height = (0.5 * self.g * self.falling_time * self.falling_time
                                + 0.5*(self.apex_x_dot**2+self.apex_y_dot**2)/self.g )* self.leg_efficiency
            self.nominal_height = saturation(self.nominal_height, 1.5, 0.01)
    
    def step(self, desired_h):
        
        take_off_speed = sqrt_safe(2 * self.g * self.nominal_height, 2 * self.g * 0.5)

        if desired_h < self.nominal_height:
            self.powered_climbing_time = self.t_p_low
        else:
            self.powered_climbing_time = (desired_h - self.nominal_height)/(take_off_speed * self.thrust_gain)+self.t_i
            self.powered_climbing_time = saturation(self.powered_climbing_time,self.t_p_high,self.t_p_low)

        self.logging_data = [self.nominal_height,self.leg_efficiency, desired_h, self.falling_time]

        return self.powered_climbing_time





