"""Powered climb control (stage 2 of docs/follow_planner_plan.md).

One push per real stance, at a fixed thrust, cut off as soon as the coast that follows will reach
the target apex. The cut is closed-loop on the measured height and vertical speed, with a lead for
the thrust that still arrives after the cut (motor spin-down, latency, half a control tick)."""

# Gazebo MulticopterMotorModel (model.sdf.jinja): omega = 0.04076521 * pwm + 380.8359, zero below
# PWM 1000; thrust = MOTOR_K * omega^2 per motor
MOTOR_K = 28e-8


def push_accel(pwm, mass, motors=4):
    """Specific thrust [m/s^2] of all motors at one PWM."""
    if pwm < 1000:
        return 0.0
    omega = 0.04076521 * pwm + 380.8359
    return motors * MOTOR_K * omega * omega / mass


class Climb:
    """One climb's push, for the apex log. end says why the push stopped:
    'target' (the coast will reach the target), 'leg' (the leg alone reaches it, so no push),
    'cap' (t_max), 'apex' (held to the apex) or 'stance' (a new stance before the apex)."""

    def __init__(self, t_start, target, z_lo):
        self.t_start = t_start
        self.target = target
        self.z_lo = z_lo
        self.t_end = None
        self.end = None
        # state at the tick that stopped the push: height above z_lo, vertical speed, cos(tilt)
        self.cut_h = self.cut_v = self.cut_cos = None

    @property
    def duration(self):
        return None if self.t_end is None else self.t_end - self.t_start


class PushController:
    def __init__(self, accel, z_lo, g_coast=9.55, t_lead=0.03, t_max=0.4, v_min=1.0,
                 stance_accel=5 * 9.80665, stance_time=0.01):
        self.accel = accel                # [m/s^2] specific thrust of the push
        self.z_lo = z_lo                  # [m] CoM height at liftoff; apex heights are above it
        self.g_coast = g_coast            # [m/s^2] effective gravity of the idle coast after the cut
        self.t_lead = t_lead              # [s] thrust still arriving after the cut, plus half a tick
        self.t_max = t_max                # [s] cap on one push
        self.v_min = v_min                # [m/s] arm only when climbing at least this fast
        self.stance_accel = stance_accel  # [m/s^2] a real stance holds the IMU above this...
        self.stance_time = stance_time    # [s] ...for this long
        self.stance_seen = False          # a real stance since the last push was armed
        self._above_since = None          # IMU stamp at which acc_z rose above stance_accel
        self.pushing = False
        self.climb = None                 # the latest climb's Climb, until pop_climb()

    def imu(self, acc_z, stamp):
        """Feed every IMU sample: the z specific force [m/s^2] and its stamp [s]."""
        if acc_z <= self.stance_accel:
            self._above_since = None
            return
        if self._above_since is None:
            self._above_since = stamp
        if stamp - self._above_since >= self.stance_time - 1e-6:
            self.stance_seen = True

    def apex_height(self, z, v, cos_tilt):
        """Apex height above z_lo if the push is cut now: the coast from height z at vertical speed
        v, plus the thrust that still arrives over t_lead."""
        v = max(v, 0.0)
        return (z - self.z_lo + v * v / (2.0 * self.g_coast)
                + v * self.accel * cos_tilt * self.t_lead / self.g_coast)

    def step(self, t, state, z, v, cos_tilt, target):
        """One control tick. t: sim time [s]; state: JSTO jumping_state (1 fall, 2 stance,
        3 climb); z: CoM height [m]; v: world-frame vertical speed [m/s]; cos_tilt: vertical
        component of the body z axis; target: apex height above z_lo for this climb.
        Returns True to push this tick."""
        if state == 1:
            # Past the apex: the climb is over, and nothing arms a push in flight
            if self.pushing:
                self._stop(t, 'apex', z, v, cos_tilt)
            self.stance_seen = False
            return False
        armed_now = False
        if state == 3 and self.stance_seen and v > self.v_min:
            # First climbing tick after a real stance: arm this climb's only push
            if self.pushing:
                self._stop(t, 'stance', z, v, cos_tilt)
            self.stance_seen = False
            self.climb = Climb(t, target, self.z_lo)
            self.pushing = armed_now = True
        if not self.pushing:
            return False
        if t - self.climb.t_start >= self.t_max:
            self._stop(t, 'cap', z, v, cos_tilt)
        elif self.apex_height(z, v, cos_tilt) >= self.climb.target:
            self._stop(t, 'leg' if armed_now else 'target', z, v, cos_tilt)
        # a stance tick in mid-climb (a false trigger) pauses the push without ending it
        return self.pushing and state == 3

    def _stop(self, t, end, z, v, cos_tilt):
        c = self.climb
        c.t_end, c.end = t, end
        c.cut_h, c.cut_v, c.cut_cos = z - c.z_lo, v, cos_tilt
        self.pushing = False

    def pop_climb(self):
        """The latest finished climb, once; None if no push was armed since the last call."""
        if self.pushing:
            return None
        c, self.climb = self.climb, None
        return c
