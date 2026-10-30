"""Takeoff solve at the steering apex (stage 4 of docs/follow_planner_plan.md).

The planner says where each hop lands and how high it peaks, and HopPlan adds the takeoff and
landing it planned. At the apex before each landing the controller re-solves that takeoff from
what the robot is actually doing: the predicted foot touchdown P and the arrival velocity v_in.

- The leg launches at v_leg = sqrt(eta) |v_in| along d_out = (cos a cos psi, cos a sin psi, sin a);
  a is the leg angle alpha_leg, and eta grows with the redirection. The stance model (fcn1) gives
  the landing attitude that turns the arrival into d_out.
- The push after liftoff is level, so it adds only vertical speed and is cut off at the apex H. The
  leg alone supplies the horizontal speed v_h = v_leg cos a.
- The flight time comes from the push model (a finite push climbs slower than a ballistic arc) and
  the g' of each phase: 9.55 for the idle coast, less for a fall steered at a tilt.

Three effects that would otherwise put landings long are part of the same forward model: the stance
moves the CoM forward (the liftoff CoM sits ahead of the touchdown CoM), the push kicks the robot
forward while the body is still tilted after liftoff, and the finite push lengthens the climb.

Plain numpy, so the solve can be unit-tested without ROS."""

import math

import numpy as np

# Tilt after liftoff, over the liftoff tilt, every LIFT_DT from liftoff. The body keeps rotating
# for ~40 ms after a forward liftoff, then levels. Stage 2's four 30000 pushes after a vertical
# arrival (16:59 and 17:06 runs) all follow this within 0.05.
LIFT_TILT = (1.0, 1.11, 1.15, 1.08, 0.93, 0.74, 0.57, 0.45, 0.35, 0.27, 0.20, 0.14, 0.09, 0.06,
             0.02, 0.0)
LIFT_DT = 0.02


def slerp(a, b, s):
    """Unit vector a fraction s of the way from unit vector a to unit vector b."""
    theta = math.acos(max(-1.0, min(1.0, float(np.dot(a, b)))))
    if theta < 1e-9:
        return np.array(a, dtype=float)
    return (math.sin((1 - s) * theta) * np.asarray(a) + math.sin(s * theta) * np.asarray(b)) / math.sin(theta)


def unit(heading, elevation):
    """Unit vector at a heading [rad] from +x and an elevation [rad] above the horizontal."""
    c = math.cos(elevation)
    return np.array([c * math.cos(heading), c * math.sin(heading), math.sin(elevation)])


def tilt_of(axis):
    """Angle [rad] between a body z axis and the vertical."""
    return math.acos(max(-1.0, min(1.0, float(axis[2]))))


class Takeoff:
    """One solved (or forward-simulated) hop from a touchdown at p.

    mode:     'push' (the push tops the leg up to the apex), 'leg' (the leg alone overshoots the
              apex: no push, the steep root), 'capped' (the push cap stops short of the apex),
              'pump' (the leg cannot supply the horizontal speed above alpha_min: hop in place)
    d_out:    the leg's unit takeoff direction; heading, alpha_leg its angles [rad]
    v_leg, v_h: leg takeoff speed and its horizontal part [m/s]
    z_land, z_lift: body z axis at touchdown and at liftoff; tilt: landing tilt [rad];
              theta_v: angle from -v_in to d_out [rad]
    apex:     apex above the liftoff CoM height of flat ground (leg above the takeoff ground) [m]
    t_push, t_up, t_down: push time, liftoff to apex, apex to the next touchdown [s]
    kick_v, kick_x: horizontal speed [m/s] and distance [m] the tilted push adds (2D vectors)
    lift_dx:  liftoff CoM minus touchdown CoM, horizontal (2D) [m]
    land:     the next foot touchdown (2D); v_land: the arrival velocity there (3D)
    v_h_need: for 'pump', the horizontal leg speed the hop needed [m/s]
    """

    def __init__(self, **kw):
        self.__dict__.update(kw)


class HopModel:
    """The hop the robot flies, with the constants it was measured with.

    eta, eta_turn, eta_ramp: stance energy ratio. eta in place and up to the redirection angle
                  eta_ramp[0], rising linearly to eta_turn at eta_ramp[1] [rad]. Stage 4 measured
                  0.765 at theta_v 15-35 deg (48 stances), 0.787 at 45-55 (24) and 0.816 at 55-65
                  (12); stage 0 measured 0.767 in place.
    k1, k2:       stance model: the landing attitude sits theta_v / k2, and the liftoff attitude
                  k1 theta_v / k2, from -v_in toward the takeoff direction. Stage 4 measured k1 1.50
                  and k2 2.46-2.55 (bins of descent angle under 70 deg; straight continuation
                  high, doglegs low); 2.53 landed with the least bias (fcn1's were 1.688 and 2.469).
    k2_vertical, k2_ramp: k2 after a near-vertical arrival (2.70 over 13 stances at 84-90 deg of
                  descent), reached linearly between the descent angles k2_ramp [rad]
    leg:          touchdown CoM height on an upright leg [m]
    g:            gravity the push works against [m/s^2] (the stage 2 push fit used Gazebo's 9.8)
    g_up:         g' of the idle coast after the push [m/s^2]
    g_down, g_down_tilt: g' of a fall steered at tilt t is g_down - g_down_tilt (1 - cos t); stage 0
                  measured 9.55 level, 9.33 at 19 deg and 8.99 at 30 deg
    push_accel:   effective push [m/s^2] (stage 2's fit to 75 pushes at 30000); push_max: cap [s]
    kick_accel:   the push's horizontal kick per sin(liftoff tilt) [m/s^2] (stage 4's fit to 63
                  pushed liftoffs, RMS 0.017 m/s; lower than the push because the motors are
                  still spinning up while the tilt is largest)
    alpha_min:    smallest leg angle the stance can launch [rad]
    """

    def __init__(self, eta=0.77, eta_turn=0.81, eta_ramp=(math.radians(30.0), math.radians(60.0)),
                 k1=1.5, k2=2.53, k2_vertical=2.70,
                 k2_ramp=(math.radians(75.0), math.radians(85.0)), leg=0.366, g=9.8, g_up=9.55,
                 g_down=9.55, g_down_tilt=4.1, push_accel=7.55, push_max=0.4, kick_accel=6.69,
                 alpha_min=math.radians(40.0), lift_tilt=LIFT_TILT, lift_dt=LIFT_DT):
        self.eta, self.eta_turn, self.eta_ramp = eta, eta_turn, eta_ramp
        self.k1, self.k2, self.leg = k1, k2, leg
        self.k2_vertical, self.k2_ramp = k2_vertical, k2_ramp
        self.g, self.g_up, self.g_down, self.g_down_tilt = g, g_up, g_down, g_down_tilt
        self.push_accel, self.push_max, self.alpha_min = push_accel, push_max, alpha_min
        self.kick_accel = kick_accel
        # F(t) = integral of the lift profile from liftoff, and G(t) = integral of F, on a 1 ms grid
        self._t = np.arange(0.0, len(lift_tilt) * lift_dt + push_max + 1e-9, 0.001)
        f = np.interp(self._t, np.arange(len(lift_tilt)) * lift_dt, lift_tilt, right=0.0)
        self._F = np.concatenate(([0.0], np.cumsum(0.5 * (f[1:] + f[:-1]) * 0.001)))
        self._G = np.concatenate(([0.0], np.cumsum(0.5 * (self._F[1:] + self._F[:-1]) * 0.001)))

    # ---- pieces of the forward model ----

    def eta_at(self, theta_v):
        """Stance energy ratio at a redirection angle theta_v [rad] (see the class docstring)."""
        lo, hi = self.eta_ramp
        s = min(max((theta_v - lo) / (hi - lo), 0.0), 1.0)
        return self.eta + s * (self.eta_turn - self.eta)

    def k2_at(self, v_in):
        """k2 for an arrival at v_in: k2 up to the descent angle k2_ramp[0], k2_vertical from
        k2_ramp[1]."""
        beta = math.atan2(-v_in[2], math.hypot(v_in[0], v_in[1]))
        lo, hi = self.k2_ramp
        s = min(max((beta - lo) / (hi - lo), 0.0), 1.0)
        return self.k2 + s * (self.k2_vertical - self.k2)

    def stance(self, v_in, d_out):
        """fcn1: the landing attitude that turns the arrival v_in into the takeoff direction d_out,
        and the attitude at liftoff. Returns (z_land, z_lift, theta_v)."""
        a = -np.asarray(v_in, dtype=float) / np.linalg.norm(v_in)
        theta_v = math.acos(max(-1.0, min(1.0, float(np.dot(a, d_out)))))
        k2 = self.k2_at(v_in)
        return slerp(a, d_out, 1.0 / k2), slerp(a, d_out, self.k1 / k2), theta_v

    def climb(self, v0, h):
        """Climb from liftoff at vertical speed v0 toward an apex h above the liftoff CoM: a level
        push until the coast reaches h, at most push_max. Returns (t_push, t_up, apex): the push
        time, the time from liftoff to the apex, and the apex reached above the liftoff CoM."""
        gc = self.g_up
        if v0 * v0 >= 2.0 * gc * h:
            return 0.0, v0 / gc, v0 * v0 / (2.0 * gc)   # the leg alone gets there: no push
        d = self.g - self.push_accel                    # net deceleration while pushing
        r = 1.0 - d / gc
        disc = v0 * v0 - 2.0 * d * (h - v0 * v0 / (2.0 * gc)) / r
        t_cut = (v0 - math.sqrt(disc)) / d if disc >= 0.0 else math.inf
        t_p = min(t_cut, self.push_max, v0 / d)
        v = v0 - d * t_p
        return t_p, t_p + v / gc, v0 * t_p - 0.5 * d * t_p * t_p + v * v / (2.0 * gc)

    def fall_g(self, tilt):
        """g' of a fall steered toward a landing tilt [rad]."""
        return self.g_down - self.g_down_tilt * (1.0 - math.cos(tilt))

    def kick(self, tilt_lift, t_push, t_end):
        """Horizontal speed the push adds along the liftoff tilt, and the distance that adds by
        t_end after liftoff. The push is commanded level, but the body is still tilted after
        liftoff: the thrust's horizontal part, kick_accel sin(tilt(t)), with tilt(t) the liftoff
        tilt times the lift profile."""
        if t_push <= 0.0 or tilt_lift <= 0.0:
            return 0.0, 0.0
        s = self.kick_accel * math.sin(tilt_lift)
        F = float(np.interp(t_push, self._t, self._F))
        G = float(np.interp(t_push, self._t, self._G))
        return s * F, s * (G + F * (t_end - t_push))

    # ---- the hop ----

    def forward(self, p, v_in, d_out, h, dz=0.0, z_next=(0.0, 0.0, 1.0)):
        """Where a leg takeoff along d_out from a touchdown at p lands.

        p: foot touchdown (x, y); v_in: arrival velocity there (3D, world); d_out: unit leg takeoff
        direction; h: target apex above the liftoff CoM height (the leg above the takeoff ground);
        dz: landing ground minus takeoff ground; z_next: body z axis at the next touchdown."""
        p = np.asarray(p, dtype=float)
        d_out = np.asarray(d_out, dtype=float)
        z_next = np.asarray(z_next, dtype=float)
        L = self.leg
        z_land, z_lift, theta_v = self.stance(v_in, d_out)
        v_leg = math.sqrt(self.eta_at(theta_v)) * float(np.linalg.norm(v_in))
        h0 = L * z_lift[2]                                  # liftoff CoM height above the ground
        v0 = v_leg * d_out[2]
        t_push, t_up, climbed = self.climb(v0, h + L - h0)
        apex = h0 + climbed - L
        if t_push > 0.0 and climbed < h + L - h0 - 1e-6:
            mode = 'capped'
        elif t_push > 0.0:
            mode = 'push'
        else:
            mode = 'leg'
        tilt_next = tilt_of(z_next)
        h_fall = h0 + climbed - dz - L * z_next[2]
        if h_fall <= 0.0:
            return None                                     # the apex is below the next touchdown
        g_fall = self.fall_g(tilt_next)
        t_down = math.sqrt(2.0 * h_fall / g_fall)
        T = t_up + t_down
        u = v_leg * d_out[:2]                               # the leg's horizontal velocity
        e = z_lift[:2]
        s = float(np.linalg.norm(e))
        e = e / s if s > 1e-9 else np.zeros(2)
        kv, kx = self.kick(tilt_of(z_lift), t_push, T)
        lift_dx = L * (z_lift[:2] - z_land[:2])
        com_lo = p + L * z_lift[:2]
        land = com_lo + u * T + kx * e - L * z_next[:2]
        v_land = np.array([u[0] + kv * e[0], u[1] + kv * e[1], -g_fall * t_down])
        return Takeoff(mode=mode, d_out=d_out, heading=math.atan2(d_out[1], d_out[0]),
                       alpha_leg=math.asin(max(-1.0, min(1.0, float(d_out[2])))), v_leg=v_leg,
                       v_h=v_leg * float(np.linalg.norm(d_out[:2])), z_land=z_land, z_lift=z_lift,
                       tilt=tilt_of(z_land), theta_v=theta_v, apex=apex, t_push=t_push, t_up=t_up,
                       t_down=t_down, kick_v=kv * e, kick_x=kx * e, lift_dx=lift_dx, land=land,
                       v_land=v_land, v_h_need=None)

    def _range(self, v_leg, a, hc, h_end, g_fall):
        """Horizontal distance the leg's own speed covers over the flight, at leg angle a: the
        climb toward hc above the liftoff CoM, then the fall to h_end below it."""
        t_push, t_up, climbed = self.climb(v_leg * math.sin(a), hc)
        h_fall = climbed - h_end
        if h_fall <= 0.0:
            return 0.0
        return v_leg * math.cos(a) * (t_up + math.sqrt(2.0 * h_fall / g_fall))

    def solve(self, p, v_in, target, h, dz=0.0, z_next=(0.0, 0.0, 1.0), tol=1e-6):
        """The leg takeoff from a touchdown at p (arriving at v_in) whose next foot touchdown is at
        target, peaking h above the liftoff CoM height (see forward). For a target the leg cannot
        reach above alpha_min, returns mode 'pump' with the horizontal speed it needed."""
        p = np.asarray(p, dtype=float)
        target = np.asarray(target, dtype=float)
        z_next = np.asarray(z_next, dtype=float)
        L = self.leg
        goal_com = target + L * z_next[:2]
        g_fall = self.fall_g(tilt_of(z_next))
        # Start from a straight vertical takeoff and iterate: the liftoff CoM, the push's kick and
        # the leg's speed (eta) depend on the takeoff, the takeoff on what is left for the leg
        psi, alpha = 0.0, 0.5 * math.pi
        f = self.forward(p, v_in, unit(psi, alpha), h, dz, z_next)
        if f is None:
            return self._pump(None, None)        # the apex does not even reach the target ground
        hint = None
        for it in range(60):
            need = goal_com - (p + L * f.z_lift[:2]) - f.kick_x
            X = float(np.linalg.norm(need))
            psi_new = math.atan2(need[1], need[0]) if X > 1e-9 else psi
            h0 = L * f.z_lift[2]
            hc, h_end = h + L - h0, dz + L * z_next[2] - h0
            v_leg = f.v_leg
            alpha_new = self._steep_root(v_leg, X, hc, h_end, g_fall, hint)
            if alpha_new is None:
                # Flatter launches turn the arrival further, and the leg keeps more energy then
                v_leg = max(v_leg, self.forward(p, v_in, unit(psi_new, self.alpha_min), h, dz, z_next).v_leg)
                alpha_new = self._steep_root(v_leg, X, hc, h_end, g_fall)
            if alpha_new is None:
                t = self._range(v_leg, self.alpha_min, hc, h_end, g_fall) / (v_leg * math.cos(self.alpha_min))
                return self._pump(f, X / t if t > 0.0 else None)
            if it > 20:                          # damp a slow oscillation
                alpha_new = 0.5 * (alpha + alpha_new)
                psi_new = psi + 0.5 * math.atan2(math.sin(psi_new - psi), math.cos(psi_new - psi))
            done = (abs(alpha_new - alpha) < tol
                    and abs(math.atan2(math.sin(psi_new - psi), math.cos(psi_new - psi))) < tol)
            psi, alpha, hint = psi_new, alpha_new, alpha_new
            f = self.forward(p, v_in, unit(psi, alpha), h, dz, z_next)
            if f is None:
                return self._pump(None, None)
            if done:
                break
        return f

    @staticmethod
    def _pump(f, v_h_need):
        """A hop the leg cannot fly from here: v_h_need is the horizontal leg speed it needed (None
        when the apex cannot even reach the target's ground)."""
        if f is None:
            f = Takeoff(d_out=None)
        f.mode, f.v_h_need = 'pump', v_h_need
        return f

    def _steep_root(self, v_leg, X, hc, h_end, g_fall, hint=None):
        """The steepest leg angle in [alpha_min, 90 deg] whose range is X, or None if no angle
        there reaches X. The range falls with the angle wherever the push tops the apex up; where
        the leg alone overshoots it, the steep root keeps the apex above the target (the shallow
        one would fly under the planned clearance). hint, a nearby root, narrows the bracket."""
        rng = self._range
        lo = hi = None
        if hint is not None:
            a, b = max(hint - 0.01, self.alpha_min), min(hint + 0.01, 0.5 * math.pi)
            if rng(v_leg, a, hc, h_end, g_fall) >= X > rng(v_leg, b, hc, h_end, g_fall):
                lo, hi = a, b
        if lo is None:
            hi = 0.5 * math.pi
            step = math.radians(2.0)
            lo = hi - step
            while lo > self.alpha_min and rng(v_leg, lo, hc, h_end, g_fall) < X:
                hi, lo = lo, lo - step
            lo = max(lo, self.alpha_min)
            if rng(v_leg, lo, hc, h_end, g_fall) < X:
                return None
        while hi - lo > 1e-9:
            mid = 0.5 * (lo + hi)
            if rng(v_leg, mid, hc, h_end, g_fall) < X:
                hi = mid
            else:
                lo = mid
        return 0.5 * (lo + hi)

    def pump_height(self, v_h):
        """Apex to hop in place to, so that the next (vertical) landing lets the leg supply v_h at
        alpha_min: v_h = sqrt(eta) sqrt(2 g' H) cos(alpha_min)."""
        return v_h * v_h / (2.0 * self.g_down * self.eta * math.cos(self.alpha_min) ** 2)

    def limit_tilt(self, v_in, d_out, tilt_max):
        """Braking tilt guard (stage 1): if d_out needs a landing tilt above tilt_max, rotate it
        toward continuing the arrival until the tilt fits. Returns (d_out, tilt d_out needed)."""
        tilt = tilt_of(self.stance(v_in, d_out)[0])
        if tilt <= tilt_max:
            return np.asarray(d_out, dtype=float), tilt
        v = np.asarray(v_in, dtype=float)
        u_continue = np.array([v[0], v[1], -v[2]]) / np.linalg.norm(v)
        lo, hi = 0.0, 1.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if tilt_of(self.stance(v_in, slerp(d_out, u_continue, mid))[0]) > tilt_max:
                lo = mid
            else:
                hi = mid
        return slerp(d_out, u_continue, hi), tilt


# ---- clearance re-check (the planner's test, on the controller's copy of the map) ----

def dilate(grid, res, radius):
    """Grid of the highest cell within radius of each cell (NaN cells count as +inf). The planner
    dilates only steep cells; dilating all of them is the same on flat-topped obstacles."""
    src = np.where(np.isfinite(grid), grid, np.inf)
    reach = radius + res * math.sqrt(2.0) / 2.0      # the planner's lookup pad
    n = int(math.ceil(reach / res))
    out = np.full(src.shape, -np.inf)
    nx, ny = src.shape
    for di in range(-n, n + 1):
        for dj in range(-n, n + 1):
            if math.hypot(di, dj) * res >= reach:
                continue
            a = src[max(0, di):nx + min(0, di), max(0, dj):ny + min(0, dj)]
            o = out[max(0, -di):nx + min(0, -di), max(0, -dj):ny + min(0, -dj)]
            np.maximum(o, a, out=o)
    return np.maximum(out, src)


def clearance_apex(field, origin, res, p, target, ground_p, ground_t, gate, max_step=0.05):
    """The least apex, above the takeoff ground, at which the hop from p to target passes the
    planner's clearance test (clearance_floor_alpha): wherever the dilated terrain under the
    centreline is above both ends, the parabola through both ends must clear it by gate. -inf when
    no terrain constrains the hop. field[i, j] is the dilated terrain of the cell whose lower corner
    is origin + (i, j) res. Points off the map do not constrain it: the planner never plans there,
    and the controller's map has nothing to say about them."""
    p = np.asarray(p, dtype=float)
    target = np.asarray(target, dtype=float)
    X = float(np.linalg.norm(target - p))
    Z = ground_t - ground_p
    if X < 1e-6:
        return -math.inf
    n = max(3, int(math.ceil(X / min(max_step, res / 3.0))) + 1)
    top = max(ground_p, ground_t)
    max_t = -math.inf                        # least tan of the launch angle over all samples
    for i in range(1, n - 1):
        u = X * i / (n - 1)
        c = p + (target - p) * (u / X)
        ci = int(math.floor((c[0] - origin[0]) / res))
        cj = int(math.floor((c[1] - origin[1]) / res))
        if not (0 <= ci < field.shape[0] and 0 <= cj < field.shape[1]):
            continue
        f = field[ci, cj]
        if f > top:
            max_t = max(max_t, (f + gate - ground_p - Z * u * u / (X * X)) / (u * (X - u) / X))
    if max_t == -math.inf:
        return -math.inf
    # The parabola z(u) = a u - k u^2 through (X, Z) with launch slope a peaks at a^2 / (4 k)
    k = (max_t * X - Z) / (X * X)
    if k <= 0.0 or max_t / (2.0 * k) >= X:
        return math.inf if k <= 0.0 else Z
    return max_t * max_t / (4.0 * k)
