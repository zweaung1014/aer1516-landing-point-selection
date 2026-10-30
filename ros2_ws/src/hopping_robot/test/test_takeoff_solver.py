"""Stage 4 of docs/follow_planner_plan.md: the takeoff solve at the apex."""
import math

import numpy as np
import pytest

from hopping_robot.JumpLib.takeoff_solver import (LIFT_DT, LIFT_TILT, HopModel, clearance_apex,
                                                  dilate, tilt_of, unit)

M = HopModel()
UP = np.array([0.0, 0.0, 1.0])
V_WARMUP = np.array([0.0, 0.0, -math.sqrt(2 * 9.55 * 0.8)])   # level fall from the 0.8 m warm-up


def simulate(m, p, v_in, d_out, h, dz=0.0, z_next=UP, dt=1e-4):
    """The same hop integrated in small steps: the CoM leaves from the liftoff attitude, the push
    runs until the coast reaches the apex (or the cap) and kicks along the liftoff tilt, then the
    idle coast and the steered fall to the next touchdown. Returns the next foot touchdown."""
    L = m.leg
    _, z_lift, theta_v = m.stance(v_in, d_out)
    pos = np.array([p[0] + L * z_lift[0], p[1] + L * z_lift[1], L * z_lift[2]])
    vel = math.sqrt(m.eta_at(theta_v)) * np.linalg.norm(v_in) * np.asarray(d_out)
    e = z_lift[:2] / max(np.linalg.norm(z_lift[:2]), 1e-12)
    s = m.kick_accel * math.sin(tilt_of(z_lift))
    g_fall = m.fall_g(tilt_of(z_next))
    pushing, t = True, 0.0
    while True:
        if pushing and (pos[2] + max(vel[2], 0.0) ** 2 / (2 * m.g_up) >= L + h or t >= m.push_max):
            pushing = False
        if pushing:
            f = np.interp(t, np.arange(len(LIFT_TILT)) * LIFT_DT, LIFT_TILT, right=0.0)
            acc = np.array([s * f * e[0], s * f * e[1], m.push_accel - m.g])
        else:
            acc = np.array([0.0, 0.0, -(m.g_up if vel[2] > 0 else g_fall)])
        vel = vel + acc * dt
        pos = pos + vel * dt
        t += dt
        if vel[2] < 0 and pos[2] <= dz + L * z_next[2]:
            return pos[:2] - L * np.asarray(z_next)[:2]


def test_stance_matches_fcn1():
    pytest.importorskip('scipy')
    from hopping_robot.JumpLib.jumping_model import InPlaneJumpingModel, JumpingModel3D
    rng = np.random.default_rng(1)
    for _ in range(50):
        v_in = np.array([*rng.uniform(-3, 3, 2), -rng.uniform(2.5, 5)])
        if rng.uniform() < 0.3:
            v_in[:2] *= 0.05                     # near-vertical arrivals take k2_vertical
        d_out = unit(rng.uniform(-math.pi, math.pi), rng.uniform(math.radians(40), math.radians(89)))
        jm = JumpingModel3D(InPlaneJumpingModel(M.k1, M.k2_at(v_in)))
        z_fcn1, _ = jm.fcn1(v_in, d_out)
        z_land, z_lift, theta_v = M.stance(v_in, d_out)
        assert np.allclose(z_land, z_fcn1, atol=1e-9)
        assert theta_v == pytest.approx(jm.model.theta_v)
        # the liftoff attitude is k1 theta_l from -v_in, in the same plane
        a = -v_in / np.linalg.norm(v_in)
        assert math.acos(np.clip(a @ z_lift, -1, 1)) == pytest.approx(M.k1 * jm.model.theta_l)


def test_k2_ramps_to_vertical_arrivals():
    assert M.k2_at((3.0, 0.0, -3.6)) == pytest.approx(M.k2)               # 50 deg descent
    assert M.k2_at((0.0, 0.0, -3.9)) == pytest.approx(M.k2_vertical)
    beta = 0.5 * (M.k2_ramp[0] + M.k2_ramp[1])
    mid = M.k2_at((math.cos(beta), 0.0, -math.sin(beta)))
    assert mid == pytest.approx(0.5 * (M.k2 + M.k2_vertical))


def test_climb():
    t_push, t_up, apex = M.climb(4.0, 0.7)       # the leg alone overshoots: no push
    assert (t_push, t_up, apex) == pytest.approx((0.0, 4.0 / 9.55, 16.0 / (2 * 9.55)))
    t_push, t_up, apex = M.climb(3.0, 0.8)
    assert apex == pytest.approx(0.8)
    assert 0.0 < t_push < 0.4
    assert t_up > math.sqrt(2 * 0.8 / 9.55)      # a finite push climbs slower than a ballistic arc
    d = M.g - M.push_accel                       # the same climb, by its definition
    v = 3.0 - d * t_push
    assert 3.0 * t_push - 0.5 * d * t_push ** 2 + v * v / (2 * 9.55) == pytest.approx(0.8)
    t_push, _, apex = M.climb(1.5, 2.0)          # out of reach in 0.4 s: capped
    assert t_push == pytest.approx(0.4)
    assert apex < 2.0


def test_fall_g_matches_stage_0():
    assert M.fall_g(0.0) == pytest.approx(9.55)
    assert M.fall_g(math.radians(19)) == pytest.approx(9.33, abs=0.01)
    assert M.fall_g(math.radians(30)) == pytest.approx(8.99, abs=0.02)


CASES = [
    # start, arrival, target, apex: the stage 3 plans' hops and a few off-plan ones
    ((0.05, 0.05), V_WARMUP, (1.35, 0.05), 0.72),                 # flat (4,0) hop 1
    ((1.35, 0.05), (1.53, 0.0, -3.71), (2.65, 0.05), 0.72),       # its hop 2, straight on
    ((0.05, 0.05), V_WARMUP, (1.65, 0.75), 0.71),                 # low_wall (4,1.5) hop 1
    ((1.65, 0.05), (1.82, 0.0, -3.83), (3.55, 0.05), 0.94),       # low_wall (4,0) hop 2
    ((3.55, 0.05), (2.02, 0.0, -4.24), (4.05, 0.05), 1.08),       # its short final hop
    ((1.3, 0.0), (1.6, 0.0, -3.7), (2.3, 0.75), 0.8),             # a 37 deg dogleg
    ((1.3, 0.0), (1.6, 0.0, -3.7), (1.9, 1.2), 0.8),              # a 63 deg dogleg
    ((0.0, 0.0), (2.4, 0.3, -4.0), (0.3, -0.1), 0.6),             # short hop, fast arrival
    ((4.0, 0.0), (0.5, 0.0, -3.9), (4.05, 0.05), 0.8),            # stop at the goal
]


@pytest.mark.parametrize('p, v_in, target, h', CASES)
def test_solve_lands_on_target(p, v_in, target, h):
    f = M.solve(p, v_in, target, h)
    assert f.mode in ('push', 'leg')
    assert np.allclose(f.land, target, atol=1e-5)
    assert f.alpha_leg >= M.alpha_min - 1e-9
    assert f.apex >= h - 1e-6
    # the forward model agrees with itself at the solved takeoff
    g = M.forward(p, v_in, f.d_out, h)
    assert np.allclose(g.land, target, atol=1e-5)


@pytest.mark.parametrize('p, v_in, target, h', CASES)
def test_solve_matches_a_step_integration(p, v_in, target, h):
    f = M.solve(p, v_in, target, h)
    assert np.allclose(simulate(M, p, v_in, f.d_out, h), target, atol=0.01)


def test_corrections_point_along_the_hop():
    """From the warm-up landing, the stance moves the CoM forward and the tilted push kicks it
    forward, so the leg flies shorter and steeper than a plain ballistic hop."""
    f = M.solve((0.0, 0.0), V_WARMUP, (1.3, 0.0), 0.72)
    assert f.lift_dx[0] > 0.02 and abs(f.lift_dx[1]) < 1e-9
    assert f.kick_x[0] > 0.05 and f.kick_v[0] > 0.1
    t = f.t_up + f.t_down
    assert f.v_h * t < 1.3 - 0.1
    short = M.forward((0.0, 0.0), V_WARMUP, unit(0.0, f.alpha_leg), 0.72)
    longer_push = M.forward((0.0, 0.0), V_WARMUP, unit(0.0, f.alpha_leg), 0.9)
    assert np.linalg.norm(longer_push.kick_x) > np.linalg.norm(short.kick_x)


def test_leg_alone_overshooting_takes_the_steep_root():
    f = M.solve((0.0, 0.0), (3.0, 0.0, -4.5), (0.5, 0.0), 0.6)
    assert f.mode == 'leg' and f.t_push == 0.0
    assert f.apex > 0.6                        # flies over the planned apex, not under it
    assert np.allclose(f.land, (0.5, 0.0), atol=1e-5)


def test_in_place():
    f = M.solve((1.0, 2.0), V_WARMUP, (1.0, 2.0), 0.8)
    assert f.alpha_leg == pytest.approx(math.pi / 2, abs=1e-6)
    assert np.allclose(f.land, (1.0, 2.0), atol=1e-6)
    assert f.apex == pytest.approx(0.8)


def test_pump_when_the_leg_cannot_supply_the_speed():
    """A 2.8 m hop from the warm-up landing needs more energy than the leg has. Hopping in place to
    the pump height (plus hopcopter's 5 cm) and trying again, as the tracker's two retries do, gets
    there: a faster arrival also shortens the climb, so the first pump falls a little short."""
    v_in, pumps = V_WARMUP, []
    for _ in range(3):
        f = M.solve((0.0, 0.0), v_in, (2.8, 0.0), 0.8)
        if f.mode != 'pump':
            break
        pumps.append(M.pump_height(f.v_h_need) + 0.05)
        v_in = np.array([0.0, 0.0, -math.sqrt(2 * 9.55 * pumps[-1])])
    assert 1 <= len(pumps) <= 2 and pumps[0] > 0.8
    assert f.mode == 'push' and np.allclose(f.land, (2.8, 0.0), atol=1e-5)


def test_eta_grows_with_the_redirection():
    assert M.eta_at(0.0) == pytest.approx(M.eta)
    assert M.eta_at(M.eta_ramp[0]) == pytest.approx(M.eta)
    assert M.eta_at(0.5 * sum(M.eta_ramp)) == pytest.approx(0.5 * (M.eta + M.eta_turn))
    assert M.eta_at(math.radians(90.0)) == pytest.approx(M.eta_turn)
    # a turned takeoff launches faster than a straight-up one from the same arrival
    v_in = (2.0, 0.0, -3.7)
    turned = M.forward((0.0, 0.0), v_in, unit(math.pi / 2, math.radians(50.0)), 0.8)
    up = M.forward((0.0, 0.0), v_in, unit(0.0, math.radians(89.0)), 0.8)
    assert turned.v_leg > up.v_leg


def test_limit_tilt_matches_the_stage_1_guard():
    pytest.importorskip('scipy')
    from hopping_robot.JumpLib.jumping_model import InPlaneJumpingModel, LinearJumpingController
    v_in = np.array([3.0, 0.0, -3.6])
    ljc = LinearJumpingController(model=InPlaneJumpingModel(M.k1, M.k2_at(v_in)))
    ljc.landing_velocity = v_in
    ljc.takeoff_velocity = UP.copy()
    needed = ljc.limit_landing_tilt(math.radians(20.0))
    d, tilt = M.limit_tilt(v_in, UP, math.radians(20.0))
    assert tilt == pytest.approx(needed)
    assert np.allclose(d, ljc.takeoff_velocity, atol=1e-6)
    assert tilt_of(M.stance(v_in, d)[0]) == pytest.approx(math.radians(20.0), abs=1e-4)


def low_wall():
    """The planner's low_wall map: 5 x 5 m at 0.1 m from (-0.5, -2.5), a 0.4 m wall at x 2.5-2.7."""
    res, origin = 0.1, (-0.5, -2.5)
    grid = np.zeros((50, 50))
    xs = origin[0] + (np.arange(50) + 0.5) * res
    grid[(xs >= 2.5 - 1e-9) & (xs < 2.7 - 1e-9), :] = 0.4
    return grid, origin, res


def test_dilate():
    grid, _, res = low_wall()
    field = dilate(grid, res, 0.33)
    xs = -0.5 + (np.arange(50) + 0.5) * res
    assert np.all(field[(xs > 2.15) & (xs < 3.05)] == 0.4)
    assert np.all(field[(xs < 2.1) | (xs > 3.1)] == 0.0)
    g = grid.copy()
    g[10, 10] = np.nan                         # an obstacle cell is never cleared
    assert dilate(g, res, 0.33)[12, 10] == np.inf


def test_clearance_apex_over_the_wall():
    grid, origin, res = low_wall()
    field = dilate(grid, res, 0.33)
    # low_wall (4,0)'s hop 2 peaks at 0.946 m, the least apex that clears the wall by 0.3 m at the
    # planner's samples (its launch angle is the clearance floor)
    need = clearance_apex(field, origin, res, (1.65, 0.05), (3.55, 0.05), 0.0, 0.0, 0.3)
    assert need == pytest.approx(1.9 * 1.9886 / 4, abs=0.002)
    # from 0.5 m further back the same wall needs a higher apex
    assert clearance_apex(field, origin, res, (1.15, 0.05), (3.55, 0.05), 0.0, 0.0, 0.3) > need + 0.1
    # hops that stay off the wall, or in place, are not constrained
    assert clearance_apex(field, origin, res, (0.05, 0.05), (1.65, 0.05), 0.0, 0.0, 0.3) == -math.inf
    assert clearance_apex(field, origin, res, (1.65, 0.05), (1.65, 0.05), 0.0, 0.0, 0.3) == -math.inf
