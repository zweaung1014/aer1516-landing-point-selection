"""Stage 2 of docs/follow_planner_plan.md: closed-loop push cutoff, armed once per real stance."""
import math

import pytest

from hopping_robot.JumpLib.push_controller import PushController, push_accel

G = 9.80665
MASS = 0.3609
Z_LO = 0.366
K = 28e-8


def make(**kw):
    return PushController(push_accel(30000, MASS), z_lo=Z_LO, **kw)


def stance(pc, t0=-0.05, ms=30):
    """A 30 ms stance at 15 g on the 1 kHz IMU, ending before t = 0."""
    for k in range(ms + 1):
        pc.imu(15 * G, t0 + 0.001 * k)
    pc.imu(0.0, t0 + 0.001 * (ms + 1))


def test_push_accel_matches_the_motor_model():
    assert push_accel(21000, MASS) == pytest.approx(4.75, abs=0.01)   # stage 0: IMU read 4.73
    assert push_accel(30000, MASS) == pytest.approx(7.98, abs=0.01)
    assert push_accel(34265, MASS) == pytest.approx(G, abs=0.02)      # hover
    assert push_accel(999, MASS) == 0.0


def test_no_push_without_a_real_stance():
    pc = make()
    for k in range(6):                       # 5 ms above 5 g: a blip, not a stance
        pc.imu(30 * G, 0.001 * k)
    pc.imu(0.0, 0.006)
    assert not pc.step(0.01, 3, Z_LO, 3.5, 1.0, 0.8)
    assert pc.pop_climb() is None


def test_one_push_per_stance():
    pc = make()
    stance(pc)
    assert pc.step(0.00, 3, Z_LO, 3.5, 1.0, 1.0)
    assert pc.step(0.01, 3, Z_LO + 0.035, 3.48, 1.0, 1.0)
    assert pc.step(0.02, 2, Z_LO + 0.07, 3.46, 1.0, 1.0) is False   # false stance tick: paused
    assert pc.step(0.03, 3, Z_LO + 0.10, 3.44, 1.0, 1.0)            # resumes, not re-armed
    assert pc.climb.t_start == 0.0


def test_slow_first_tick_arms_on_the_next():
    pc = make()
    stance(pc)
    assert not pc.step(0.00, 3, Z_LO, 0.8, 1.0, 0.8)    # first-tick speed reads low
    assert pc.step(0.01, 3, Z_LO + 0.03, 3.4, 1.0, 0.8)
    assert pc.climb.t_start == 0.01


def test_cut_when_the_coast_reaches_the_target():
    pc = make(t_lead=0.0)
    stance(pc)
    assert pc.step(0.0, 3, Z_LO, 3.5, 1.0, 0.8)
    h = 0.8 - 3.0 ** 2 / (2 * 9.55)                       # coasting from here at 3 m/s reaches 0.8
    assert pc.step(0.1, 3, Z_LO + h - 0.01, 3.0, 1.0, 0.8)
    assert not pc.step(0.11, 3, Z_LO + h + 0.001, 3.0, 1.0, 0.8)
    assert not pc.step(0.12, 3, Z_LO, 3.0, 1.0, 0.8)     # a cut is final
    c = pc.pop_climb()
    assert (c.end, c.target) == ('target', 0.8)
    assert c.duration == pytest.approx(0.11)
    assert c.cut_h == pytest.approx(h + 0.001)
    assert pc.pop_climb() is None


def test_lead_cuts_earlier():
    pc = make(t_lead=0.02)
    v = 3.0
    lead = v * push_accel(30000, MASS) * 0.02 / 9.55
    assert pc.apex_height(Z_LO, v, 1.0) == pytest.approx(v * v / (2 * 9.55) + lead)
    assert pc.apex_height(Z_LO, v, 0.5) == pytest.approx(v * v / (2 * 9.55) + lead / 2)
    assert pc.apex_height(Z_LO + 0.5, -1.0, 1.0) == pytest.approx(0.5)


def test_leg_alone_reaches_the_target():
    pc = make()
    stance(pc)
    assert not pc.step(0.0, 3, Z_LO, 4.0, 1.0, 0.6)
    c = pc.pop_climb()
    assert (c.end, c.duration) == ('leg', 0.0)


def test_cap():
    pc = make(t_max=0.4)
    stance(pc)
    assert pc.step(0.0, 3, Z_LO, 3.0, 1.0, 5.0)
    assert pc.step(0.39, 3, Z_LO + 1.0, 2.3, 1.0, 5.0)
    assert not pc.step(0.40, 3, Z_LO + 1.0, 2.3, 1.0, 5.0)
    assert pc.pop_climb().end == 'cap'


def test_apex_ends_the_push_and_nothing_arms_in_flight():
    pc = make()
    stance(pc)
    assert pc.step(0.0, 3, Z_LO, 3.0, 1.0, 5.0)
    assert not pc.step(0.3, 1, Z_LO + 1.0, -0.01, 1.0, 5.0)
    assert pc.pop_climb().end == 'apex'
    stance(pc, t0=0.31)                      # a spike in flight that looks like a stance
    assert not pc.step(0.35, 1, Z_LO + 1.0, -0.5, 1.0, 0.8)
    assert not pc.step(0.36, 3, Z_LO + 1.0, 2.0, 1.0, 0.8)   # a false 1 -> 3 does not push


def fly(target, v0, t_lead):
    """Apex height reached from liftoff at v0, with the motors modelled as in model.sdf.jinja:
    rotor speed first-order (12.5 ms up, 25 ms down), thrust ~ omega^2, idle thrust that leaves
    the 9.55 m/s^2 coast measured in stage 0. Control ticks every 10 ms see the newest 200 Hz
    odom sample (2.5 ms old) and its speed differentiated over 5 ms."""
    pc = make(t_lead=t_lead)
    stance(pc)
    w_push = 0.04076521 * 30000 + 380.8359
    w_idle = math.sqrt((G - 9.55) * MASS / (4 * K))
    z, v, w, push = Z_LO, v0, w_idle, False
    odom_z = odom_v = None
    dt = 1e-4
    for i in range(20000):
        if i % 50 == 0:                      # odom
            odom_v = v0 if odom_z is None else (z - odom_z) / 0.005
            odom_z = z
        if i % 100 == 25:                    # control tick
            push = pc.step(i * dt, 3, odom_z, odom_v, 1.0, target)
        w_cmd = w_push if push else w_idle
        w += (w_cmd - w) * dt / (0.0125 if w_cmd > w else 0.025)
        v += (4 * K * w * w / MASS - G) * dt
        z += v * dt
        if v <= 0.0:
            return z - Z_LO
    raise AssertionError('no apex')


@pytest.mark.parametrize('v0', [3.0, 3.5])
@pytest.mark.parametrize('target', [0.8, 1.0, 1.2])
def test_closed_loop_apex_with_lead(target, v0):
    # The lead this motor model needs: spin-down (0.65 tau_down above idle, as thrust ~ omega^2),
    # the odom sample's age and half a tick
    assert fly(target, v0, t_lead=0.024) == pytest.approx(target, abs=0.015)


def test_without_lead_the_apex_overshoots():
    assert fly(1.0, 3.5, t_lead=0.0) - 1.0 > 0.03
