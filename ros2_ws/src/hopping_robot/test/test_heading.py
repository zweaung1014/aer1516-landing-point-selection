"""Unit tests for JumpLib/heading.py: the landing attitude at any heading, the heading that faces a
hop's target, and the yaw rate command."""
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from hopping_robot.JumpLib.heading import face_heading, heading_of, tilt_rp, wrap_pi, yaw_rate_cmd


def axis(roll_deg, pitch_deg, yaw):
    """Body z axis of the ZYX attitude (yaw [rad], pitch, roll [deg])."""
    r = Rotation.from_euler('ZYX', [yaw, math.radians(pitch_deg), math.radians(roll_deg)])
    return r.apply([0.0, 0.0, 1.0])


def unit_tilted(tilt_deg, azimuth_deg):
    t, a = math.radians(tilt_deg), math.radians(azimuth_deg)
    return np.array([math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), math.cos(t)])


def test_tilt_rp_at_heading_zero_is_the_apex_conversion():
    # Before the heading was commanded, hopcopter converted z_land at the apex yaw (0) this way
    for z in (unit_tilted(15, 0), unit_tilted(20, 90), unit_tilted(8, -135), unit_tilted(0, 0)):
        v_b = Rotation.from_rotvec([0.0, 0.0, 0.0]).apply(z)
        roll = -math.asin(v_b[1])
        pitch = math.asin(v_b[0] / math.cos(roll))
        expected = (math.degrees(roll), math.degrees(pitch))
        assert tilt_rp(z, 0.0) == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize('yaw_deg', [-180, -135, -90, -30, 0, 45, 90, 150, 179])
@pytest.mark.parametrize('tilt,az', [(0, 0), (5, 30), (12, -100), (20, 180), (20, 75), (30, -45)])
def test_tilt_rp_points_the_body_axis_along_z_land_at_any_heading(yaw_deg, tilt, az):
    z = unit_tilted(tilt, az)
    yaw = math.radians(yaw_deg)
    roll, pitch = tilt_rp(z, yaw)
    assert axis(roll, pitch, yaw) == pytest.approx(z, abs=1e-9)
    # the tilt itself does not depend on the heading
    cos_tilt = math.cos(math.radians(roll)) * math.cos(math.radians(pitch))
    assert math.degrees(math.acos(cos_tilt)) == pytest.approx(tilt, abs=1e-6)


def test_tilt_rp_angles_fixed_at_the_apex_would_swing_the_tilt_with_the_heading():
    # Why roll/pitch are recomputed every tick: angles fixed at heading 0 point the landing axis
    # 90 deg off once the robot has turned 90 deg
    z = unit_tilted(15, 0)
    roll, pitch = tilt_rp(z, 0.0)
    turned = axis(roll, pitch, math.radians(90))
    assert turned == pytest.approx(unit_tilted(15, 90), abs=1e-9)


def test_face_heading_faces_a_moving_hops_target():
    assert face_heading('track', (0.0, 0.0), (1.0, 1.0), 0.0) == pytest.approx(math.radians(45))
    assert face_heading('retry', (2.0, 0.0), (1.0, 0.0), 0.0) == pytest.approx(math.pi)
    assert face_heading('track', (0.0, 0.0), (0.0, -2.0), 1.0) == pytest.approx(-math.pi / 2)


def test_face_heading_keeps_the_heading_in_place_and_for_small_corrections():
    for mode in ('goal', 'hold', 'idle'):
        assert face_heading(mode, (0.0, 0.0), (3.0, 3.0), 0.7) == 0.7
    assert face_heading('track', (0.0, 0.0), (0.2, 0.1), -1.2) == -1.2
    assert face_heading('track', (0.0, 0.0), (0.2, 0.1), -1.2, min_dist=0.1) == \
        pytest.approx(math.atan2(0.1, 0.2))


def test_wrap_pi():
    assert wrap_pi(math.radians(190)) == pytest.approx(math.radians(-170))
    assert wrap_pi(math.radians(-190)) == pytest.approx(math.radians(170))
    assert wrap_pi(0.3) == pytest.approx(0.3)


def test_yaw_rate_cmd_turns_the_short_way_and_saturates():
    # 170 deg from -170 deg: 20 deg the short way, clockwise (negative)
    assert yaw_rate_cmd(math.radians(170), math.radians(-170), 90.0, 90.0) == \
        pytest.approx(-90.0 * math.radians(20))
    assert yaw_rate_cmd(math.radians(90), 0.0, 90.0, 60.0) == 60.0
    assert yaw_rate_cmd(math.radians(-90), 0.0, 90.0, 60.0) == -60.0
    assert yaw_rate_cmd(0.5, 0.5, 90.0, 60.0) == 0.0
    # a negative cap is taken as its size, never as a spin
    assert yaw_rate_cmd(0.1, 0.0, 90.0, -60.0) == pytest.approx(90.0 * 0.1)


@pytest.mark.parametrize('yaw_deg', [-170, -60, 0, 35, 120, 179])
@pytest.mark.parametrize('roll,pitch', [(0, 0), (10, 15), (14, 14), (-20, 20), (25, -5)])
def test_heading_of_is_the_intrinsic_yaw_under_any_tilt(yaw_deg, roll, pitch):
    q = Rotation.from_euler('ZYX', [yaw_deg, pitch, roll], degrees=True).as_quat()
    assert math.degrees(heading_of(*q)) == pytest.approx(yaw_deg, abs=1e-9)


def test_tilt_rp_round_trip_through_the_heading_of_the_attitude():
    # A tilted body's heading, then roll/pitch for its own z axis, rebuild the same attitude's axis
    r = Rotation.from_euler('ZYX', [70, 12, -9], degrees=True)
    yaw = heading_of(*r.as_quat())
    roll, pitch = tilt_rp(r.apply([0.0, 0.0, 1.0]), yaw)
    assert (roll, pitch) == pytest.approx((-9.0, 12.0), abs=1e-9)
