"""Heading helpers for hopcopter.

The landing attitude at any heading, the heading that faces a hop's target, and the yaw rate
command toward it. Pure Python, unit-tested in test/test_heading.py.

The robot hops on its leg along the body z axis, so its heading does not enter the hop: the
takeoff solve, the touchdown march and the planner all work with world-frame vectors. The heading
only matters where a world-frame landing axis becomes the firmware's roll and pitch, which are
Euler angles under the heading.
"""
import math


def tilt_rp(z_land, yaw):
    """Roll and pitch [deg] that point the body z axis along z_land at heading yaw [rad].

    z_land is a world-frame unit vector: z_land = Rz(yaw) Ry(pitch) Rx(roll) e_z (ZYX Euler angles,
    the convention of LinearJumpingController.inverse_jumping_model and of hopcopter's commands).
    """
    c, s = math.cos(yaw), math.sin(yaw)
    x = c * z_land[0] + s * z_land[1]           # z_land in the heading frame: Rz(-yaw) z_land
    y = -s * z_land[0] + c * z_land[1]
    roll = -math.asin(max(-1.0, min(1.0, y)))
    pitch = math.asin(max(-1.0, min(1.0, x / math.cos(roll))))
    return math.degrees(roll), math.degrees(pitch)


def face_heading(mode, foot, aim, current, min_dist=0.3):
    """Return the heading [rad] to hold after a steering decision.

    Toward the aim point of a hop that moves (tracker mode 'track' or 'retry', at least min_dist
    from the predicted foot touchdown), else the current heading (hops in place: 'goal', 'hold',
    'idle', and small corrections).
    """
    if mode in ('track', 'retry'):
        dx, dy = aim[0] - foot[0], aim[1] - foot[1]
        if math.hypot(dx, dy) >= min_dist:
            return math.atan2(dy, dx)
    return current


def heading_of(x, y, z, w):
    """Return the heading [rad] of the attitude quaternion (x, y, z, w).

    The intrinsic ZYX yaw atan2(R10, R00): the direction of the body x axis seen from above, under
    which the firmware takes its roll and pitch. (scipy's lowercase as_euler('zyx') is extrinsic:
    its first angle drifts from the heading when the body tilts both ways.)
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(a):
    """Return the angle a [rad] wrapped to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_rate_cmd(desired, actual, kp, rate_max):
    """Return the yaw rate [deg/s, counterclockwise positive] toward heading `desired` [rad].

    kp [deg/s per rad] times the error from `actual` [rad] the short way round, capped at
    +-|rate_max|. (hopcopter sends its negative: the legacy commander inverts the yaw rate.)
    """
    rate_max = abs(rate_max)
    return max(-rate_max, min(rate_max, kp * wrap_pi(desired - actual)))
