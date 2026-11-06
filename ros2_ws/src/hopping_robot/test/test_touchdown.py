"""The touchdown march (JumpLib/touchdown.py): where the current fall meets the ground."""
import math

import numpy as np
import pytest

from hopping_robot.JumpLib.touchdown import march_touchdown

LEG = 0.366
G = 9.55
UP = np.array([0.0, 0.0, 1.0])


def closed_form(z0, w, ground):
    """Fall time of the foot from z0 - LEG down to ground, starting at downward speed w."""
    drop = z0 - LEG - ground
    return (math.sqrt(w * w + 2.0 * G * drop) - w) / G


def flat(z):
    return lambda x, y: z


def pads(*discs, floor=0.0):
    """Ground with discs (x, y, r, z) over a floor."""
    def ground(x, y):
        for px, py, r, z in discs:
            if (x - px) ** 2 + (y - py) ** 2 < r * r:
                return z
        return floor
    return ground


@pytest.mark.parametrize('ground', [0.0, 1.2, 2.0])
@pytest.mark.parametrize('v', [(0.0, 0.0, 0.0), (1.8, -0.4, 0.0), (0.5, 0.5, -0.3)])
def test_flat_matches_closed_form(ground, v):
    z0 = ground + 1.2
    td = march_touchdown((0.0, 0.0, z0), v, UP, LEG, G, flat(ground), ground)
    t = closed_form(z0, max(-v[2], 0.0), ground)
    assert td.kind == 'top'
    assert td.t == pytest.approx(t, abs=1e-12)
    assert td.ground == ground
    assert td.h == pytest.approx(z0 - LEG - ground)
    assert td.com == pytest.approx([v[0] * t, v[1] * t])


def test_foot_offset_along_axis():
    """The foot lands off the CoM along the landing attitude, and the lookup is under the foot."""
    tilt = math.radians(15.0)
    axis = np.array([math.sin(tilt), 0.0, math.cos(tilt)])
    td = march_touchdown((0.0, 0.0, 1.5), (1.0, 0.0, 0.0), axis, LEG, G, flat(0.0), 0.0)
    assert td.com - td.foot == pytest.approx(LEG * axis[:2])
    t = (math.sqrt(2.0 * G * (1.5 - LEG * axis[2]))) / G
    assert td.t == pytest.approx(t)


def test_pad_over_a_gap():
    """Apex 3 of the 2026-11-05 lilypads run: over a gap, the pad at (3.5, 0.2) in the way. The
    old guess (ground under the apex, then once more under that landing) put it at x 3.92 on the
    floor; the robot landed at (3.33, 0.30) on the 1.9 m pad."""
    ground = pads((2.0, 0.9, 0.6, 1.8), (3.5, 0.2, 0.35, 1.9), (4.9, 0.4, 0.7, 2.0))
    td = march_touchdown((2.6044, 0.4499, 3.0246), (1.7783, -0.3611, 0.0), UP, LEG, G,
                         ground, 0.0)
    assert td.kind == 'top'
    assert td.ground == 1.9
    assert td.t == pytest.approx(closed_form(3.0246, 0.0, 1.9))
    assert td.foot == pytest.approx([3.31, 0.31], abs=0.02)
    assert td.steps <= 10


def test_step_up_and_down():
    up = lambda x, y: 0.5 if x >= 0.4 else 0.0        # noqa: E731
    td = march_touchdown((0.0, 0.0, 1.4), (2.0, 0.0, 0.0), UP, LEG, G, up, 0.0)
    assert (td.kind, td.ground) == ('top', 0.5)
    assert td.t == pytest.approx(closed_form(1.4, 0.0, 0.5))
    down = lambda x, y: 0.5 if x < 0.5 else 0.0       # noqa: E731
    td = march_touchdown((0.0, 0.0, 1.4), (2.0, 0.0, 0.0), UP, LEG, G, down, 0.0)
    assert (td.kind, td.ground) == ('top', 0.0)
    assert td.t == pytest.approx(closed_form(1.4, 0.0, 0.0))


def test_miss_falls_to_the_floor():
    """Short of the pad: the march goes on to the gap floor."""
    ground = pads((0.0, 0.0, 0.4, 2.0), (3.0, 0.0, 0.4, 2.0))
    td = march_touchdown((0.5, 0.0, 3.0), (1.0, 0.0, 0.0), UP, LEG, G, ground, 0.0)
    assert (td.kind, td.ground) == ('top', 0.0)


def test_side_of_a_pad():
    """The foot is below the pad's top when it comes over it: contact at the rim."""
    ground = pads((2.0, 0.0, 0.4, 2.0))
    td = march_touchdown((0.0, 0.0, 2.8), (4.0, 0.0, 0.0), UP, LEG, G, ground, 0.0)
    assert td.kind == 'edge'
    assert td.ground == 2.0
    assert td.foot[0] == pytest.approx(1.6, abs=0.05)


def test_steps_stay_sparse():
    """A long fall at speed stays at one lookup per 0.2 m of travel (or per 0.1 s), plus the
    refinement."""
    for v in [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (3.0, 1.0, 0.0), (6.0, 0.0, 0.0)]:
        z0 = 3.4
        td = march_touchdown((0.0, 0.0, z0), v, UP, LEG, G, flat(0.0), 0.0)
        t = closed_form(z0, 0.0, 0.0)
        n = math.ceil(t / min(0.1, 0.2 / max(math.hypot(v[0], v[1]), 1e-9)))
        assert td.steps <= n + 3
