"""Stage 1 of docs/follow_planner_plan.md: waypoint decisions at the apex, braking tilt guard."""
import math

import numpy as np
import pytest

from hopping_robot.JumpLib.waypoint_tracker import WaypointTracker

# 2026-10-27 10:52 plan: start, WP1, goal; z is the target apex of the hop landing there
PATH_1052 = [(0.05, 0.05, 0.8), (1.15, 1.95, 0.700001), (4.05, 0.05, 1.150568)]
STRAIGHT = [(0.0, 0.0, 0.8), (2.0, 0.0, 0.7), (4.0, 0.0, 0.7)]


def decide(tracker, x, y):
    d = tracker.decide(x, y)
    tracker.apply(d)
    return d


def test_replay_1052():
    """The apexes before the two steered landings of the 10:52 run, with their real landings."""
    t = WaypointTracker()
    t.set_path(PATH_1052)
    d = decide(t, -0.069, -0.117)       # apex 790.42; landed at 790.84
    assert (d.mode, d.land, d.aim_x, d.aim_y) == ('track', 1, 1.15, 1.95)
    assert d.apex == pytest.approx(0.70, abs=1e-3)
    d = decide(t, 1.158, 1.992)         # apex 791.23; landed at 791.54 next to WP1
    assert (d.mode, d.land, d.aim_x, d.aim_y) == ('track', 2, 4.05, 0.05)
    assert d.apex == pytest.approx(1.151, abs=1e-3)


def test_no_path_hops_in_place_at_hold_point():
    t = WaypointTracker()
    t.hold_xy = (0.1, 0.2)
    d = decide(t, 0.5, 0.5)
    assert (d.mode, d.aim_x, d.aim_y, d.apex) == ('idle', 0.1, 0.2, None)


def test_decide_does_not_change_tracker():
    t = WaypointTracker()
    t.set_path(STRAIGHT)
    t.decide(0.0, 0.0)
    assert t.land is None and t.retries == 0


def test_short_landing_within_reach_advances():
    t = WaypointTracker(reach=0.8)
    t.set_path(STRAIGHT)
    decide(t, 0.0, 0.0)                 # aim at WP1
    d = decide(t, 1.35, 0.1)            # 0.66 m short of WP1
    assert (d.mode, d.land) == ('track', 2)


def test_missed_landing_retries_then_holds():
    t = WaypointTracker(reach=0.8, max_retries=2)
    t.set_path(STRAIGHT)
    decide(t, 0.0, 0.0)
    for k in (1, 2):
        d = decide(t, 0.9, 0.0)         # 1.1 m short of WP1
        assert (d.mode, d.land, d.retries, d.aim_x) == ('retry', 1, k, 2.0)
    d = decide(t, 0.9, 0.0)
    assert (d.mode, d.aim_x, d.aim_y) == ('hold', 0.9, 0.0)
    d = decide(t, 1.0, 0.0)             # keeps holding where it gave up
    assert (d.mode, d.aim_x) == ('hold', 0.9)


def test_retry_counter_resets_after_reaching_the_waypoint():
    t = WaypointTracker(reach=0.8, max_retries=2)
    t.set_path(STRAIGHT)
    decide(t, 0.0, 0.0)
    decide(t, 0.9, 0.0)                 # retry 1
    d = decide(t, 1.9, 0.0)             # made it
    assert (d.mode, d.land, d.retries) == ('track', 2, 0)


def test_overshoot_catches_up():
    t = WaypointTracker(reach=0.8)
    t.set_path(STRAIGHT)
    decide(t, 0.0, 0.0)                 # aim at WP1 (2, 0)
    d = decide(t, 3.0, 0.0)             # flew 1 m past it: aim at the goal, not back
    assert (d.mode, d.land, d.aim_x) == ('retry', 2, 4.0)
    d = decide(t, 4.2, 0.0)
    assert (d.mode, d.land, d.aim_x, d.apex) == ('goal', 2, 4.0, None)


def test_landing_closer_to_the_next_waypoint_skips_ahead():
    t = WaypointTracker(reach=0.8)
    t.set_path([(0.0, 0.0, 0.8), (1.0, 0.0, 0.7), (1.5, 0.0, 0.7)])
    decide(t, 0.0, 0.0)
    d = decide(t, 1.4, 0.0)             # past WP1 and nearer the goal
    assert (d.mode, d.land) == ('goal', 2)


def test_close_waypoint_not_skipped_unless_past():
    t = WaypointTracker(reach=0.8)
    t.set_path([(0.0, 0.0, 0.8), (1.0, 0.0, 0.7), (1.5, 0.0, 0.7)])
    decide(t, 0.0, 0.0)
    d = decide(t, 0.95, 0.0)            # just short of WP1, though WP2 is close too
    assert (d.mode, d.land, d.aim_x) == ('track', 2, 1.5)


def test_short_landing_beside_a_corner_is_not_past_it():
    t = WaypointTracker(reach=0.5)
    t.set_path([(0.0, 0.0, 0.8), (2.0, 0.0, 0.7), (2.0, 2.0, 0.7)])
    decide(t, 0.0, 0.0)
    d = decide(t, 1.2, 0.3)             # short of the corner, but on the far side of its y
    assert (d.mode, d.land, d.aim_x, d.aim_y) == ('retry', 1, 2.0, 0.0)


def test_new_path_mid_run_syncs_to_nearest_waypoint():
    """09:43:33: replanned from a stale start while the robot was at (0.78, 1.64)."""
    t = WaypointTracker(reach=0.8)
    t.set_path(PATH_1052)
    d = decide(t, 0.78, 1.64)           # 0.48 m from WP1
    assert (d.mode, d.land, d.aim_x) == ('track', 2, 4.05)


def test_far_from_new_path_holds_where_it_lands():
    t = WaypointTracker(reach=0.8)
    t.set_path(STRAIGHT)
    d = decide(t, 1.0, 3.0)
    assert (d.mode, d.aim_x, d.aim_y, d.apex) == ('hold', 1.0, 3.0, None)
    t.set_path(STRAIGHT)                # a new path releases the hold
    d = decide(t, 0.1, 0.0)
    assert (d.mode, d.land) == ('track', 1)


def test_goal_overshoot_returns_to_goal():
    t = WaypointTracker(reach=0.8)
    t.set_path(STRAIGHT)
    decide(t, 0.0, 0.0)
    decide(t, 2.0, 0.0)
    assert decide(t, 4.1, 0.0).mode == 'goal'
    d = decide(t, 5.0, 0.0)             # partial brake carried it past the goal
    assert (d.mode, d.land, d.aim_x) == ('retry', 2, 4.0)


def test_clear_hops_in_place_where_it_lands():
    t = WaypointTracker()
    t.hold_xy = (0.0, 0.0)
    t.set_path(STRAIGHT)
    decide(t, 0.0, 0.0)
    t.clear()
    d = decide(t, 1.5, 0.2)
    assert (d.mode, d.aim_x, d.aim_y) == ('idle', 1.5, 0.2)
    assert decide(t, 1.7, 0.2).aim_x == 1.5


# --- Braking tilt guard (needs scipy, as the controller does) ---

def make_ljc():
    pytest.importorskip('scipy')
    from hopping_robot.JumpLib.jumping_model import InPlaneJumpingModel, LinearJumpingController
    return LinearJumpingController(high_ballistic_enable=True, model=InPlaneJumpingModel(1.688, 2.469),
                                   g=9.6, normal_gain=1)


def test_full_brake_tilt_matches_closed_form():
    ljc = make_ljc()
    ljc.landing_velocity = np.array([3.0, 0.0, -3.6])
    phi = math.atan2(3.0, 3.6)
    assert ljc.landing_tilt(np.array([0.0, 0.0, 1.0])) == pytest.approx((1 - 1 / 2.469) * phi, abs=1e-9)


def test_guard_limits_a_steep_brake_by_continuing_forward():
    ljc = make_ljc()
    ljc.landing_velocity = np.array([3.0, 0.0, -3.6])  # arrival 39.8 deg off vertical
    ljc.takeoff_velocity = np.array([0.0, 0.0, 1.0])   # full brake needs 23.7 deg
    tilt_max = math.radians(20.0)
    planned = ljc.limit_landing_tilt(tilt_max)
    assert math.degrees(planned) == pytest.approx(23.7, abs=0.1)
    assert ljc.landing_tilt(ljc.takeoff_velocity) <= tilt_max + 1e-4
    assert ljc.landing_tilt(ljc.takeoff_velocity) == pytest.approx(tilt_max, abs=1e-3)
    assert ljc.takeoff_velocity[0] > 0.0                # rotated toward continuing
    assert np.linalg.norm(ljc.takeoff_velocity) == pytest.approx(1.0)


def test_guard_leaves_feasible_takeoffs_alone():
    ljc = make_ljc()
    ljc.landing_velocity = np.array([3.0, 0.0, -3.6])
    u = np.array([3.0, 0.0, 3.6]) / np.linalg.norm([3.0, 0.0, 3.6])  # straight continuation
    ljc.takeoff_velocity = u.copy()
    ljc.limit_landing_tilt(math.radians(20.0))
    assert np.allclose(ljc.takeoff_velocity, u)
    ljc.landing_velocity = np.array([0.0, 0.0, -4.0])  # in place
    ljc.takeoff_velocity = np.array([0.0, 0.0, 1.0])
    assert ljc.limit_landing_tilt(math.radians(20.0)) == pytest.approx(0.0, abs=1e-9)
