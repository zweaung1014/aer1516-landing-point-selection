"""Where the current fall touches down, found by marching along it over the elevation map.

At the apex the controller knows the CoM position and world velocity, and the landing attitude
it is about to command. The fall from there is ballistic at the g' of that attitude, and the foot
hangs one leg length below the CoM along it. The touchdown is where the foot first meets the
ground under it.

Guessing the ground first (under the apex), falling to it, and looking up the ground there misses
every surface in between: over a gap the guess is the gap floor, the fall comes out ~2 m too
long, and the pad the robot actually lands on is never looked at. So instead the fall is sampled
every `ds` of horizontal travel (at most `dt_max` apart; `ds` is half a map cell, so no cell is
stepped over), and stopped at the first sample where the foot is at or below the ground. One
closed-form solve then gives the exact time the foot reaches that surface's height. If that time
falls where the ground under the foot is still lower, the foot reaches the surface from the side
(its rim), and a short bisection finds where.

A foot can also reach a surface's top between two samples and pass its far rim before the next:
at the next sample it is above the lower ground beyond, and the surface is never seen. So whenever
the ground drops from one sample to the next, the closed form is also solved for the previous
sample's surface, and if the foot is still over that surface when it gets down to it, that is the
touchdown. (2026-11-06 lilypads run: the foot landed 2 cm inside the far rim of the pad at
(7.8, -0.5), between samples 0.2 m apart, and the march went on to the gap floor 0.8 m further.)

Plain Python, so it can be unit-tested without ROS."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Touchdown:
    """t: fall time [s]; com, foot: (x, y) at touchdown; h: foot drop from the apex [m];
    ground: ground height there; kind: 'top' (lands on top of a surface) or 'edge' (meets a
    surface from its side); steps: ground lookups made."""

    t: float
    com: np.ndarray
    foot: np.ndarray
    h: float
    ground: float
    kind: str
    steps: int


def march_touchdown(p, v, axis, leg, g_fall, ground_at, z_floor, ds=0.05, dt_max=0.1,
                    n_bisect=3):
    """The touchdown of a fall from CoM p = (x, y, z) at world velocity v.

    axis: body z axis at touchdown (the foot is at CoM - leg * axis); g_fall: the fall's g';
    ground_at(x, y): ground height; z_floor: a height no ground is below (the march ends there).
    ds [m], dt_max [s]: sample spacing along the fall. A rising v[2] (the apex is detected a tick
    after it) is treated as zero, as the closed form always did."""
    axis = np.asarray(axis, dtype=float)
    vx, vy = float(v[0]), float(v[1])
    w = max(-float(v[2]), 0.0)
    off = leg * axis
    fz0 = float(p[2]) - off[2]                 # foot height at the apex
    vh = math.hypot(vx, vy)
    dt = dt_max if vh * dt_max <= ds else ds / vh

    def foot_at(t):
        return np.array([p[0] + vx * t - off[0], p[1] + vy * t - off[1]])

    def fz(t):
        return fz0 - w * t - 0.5 * g_fall * t * t

    def t_to(z):                               # time for the foot to fall to height z
        drop = fz0 - z
        if drop <= 0.0:
            return 0.0
        return (math.sqrt(w * w + 2.0 * g_fall * drop) - w) / g_fall

    def result(t, ground, kind, steps):
        foot = foot_at(t)
        return Touchdown(t, foot + off[:2], foot, fz0 - fz(t), ground, kind, steps)

    steps = 1
    g0 = ground_at(*foot_at(0.0))
    if fz0 <= g0:                              # already at the ground (degenerate)
        return result(0.0, g0, 'top', steps)
    t_end = t_to(min(z_floor, g0))
    t_prev, g_prev = 0.0, g0
    while True:
        t = min(t_prev + dt, t_end)
        g = ground_at(*foot_at(t))
        steps += 1
        if g < g_prev:                         # the ground dropped: did the foot land before it?
            ts = t_to(g_prev)
            if t_prev < ts <= t:
                steps += 1
                if abs(ground_at(*foot_at(ts)) - g_prev) < 1e-6:
                    return result(ts, g_prev, 'top', steps)
        if fz(t) <= g or t >= t_end:
            break
        t_prev, g_prev = t, g
    # The foot reached a surface of height g between t_prev and t
    ts = t_to(g)
    if t_prev <= ts <= t:
        g_s = ground_at(*foot_at(ts))
        steps += 1
        if abs(g_s - g) < 1e-6:
            return result(ts, g, 'top', steps)
    # Its side: the foot is below the surface's top when it first comes over it. Bisect for where
    lo, hi = t_prev, t
    for _ in range(n_bisect):
        mid = 0.5 * (lo + hi)
        g_m = ground_at(*foot_at(mid))
        steps += 1
        if fz(mid) <= g_m:
            hi, g = mid, g_m
        else:
            lo = mid
    return result(hi, g, 'edge', steps)
