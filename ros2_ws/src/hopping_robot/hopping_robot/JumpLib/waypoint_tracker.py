import math


class Decision:
    """What the next hop aims at, decided at a steering apex.

    mode:   'track' (the flight lands on p[land-1], aim at p[land]), 'retry' (missed p[land],
            aim at it again), 'goal' (on the last waypoint, hop in place there), 'hold' (gave up
            on the path, hop in place), 'idle' (no path, hop in place)
    land:   index of the waypoint the next flight is meant to land on (None without one)
    aim_x, aim_y: where the next hop aims
    apex:   target apex for the next hop, or None for "hop in place at the default height"
    """

    def __init__(self, mode, land, retries, aim_x, aim_y, apex, reason=''):
        self.mode = mode
        self.land = land
        self.retries = retries
        self.aim_x = aim_x
        self.aim_y = aim_y
        self.apex = apex
        self.reason = reason


class WaypointTracker:
    """Which waypoint each hop aims at.

    The planner plans exactly one hop per waypoint, and the controller commits to the next
    landing and push at the apex, so the target changes there: one decision per real
    touchdown, made from where the current flight will land (P), never from a counter or a
    distance check. A missed or false apex therefore costs at most one hop, not the rest of
    the path.

    path[k] = (x, y, target_apex): waypoint k and the planned apex of the hop that lands on it.
    path[0] is the robot's position at planning time.
    """

    def __init__(self, reach=0.8, max_retries=2):
        self.reach = reach              # R_far: a landing this close to a waypoint counts as on it
        self.max_retries = max_retries  # missed landings in a row before giving up on the path
        self.path = []
        self.land = None                # waypoint the current flight is meant to land on
        self.retries = 0
        self.holding = False
        self.hold_xy = None             # where to hop in place; None: wherever the flight lands

    def set_path(self, path):
        """A new path: sync to it at the next decision, from where that flight lands."""
        self.path = [(float(x), float(y), float(h)) for x, y, h in path]
        self.land = None
        self.retries = 0
        self.holding = False

    def clear(self):
        """Drop the path and hop in place where the robot next lands."""
        self.path = []
        self.land = None
        self.retries = 0
        self.holding = False
        self.hold_xy = None

    def _dist(self, px, py, k):
        x, y, _ = self.path[k]
        return math.hypot(px - x, py - y)

    def _past(self, px, py, k):
        """P is beyond p[k]: ahead of it along the next segment, and along the previous one if
        there is one (so a short landing next to a corner does not count as past it)."""
        x, y, _ = self.path[k]
        nx, ny, _ = self.path[k + 1]
        if (px - x) * (nx - x) + (py - y) * (ny - y) <= 0.0:
            return False
        if k > 0:
            ax, ay, _ = self.path[k - 1]
            if (px - x) * (x - ax) + (py - y) * (y - ay) <= 0.0:
                return False
        return True

    def _hold(self, px, py, land, retries, reason):
        if self.hold_xy is not None:
            px, py = self.hold_xy
        return Decision('hold', land, retries, px, py, None, reason)

    def decide(self, px, py):
        """The decision for a flight predicted to land (foot) at (px, py). Does not change the
        tracker; apply() commits it."""
        if not self.path:
            x, y = self.hold_xy if self.hold_xy is not None else (px, py)
            return Decision('idle', None, 0, x, y, None)
        if self.holding:
            return self._hold(px, py, self.land, self.retries, 'holding')

        n = len(self.path) - 1
        land, retries = self.land, self.retries
        if land is None:
            # New path: start from the waypoint this flight is landing on
            land = min(range(n + 1), key=lambda k: self._dist(px, py, k))
            if self._dist(px, py, land) > self.reach:
                return Decision('hold', None, 0, px, py, None, 'far from the new path')

        # Catch up over waypoints the flight has already passed: far past one, or closer to
        # the next. A close waypoint is never skipped otherwise; short hops can be deliberate.
        while land < n and self._past(px, py, land) and (
                self._dist(px, py, land) > self.reach
                or self._dist(px, py, land + 1) < self._dist(px, py, land)):
            land += 1
            retries = 0

        if self._dist(px, py, land) <= self.reach:
            if land == n:
                x, y, _ = self.path[n]
                return Decision('goal', n, 0, x, y, None)
            x, y, h = self.path[land + 1]
            return Decision('track', land + 1, 0, x, y, h)

        retries += 1
        if retries > self.max_retries:
            return Decision('hold', land, retries, px, py, None,
                            f'missed waypoint {land} {retries} times')
        x, y, h = self.path[land]
        return Decision('retry', land, retries, x, y, h)

    def apply(self, d):
        """Commit a decision returned by decide()."""
        self.land = d.land
        self.retries = d.retries
        if d.mode == 'hold':
            self.holding = True
            self.hold_xy = (d.aim_x, d.aim_y)
        elif d.mode == 'idle':
            self.hold_xy = (d.aim_x, d.aim_y)
