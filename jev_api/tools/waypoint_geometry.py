"""Waypoint-only geometry in a fixed east/north frame. No reference path or bow state."""

import math

EARTH_RADIUS = 6378137.0
KNOT_MPS = 1852 / 3600


def xy(lon_lat, origin):
    lon, lat = lon_lat
    lon0, lat0 = origin
    return (
        (lon - lon0) * math.pi / 180 * EARTH_RADIUS * math.cos(math.radians(lat0)),
        (lat - lat0) * math.pi / 180 * EARTH_RADIUS,
    )


def circle_interval(a, b, center, radius):
    dx, dy = b[0] - a[0], b[1] - a[1]
    ox, oy = a[0] - center[0], a[1] - center[1]
    aa = dx * dx + dy * dy
    if aa < 1e-16:
        return (0.0, 1.0) if ox * ox + oy * oy <= radius * radius else None
    bb, cc = 2 * (ox * dx + oy * dy), ox * ox + oy * oy - radius * radius
    disc = bb * bb - 4 * aa * cc
    if disc < 0:
        return None
    low, high = max(0.0, (-bb - math.sqrt(disc)) / (2 * aa)), min(
        1.0, (-bb + math.sqrt(disc)) / (2 * aa)
    )
    return (low, high) if low <= high else None


class WaypointGeometry:
    def __init__(self, observation):
        task, boat = observation["task"], observation["boat"]
        origin = (boat["longitude"], boat["latitude"])
        self.waypoints = [
            xy((p["routeLon"], p["routeLat"]), origin) for p in task["planWaypointList"]
        ]
        self.next_index = task["next_waypoint_index"]
        if not 1 <= self.next_index < len(self.waypoints):
            raise ValueError("Expected an active waypoint after the start")
        self.radius = task["arrival_radius_m"]
        self.target = self.waypoints[self.next_index]
        self.upcoming = [
            {
                "index": i,
                "east_north_m": self.waypoints[i],
                "speed_limit_mps": task["planWaypointList"][i]["routeSpeed"],
            }
            for i in range(self.next_index, len(self.waypoints))
        ]

    def motion_facts(self, motion, waypoint_index=None, radius=None):
        center = self.waypoints[
            self.next_index if waypoint_index is None else waypoint_index
        ]
        radius = self.radius if radius is None else radius
        entered = False
        maximum = None
        for i, (a, b) in enumerate(zip(motion["path"], motion["path"][1:])):
            interval = circle_interval(a, b, center, radius)
            if interval is None:
                continue
            entered = True
            va, vb = motion["path_speeds"][i : i + 2]
            peak = max(abs(va + (vb - va) * u) for u in interval)
            maximum = peak if maximum is None else max(maximum, peak)
        return {
            "enters_checkpoint": entered,
            "max_speed_in_circle_mps": maximum,
            "end_checkpoint_distance_m": math.dist(motion["endpoint"], center),
        }
