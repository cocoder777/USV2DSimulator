"""Geographic decoding and ordered route context, without selecting a control action."""

import bisect
import math

EARTH_RADIUS = 6378137.0


def xy(lon_lat, origin):
    lon, lat = lon_lat
    lon0, lat0 = origin
    return (
        (lon - lon0) * math.pi / 180 * EARTH_RADIUS * math.cos(math.radians(lat0)),
        (lat - lat0) * math.pi / 180 * EARTH_RADIUS,
    )


def local(point, position, heading):
    dx, dy = point[0] - position[0], point[1] - position[1]
    c, s = math.cos(heading), math.sin(heading)
    return [round(dx * c + dy * s, 3), round(-dx * s + dy * c, 3)]


class RouteContext:
    def __init__(self):
        self.key = None
        self.progress = None

    def prepare(self, observation, run_key=None):
        task, boat = observation["task"], observation["boat"]
        key = (
            run_key,
            task["route_id"],
            task["reference_lon_lat"],
            task["planWaypointList"],
        )
        if key != self.key:
            self.key, self.progress = key, None
            self.points = [
                xy(p, task["origin_lon_lat"]) for p in task["reference_lon_lat"]
            ]
            self.waypoints = [
                xy(p, task["origin_lon_lat"]) for p in task["planWaypointList"]
            ]
            self.lengths = [0.0]
            for a, b in zip(self.points, self.points[1:]):
                self.lengths.append(self.lengths[-1] + math.dist(a, b))
            self.waypoint_indices = []
            lower = 0
            for p in self.waypoints:
                lower = min(
                    range(lower, len(self.points)),
                    key=lambda i: math.dist(p, self.points[i]),
                )
                self.waypoint_indices.append(lower)
        position = xy(boat["lon_lat"], task["origin_lon_lat"])
        heading = math.radians(boat["heading_deg"])
        next_index = task["next_waypoint_index"]
        if not 1 <= next_index < len(self.waypoints):
            raise ValueError("Expected an active waypoint after the start")
        start = self.waypoint_indices[next_index - 1]
        end = self.waypoint_indices[next_index]
        # Allow the approach to a waypoint which was passed at the edge of its arrival circle.
        lower_s = max(0, self.lengths[start] - 2 * task["arrival_radius_m"])
        window = (
            max(abs(v) for v in observation["action_space"]["speeds_mps"])
            * observation["planning"]["action_duration_s"]
            + 2 * task["arrival_radius_m"]
        )
        if self.progress is not None:
            lower_s = max(lower_s, self.progress - window)
            upper_s = min(self.lengths[end], self.progress + window)
        else:
            upper_s = self.lengths[end]
        low = max(0, bisect.bisect_right(self.lengths, lower_s) - 1)
        high = min(end, bisect.bisect_left(self.lengths, upper_s) + 1)
        best = None
        for i in range(low, high):
            a, b = self.points[i], self.points[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            size2 = dx * dx + dy * dy
            if size2 == 0:
                continue
            t = max(
                0,
                min(1, ((position[0] - a[0]) * dx + (position[1] - a[1]) * dy) / size2),
            )
            p = (a[0] + t * dx, a[1] + t * dy)
            candidate = (
                math.dist(position, p),
                self.lengths[i] + t * math.sqrt(size2),
                p,
            )
            if best is None or candidate[0] < best[0]:
                best = candidate
        if best is None:
            raise ValueError("No nonzero reference segment in current route window")
        distance, self.progress, projection = best

        def point_at(s):
            i = min(
                len(self.points) - 2, max(0, bisect.bisect_right(self.lengths, s) - 1)
            )
            span = self.lengths[i + 1] - self.lengths[i]
            t = (s - self.lengths[i]) / span if span else 0
            return tuple(
                a + (b - a) * t for a, b in zip(self.points[i], self.points[i + 1])
            )

        self.leg_end_s = self.lengths[end]
        preview_s = sorted(
            set(min(self.leg_end_s, self.progress + d) for d in (1, 2, 3, 4, 6))
        )
        begin_s = max(0, self.progress - 2)
        finish_s = min(self.lengths[-1], self.progress + max(6, 2 * window))
        indices = range(
            bisect.bisect_right(self.lengths, begin_s),
            bisect.bisect_left(self.lengths, finish_s),
        )
        self.reference_window = [(begin_s, local(point_at(begin_s), position, heading))]
        self.reference_window.extend(
            (self.lengths[i], local(self.points[i], position, heading)) for i in indices
        )
        self.reference_window.append(
            (finish_s, local(point_at(finish_s), position, heading))
        )
        self.current_target_local = local(self.waypoints[next_index], position, heading)
        self.arrival_radius = task["arrival_radius_m"]
        self.cross_track_distance = distance
        return {
            "route": {
                "remaining_to_checkpoint_m": round(self.leg_end_s - self.progress, 3),
                "after_checkpoint_forward_left_m": (
                    [
                        local(
                            point_at(min(self.lengths[-1], self.leg_end_s + d)),
                            position,
                            heading,
                        )
                        for d in (1, 2, 3)
                    ]
                    if next_index < len(self.waypoints) - 1
                    else []
                ),
                "nearest_point_forward_left_m": local(projection, position, heading),
                "cross_track_distance_m": round(distance, 3),
                "ahead_points_forward_left_m": [
                    local(point_at(s), position, heading) for s in preview_s
                ],
            },
            "target": {
                "forward_left_m": local(self.waypoints[next_index], position, heading),
                "arrival_radius_m": task["arrival_radius_m"],
            },
        }

    def motion_facts(self, path):
        """Geometry only: no score, preferred action, filtering or ranking."""

        def project(point, minimum_s, maximum_s):
            best = (float("inf"), self.progress, 0.0)
            for (sa, a), (sb, b) in zip(
                self.reference_window, self.reference_window[1:]
            ):
                if sb < minimum_s or sa > maximum_s:
                    continue
                dx, dy = b[0] - a[0], b[1] - a[1]
                size = dx * dx + dy * dy
                t = (
                    max(
                        0,
                        min(
                            1, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / size
                        ),
                    )
                    if size
                    else 0
                )
                if sb > sa:
                    t = max(
                        (max(sa, minimum_s) - sa) / (sb - sa),
                        min((min(sb, maximum_s) - sa) / (sb - sa), t),
                    )
                distance = math.hypot(
                    point[0] - a[0] - t * dx, point[1] - a[1] - t * dy
                )
                if distance < best[0]:
                    best = (
                        distance,
                        sa + (sb - sa) * t,
                        math.degrees(math.atan2(dy, dx)),
                    )
            return best

        # Route order matters: a nearby crossing or future leg is not interchangeable
        # with the active leg. Open the following leg only after this predicted
        # movement has entered the mandatory circle.
        projections = []
        travelled = 0.0
        entered = False
        previous = path[0]
        for point in path:
            travelled += math.dist(previous, point)
            dx, dy = point[0] - previous[0], point[1] - previous[1]
            size = dx * dx + dy * dy
            t = (
                max(
                    0,
                    min(
                        1,
                        (
                            (self.current_target_local[0] - previous[0]) * dx
                            + (self.current_target_local[1] - previous[1]) * dy
                        )
                        / size,
                    ),
                )
                if size
                else 0
            )
            entered |= (
                math.dist(
                    self.current_target_local,
                    (previous[0] + t * dx, previous[1] + t * dy),
                )
                <= self.arrival_radius
            )
            budget = travelled + self.cross_track_distance
            projections.append(
                project(
                    point,
                    self.progress - budget,
                    min(
                        self.progress + budget,
                        self.reference_window[-1][0] if entered else self.leg_end_s,
                    ),
                )
            )
            previous = point
        target_distances = [math.dist(p, self.current_target_local) for p in path]
        for a, b in zip(path, path[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            size = dx * dx + dy * dy
            t = (
                max(
                    0,
                    min(
                        1,
                        (
                            (self.current_target_local[0] - a[0]) * dx
                            + (self.current_target_local[1] - a[1]) * dy
                        )
                        / size,
                    ),
                )
                if size
                else 0
            )
            target_distances.append(
                math.hypot(
                    self.current_target_local[0] - a[0] - t * dx,
                    self.current_target_local[1] - a[1] - t * dy,
                )
            )
        entered = min(target_distances) <= self.arrival_radius
        return {
            "route_error_rms_m": math.sqrt(
                sum(p[0] ** 2 for p in projections) / len(projections)
            ),
            "route_error_max_m": max(p[0] for p in projections),
            "route_error_end_m": projections[-1][0],
            "along_route_advance_m": projections[-1][1] - projections[0][1],
            "end_route_tangent_deg": projections[-1][2],
            "enters_checkpoint": entered,
            "end_checkpoint_distance_m": math.dist(path[-1], self.current_target_local),
        }
