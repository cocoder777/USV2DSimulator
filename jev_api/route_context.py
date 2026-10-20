"""Geographic decoding and ordered route context, without selecting a control action."""
import bisect
import math

EARTH_RADIUS = 6378137.0


def xy(lon_lat, origin):
    lon, lat = lon_lat
    lon0, lat0 = origin
    return ((lon - lon0) * math.pi / 180 * EARTH_RADIUS * math.cos(math.radians(lat0)),
            (lat - lat0) * math.pi / 180 * EARTH_RADIUS)


def local(point, position, heading):
    dx, dy = point[0] - position[0], point[1] - position[1]
    c, s = math.cos(heading), math.sin(heading)
    return [round(dx * c + dy * s, 3), round(-dx * s + dy * c, 3)]


class RouteContext:
    def __init__(self):
        self.key = None
        self.progress = None

    def prepare(self, observation, run_key=None):
        task, boat = observation['task'], observation['boat']
        key = (run_key, task['route_id'], task['reference_lon_lat'], task['planWaypointList'])
        if key != self.key:
            self.key, self.progress = key, None
            self.points = [xy(p, task['origin_lon_lat']) for p in task['reference_lon_lat']]
            self.waypoints = [xy(p, task['origin_lon_lat']) for p in task['planWaypointList']]
            self.lengths = [0.0]
            for a, b in zip(self.points, self.points[1:]):
                self.lengths.append(self.lengths[-1] + math.dist(a, b))
            self.waypoint_indices = []
            lower = 0
            for p in self.waypoints:
                lower = min(range(lower, len(self.points)), key=lambda i: math.dist(p, self.points[i]))
                self.waypoint_indices.append(lower)
        position = xy(boat['lon_lat'], task['origin_lon_lat'])
        heading = math.radians(boat['heading_deg'])
        next_index = task['next_waypoint_index']
        if not 1 <= next_index < len(self.waypoints):
            raise ValueError('Expected an active waypoint after the start')
        start = self.waypoint_indices[next_index - 1]
        end = self.waypoint_indices[next_index]
        # Allow the approach to a waypoint which was passed at the edge of its arrival circle.
        lower_s = max(0, self.lengths[start] - 2 * task['arrival_radius_m'])
        if self.progress is not None:
            window = max(abs(v) for v in observation['action_space']['speeds_mps']) * observation['planning']['action_duration_s'] + 2 * task['arrival_radius_m']
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
            t = max(0, min(1, ((position[0]-a[0])*dx + (position[1]-a[1])*dy) / size2))
            p = (a[0] + t*dx, a[1] + t*dy)
            candidate = (math.dist(position, p), self.lengths[i] + t * math.sqrt(size2), p)
            if best is None or candidate[0] < best[0]:
                best = candidate
        if best is None:
            raise ValueError('No nonzero reference segment in current route window')
        distance, self.progress, projection = best
        def point_at(s):
            i = min(len(self.points)-2, max(0, bisect.bisect_right(self.lengths, s)-1))
            span = self.lengths[i+1] - self.lengths[i]
            t = (s-self.lengths[i])/span if span else 0
            return tuple(a+(b-a)*t for a,b in zip(self.points[i],self.points[i+1]))
        preview_s = sorted(set(min(self.lengths[-1], self.progress + d) for d in (1,2,3,4,6)))
        return {
            'route': {
                'nearest_point_forward_left_m': local(projection, position, heading),
                'cross_track_distance_m': round(distance, 3),
                'ahead_points_forward_left_m': [local(point_at(s), position, heading) for s in preview_s],
            },
            'target': {
                'forward_left_m': local(self.waypoints[next_index], position, heading),
                'arrival_radius_m': task['arrival_radius_m'],
            },
        }
