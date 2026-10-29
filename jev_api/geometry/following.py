"""Soft rear-point tracking and sector reporting; no action selection."""

import math

from geometry.coordinates import KNOT_MPS, xy
from geometry.relative_motion import direction, course_error


def sector_facts(relative, course, lower, upper, half_angle_deg=90):
    """Own minus target; closed rear half-annulus, bearing 0=front/180=rear."""
    forward = direction(course)
    along = sum(a*b for a, b in zip(relative, forward))
    across = relative[0]*forward[1] - relative[1]*forward[0]
    distance = math.hypot(*relative)
    out = distance - min(upper, max(lower, distance))
    # Outside the angular bounds, project onto either radial edge and clamp its
    # radius. Only report distance to the SET; never choose a bypass or control.
    cosine, sine = math.cos(math.radians(half_angle_deg)), math.sin(math.radians(half_angle_deg))
    angular_inside = -along >= distance*cosine-1e-9
    edge_radius = min(upper, max(lower, -along*cosine+abs(across)*sine))
    error = abs(out) if angular_inside else math.hypot(-along-edge_radius*cosine, abs(across)-edge_radius*sine)
    bearing = (math.degrees(math.atan2(across, along)) % 360) if distance > 1e-9 else None
    return dict(distance_m=distance, relative_bearing_deg=bearing,
                along_behind_m=-along, cross_track_m=across,
                distance_out_m=out, sector_error_m=error,
                in_range=lower <= distance <= upper,
                in_sector=lower <= distance <= upper and angular_inside)


def observed_motion(state, local, speed, course, stationary):
    """History estimates motion only; this sample must already be complete."""
    tick = local['tick']
    previous = state.get('following_motion')
    yaw = acceleration = 0.0
    if previous and tick > previous['tick']:
        dt = (tick-previous['tick'])*local['execution']['fixed_dt_s']
        if 0 < dt <= 5:
            alpha = dt/(.7+dt)
            if not stationary and not previous['stationary']:
                yaw = previous['yaw'] + alpha*(course_error(course, previous['course'])/dt-previous['yaw'])
            acceleration = previous['acceleration'] + alpha*((speed-previous['speed'])*KNOT_MPS/dt-previous['acceleration'])
    state['following_motion'] = dict(tick=tick, speed=speed, course=course, stationary=stationary,
                                     yaw=yaw, acceleration=acceleration)
    return dict(yaw_deg_s=yaw if abs(yaw) >= .05 else 0,
                acceleration_mps2=acceleration if abs(acceleration) >= .15 else 0)


class FollowingGeometry:
    def __init__(self, observation):
        self.task = observation['task']
        boat, target = observation['boat'], observation['target']
        delta = xy((target['longitude'], target['latitude']), (boat['longitude'], boat['latitude']))
        self.relative = tuple(-x for x in delta)
        self.course = self.task['reference_course_deg']
        self.speed = target['targetSpeed']*KNOT_MPS
        estimate = self.task.get('target_motion', {})
        self.yaw = estimate.get('yaw_deg_s', 0)
        self.acceleration = estimate.get('acceleration_mps2', 0)
        self.combined_radius = boat['collision_radius_m']+target['radius_m']
        self.distance = (self.task['followDistanceMinM']+self.task['followDistanceMaxM'])/2
        self.tolerance = min(5.0, (self.task['followDistanceMaxM']-self.task['followDistanceMinM'])/20)
        self._target_cache = {}
        self.current = self.position(self.relative, self.course)

    def position(self, relative, course):
        facts = sector_facts(relative, course, self.task['followDistanceMinM'], self.task['followDistanceMaxM'])
        # Shared accompaniment coordinates: along forward, across starboard.
        forward = direction(course)
        error = tuple(relative[j]+self.distance*forward[j] for j in (0,1))
        facts['point_error_m'] = math.hypot(*error)
        facts['tracking_error_m'] = max(0, facts['point_error_m']-self.tolerance)
        return facts

    def guidance(self, correction_time):
        # Follow the point behind the CURRENT direction, with translation and
        # gradual position correction. Rebuild next interval rather than steer
        # toward a tail point extrapolated around a long hypothetical turn.
        # This is a course reference, never a chosen speed or candidate.
        forward = direction(self.course)
        error = tuple(self.relative[j]+self.distance*forward[j] for j in (0,1))
        length = math.hypot(*error)
        scale = max(0, 1-self.tolerance/length) if length else 0
        v = tuple(self.speed*forward[j]-scale*error[j]/correction_time for j in (0,1))
        return dict(course_deg=math.degrees(math.atan2(v[0],v[1]))%360 if math.hypot(*v)>1e-8 else None,
                    correction_time_s=correction_time)

    def target_at(self, t):
        key = round(t, 8)
        if key not in self._target_cache:
            x = y = 0.0
            steps = max(1, math.ceil(t/.1))
            dt = t/steps
            for i in range(steps):
                middle = (i+.5)*dt
                speed = max(0, self.speed+self.acceleration*min(middle, 2))
                forward = direction(self.course+self.yaw*middle)
                x += speed*forward[0]*dt
                y += speed*forward[1]*dt
            self._target_cache[key] = ((x, y), (self.course+self.yaw*t) % 360,
                                      max(0, self.speed+self.acceleration*min(t, 2)))
        return self._target_cache[key]

    def forecast(self, motion, t, dt):
        previous = None
        minimum, maximum = math.inf, 0.0
        for i, point in enumerate(motion['path']):
            displacement, course, _ = self.target_at(min(i*dt, t))
            relative = tuple(self.relative[j]+point[j]-displacement[j] for j in (0, 1))
            maximum = max(maximum, math.hypot(*relative))
            if previous is not None:
                v = tuple(relative[j]-previous[j] for j in (0, 1))
                vv = sum(x*x for x in v)
                s = min(1, max(0, -sum(a*b for a, b in zip(previous, v))/vv)) if vv else 0
                minimum = min(minimum, math.hypot(*(previous[j]+s*v[j] for j in (0, 1))))
            previous = relative
        facts = self.position(relative, course)
        # Track one current-direction reference, translated linearly until the
        # next observation. Keep the curved target forecast above for safety.
        forward = direction(self.course)
        point_error = math.hypot(*(self.relative[j]+motion['endpoint'][j]
                                  +(self.distance-self.speed*t)*forward[j] for j in (0,1)))
        facts.update(point_error_m=point_error, tracking_error_m=max(0,point_error-self.tolerance))
        facts.update(minimum_m=minimum, maximum_m=maximum)
        return facts
