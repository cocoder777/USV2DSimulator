"""Rear-region and two-sided bypass facts; no action selection."""

import math

from geometry.coordinates import KNOT_MPS, xy
from geometry.relative_motion import direction, course_error


def request_phase(geometry, previous=None):
    """Classify the complete snapshot, without producing a control command."""
    current = geometry.current
    # Scale the neighbourhood to the available band, not a map or scenario.
    width = geometry.task['followDistanceMaxM'] - geometry.protected_radius
    enter, leave = width / 10, width / 5
    threshold = leave if previous == 'maintain' else enter
    unsafe = current['distance_m'] < geometry.protected_radius
    # After a loss, settle in the interior before handing back to point
    # maintenance. A fast crossing of the official edge is not acquisition.
    near = previous != 'recover' and current['sector_error_m'] <= threshold
    acquired = current['in_sector'] and (previous != 'recover' or geometry.region(geometry.relative)['in_sector'])
    phase = 'maintain' if not unsafe and (acquired or near) else 'recover'
    reason = ('too_close' if unsafe else 'inside' if acquired else 'settling' if current['in_sector'] else
              'near_boundary' if near else 'ahead' if current['along_behind_m'] < 0 else 'out_of_range')
    return dict(phase=phase, reason=reason, near_enter_m=enter, near_leave_m=leave)


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
        self.protected_radius = max(self.combined_radius, self.task['followDistanceMinM'])
        width = self.task['followDistanceMaxM'] - self.protected_radius
        self.margin = width / 4
        self.inner = self.protected_radius + self.margin
        self.outer = self.task['followDistanceMaxM'] - self.margin
        # A margin of the same physical size at the radial and angular edges.
        self.angle_margin = math.degrees(math.asin(min(1, self.margin/self.inner))) if self.inner else 0
        self.distance = (self.task['followDistanceMinM']+self.task['followDistanceMaxM'])/2
        self.tolerance = min(5.0, (self.task['followDistanceMaxM']-self.task['followDistanceMinM'])/20)
        self._target_cache = {}
        self.current = self.position(self.relative, self.course)

    def position(self, relative, course):
        return sector_facts(relative, course, self.task['followDistanceMinM'], self.task['followDistanceMaxM'])

    def region(self, relative):
        return sector_facts(relative, self.course, self.inner, self.outer, 90-self.angle_margin)

    def point_error(self, relative):
        forward = direction(self.course)
        return max(0, math.hypot(*(relative[j]+self.distance*forward[j] for j in (0,1)))-self.tolerance)

    def guidance(self, correction_time):
        """Retain V4's soft point reference while maintaining acquired tracking."""
        forward = direction(self.course)
        error = tuple(self.relative[j]+self.distance*forward[j] for j in (0,1))
        length = math.hypot(*error)
        scale = max(0,1-self.tolerance/length) if length else 0
        velocity = tuple(self.speed*forward[j]-scale*error[j]/correction_time for j in (0,1))
        return dict(course_deg=math.degrees(math.atan2(*velocity))%360 if math.hypot(*velocity)>1e-8 else None,
                    correction_time_s=correction_time)

    def bypass(self, relative, correction_time=12):
        """CW/CCW tangent-arc distances to the rear interior, never action scores.

        Both topologies stay visible to Jev. The set translates with the target;
        these geometric lower bounds omit own turn limits and other obstacles,
        whose bounded-motion safety previews remain independently mandatory.
        An unsafe start includes radial escape before the outside-circle path.
        """
        facts = sector_facts(relative, self.course, self.inner, self.outer)
        radius = math.hypot(*relative)
        angle = facts['relative_bearing_deg'] if radius > 1e-9 else 0
        first, last = 90+self.angle_margin, 270-self.angle_margin
        if first <= angle <= last:
            gap = abs(radius-min(self.outer, max(self.inner, radius)))
            scale = min(self.outer, max(self.inner, radius))/radius-1 if radius else 0
            vectors = [tuple(v*scale for v in relative)]*2
            return [gap, gap], self._reference_courses(vectors, [gap, gap], correction_time)
        # The bypass circle is the actual target protection radius. The goal
        # has additional room; feasibility at speed is NOT claimed by this path.
        circle = self.protected_radius
        escape = max(0, circle-radius)
        radius = max(radius, circle)
        tangent_start = math.acos(min(1, circle/radius)) if radius else math.pi/2
        result, vectors = [], []
        for sign,edge,delta in ((1,first,(first-angle) % 360),(-1,last,(angle-last) % 360)):
            delta = math.radians(delta)
            end_radius = min(self.outer, max(self.inner, radius*math.cos(delta)))
            tangent_end = math.acos(min(1, circle/end_radius)) if end_radius else math.pi/2
            if delta <= tangent_start+tangent_end:
                length = math.sqrt(max(0, radius**2+end_radius**2-2*radius*end_radius*math.cos(delta)))
                end = direction(self.course+edge)
                vector = tuple(end_radius*end[j]-relative[j] for j in (0,1))
            else:
                end_radius = self.inner
                tangent_end = math.acos(min(1, circle/end_radius)) if end_radius else math.pi/2
                length = (math.sqrt(max(0, radius**2-circle**2))
                          + circle*max(0, delta-tangent_start-tangent_end)
                          + math.sqrt(max(0, end_radius**2-circle**2)))
                # Initial tangent direction, including the limiting case on
                # the circle. This is geometry for BOTH sides, not a command.
                vector = direction(self.course+angle+sign*(90+math.degrees(tangent_start)))
            if escape:
                vector = direction(self.course+angle)
            result.append(escape+length)
            vectors.append(vector)
        return result, self._reference_courses(vectors, result, correction_time)

    def _reference_courses(self, vectors, lengths, correction_time):
        forward = direction(self.course)
        courses = []
        for vector,length in zip(vectors,lengths):
            norm = math.hypot(*vector)
            velocity = tuple(self.speed*forward[j]+(vector[j]*length/(norm*correction_time) if norm else 0) for j in (0,1))
            course = math.degrees(math.atan2(*velocity))%360 if math.hypot(*velocity)>1e-8 else None
            # Tiny negative atan2 values can round up to 360 under modulo.
            courses.append(0.0 if course is not None and course >= 360 else course)
        return courses

    def bypass_distances(self, relative):
        return self.bypass(relative)[0]

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
        forward = direction(self.course)
        linear_relative = tuple(self.relative[j]+motion['endpoint'][j]-self.speed*t*forward[j] for j in (0,1))
        facts['tracking_error_m'] = self.point_error(linear_relative)
        facts['coast_region_error_m'] = self.region(linear_relative)['sector_error_m']
        displacement,region_course,_ = self.target_at(t)
        predicted = tuple(self.relative[j]+motion['endpoint'][j]-displacement[j] for j in (0,1))
        # Rotate this position back into the current reference frame so the
        # same set-distance/path geometry applies to the observed rotating set.
        change = math.radians(region_course-self.course)
        relative = (predicted[0]*math.cos(change)-predicted[1]*math.sin(change),
                    predicted[0]*math.sin(change)+predicted[1]*math.cos(change))
        facts['region_error_m'] = self.region(relative)['sector_error_m']
        facts['bypass_remaining_m'] = self.bypass_distances(relative)
        facts.update(minimum_m=minimum, maximum_m=maximum)
        return facts
