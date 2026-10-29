"""Measured and predicted rotating-frame accompaniment facts, never action selection."""

import math

from geometry.coordinates import KNOT_MPS, xy
from geometry.relative_motion import direction


def interval_error(value, lower, upper):
    return value - min(upper, max(lower, value))


def project(vector, course):
    forward = direction(course)
    right = (forward[1], -forward[0])
    return sum(a*b for a, b in zip(vector, forward)), sum(a*b for a, b in zip(vector, right))


def position_facts(relative_position, task, course):
    along, across = project(relative_position, course)
    along_out = interval_error(along, task['offsetAlongMinM'], task['offsetAlongMaxM'])
    across_out = interval_error(across, task['offsetAcrossMinM'], task['offsetAcrossMaxM'])
    return dict(along_m=along, across_m=across, along_out_m=along_out, across_out_m=across_out,
                position_error_m=math.hypot(along_out, across_out), in_position=along_out == 0 and across_out == 0)


class AccompanyGeometry:
    def __init__(self, observation):
        self.task = observation['task']
        # Use the middle half as an operating region; customer reporting keeps the full band.
        self.control_task = dict(self.task)
        for axis in ('Along', 'Across'):
            lo, hi = (f'offset{axis}MinM', f'offset{axis}MaxM')
            margin = (self.task[hi]-self.task[lo])*.25
            self.control_task[lo] += margin
            self.control_task[hi] -= margin
        self.course = self.task['reference_course_deg']
        boat, target = observation['boat'], observation['target']
        delta = xy((target['longitude'], target['latitude']), (boat['longitude'], boat['latitude']))
        self.relative_position = (-delta[0], -delta[1])  # own minus target
        self.target_speed = target['targetSpeed'] * KNOT_MPS
        estimate = self.task.get('target_motion', {})
        self.yaw = estimate.get('yaw_deg_s', 0)
        self.acceleration = estimate.get('acceleration_mps2', 0)
        self._target_cache = {}
        self.current = position_facts(self.relative_position, self.task, self.course)
        control = position_facts(self.relative_position, self.control_task, self.course)
        self.current.update(control_error_m=max(0, control['position_error_m']-.05),
                            in_control_region=self.current['in_position'] and control['position_error_m'] <= .05)
        own_course = boat['targetCourse'] if boat['targetCourse'] is not None else observation['execution']['heading_course_deg']
        # The situation course is already the movement direction, including reverse.
        own_velocity = tuple(abs(boat['targetSpeed'])*KNOT_MPS*x for x in direction(own_course))
        self.current.update(self.frame_rates(self.relative_position, own_velocity, 0))

    def target_at(self, t):
        """Observed constant turn, with only two seconds of acceleration extrapolation."""
        key = round(t, 8)
        if key not in self._target_cache:
            x = y = 0.0
            steps = max(1, math.ceil(t / .1))
            dt = t / steps
            for i in range(steps):
                middle = (i + .5) * dt
                speed = max(0, self.target_speed + self.acceleration * min(middle, 2))
                f = direction(self.course + self.yaw * middle)
                x += speed * f[0] * dt
                y += speed * f[1] * dt
            self._target_cache[key] = ((x, y), (self.course + self.yaw*t) % 360,
                                      max(0, self.target_speed + self.acceleration*min(t, 2)))
        return self._target_cache[key]

    def frame_rates(self, relative, own_velocity, t):
        _, course, speed = self.target_at(t)
        target_velocity = tuple(speed*x for x in direction(course))
        along, across = project(relative, course)
        va, vb = project(tuple(a-b for a, b in zip(own_velocity, target_velocity)), course)
        omega = math.radians(self.yaw)
        return dict(along_rate_mps=va + omega*across, across_rate_mps=vb - omega*along)

    def reference(self):
        """Kinematic motion of the band centre; it is not a chosen control command."""
        a = (self.task['offsetAlongMinM'] + self.task['offsetAlongMaxM']) / 2
        b = (self.task['offsetAcrossMinM'] + self.task['offsetAcrossMaxM']) / 2
        omega = math.radians(self.yaw)
        return dict(along_m=a, across_m=b,
                    centre_error_m=[self.current['along_m']-a, self.current['across_m']-b],
                    centre_velocity_along_mps=self.target_speed-omega*b,
                    centre_velocity_across_mps=omega*a)

    def guidance(self, correction_time):
        """LOS direction from formation kinematics and gradual position recovery, never a speed command."""
        a, b = self.current['along_m'], self.current['across_m']
        oa = interval_error(a, self.control_task['offsetAlongMinM'], self.control_task['offsetAlongMaxM'])
        ob = interval_error(b, self.control_task['offsetAcrossMinM'], self.control_task['offsetAcrossMaxM'])
        distance = math.hypot(oa, ob)
        scale = max(0, 1-.05/distance) if distance else 0
        omega = math.radians(self.yaw)
        va = self.target_speed - omega*b - scale*oa/correction_time
        vb = omega*a - scale*ob/correction_time
        course = (self.course+math.degrees(math.atan2(vb, va))) % 360 if abs(va)+abs(vb)>1e-8 else None
        return dict(course_deg=course, position_correction_time_s=correction_time)

    def forecast(self, motion, t):
        displacement, course, _ = self.target_at(t)
        relative = tuple(self.relative_position[j] + motion['endpoint'][j] - displacement[j] for j in (0, 1))
        velocity = tuple(motion['end_speed']*x for x in direction(motion['end_course_deg']))
        facts = position_facts(relative, self.task, course)
        facts['control_error_m'] = max(0, position_facts(relative, self.control_task, course)['position_error_m']-.05)
        facts.update(self.frame_rates(relative, velocity, t))
        return facts

    def clearance(self, motion, t, dt, combined_radius):
        previous = None
        minimum = math.inf
        for i, point in enumerate(motion['path']):
            target, _, _ = self.target_at(min(i*dt, t))
            relative = tuple(self.relative_position[j]+point[j]-target[j] for j in (0, 1))
            if previous is not None:
                v = tuple(relative[j]-previous[j] for j in (0, 1))
                vv = sum(x*x for x in v)
                s = min(1, max(0, -sum(x*y for x, y in zip(previous, v))/vv)) if vv else 0
                minimum = min(minimum, math.hypot(*(previous[j]+s*v[j] for j in (0, 1))))
            previous = relative
        return minimum-combined_radius
