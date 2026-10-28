"""Target-frame position-band and relative-velocity facts for accompaniment."""

import math

from geometry.coordinates import KNOT_MPS, xy
from geometry.relative_motion import course_error, direction


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
        self.course = self.task['reference_course_deg']
        boat, target = observation['boat'], observation['target']
        delta = xy((target['longitude'], target['latitude']), (boat['longitude'], boat['latitude']))
        self.relative_position = (-delta[0], -delta[1])  # own minus target
        self.target_speed = target['targetSpeed'] * KNOT_MPS
        self.target_velocity = tuple(self.target_speed*x for x in direction(self.course))
        self.current = position_facts(self.relative_position, self.task, self.course)
        own_course = boat['targetCourse'] if boat['targetCourse'] is not None else observation['execution']['heading_course_deg']
        # The situation course is already the movement direction, including reverse.
        own_velocity = tuple(abs(boat['targetSpeed'])*KNOT_MPS*x for x in direction(own_course))
        self.current.update(self.relative_velocity(own_velocity))

    def relative_velocity(self, own_velocity):
        along, across = project(tuple(a-b for a, b in zip(own_velocity, self.target_velocity)), self.course)
        return dict(relative_along_mps=along, relative_across_mps=across)

    def candidate(self, motion, duration, synchronize):
        relative = tuple(self.relative_position[j] + motion['endpoint'][j] - self.target_velocity[j]*duration
                         for j in (0, 1))
        facts = position_facts(relative, self.task, self.course)
        own_velocity = tuple(motion['end_speed']*x for x in direction(motion['end_course_deg']))
        facts.update(self.relative_velocity(own_velocity))
        if synchronize:
            facts['heading_error_deg'] = abs(course_error(self.course, motion['end_course_deg']))
        return facts
