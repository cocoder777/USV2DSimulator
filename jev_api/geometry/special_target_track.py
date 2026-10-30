"""Task-6 terminal geometry and observed-target forecasts; never selects actions."""

import math

from geometry.coordinates import KNOT_MPS, xy
from geometry.relative_motion import course_error, direction
from geometry.motion import bounded_turn


class SpecialTrackGeometry:
    def __init__(self, obs):
        self.obs, self.task = obs, obs['task']
        boat, target = obs['boat'], obs['target']
        delta = xy((target['longitude'], target['latitude']), (boat['longitude'], boat['latitude']))
        self.relative = (-delta[0], -delta[1])
        self.course = self.task['reference_course_deg']
        self.speed = target['targetSpeed'] * KNOT_MPS
        estimate = self.task.get('target_motion', {})
        self.yaw = estimate.get('yaw_deg_s', 0)
        self.acceleration = estimate.get('acceleration_mps2', 0)
        self.cache = {}
        expected = self.task['expectedImpactSpeed']*KNOT_MPS
        execution = obs['execution']
        for key in ('max_yaw_rate_deg_s','max_angular_acceleration_deg_s2','max_acceleration_mps2','max_deceleration_mps2'):
            value = execution[key]
            if type(value) not in (int,float) or not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid special-track physical limit: '+key)
        nominal_radius = expected/math.radians(execution['max_yaw_rate_deg_s'])
        turn_time = 180/execution['max_yaw_rate_deg_s']+2*execution['max_yaw_rate_deg_s']/execution['max_angular_acceleration_deg_s2']
        closing = max(0,expected+self.speed*math.cos(math.radians(self.task['expectedImpactAngleDeg'])))
        acceleration_room = expected*expected/(2*execution['max_acceleration_mps2'])
        # Also reserve the distance the target closes during turn/speed setup;
        # two turn radii alone left too little time to regain terminal speed.
        self.setup_length = max(2*boat['collision_radius_m'],2*nominal_radius,closing*turn_time+acceleration_room)
        self.setup_capture_radius = max(boat['collision_radius_m'],nominal_radius)
        self.current = self.facts(self.relative, self.course, boat['targetSpeed'] * KNOT_MPS,
                                  obs['execution']['heading_course_deg'])

    def setup_error(self, relative, course):
        f = direction(course+self.task['expectedImpactAngleDeg'])
        return math.hypot(*(relative[j]-(self.task['expectedDistance']+self.setup_length)*f[j] for j in (0, 1)))

    def terminal_guidance(self, horizon):
        # A short observed-motion interception reference; no chosen command/speed.
        travel = self.task['expectedImpactSpeed']*KNOT_MPS+self.speed
        t = min(horizon,max(self.obs['planning']['action_duration_s'],self.current['position_error_m']/max(travel,1e-6)))
        target, course = self.target_at(t)
        forward = direction(course+self.task['expectedImpactAngleDeg'])
        delta = tuple(target[j]+self.task['expectedDistance']*forward[j]-self.relative[j] for j in (0,1))
        return dict(course_deg=math.degrees(math.atan2(*delta))%360 if math.hypot(*delta)>1e-8 else self.current['terminal_heading_deg'],
                    target_prediction_time_s=t)

    def approach(self):
        """Classify from an incoming ray and physical settling times, not scenario labels."""
        e = self.obs['execution']; c = self.current
        incoming = direction(c['terminal_heading_deg'])
        side = (incoming[1], -incoming[0])
        forward = direction(self.course+self.task['expectedImpactAngleDeg'])
        q = tuple(self.relative[j]-self.task['expectedDistance']*forward[j] for j in (0, 1))
        outside = -sum(x*y for x,y in zip(q,incoming))
        lateral = sum(x*y for x,y in zip(q,side))
        # Settling time from the SAME angular plant used in motion prediction.
        delta = -course_error(c['terminal_heading_deg'],e['heading_course_deg'])
        lo, hi = 0.0, 360/e['max_yaw_rate_deg_s']+4*e['max_yaw_rate_deg_s']/e['max_angular_acceleration_deg_s2']
        for _ in range(32):
            mid = (lo+hi)/2
            angle, yaw = bounded_turn(delta,-e['yaw_rate_deg_s'],e['max_yaw_rate_deg_s'],e['max_angular_acceleration_deg_s2'],mid)
            if abs(angle-delta)<1e-6 and abs(yaw)<1e-6: hi=mid
            else: lo=mid
        expected = self.task['expectedImpactSpeed']*KNOT_MPS
        actual = self.obs['boat']['targetSpeed']*KNOT_MPS
        speed_time = abs(expected-actual)/(e['max_acceleration_mps2'] if expected>=actual else e['max_deceleration_mps2'])
        vt = tuple(self.speed*x for x in direction(self.course))
        omega = math.radians(self.yaw)
        offset = tuple(self.task['expectedDistance']*x for x in forward)
        goal_velocity = (vt[0]+omega*offset[1],vt[1]-omega*offset[0])
        closing = expected-sum(x*y for x,y in zip(goal_velocity,incoming))
        radius = self.obs['boat']['collision_radius_m']
        available = max(0,outside-radius)/closing if closing>1e-6 else 0
        ready = outside>0 and abs(lateral)<=radius and closing>0 and max(hi,speed_time)<=max(available,self.obs['planning']['action_duration_s'])
        # Reaching the setup area must also hand over, otherwise an outward-facing
        # boat would wait forever for inward heading readiness before it may turn.
        at_setup = self.setup_error(self.relative,self.course) <= self.setup_capture_radius
        ready = ready or at_setup
        setup_offset = tuple((self.task['expectedDistance']+self.setup_length)*x for x in forward)
        correction_time = 180/e['max_yaw_rate_deg_s']+2*e['max_yaw_rate_deg_s']/e['max_angular_acceleration_deg_s2']
        setup_velocity = (vt[0]+omega*setup_offset[1]-(self.relative[0]-setup_offset[0])/correction_time,
                          vt[1]-omega*setup_offset[0]-(self.relative[1]-setup_offset[1])/correction_time)
        setup_course = math.degrees(math.atan2(*setup_velocity))%360 if math.hypot(*setup_velocity)>1e-8 else None
        return dict(stage='terminal_approach' if ready else 'setup_approach',
            incoming_axis_distance_m=outside, incoming_axis_lateral_m=lateral,
            estimated_time_to_capture_s=available, heading_settle_s=hi, speed_settle_s=speed_time,
            setup_point_distance_beyond_terminal_m=self.setup_length,
            setup_handover_radius_m=self.setup_capture_radius,
            current_setup_error_m=self.setup_error(self.relative,self.course),
            setup_guidance_course_deg=setup_course)

    def target_at(self, t):
        key = round(t, 8)
        if key not in self.cache:
            x = y = 0.0
            count = max(1, math.ceil(t / .1))
            dt = t / count
            for i in range(count):
                mid = (i + .5) * dt
                speed = max(0, self.speed + self.acceleration * min(mid, 2))
                f = direction(self.course + self.yaw * mid)
                x += speed * f[0] * dt
                y += speed * f[1] * dt
            self.cache[key] = ((x, y), (self.course + self.yaw * t) % 360)
        return self.cache[key]

    def facts(self, relative, course, speed, heading):
        distance = math.hypot(*relative)
        bearing = (math.degrees(math.atan2(relative[0], relative[1])) - course) % 360
        f = direction(course + self.task['expectedImpactAngleDeg'])
        error = tuple(relative[j] - self.task['expectedDistance'] * f[j] for j in (0, 1))
        pointing = (math.degrees(math.atan2(-relative[0], -relative[1]))) % 360
        return dict(distance_m=distance, bearing_deg=bearing,
                    distance_error_m=distance-self.task['expectedDistance'],
                    bearing_error_deg=course_error(bearing, self.task['expectedImpactAngleDeg']),
                    speed_error_mps=speed-self.task['expectedImpactSpeed']*KNOT_MPS,
                    pointing_error_deg=course_error(heading, pointing),
                    position_error_m=math.hypot(*error), actual_speed_mps=speed,
                    heading_deg=heading, pointing_course_deg=pointing,
                    terminal_heading_deg=(course+self.task['expectedImpactAngleDeg']+180) % 360)

    def at(self, position, speed, heading, t):
        target, course = self.target_at(t)
        relative = tuple(self.relative[j] + position[j] - target[j] for j in (0, 1))
        return self.facts(relative, course, speed, heading)

    def path_facts(self, motion, duration, dt):
        """Swept clearance and closest terminal-point sample share the same timeline."""
        previous = None
        minimum = math.inf
        closest = capture = None
        combined = self.obs['boat']['collision_radius_m'] + self.obs['target']['radius_m']
        for i, point in enumerate(motion['path']):
            t = min(i*dt, duration)
            target, course = self.target_at(t)
            relative = tuple(self.relative[j] + point[j] - target[j] for j in (0, 1))
            f = direction(course+self.task['expectedImpactAngleDeg'])
            error = math.hypot(*(relative[j]-self.task['expectedDistance']*f[j] for j in (0, 1)))
            if closest is None or error < closest['position_error_m']:
                closest = dict(t_s=t, position=point, position_error_m=error)
            if capture is None and self.obs.get('operation') != 'stop' and error <= self.obs['boat']['collision_radius_m']:
                capture = dict(t_s=t, position=point, position_error_m=error)
            if previous is not None:
                v = tuple(relative[j]-previous[j] for j in (0, 1))
                vv = sum(x*x for x in v)
                fraction = min(1, max(0, -sum(x*y for x, y in zip(previous, v))/vv)) if vv else 0
                minimum = min(minimum, math.hypot(*(previous[j]+fraction*v[j] for j in (0, 1))))
            else:
                minimum = math.hypot(*relative)
            if capture is not None and 'clearance_m' not in capture:
                capture['clearance_m'] = minimum-combined
            previous = relative
        selected = capture if capture is not None else closest
        selected['is_capture'] = capture is not None
        # The local experiment freezes on capture, so motion after it cannot
        # collide. A collision BEFORE capture still leaves a nonpositive prefix.
        return capture['clearance_m'] if capture is not None else minimum-combined, selected
