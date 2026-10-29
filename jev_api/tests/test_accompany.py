"""Task-3 wire adaptation, frame signs, speed caps and distinct model requests."""

import copy
import json
import math
import unittest

import test_navigation as navigation
from test_navigation import geo, packet
from choice_control import build_payload, ACTION_CHOICES
from geometry.accompany import project, interval_error
from protocol.frames import TASK, STOP
from protocol.unity_snapshot import UnitySnapshotAdapter
from protocol.status import status_content
from validation import valid_request


def wire(sync=0, cap=0, stationary=False):
    task, records = navigation.FollowingTests().wire()
    task['code'] = 'accompanyMode'
    task['params'] = dict(platCode=1001, targetPlatCode=3001, offsetAlongMinM=-160,
        offsetAlongMaxM=-140, offsetAcrossMinM=70, offsetAcrossMaxM=90, syncHeading=sync, maxSpeed=cap)
    task['unity_only']['collision_radii_m'].pop('2001')
    task['unity_only']['execution']['obstacle_avoidance_radii_m'] = {}
    task['unity_only']['target_reference_course_deg'] = 0
    records = records[:2]
    records[1].update(longitude=geo(-80, 150)[0], latitude=geo(-80, 150)[1])
    if stationary:
        records[1].update(targetSpeed=0, targetCourse=None)
    return task, records


def adapted(sync=0, cap=0, stationary=False):
    task, records = wire(sync, cap, stationary)
    adapter = UnitySnapshotAdapter('mps')
    adapter.add_task(task, TASK)
    for record in records:
        adapter.add_situation(record)
    _, request, error = adapter.ready()[0]
    assert error is None, error
    return task, adapter, request


class AccompanyTests(unittest.TestCase):
    def test_body_axes_and_closed_interval_error(self):
        self.assertEqual(project((80, -150), 0), (-150, 80))
        along, across = project((-150, -80), 90)
        self.assertAlmostEqual(along, -150)
        self.assertAlmostEqual(across, 80)
        self.assertEqual([interval_error(v, -160, -140) for v in (-200, -160, -150, -140, -100)], [-40, 0, 0, 0, 40])

    def test_signed_reverse_speed_uses_actual_movement_direction(self):
        from geometry.accompany import AccompanyGeometry
        from geometry.coordinates import KNOT_MPS
        _, _, request = adapted(stationary=True)
        obs = request['observation']
        obs['boat'].update(targetSpeed=-1, targetCourse=180)
        self.assertAlmostEqual(AccompanyGeometry(obs).current['along_rate_mps'], -KNOT_MPS)

    def test_task_cap_generates_all_eight_speed_bins(self):
        for cap, maximum in ((0, 50), (18, 18), (100, 50)):
            _, _, request = adapted(cap=cap)
            self.assertTrue(valid_request(request))
            self.assertEqual(request['observation']['action_space']['speeds_knots'],
                             [maximum*f for f in (0, .05, .1, .2, .35, .5, .75, 1)])

    def test_variants_keep_position_only_free_of_heading_objective(self):
        for sync in (0, 1):
            _, _, request = adapted(sync)
            obs = request['observation']; before = copy.deepcopy(obs)
            payload = build_payload(obs)
            self.assertEqual(obs, before)
            self.assertEqual('heading_sync' in payload['state'], bool(sync))
            self.assertEqual(payload['state']['task']['objective'], 'position_and_heading' if sync else 'position_only')
            choices = payload['questions']['action']['criteria']
            self.assertEqual(list(choices), list(ACTION_CHOICES))
            for candidate in choices.values():
                facts = json.loads(candidate)
                self.assertEqual('heading_error_deg' in facts, bool(sync))
                self.assertEqual(len(facts['position_m']), 3)
                self.assertGreater(facts['min_clearance_m'], 0)
            equal_speed = json.loads(choices['speed_3__north'])
            self.assertEqual(equal_speed['position_m'][0], [-150, 80])
            self.assertEqual(equal_speed['end_rate_mps'], [0, 0])
            self.assertEqual(equal_speed['preview_rate_mps'], [0, 0])

    def test_static_target_uses_supplied_reference_without_history(self):
        task, records = wire(stationary=True)
        task['unity_only']['target_reference_course_deg'] = 45
        adapter = UnitySnapshotAdapter('mps'); adapter.add_task(task, TASK)
        for r in records: adapter.add_situation(r)
        obs = adapter.ready()[0][1]['observation']
        self.assertEqual(obs['task']['reference_course_deg'], 45)
        self.assertTrue(valid_request(packet(obs)))
        build_payload(obs)

    def test_model_units_preserve_position_and_heading_facts(self):
        from geometry.coordinates import KNOT_MPS
        _, _, request = adapted(1, 18)
        obs = request['observation']
        mps = build_payload(obs)
        obs['model_speed_unit'] = 'knots'
        knots = build_payload(obs)
        self.assertEqual(knots['state']['units']['speed'], 'knots')
        for name in ACTION_CHOICES:
            a = json.loads(mps['questions']['action']['criteria'][name])
            b = json.loads(knots['questions']['action']['criteria'][name])
            for key in ('position_m', 'min_clearance_m', 'heading_error_deg'):
                self.assertEqual(a[key], b[key])
            for va, vb in zip(a['preview_rate_mps'], b['preview_rate_knots']):
                self.assertAlmostEqual(va/KNOT_MPS, vb, delta=.0015)

    def test_rotating_frame_requires_offset_velocity(self):
        from geometry.accompany import AccompanyGeometry
        _, _, request = adapted()
        obs = request['observation']
        obs['task']['target_motion'] = dict(yaw_deg_s=2, acceleration_mps2=0)
        g = AccompanyGeometry(obs)
        a, b = g.current['along_m'], g.current['across_m']
        omega = math.radians(2)
        # North-facing target: forward is north, starboard is east.
        velocity = (omega*a, g.target_speed-omega*b)
        rates = g.frame_rates(g.relative_position, velocity, 0)
        self.assertAlmostEqual(rates['along_rate_mps'], 0, places=8)
        self.assertAlmostEqual(rates['across_rate_mps'], 0, places=8)
        uncorrected = g.frame_rates(g.relative_position, (0, g.target_speed), 0)
        self.assertGreater(math.hypot(*uncorrected.values()), 5)
        self.assertAlmostEqual(g.guidance(12)['course_deg'], 294.18881431966855, places=4)
        # Target's observed turn predicts a circular arc, not the tangent line.
        position, course, speed = g.target_at(10)
        self.assertAlmostEqual(course, 20)
        self.assertAlmostEqual(speed, g.target_speed)
        self.assertAlmostEqual(position[0], g.target_speed/omega*(1-math.cos(omega*10)), delta=.001)

    def test_motion_estimation_uses_simulation_ticks_and_resets(self):
        task, records = wire()
        adapter = UnitySnapshotAdapter('mps')
        adapter.add_task(task, TASK)
        for record in records: adapter.add_situation(record)
        adapter.ready()
        next_task = copy.deepcopy(task)
        next_task['unity_only'].update(decision_id='next', tick=50, situation_timestamp=124)
        adapter.add_task(next_task, TASK)
        for record in records:
            r = dict(record, timestamp=124)
            if r['platformId'] == 3001: r['targetCourse'] = 2
            adapter.add_situation(r)
        obs = adapter.ready()[0][1]['observation']
        yaw = obs['task']['target_motion']['yaw_deg_s']
        self.assertGreater(yaw, 1)
        self.assertLessEqual(yaw, 2)
        # Independent adapter/episode has no previous motion estimate.
        _, _, fresh = adapted()
        self.assertEqual(fresh['observation']['task']['target_motion']['yaw_deg_s'], 0)

    def test_near_band_recovery_remains_a_position_objective(self):
        _, _, request = adapted(1)
        obs = request['observation']
        lon, lat = geo(11.3, -19.4)  # own at along=-169.4, across=91.3
        obs['boat'].update(longitude=lon, latitude=lat)
        payload = build_payload(obs)
        self.assertEqual(payload['state']['task']['stage'], 'acquire_position')
        self.assertFalse(payload['state']['heading_sync']['active'])
        self.assertEqual(list(payload['questions']['action']['criteria']), list(ACTION_CHOICES))
        choices = payload['questions']['action']['criteria']
        matching = json.loads(choices['speed_3__north'])
        correcting = json.loads(choices['speed_4__north_northwest'])
        self.assertAlmostEqual(matching['control_gain_m'], 0, delta=.001)
        self.assertGreater(correcting['control_gain_m'], 0)
        self.assertNotIn('heading_error_deg', correcting)
        self.assertEqual(len(correcting['band_error_m']), 3)

    def test_interior_reference_does_not_change_customer_position_status(self):
        task, adapter, request = adapted(1)
        obs = request['observation']
        lon, lat = geo(7, -7)  # along=-157, across=87: in customer band, outside middle half
        obs['boat'].update(longitude=lon, latitude=lat)
        payload = build_payload(obs)
        self.assertTrue(payload['state']['relative']['in_position'])
        self.assertFalse(payload['state']['relative']['in_control_region'])
        self.assertEqual(payload['state']['task']['control_along_m'], [-155, -145])
        self.assertEqual(payload['state']['task']['control_across_m'], [75, 85])
        self.assertEqual(payload['state']['task']['stage'], 'acquire_position')
        status = status_content(task, 1, 'accompanying', adapter.tasks, observation=obs)
        self.assertEqual(status['params']['inPosition'], 1)
        self.assertEqual(status['params']['alongOutM'], 0)
        self.assertEqual(status['params']['acrossOutM'], 0)

    def test_static_reference_has_no_spurious_direction_at_control_boundary(self):
        from geometry.accompany import AccompanyGeometry
        _, _, request = adapted(1, stationary=True)
        obs = request['observation']
        lon, lat = geo(5.02, 0)  # along=-150, across=85.02, within observation tolerance
        obs['boat'].update(longitude=lon, latitude=lat)
        g = AccompanyGeometry(obs)
        self.assertTrue(g.current['in_position'])
        self.assertTrue(g.current['in_control_region'])
        self.assertEqual(g.current['control_error_m'], 0)
        self.assertIsNone(g.guidance(12)['course_deg'])

    def test_current_status_and_stop_keep_customer_code_and_parameters(self):
        task, adapter, request = adapted(1, 18)
        content = status_content(task, 1, 'accompanying', adapter.tasks, observation=request['observation'])
        self.assertEqual(content['code'], 'accompanyMode')
        self.assertEqual(content['params']['inPosition'], 1)
        self.assertAlmostEqual(content['params']['currentAlongM'], -150, places=4)
        self.assertAlmostEqual(content['params']['currentAcrossM'], 80, places=4)
        self.assertNotIn('currentRoute', content['params'])
        stop = copy.deepcopy(task); stop.pop('params')
        stop['unity_only'].update(decision_id='stop', tick=50, situation_timestamp=124)
        adapter.add_task(stop, STOP)
        _, records = wire()
        for r in records: adapter.add_situation(dict(r, timestamp=124))
        _, request, error = adapter.ready()[0]
        self.assertIsNone(error)
        self.assertTrue(valid_request(request))
        obs = request['observation']
        self.assertEqual(obs['task']['code'], 'accompanyMode')
        self.assertEqual(obs['operation'], 'stop')
        payload = build_payload(obs)
        self.assertEqual(payload['state']['task']['stage'], 'immediate_stop')
        self.assertNotIn('heading_sync', payload['state'])

    def test_invalid_parameters_and_incomplete_target_are_rejected(self):
        for field, value in (('syncHeading', 2), ('syncHeading', True), ('maxSpeed', -1),
                             ('offsetAlongMinM', 1), ('targetPlatCode', 1001)):
            task, _ = wire(); task['params'][field] = value
            with self.assertRaises(ValueError): UnitySnapshotAdapter().add_task(task, TASK)
        task, records = wire()
        adapter = UnitySnapshotAdapter(); adapter.add_task(task, TASK)
        adapter.add_situation(records[0])
        self.assertEqual(adapter.ready(), [])


if __name__ == '__main__':
    unittest.main()
