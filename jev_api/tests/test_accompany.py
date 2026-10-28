"""Task-3 wire adaptation, frame signs, speed caps and distinct model requests."""

import copy
import json
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
        self.assertAlmostEqual(AccompanyGeometry(obs).current['relative_along_mps'], -KNOT_MPS)

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
                self.assertNotIn('preview', candidate)
            equal_speed = json.loads(choices['speed_3__north'])
            self.assertTrue(equal_speed['in_position'])
            self.assertAlmostEqual(equal_speed['position_error_m'], 0)
            self.assertAlmostEqual(equal_speed['relative_along_mps'], 0)
            self.assertAlmostEqual(equal_speed['relative_across_mps'], 0)

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
            for key in ('along_m', 'across_m', 'position_error_m', 'heading_error_deg'):
                self.assertEqual(a[key], b[key])
            self.assertAlmostEqual(a['relative_along_mps']/KNOT_MPS, b['relative_along_knots'], delta=.0015)

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
