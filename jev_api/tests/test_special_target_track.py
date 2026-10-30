"""Special-track batch identity, terminal-frame signs, unified choices and state."""

import copy
import math
import unittest

from test_accompany import wire as accompany_wire
from test_navigation import packet, geo
from choice_control import build_payload, decode_action
from geometry.coordinates import KNOT_MPS
from geometry.special_target_track import SpecialTrackGeometry
from geometry.motion import predict_ground_motion
from protocol.frames import TASK, STOP
from protocol.status import status_content
from protocol.unity_snapshot import UnitySnapshotAdapter
from validation import valid_request
from tasks import task6_special_target_track


def wire(stationary=False):
    task, records = accompany_wire(stationary=stationary)
    task['code'] = 'SpecialTargetTrackMode'
    task['params'] = dict(platCode=1001, targetBatch=30015, expectedAngleDeg=0,
                          expectedSpeed=25, expectedDistance=15)
    records[1]['batch'] = 30015
    return task, records


def adapted(stationary=False):
    task, records = wire(stationary)
    adapter = UnitySnapshotAdapter('mps'); adapter.add_task(task, TASK)
    for r in records: adapter.add_situation(r)
    _, request, error = adapter.ready()[0]
    assert error is None, error
    return task, adapter, request


class SpecialTrackTests(unittest.TestCase):
    def test_batch_resolves_different_platform_id_and_snapshot_barrier(self):
        task, records = wire()
        # The adapter must wait for the actual target platform, not platform 30015.
        adapter = UnitySnapshotAdapter(); adapter.add_task(task, TASK)
        adapter.add_situation(records[0]); self.assertEqual(adapter.ready(), [])
        adapter.add_situation(records[1]); _, request, error = adapter.ready()[0]
        self.assertIsNone(error); self.assertTrue(valid_request(request))
        self.assertEqual(request['observation']['target']['platformId'], 3001)
        self.assertEqual(request['observation']['target']['batch'], 30015)
        self.assertEqual(request['observation']['obstacles'], [])

    def test_missing_batch_and_invalid_parameters_fail(self):
        task, records = wire(); records[1]['batch'] = 999
        adapter = UnitySnapshotAdapter(); adapter.add_task(task, TASK)
        for r in records: adapter.add_situation(r)
        self.assertIn('batch', adapter.ready()[0][2])
        for key, value in (('expectedDistance', 0), ('expectedDistance', -1),
                           ('expectedDistance', None), ('expectedDistance', math.inf),
                           ('expectedDistance', math.nan), ('expectedDistance', True),
                           ('expectedDistance', '1'), ('expectedAngleDeg', 360),
                           ('expectedSpeed', -1), ('targetBatch', -1)):
            task, _ = wire(); task['params'][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                UnitySnapshotAdapter().add_task(task, TASK)
        task, records = wire(); task['params']['expectedSpeed'] = 51
        adapter = UnitySnapshotAdapter(); adapter.add_task(task, TASK)
        for r in records: adapter.add_situation(r)
        self.assertIn('capability', adapter.ready()[0][2])
        for key in ('max_yaw_rate_deg_s','max_angular_acceleration_deg_s2','max_acceleration_mps2','max_deceleration_mps2'):
            task, records=wire(); task['unity_only']['execution'][key]=0
            adapter=UnitySnapshotAdapter(); adapter.add_task(task,TASK)
            for r in records: adapter.add_situation(r)
            self.assertIn('physical limit',adapter.ready()[0][2])

    def test_distance_defaults_to_one_metre_and_explicit_values_are_preserved(self):
        for supplied in (None, 1, 10, 15):
            with self.subTest(supplied=supplied):
                task, records = wire()
                if supplied is None:
                    del task['params']['expectedDistance']
                else:
                    task['params']['expectedDistance'] = supplied
                adapter = UnitySnapshotAdapter(); adapter.add_task(task, TASK)
                for record in records: adapter.add_situation(record)
                _, request, error = adapter.ready()[0]
                self.assertIsNone(error)
                self.assertTrue(valid_request(request))
                distance = 1.0 if supplied is None else supplied
                self.assertEqual(adapter.tasks[(task['simId'], task['id'])]['expectedDistance'], distance)
                self.assertEqual(request['observation']['task']['expectedDistance'], distance)
                geometry = SpecialTrackGeometry(request['observation'])
                self.assertAlmostEqual(geometry.facts((0, distance), 0, 0, 180)['position_error_m'], 0)

    def test_terminal_geometry_distinguishes_position_and_bow_angles(self):
        _, _, request = adapted(); g = SpecialTrackGeometry(request['observation'])
        for bearing in (0, 90, 180, 270):
            g.task['expectedAngleDeg'] = bearing
            a = math.radians(bearing)
            relative = (15*math.sin(a), 15*math.cos(a))
            f = g.facts(relative, 0, 25*KNOT_MPS, (bearing+180)%360)
            self.assertAlmostEqual(f['bearing_error_deg'], 0)
            self.assertAlmostEqual(f['pointing_error_deg'], 0)
            self.assertAlmostEqual(f['position_error_m'], 0)
            self.assertAlmostEqual(f['speed_error_mps'], 0)
        f = g.facts((0, 15), 0, 0, 0)
        self.assertEqual(abs(f['pointing_error_deg']), 180)

    def test_stationary_direction_motion_history_and_stop(self):
        task, adapter, request = adapted(stationary=True)
        self.assertTrue(valid_request(request))
        self.assertEqual(request['observation']['task']['reference_course_deg'], 0)
        fresh_task, records = wire(stationary=True)
        fresh_task['unity_only'].update(decision_id='stop', tick=50, situation_timestamp=124)
        adapter.add_task(fresh_task, STOP)
        for r in records: adapter.add_situation(dict(r, timestamp=124))
        obs = adapter.ready()[0][1]['observation']
        request = packet(obs); request['type'] = 'stop_request'
        self.assertTrue(valid_request(request))
        payload = build_payload(obs)
        self.assertEqual(len(payload['questions']['action']['criteria']), 128)
        self.assertEqual(payload['state']['task']['stage'], 'immediate_stop')
        self.assertNotIn('terminal', payload['questions']['action']['instructions'].lower())

    def test_first_capture_and_geometry_based_request_stage(self):
        _, _, request = adapted(stationary=True); obs = request['observation']
        obs['boat']['targetSpeed'] = 25
        obs['execution']['heading_course_deg'] = 180
        obs['execution']['yaw_rate_deg_s'] = 0
        obs['execution']['active_target_course_deg'] = 180
        obs['execution']['active_course_error_deg'] = 0
        g = SpecialTrackGeometry(obs); g.relative = (0, 30)
        motion = predict_ground_motion(25*KNOT_MPS,25*KNOT_MPS,180,obs['execution'],2,route_details=False)
        clearance, first = g.path_facts(motion,2,obs['execution']['fixed_dt_s'])
        self.assertGreater(clearance,0)  # Later virtual contact cannot occur after capture freezes Unity.
        self.assertTrue(first['is_capture'])
        self.assertLess(first['t_s'],1)
        self.assertGreater(first['position_error_m'],5)
        obs['operation']='stop'
        stopped_clearance, stopped_pass=g.path_facts(motion,2,obs['execution']['fixed_dt_s'])
        self.assertLess(stopped_clearance,0)  # STOP suspends capture, so it must see the full collision risk.
        self.assertFalse(stopped_pass['is_capture'])
        obs['operation']='execute'
        g.relative=(0,100); g.current=g.facts(g.relative,0,25*KNOT_MPS,180)
        self.assertEqual(g.approach()['stage'],'terminal_approach')
        g.relative=(40,100); g.current=g.facts(g.relative,0,25*KNOT_MPS,180)
        self.assertEqual(g.approach()['stage'],'setup_approach')
        g.relative=(100,100); g.current=g.facts(g.relative,0,25*KNOT_MPS,180)
        self.assertEqual(g.approach()['stage'],'setup_approach')

    def test_terminal_handover_is_latched_but_new_episode_resets_it(self):
        task, records = wire(stationary=True)
        records[1].update(longitude=geo(0,0)[0],latitude=geo(0,0)[1])
        records[0].update(longitude=geo(0,330)[0],latitude=geo(0,330)[1])
        adapter=UnitySnapshotAdapter(); adapter.add_task(task,TASK)
        for r in records: adapter.add_situation(r)
        obs=adapter.ready()[0][1]['observation']
        self.assertEqual(obs['task']['request_stage'],'terminal_approach')
        self.assertGreater(SpecialTrackGeometry(obs).current['position_error_m'], obs['boat']['collision_radius_m'])
        self.assertEqual(task6_special_target_track.status_params(task['params'], obs)['inTerminal'], 1)
        next_task=copy.deepcopy(task)
        next_task['unity_only'].update(decision_id='next',tick=50,situation_timestamp=124)
        records[0].update(longitude=geo(80,100)[0],latitude=geo(80,100)[1])
        records=[dict(r,timestamp=124) for r in records]
        adapter.add_task(next_task,TASK)
        for r in records: adapter.add_situation(r)
        obs=adapter.ready()[0][1]['observation']
        self.assertEqual(SpecialTrackGeometry(obs).approach()['stage'],'setup_approach')
        self.assertEqual(obs['task']['request_stage'],'terminal_approach')
        self.assertEqual(task6_special_target_track.status_params(task['params'], obs)['inTerminal'], 1)
        next_task=dict(next_task,simId='new-episode',id='new-run')
        adapter.add_task(next_task,TASK)
        for r in records: adapter.add_situation(r)
        obs=adapter.ready()[0][1]['observation']
        self.assertEqual(obs['task']['request_stage'],'setup_approach')
        self.assertEqual(task6_special_target_track.status_params(task['params'], obs)['inTerminal'], 0)

    def test_in_terminal_follows_decision_phase_independently_of_capture(self):
        task, adapter, request = adapted(stationary=True)
        obs = request['observation']
        target = obs['target']
        # Put the boat inside the capture circle, then report each decision phase.
        target.update(longitude=geo(0, 0)[0], latitude=geo(0, 0)[1])
        obs['boat'].update(longitude=geo(0, 15)[0], latitude=geo(0, 15)[1])
        self.assertLess(SpecialTrackGeometry(obs).current['position_error_m'], obs['boat']['collision_radius_m'])
        for stage, expected in (('setup_approach', 0), ('terminal_approach', 1), ('setup_approach', 0)):
            with self.subTest(stage=stage):
                obs['task']['request_stage'] = stage
                response = status_content(task, 1, 'tracking', adapter.tasks, observation=obs)
                self.assertEqual(response['params']['inTerminal'], expected)
                self.assertEqual(build_payload(obs)['state']['task']['stage'], stage)
                # Completion and STOP events preserve Unity's mirrored phase and endpoint diagnostics.
                endpoint = dict(task, params=dict(response['params'], unity_only=dict(position_error_m=2)))
                for description in ('position_reached', 'stopped'):
                    final = status_content(endpoint, 2, description, adapter.tasks)['params']
                    self.assertEqual(final['inTerminal'], expected)
                    self.assertEqual(final['unity_only']['position_error_m'], 2)

    def test_payload_128_fixed_choices_units_and_status(self):
        task, adapter, request = adapted(); obs = request['observation']; before = copy.deepcopy(obs)
        mps = build_payload(obs)
        self.assertEqual(obs, before)
        self.assertEqual(mps['state']['task']['stage'], 'setup_approach')
        self.assertEqual(mps['state']['execution']['hull_length_m'], obs['execution']['hull_length_m'])
        self.assertEqual(len(mps['questions']['action']['criteria']), 128)
        row = task6_special_target_track.build_payload(obs)['questions']['action']['criteria']['speed_4__south']
        self.assertEqual(row['course_deg'], 180)
        self.assertAlmostEqual(row['command_speed_mps'], 17.5*KNOT_MPS, delta=.001)
        self.assertEqual(len(row['setup_error_m']), 2)
        obs['model_speed_unit'] = 'knots'; knots = build_payload(obs)
        text_knots = knots['questions']['action']['criteria']['speed_4__south']
        self.assertIn('Set 17.500 knots, absolute course 180 deg.', text_knots)
        self.assertIn('Setup point error after action/if held',text_knots)
        self.assertIn(f"Set {17.5*KNOT_MPS:.3f} m/s", mps['questions']['action']['criteria']['speed_4__south'])
        for name in mps['questions']['action']['criteria']:
            action = decode_action({'answers':{'action':{'type':'choice','choice':name}}}, obs)
            self.assertEqual(action['course'] % 22.5, 0)
        response = status_content(task, 1, 'tracking', adapter.tasks,
            result=dict(decision_id='d', tick=0), control_stamp=456, observation=obs)
        params = response['params']
        self.assertEqual(params['targetBatch'], 30015)
        self.assertEqual(params['inTerminal'], 0)
        self.assertIn('pointing_error_deg', params['unity_only'])
        self.assertEqual(params['unity_only']['control_timestamp'], 456)
        self.assertFalse(params['unity_only']['four_conditions_accepted'])
        obs['task']['request_stage']='terminal_approach'
        terminal=build_payload(obs)
        self.assertNotIn('hull_length_m', terminal['state']['execution'])
        self.assertIn('Reference deviation command/actual end',terminal['questions']['action']['criteria']['speed_4__south'])
        self.assertIn('guidance',terminal['state'])
        # A Unity terminal event carries actual endpoint errors, not the previous observation.
        end = dict(task, params=dict(targetBatch=30015, relativeDistanceM=16,
            unity_only=dict(pointing_error_deg=2, position_reached=True, four_conditions_accepted=False)))
        self.assertEqual(status_content(end, 2, 'position_reached', adapter.tasks)['params']['relativeDistanceM'],16)


if __name__ == '__main__':
    unittest.main()
