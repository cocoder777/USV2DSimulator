"""Prevent accompaniment's finer grid from changing the other tasks or STOP."""

import copy
import unittest

import test_navigation as navigation
from test_navigation import observation, packet
from test_accompany import adapted
from choice_control import build_payload, decode_action
from protocol.frames import TASK
from protocol.unity_snapshot import UnitySnapshotAdapter
from validation import valid_request


class TaskActionSpaceTests(unittest.TestCase):
    def test_each_task_and_stop_use_their_own_grid_and_decode_bearings(self):
        task, records = navigation.FollowingTests().wire()
        adapter = UnitySnapshotAdapter()
        adapter.add_task(task, TASK)
        for record in records:
            adapter.add_situation(record)
        following = adapter.ready()[0][1]['observation']
        accompany = adapted(cap=18)[2]['observation']
        # Switch repeatedly so a previously built accompaniment cannot change a later route.
        for original, count, step in ((observation(), 64, 45), (accompany, 128, 22.5),
                                      (following, 64, 45), (observation(), 64, 45)):
            for operation in ('execute', 'stop'):
                with self.subTest(code=original['task']['code'], operation=operation):
                    obs = copy.deepcopy(original)
                    obs['operation'] = operation
                    if operation == 'stop':
                        obs['task']['stop_speed_mps'] = 0.000001
                    request = packet(obs)
                    request['type'] = 'stop_request' if operation == 'stop' else 'decision_request'
                    self.assertTrue(valid_request(request))
                    choices = build_payload(obs)['questions']['action']['criteria']
                    self.assertEqual(len(choices), count)
                    for name in choices:
                        result = decode_action({'answers': {'action': {'type': 'choice', 'choice': name}}}, obs)
                        self.assertEqual(result['course'] % step, 0)
                    east = decode_action({'answers': {'action': {'type': 'choice', 'choice': 'speed_7__east'}}}, obs)
                    self.assertEqual(east['course'], 90)
                    self.assertEqual(east['speed'], 18 if count == 128 else 50)
                    fine = {'answers': {'action': {'type': 'choice', 'choice': 'speed_1__north_northeast'}}}
                    if count == 128:
                        self.assertEqual(decode_action(fine, obs)['course'], 22.5)
                    else:
                        with self.assertRaises(ValueError):
                            decode_action(fine, obs)

    def test_validation_rejects_a_grid_from_another_task(self):
        route = observation()
        accompany = adapted()[2]['observation']
        route['action_space']['courses_deg'], accompany['action_space']['courses_deg'] = (
            accompany['action_space']['courses_deg'], route['action_space']['courses_deg'])
        self.assertFalse(valid_request(packet(route)))
        self.assertFalse(valid_request(packet(accompany)))
        route = observation()
        route['action_space']['courses_deg'][1] = 22.5
        self.assertFalse(valid_request(packet(route)))


if __name__ == '__main__':
    unittest.main()
