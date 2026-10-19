"""Test diagnostic design independently of model outputs and PD equality."""
import copy
import random
import unittest
from control_failure_cases import SEED, FOUR_SEED, observations, payload, orders, acceptable, evaluate


class DiagnosticTests(unittest.TestCase):
    def test_candidate_coverage_and_no_fixture_mutation(self):
        for name, obs in observations():
            before = copy.deepcopy(obs)
            for backend in ('laya','kev'):
                for family, count in (('four',4), ('linear8',8), ('angular7',7), ('joint56',56)):
                    for style in ('short','brief','full'):
                        p, mapping = payload(name, obs, family, style, backend)
                        self.assertEqual(len(mapping), count)
                        self.assertEqual(len({x['physical_id'] for x in mapping.values()}), count)
                        self.assertTrue(any(acceptable(name, obs, x['action'],family)['reasonable'] for x in mapping.values()))
            self.assertEqual(obs, before)

    def test_counterfactuals_only_change_named_field(self):
        states = dict(observations())
        for a,b,field,section in [('left_rest','right_rest','position_xy_m','target'),
                                 ('ahead_reverse','ahead_forward','speed_mps','boat'),
                                 ('inside_forward','inside_reverse','speed_mps','boat'),
                                 ('yaw_left','yaw_right','yaw_rate_deg_s','boat')]:
            x,y = copy.deepcopy(states[a]),copy.deepcopy(states[b])
            x[section].pop(field); y[section].pop(field)
            self.assertEqual(x,y)

    def test_six_random_orders_match_official_rng(self):
        for n in (4,7,8,56):
            keys = list(range(n)); rng = random.Random(FOUR_SEED if n == 4 else SEED)
            expected = [keys]
            for _ in range(6):
                arr = keys.copy(); rng.shuffle(arr); expected.append(arr)
            self.assertEqual(orders(keys), expected)
            self.assertEqual(len({tuple(x) for x in expected[1:]}), 6)
            self.assertNotIn(tuple(keys), {tuple(x) for x in expected[1:]})

    def test_multiple_actions_valid_and_no_pd_exact_requirement(self):
        name, obs = next(observations())
        for a in (4,5,6,7):
            self.assertTrue(acceptable(name, obs, {'acceleration_index':a, 'turn_index':3})['reasonable'])
        self.assertFalse(acceptable(name, obs, {'acceleration_index':0, 'turn_index':3})['reasonable'])

    def test_mapping_independent_of_position(self):
        name, obs = next(observations())
        _, mapping = payload(name,obs,'joint56','brief','laya')
        expected = mapping['k52']
        for order in orders(mapping):
            reordered = {k:mapping[k] for k in order}
            ans = {'choice':'k52','probabilities':{k:float(k=='k52') for k in order}}
            self.assertEqual(evaluate(name,obs,'joint56',reordered,ans)['selected'],expected)


if __name__ == '__main__':
    unittest.main()
