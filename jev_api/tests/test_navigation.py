import copy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from choice_control import build_payload, decode_action, ACTION_CHOICES
from route_context import RouteContext, xy, EARTH_RADIUS
from boat_client import valid_request
from decision_worker import PROTOCOL_VERSION

ORIGIN = [120.5, 30.2]
def geo(x, y):
    return [round(ORIGIN[0] + math.degrees(x / EARTH_RADIUS / math.cos(math.radians(ORIGIN[1]))), 8),
            round(ORIGIN[1] + math.degrees(y / EARTH_RADIUS), 8)]

def observation(points=None, waypoints=None, next_index=1):
    if points is None:
        route = json.loads((Path(__file__).resolve().parents[2] / 'Assets/StreamingAssets/fixed_route.json').read_text())
        points = [(p['x'],p['y']) for p in route['path']['samples']]
        waypoints = [(p['x'],p['y']) for p in route['waypoints']]
    return {'planning': {'action_duration_s': .4, 'simulation_paused_while_deciding': True},
            'action_space': {'speeds_mps': [-1,-.5,0,.5,1.25,2.5,3.75,5], 'heading_offsets_deg': [0,45,90,135,180,-135,-90,-45]},
            'boat': {'lon_lat':geo(*points[0]),'heading_deg':0},
            'task': {'route_id':'test','reference_lon_lat':[geo(*p) for p in points],
                     'origin_lon_lat':ORIGIN, 'planWaypointList':[geo(*p) for p in waypoints],
                     'planWaypointNum':len(waypoints), 'next_waypoint_index':next_index,
                     'currentRoute':next_index-1, 'arrival_radius_m':1, 'stop_speed_mps':.1}}

class NavigationTests(unittest.TestCase):
    def test_geographic_roundtrip(self):
        for p in [(0,0),(10,5),(-200,400)]:
            q=xy(geo(*p),ORIGIN)
            self.assertLess(math.dist(p,q),.002)

    def test_local_left_and_right(self):
        obs=observation([(0,0),(0,10)],[(0,0),(0,10)])
        self.assertEqual(build_payload(obs)['state']['target']['forward_left_m'],[0,10])
        obs['boat']['heading_deg']=180
        self.assertEqual(build_payload(obs)['state']['target']['forward_left_m'],[0,-10])

    def test_all_choices_decode_and_negative_motion(self):
        payload=build_payload(observation())
        self.assertEqual(len(payload['questions']['action']['criteria']),64)
        for name, action in ACTION_CHOICES.items():
            self.assertEqual(decode_action({'answers':{'action':{'type':'choice','choice':name}}}),action)
        self.assertIn('[0.000,-0.400]',payload['questions']['action']['criteria']['reverse_full__left'])
        with self.assertRaises(ValueError):
            decode_action({'answers':{'action':{'type':'choice','choice':'old_acceleration'}}})

    def test_minimal_model_input(self):
        state=build_payload(observation())['state']
        self.assertEqual(set(state),{'route','target','action_duration_s'})
        obs=observation(next_index=4)
        self.assertIn('FINAL',build_payload(obs)['questions']['action']['instructions'])

    def test_crossing_stays_on_active_leg(self):
        p=[(0,0),(5,5),(10,10),(0,10),(5,5),(10,0)]
        obs=observation(p,[p[0],p[2],p[3],p[-1]])
        obs['boat']['lon_lat']=geo(5,5)
        ctx=RouteContext()
        ctx.prepare(obs,'a')
        self.assertLess(ctx.progress,10)
        obs['task']['next_waypoint_index']=3
        ctx.prepare(obs,'b')
        self.assertGreater(ctx.progress,20)
        # Explicit run identity resets progress even with identical route data.
        obs['task']['next_waypoint_index']=1
        obs['boat']['lon_lat']=geo(0,0)
        ctx.prepare(obs,'c')
        self.assertLess(ctx.progress,.01)

    def test_protocol_version_and_shape(self):
        p=dict(type='decision_request',version=PROTOCOL_VERSION,session_id='s',episode_id='e',run_id='r',decision_id='d',tick=0,observation=observation())
        self.assertTrue(valid_request(p))
        p['version']=5
        self.assertFalse(valid_request(p))

    def test_all_maps_fit_udp_budget(self):
        root=Path(__file__).resolve().parents[2]/'Assets/StreamingAssets'
        for f in [root/'fixed_route.json',*(root/'Maps').glob('*.json')]:
            r=json.loads(f.read_text())
            obs=observation([(p['x'],p['y']) for p in r['path']['samples']],[(p['x'],p['y']) for p in r['waypoints']])
            # Compact JSON matches Unity wire formatting; reserve space for envelope and metadata.
            self.assertLess(len(json.dumps(obs,separators=(',',':')).encode())+1000,65000,f.name)

if __name__=='__main__': unittest.main()
