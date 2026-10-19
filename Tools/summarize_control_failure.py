"""Analyze saved probes in physical-action space; no inference or controller substitution."""
import argparse
import csv
import itertools
import json
import math
from pathlib import Path
from statistics import mean

from control_failure_cases import acceptable, summarize, evaluate


def save(p, value):
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def softmax(xs):
    mx = max(xs)
    ps = [math.exp(x-mx) for x in xs]
    return [x/sum(ps) for x in ps]


def analyze(directory):
    rows = [json.loads(line) for line in (directory/'rows.jsonl').read_text(encoding='utf-8').splitlines()]
    recheck = directory/'four-recheck/rows.jsonl'
    if recheck.exists():
        rows = [r for r in rows if r['family'] != 'four'] + [json.loads(line) for line in recheck.read_text(encoding='utf-8').splitlines()]
    assert len(rows) == 1008, 'Do not interpret an incomplete run as a completed experiment'
    for family in ('four','linear8','angular7','joint56'):
        one = [r for r in rows if r['state_name']=='ahead_rest' and r['family']==family and r['style']=='short']
        assert len({tuple(r['order']) for r in one if r['order_index'] > 0}) == 6, 'Six distinct random permutations are required'
    # Re-evaluate every saved response under one explicit permissive rubric.
    # Original raw observations/responses remain immutable; early in-flight
    # evaluation fields used a stricter straight-heading preference.
    for r in rows:
        if not r['error']:
            r['evaluation'] = evaluate(r['state_name'], r['observation'], r['family'], r['mapping'], r['answer'])
    groups, index, checks = {}, {}, {'rows': len(rows), 'complete_encodings': 0, 'logits_match': 0, 'mapping_match': 0}
    decisions = []
    for r in rows:
        key = r['state_name'], r['family'], r['style']
        groups.setdefault(key, []).append(r)
        index[(*key, r['order_index'])] = r
        if r['error']:
            continue
        e = r['encoding']
        checks['complete_encodings'] += e['instruction_complete'] and not e['state_stats']['truncated'] and all(o['complete'] for o in e['options'])
        assert len(r['logits']) == 1
        logit = r['logits'][0]
        z = logit.get('calibrated_logits') or [x/logit['temperature'] for x in logit['raw_logits']]
        # Kev softmax uses BF16 before conversion; permit its rounding, report actual error.
        probs = softmax(z)
        difference = max(abs(p-r['answer']['probabilities'][k]) for k,p in zip(r['order'],probs))
        logit['probability_reconstruction_max_error'] = difference
        checks['logits_match'] += difference < .004
        # Raw logits, not rounded response probabilities, determine the winner.
        checks['mapping_match'] += r['order'][max(range(len(z)), key=z.__getitem__)] == r['answer']['choice']
        ev = r['evaluation']
        decisions.append({'state': r['state_name'], 'family': r['family'], 'style': r['style'], 'order': r['order_index'],
                          'choice': r['answer']['choice'], 'physical_id': ev['selected']['physical_id'],
                          'a': ev['selected']['vector'][0], 'alpha': ev['selected']['vector'][1],
                          'pd_a': ev['pd_vector'][0], 'pd_alpha': ev['pd_vector'][1],
                          **ev['checks'], 'logits_probability_error': difference,
                          'top3': json.dumps(ev['top3'], ensure_ascii=False)})
    with (directory/'decisions.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=list(decisions[0])); writer.writeheader(); writer.writerows(decisions)
    spread, state_summary, ensembles = [], [], []
    for (state, family, style), runs in groups.items():
        valid = [r for r in runs if not r['error']]
        if not valid:
            continue
        pspreads = []
        for k, mapping in valid[0]['mapping'].items():
            ps = [r['answer']['probabilities'][k] for r in valid]
            pspreads.append(max(ps)-min(ps))
            spread.append({'state': state, 'family': family, 'style': style, 'key': k, 'physical_id': mapping['physical_id'],
                           'min_p': min(ps), 'max_p': max(ps), 'spread': max(ps)-min(ps), 'probabilities': ps})
        state_summary.append({'state': state, 'family': family, 'style': style,
                              'reasonable': sum(r['evaluation']['checks']['reasonable'] for r in valid),
                              'total': len(valid), 'choices': [r['evaluation']['selected']['physical_id'] for r in valid],
                              'max_probability_spread': max(pspreads), 'mean_probability_spread': mean(pspreads)})
        # Independent permutation-mean experiment, evaluated after the complete matrix.
        mean_p = {k: mean(r['answer']['probabilities'][k] for r in valid) for k in valid[0]['mapping']}
        ans = {'choice': max(mean_p, key=mean_p.get), 'probabilities': mean_p}
        ensembles.append({'state': state, 'family': family, 'style': style,
                          'evaluation': evaluate(state, valid[0]['observation'], family, valid[0]['mapping'], ans)})
    pairs = []
    for a, b, changed in [('left_rest','right_rest','target_y'), ('ahead_reverse','ahead_forward','speed_sign'),
                           ('yaw_left','yaw_right','yaw_sign'), ('inside_forward','inside_reverse','speed_sign')]:
        for family, style in itertools.product(('four','linear8','angular7','joint56'), ('short','brief','full')):
            detail = []
            for j in range(7):
                ra, rb = index[a,family,style,j], index[b,family,style,j]
                if ra['error'] or rb['error']:
                    continue
                assert ra['order'] == rb['order']
                pa, pb = ra['answer']['probabilities'], rb['answer']['probabilities']
                detail.append({'order_index': j, 'a': ra['evaluation']['selected'], 'b': rb['evaluation']['selected'],
                               'changed_action': ra['evaluation']['selected']['physical_id'] != rb['evaluation']['selected']['physical_id'],
                               'both_reasonable': ra['evaluation']['checks']['reasonable'] and rb['evaluation']['checks']['reasonable'],
                               'probability_total_variation': .5*sum(abs(pa[k]-pb[k]) for k in pa)})
            pairs.append({'states': [a,b], 'changed_variable': changed, 'family': family, 'style': style,
                          'action_changes': sum(r['changed_action'] for r in detail),
                          'both_reasonable': sum(r['both_reasonable'] for r in detail), 'total': len(detail), 'runs': detail})
    split = []
    for state in dict.fromkeys(r['state_name'] for r in rows):
        for style in ('short','brief','full'):
            for j in range(7):
                a, t = index[state,'linear8',style,j], index[state,'angular7',style,j]
                action = {'acceleration_index': a['evaluation']['selected']['action']['acceleration_index'],
                          'turn_index': t['evaluation']['selected']['action']['turn_index']}
                split.append({'state': state, 'style': style, 'order': j, 'action': action,
                              'checks': acceptable(state, a['observation'], action)})
    extras = {'split_axis_recombination': [{ 'style': s, 'reasonable': sum(r['checks']['reasonable'] for r in split if r['style']==s), 'total': 84} for s in ('short','brief','full')],
              'permutation_mean_joint': [{'style': s, 'reasonable': sum(r['evaluation']['checks']['reasonable'] for r in ensembles if r['style']==s and r['family']=='joint56'), 'total': 12} for s in ('short','brief','full')],
              'split_rows': split, 'ensemble_rows': ensembles}
    save(directory/'analysis.json', {'rubric_version': 2, 'raw_evaluation_fields_superseded': True,
                                    'summary': summarize(rows), 'validation': checks, 'states': state_summary,
                                    'counterfactuals': pairs, 'minimal_changes': extras})
    save(directory/'physical-probability-spread.json', spread)
    print(json.dumps({'backend': directory.name, 'validation': checks, 'minimal_changes': {k:v for k,v in extras.items() if not k.endswith('_rows')}}))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('directories', nargs='+', type=Path)
    for directory in ap.parse_args().directories:
        analyze(directory)
