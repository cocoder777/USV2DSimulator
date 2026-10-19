"""Pinned native/API audit and offline factorial experiment. No Unity writes.

Run in each deployment's Python environment with its environment.ps1 loaded.
Uses the ACTUAL FastAPI ASGI app and JSON request bytes, without opening a port.
Instrumentation observes tensors and never changes inputs, logits or responses.
"""
import argparse
import asyncio
import copy
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import runpy
import sys
import time

from control_failure_cases import ROOT, FIXTURE, SEED, FOUR_SEED, observations, payload, orders, evaluate, summarize


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def digest(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


async def asgi_post(app, path, wire):
    sent, used = [], False
    event = asyncio.Event()
    async def receive():
        nonlocal used
        if not used:
            used = True
            return {'type': 'http.request', 'body': wire.encode(), 'more_body': False}
        await event.wait()
        return {'type': 'http.disconnect'}
    async def send(message):
        sent.append(message)
        if message['type'] == 'http.response.body' and not message.get('more_body', False):
            event.set()
    scope = {'type': 'http', 'asgi': {'version': '3.0', 'spec_version': '2.4'},
             'http_version': '1.1', 'method': 'POST', 'scheme': 'http', 'path': path,
             'raw_path': path.encode(), 'query_string': b'', 'root_path': '',
             'headers': [(b'content-type', b'application/json')],
             'server': ('diagnostic-asgi', 0), 'client': ('127.0.0.1', 0)}
    await app(scope, receive, send)
    status = next(m['status'] for m in sent if m['type'] == 'http.response.start')
    raw = b''.join(m.get('body', b'') for m in sent if m['type'] == 'http.response.body').decode()
    return status, raw


class Runtime:
    def __init__(self, backend, deployment):
        self.backend, self.deployment, self.captured = backend, deployment, []
        self.manifest = json.loads((deployment / 'models.json').read_text(encoding='utf-8-sig'))
        if backend == 'laya':
            # run_name != __main__ prevents uvicorn opening a port. No deployment file is edited.
            scope = runpy.run_path(str(deployment / 'serve-local.py'), run_name='diagnostic_service')
            self.app, self.agent = scope['app'], scope['agent']
            self.tok, self.model = self.agent.tok, self.agent.model
            original = self.agent._decode_answers
            from laya.agent import _option_logits
            from laya.common import QTYPES, temp_bucket
            def capture(logits, act, items, ids, internal, offset, lang=None):
                rows = _option_logits(logits, items, offset)
                for qid, row in zip(ids, rows):
                    q = internal[qid]
                    temp = self.agent.temperature_by_options.get(temp_bucket(QTYPES[q['t']], len(row)), self.agent.temperature[QTYPES[q['t']]])
                    self.captured.append({'question': qid, 'raw_logits': row.tolist(), 'temperature': temp})
                return original(logits, act, items, ids, internal, offset, lang)
            self.agent._decode_answers = capture
        else:
            from kev.checkpoint import Checkpoint, LoadOptions
            from kev.serve import app, Server
            self.checkpoint = Checkpoint(self.manifest['kev_path'])
            self.options = LoadOptions.from_env()
            self.tok, self.model = self.checkpoint.load('cuda', self.options)
            self.server = Server(self.checkpoint, self.tok, self.model, 'cuda')
            self.app = app
            app.state.server = self.server
            projections = {}
            self.model.head.k.register_forward_hook(lambda m, a, out: projections.update(k=out.detach()))
            self.model.head.q.register_forward_hook(lambda m, a, out: projections.update(q=out.detach()))
            original = self.model.head.forward
            def capture(*args, **kwargs):
                z = original(*args, **kwargs)
                raw = (projections['k'] @ projections['q']) * self.model.head.scale
                self.captured.append({'calibrated_logits': z.detach().float().cpu().tolist(),
                                      'temperature': self.model.head.temperature,
                                      'raw_logits': raw.float().cpu().tolist(),
                                      'raw_capture': 'same dot product from observed k/q projections before temperature'})
                return z
            self.model.head.forward = capture

    def post(self, p, permute=False):
        self.captured.clear()
        seed = FOUR_SEED if len(p['questions']['action_0']['criteria']) == 4 else SEED
        wire = json.dumps({'request': p, 'question': 'action_0', 'n_perm': 7, 'seed': seed} if permute else p)
        status, raw = asyncio.run(asgi_post(self.app, '/v1/systemone/permute' if permute else '/v1/systemone', wire))
        return {'wire': wire, 'status': status, 'raw_response_text': raw,
                'response': json.loads(raw), 'logits': copy.deepcopy(self.captured)}

    def encoding(self, p, head_max=None, max_len=None):
        tok = self.tok
        if self.backend == 'laya':
            from laya.common import build_head, build_sequence, render_options, serialize_state, encode_text
            q = self.agent._to_internal(p['questions']['action_0'])
            hm, ml = head_max or p.get('head_max_len', 3072), max_len or p.get('max_len', 4096)
            head, markers, stats = build_head(tok, q, hm)
            ids, _, _, state_stats = build_sequence(tok, p['state'], q, ml, hm, return_stats=True, return_truncation_stats=True)
            full_ins = encode_text(tok, f"{q['t']} question: {q['ins']}", add_special_tokens=False)['input_ids']
            retained_ins = head[1:markers[0]-1]
            options = []
            for i, text in enumerate(render_options(q)):
                full = encode_text(tok, ' '+text, add_special_tokens=False)['input_ids']
                end = markers[i+1] if i+1 < len(markers) else len(head)-1
                kept = head[markers[i]+1:end]
                options.append({'key': list(q['crit'])[i], 'slot': i, 'start': markers[i], 'end': end,
                                'rendered': text, 'full_tokens': len(full), 'retained_tokens': len(kept),
                                'retained_ids': kept, 'retained_text': tok.decode(kept), 'complete': full == kept})
            return {'max_len': ml, 'head_max_len': hm, 'head_tokens': len(head), 'stats': stats,
                    'state_serialized': serialize_state(p['state']), 'state_stats': state_stats,
                    'instruction_full_tokens': len(full_ins), 'instruction_retained_tokens': len(retained_ins),
                    'instruction_complete': full_ins == retained_ins, 'options': options,
                    'input_ids': ids, 'input_tokens': tok.convert_ids_to_tokens(ids), 'input_decoded': tok.decode(ids)}
        from kev.api import SystemOneRequest, to_record
        from kev.model import admit, user_tokens
        record, meta = to_record(SystemOneRequest(**p))
        enc = admit(self.model, tok, record, truncate=False)
        options = []
        for i, text in enumerate(record['questions'][0]['options']):
            positions = [j for j, (seg, opt) in enumerate(zip(enc['seg'], enc['opt'])) if seg == 1 and opt == i]
            kept = [enc['ids'][j] for j in positions]
            full = user_tokens(tok, text)
            options.append({'key': meta[0]['keys'][i], 'slot': i, 'start': positions[0], 'end': positions[-1]+1,
                            'scoring_boundary': enc['opt_idx'][0][i], 'position_ids': [enc['pos'][j] for j in positions],
                            'rendered': text, 'full_tokens': len(full), 'retained_tokens': len(kept)-2,
                            'retained_ids': kept, 'retained_text': tok.decode(kept), 'complete': full == kept[1:-1]})
        ni = len(user_tokens(tok, record['questions'][0]['instr']))
        return {'record': record, 'state_serialized': record['state'], 'state_stats': {'state_tokens': enc['state_tokens'], 'truncated': enc['state_truncated']},
                'instruction_full_tokens': ni, 'instruction_retained_tokens': ni, 'instruction_complete': True,
                'options': options, 'encoding': enc, 'input_ids': enc['ids'], 'input_tokens': tok.convert_ids_to_tokens(enc['ids']),
                'input_decoded': tok.decode(enc['ids'])}

    def native(self, p):
        self.captured.clear()
        if self.backend == 'laya':
            from laya.agent import Agent
            # Official Agent.predict, bypass HTTP/router; original encoding temporarily bypasses deployment guard.
            original = self.agent._encode_state
            self.agent._encode_state = Agent._encode_state.__get__(self.agent)
            try:
                value = Agent.predict(self.agent, p['state'], p['questions'], max_len=p.get('max_len', 4096), head_max_len=p.get('head_max_len', 3072))
            finally:
                self.agent._encode_state = original
            return value
        from kev.api import SystemOneRequest, to_record, to_answers
        from kev.model import admit
        record, meta = to_record(SystemOneRequest(**p))
        with self.server.lock:
            enc = admit(self.model, self.tok, record, truncate=False)
            ps = [v.tolist() for v in self.model.probs(enc)]
        return {'answers': to_answers(ps, meta)}

    def close(self):
        if self.backend == 'kev':
            self.server.close()
        else:
            self.agent.__exit__(None, None, None)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--backend', choices=['laya', 'kev'], required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--audit-only', action='store_true')
    ap.add_argument('--four-only', action='store_true', help='Rerun only K=4 with six unique random permutations')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    deployment = Path('D:/AI/Laya' if args.backend == 'laya' else 'D:/AI/Kev-0.8B')
    runtime = Runtime(args.backend, deployment)
    assert not runtime.model.training
    protected = [ROOT / 'Tools/classic_control.py', ROOT / 'Tools/control_benchmark.py', ROOT / 'Tools/scenarios/control_benchmark_scenarios.json', ROOT / 'jev_api/choice_control.py', ROOT / 'jev_api/boat_client.py']
    protected += list((ROOT / 'Assets/Scripts').glob('*.cs'))
    integrity = {str(p): digest(p) for p in protected}
    save(args.output / 'provenance.json', {'manifest': runtime.manifest, 'fixture': str(FIXTURE), 'fixture_sha256': digest(FIXTURE),
         'protected_before': integrity, 'transport': 'actual service ASGI app with exact JSON bytes; no live TCP port',
         'parity_scope': 'same loaded pinned weights; native entry bypasses HTTP/router, not an independent weight load',
         'env': {k: v for k, v in os.environ.items() if k.startswith(('KEV_', 'LAYA_')) and 'KEY' not in k},
         'model_dtype': str(runtime.model.dtype) if args.backend == 'kev' else str(runtime.agent.dtype),
         'python': sys.version, 'packages': {name: importlib.metadata.version(name) for name in ('torch', 'transformers', 'tokenizers', 'fastapi')},
         'eval_mode': not runtime.model.training,
         'option_isolation': runtime.model.option_isolation if args.backend == 'kev' else runtime.agent.parallel_options,
         'load_options': str(runtime.options) if args.backend == 'kev' else {'backend': runtime.agent.backend, 'cpu_fallback_count': runtime.agent.cpu_fallback_count},
         'tokenizer_special_tokens': runtime.tok.special_tokens_map,
         'module': sys.modules['laya.agent' if args.backend == 'laya' else 'kev.model'].__file__})
    rows, audit = [], []
    try:
        for name, obs in list(observations())[:2]:
            p, mapping = payload(name, obs, 'joint56', 'full', args.backend)
            encoded = runtime.encoding(p)
            served, native = runtime.post(p), runtime.native(p)
            a, b = served['response']['answers']['action_0'], native['answers']['action_0']
            audit.append({'name': name, 'payload': p, 'encoding': encoded, 'service': served, 'native': native,
                          'same_choice': a['choice'] == b['choice'],
                          'max_probability_difference': max(abs(v-b['probabilities'][k]) for k,v in a['probabilities'].items()),
                          'mapping': mapping})
        if args.backend == 'laya':
            from choice_control import build_payload
            obs = next(observations())[1]
            defaults = json.loads((Path(runtime.manifest['path'])/'rl_agent_config.json').read_text())
            save(args.output/'checkpoint-config.json', defaults)
            for profile in ('compact', 'original'):
                p = build_payload(obs, 'multilingual', profile)
                for ml, hm in ((defaults['max_len'], defaults['head_max_len']), (4096, 3072)):
                    p.update(max_len=ml, head_max_len=hm)
                    enc = runtime.encoding(p)
                    reply = runtime.post(p)
                    audit.append({'name': f'{profile}-{ml}-{hm}', 'payload': copy.deepcopy(p), 'encoding': enc, 'service': reply})
        save(args.output/'native-audit.json', audit)
        print(json.dumps({'event': 'audit_complete', 'backend': args.backend, 'cases': len(audit)}), flush=True)
        if args.audit_only:
            return
        calls = (args.output/'api-calls.jsonl').open('w', encoding='utf-8')
        records = (args.output/'rows.jsonl').open('w', encoding='utf-8')
        with calls, records:
            for name, obs in observations():
                for family in (('four',) if args.four_only else ('four', 'linear8', 'angular7', 'joint56')):
                    for style in ('short', 'brief', 'full'):
                        p, mapping = payload(name, obs, family, style, args.backend)
                        expected_orders = orders(mapping)
                        base = {'state_name': name, 'observation': obs, 'family': family, 'style': style, 'mapping': mapping}
                        started = time.perf_counter()
                        if args.backend == 'kev':
                            call = runtime.post(p, permute=True)
                            calls.write(json.dumps({**base, **call}, ensure_ascii=False)+'\n'); calls.flush()
                            if call['status'] != 200:
                                raise RuntimeError(call['raw_response_text'])
                            runs = call['response']['runs']
                            assert [r['order'] for r in runs] == expected_orders
                        for j, order in enumerate(expected_orders):
                            one = copy.deepcopy(p)
                            one['questions']['action_0']['criteria'] = {k: p['questions']['action_0']['criteria'][k] for k in order}
                            if args.backend == 'laya':
                                call = runtime.post(one)
                                calls.write(json.dumps({**base, 'order_index': j, **call}, ensure_ascii=False)+'\n'); calls.flush()
                                answer = call['response'].get('answers', {}).get('action_0')
                            else:
                                answer = runs[j]
                            enc = runtime.encoding(one)
                            row = {**base, 'order_index': j, 'order': order, 'payload': one, 'encoding': enc,
                                   'answer': answer, 'logits': call['logits'] if args.backend == 'laya' else call['logits'][j:j+1],
                                   'error': None if answer else call['raw_response_text']}
                            if answer:
                                row['evaluation'] = evaluate(name, obs, family, mapping, answer)
                            rows.append(row)
                            records.write(json.dumps(row, ensure_ascii=False)+'\n'); records.flush()
                        save(args.output/'summary.json', summarize(rows))
                        print(json.dumps({'event': 'group', 'state': name, 'family': family, 'style': style, 'rows': len(rows),
                                          'elapsed_s': round(time.perf_counter()-started, 2)}), flush=True)
        save(args.output/'summary.json', summarize(rows))
    finally:
        save(args.output/'integrity-after.json', {'unchanged': all(digest(Path(p)) == h for p,h in integrity.items()),
                                                'after': {p: digest(Path(p)) for p in integrity}})
        runtime.close()


if __name__ == '__main__':
    main()
