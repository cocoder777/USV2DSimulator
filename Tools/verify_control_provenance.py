"""Verify pinned public upstream sources and local model files without changing deployments."""
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'Logs/control-failure-audit-20261018/provenance'


def main():
    OUT.mkdir(parents=True, exist_ok=False)
    report = {}
    for name, root, repo, files in [
        ('laya', Path('D:/AI/Laya'), 'NandhaKishorM/laya', ['laya/common.py', 'laya/agent.py', 'laya/serve.py', 'laya/router.py']),
        ('kev', Path('D:/AI/Kev-0.8B'), 'jaredpalmer/kev', ['kev/api.py', 'kev/model.py', 'kev/serve.py', 'kev/checkpoint.py'])]:
        manifest = json.loads((root/'models.json').read_text(encoding='utf-8-sig'))
        item = {'source_commit': manifest['source_commit'], 'sources': [], 'model_files': [],
                'git_status': subprocess.check_output(['git', '-C', str(root/'source'), 'status', '--short'], text=True)}
        for file in files:
            url = f"https://raw.githubusercontent.com/{repo}/{manifest['source_commit']}/{file}"
            local = (root/'source'/file).read_bytes().replace(b'\r\n', b'\n')
            try:
                official = urllib.request.urlopen(url, timeout=30).read().replace(b'\r\n', b'\n')
                (OUT/f'{name}-{Path(file).name}').write_bytes(official)
                item['sources'].append({'file': file, 'url': url, 'matches_official': local == official,
                                        'sha256': hashlib.sha256(local).hexdigest()})
            except Exception as exc:
                item['sources'].append({'file': file, 'url': url, 'error': str(exc)})
        for file, expected in manifest['files'].items():
            path = Path(manifest['path'])/file if name == 'laya' else Path(file)
            with path.open('rb') as stream:
                sha = hashlib.file_digest(stream, 'sha256').hexdigest()
            item['model_files'].append({'file': str(path), 'sha256': sha, 'expected_sha256': expected['sha256'],
                                        'matches_manifest': sha == expected['sha256']})
        report[name] = item
        (OUT/'verification.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps({'model': name, 'source_matches': sum(x.get('matches_official', False) for x in item['sources']),
                          'model_matches': sum(x['matches_manifest'] for x in item['model_files']),
                          'model_total': len(item['model_files'])}), flush=True)


if __name__ == '__main__':
    main()
