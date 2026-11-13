"""Stage immutable official source for review only; never build or execute it."""

import hashlib
import json
from pathlib import Path
import urllib.request


COMMIT = 'e4ab5dc1c7f3ff46274162a1ef302253936578aa'
REPOSITORY = 'DeepRoboticsLab/Lite3_ROS'


def fetch(url):
    request = urllib.request.Request(url, headers={'User-Agent': 'lite3-source-review'})
    with urllib.request.urlopen(request, timeout=20) as response:
        return response.read()


def main():
    root = Path(__file__).resolve().parents[1]
    destination = root / 'vendor' / 'Lite3_ROS'
    tree = json.loads(fetch(f'https://api.github.com/repos/{REPOSITORY}/git/trees/{COMMIT}?recursive=1'))
    if tree.get('truncated'):
        raise RuntimeError('Refusing an incomplete repository tree')
    # Restrict writes to the known review files; no hooks, submodules, or scripts run.
    entries = [entry for entry in tree['tree'] if entry['type'] == 'blob' and (
        entry['path'] in ('LICENSE', 'README.md', 'README_ZH.md') or
        entry['path'].startswith(('src/transfer/', 'src/transfer_interfaces/'))
    )]
    destination.mkdir(parents=True, exist_ok=True)
    (root / 'vendor' / 'COLCON_IGNORE').touch()
    manifest = {'repository': REPOSITORY, 'commit': COMMIT, 'files': []}
    for entry in entries:
        path = Path(entry['path'])
        if path.is_absolute() or '..' in path.parts:
            raise RuntimeError('Refusing unsafe upstream path')
        data = fetch(f'https://raw.githubusercontent.com/{REPOSITORY}/{COMMIT}/{entry["path"]}')
        blob = b'blob ' + str(len(data)).encode('ascii') + b'\0' + data
        if hashlib.sha1(blob).hexdigest() != entry['sha']:
            raise RuntimeError(f'Upstream blob verification failed: {path}')
        target = destination / path
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists() and target.read_bytes() != data:
            raise RuntimeError(f'Refusing to overwrite a modified review file: {target}')
        target.write_bytes(data)
        manifest['files'].append({'path': str(path), 'sha256': hashlib.sha256(data).hexdigest()})
    (destination / 'SOURCE_MANIFEST.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'Staged {len(entries)} verified source files at {destination}; none executed')


if __name__ == '__main__':
    main()
