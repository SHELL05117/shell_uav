#!/usr/bin/env python3
"""Verify an extracted handoff folder or the ZIP without executing project code."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import zipfile


def digest(stream):
    h = hashlib.sha256()
    while True:
        block = stream.read(4 * 1024 * 1024)
        if not block:
            return h.hexdigest()
        h.update(block)


def safe_name(name):
    p = PurePosixPath(name)
    if p.is_absolute() or '..' in p.parts or '\\' in name:
        raise ValueError('Unsafe manifest name: ' + name)
    return name


def verify(target):
    errors = []
    if target.is_file():
        with zipfile.ZipFile(target) as z:
            names = z.namelist()
            if len(names) != len(set(names)):
                raise ValueError('Duplicate ZIP members')
            manifests = [n for n in names if n.count('/') == 1 and n.endswith('/MANIFEST.json')]
            if len(manifests) != 1:
                raise ValueError('Expected one root MANIFEST.json')
            prefix = manifests[0].rsplit('/', 1)[0] + '/'
            manifest = json.loads(z.read(manifests[0]))
            expected = {prefix + safe_name(e['path']) for e in manifest['files']}
            actual = set(names) - {prefix + 'MANIFEST.json', prefix + 'SHA256SUMS.txt'}
            if expected != actual:
                errors.append('ZIP member set differs from manifest')
            for e in manifest['files']:
                name = prefix + safe_name(e['path'])
                info = z.getinfo(name)
                with z.open(name) as f:
                    got = digest(f)
                if info.file_size != e['bytes'] or got != e['sha256']:
                    errors.append(e['path'])
    else:
        root = target.resolve()
        manifest = json.loads((root / 'MANIFEST.json').read_text(encoding='utf-8'))
        for e in manifest['files']:
            p = root.joinpath(*PurePosixPath(safe_name(e['path'])).parts)
            if not p.is_file() or not p.resolve().is_relative_to(root):
                errors.append(e['path'])
                continue
            with p.open('rb') as f:
                got = digest(f)
            if p.stat().st_size != e['bytes'] or got != e['sha256']:
                errors.append(e['path'])
    result = dict(passed=not errors, files=len(manifest['files']),
                  payload_bytes=sum(e['bytes'] for e in manifest['files']),
                  source_commit=manifest['source_commit'], errors=errors)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return result


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('target', type=Path, nargs='?', default=Path(__file__).resolve().parents[1])
    args = ap.parse_args()
    raise SystemExit(0 if verify(args.target)['passed'] else 1)
