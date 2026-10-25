#!/usr/bin/env python3
"""Read-only input audit and isolated extraction of SHELL's 1025 packages.

Uses Windows/libarchive tar for 7z, Python tarfile for the gzip tar. Never runs
package code, restores links, overwrites src, or installs any package.
"""
import collections
import difflib
import hashlib
import json
from pathlib import Path, PurePosixPath
import subprocess
import sys
import tarfile


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def safe_names(names):
    seen = set()
    for name in names:
        path = PurePosixPath(name)
        if path.is_absolute() or '..' in path.parts or ':' in name or '\\' in name:
            raise ValueError('Unsafe archive member: ' + name)
        folded = str(path).casefold()
        if folded in seen:
            raise ValueError('Duplicate/case-colliding archive member: ' + name)
        seen.add(folded)


def audit(workspace, evidence):
    stage = workspace/'temp1025'
    output = stage/'inspection_20261025'
    output.mkdir(exist_ok=False)
    evidence.mkdir(parents=True, exist_ok=True)
    manifests = {}
    archives = {}
    for filename in ('npu-fix-1.7.5.gz', 'pack.7z'):
        archive = stage/filename
        original_hash = digest(archive)
        listing = subprocess.run(['tar', '-tf', str(archive)], check=True,
                                 stdout=subprocess.PIPE).stdout.decode('utf-8').splitlines()
        verbose = subprocess.run(['tar', '-tvf', str(archive)], check=True,
                                 stdout=subprocess.PIPE).stdout.decode('utf-8', 'replace').splitlines()
        safe_names(listing)
        if len(verbose) != len(listing) or len(listing) > 10000:
            raise ValueError('Unexpected archive listing shape')
        if any(not row or row[0] not in '-d' or ' link to ' in row for row in verbose):
            raise ValueError('Refusing non-regular/non-directory archive member')
        (evidence/(filename+'.members.txt')).write_text('\n'.join(listing)+'\n', encoding='utf-8')
        if filename.endswith('.gz'):
            with tarfile.open(archive, 'r:gz') as source:
                if any(not (m.isfile() or m.isdir()) for m in source.getmembers()):
                    raise ValueError('Unsupported tar member type')
                if sum(m.size for m in source.getmembers()) > 512*1024*1024:
                    raise ValueError('Unexpected expanded size')
                for member in source:
                    target = output/member.name
                    if member.isdir():
                        target.mkdir(parents=True, exist_ok=True)
                    else:
                        target.parent.mkdir(parents=True, exist_ok=True)
                        target.write_bytes(source.extractfile(member).read())
        else:
            # All names/types checked above. Extract once to avoid repeatedly
            # decompressing a solid 7z archive for every individual member.
            subprocess.run(['tar', '-xf', str(archive), '-C', str(output)], check=True)
        root = output/('npu-fix-1.7.5' if filename.endswith('.gz') else 'pack')
        rows = []
        for path in sorted(root.rglob('*')):
            if path.is_file():
                rows.append(dict(path=path.relative_to(output).as_posix(),
                                 size=path.stat().st_size, sha256=digest(path)))
        if digest(archive) != original_hash:
            raise ValueError('Input archive changed during audit')
        manifests[filename] = rows
        archives[filename] = dict(size=archive.stat().st_size, sha256=original_hash,
            entries=len(listing), files=len(rows), expanded_bytes=sum(r['size'] for r in rows))
    npu = output/'npu-fix-1.7.5'
    checksums = []
    for line in (npu/'SHA256SUMS').read_text().splitlines():
        if not line.strip():
            continue
        expected, name = line.split(None, 1)
        name = name.lstrip('*')
        safe_names([name])
        path = npu/name
        checksums.append(dict(path=name, passed=path.is_file() and digest(path)==expected.lower()))
    incoming = output/'pack/src'
    comparisons = []
    diffs = []
    for path in sorted(incoming.rglob('*')):
        if not path.is_file():
            continue
        relative = path.relative_to(incoming)
        local = workspace/'src'/relative
        other = path.read_bytes()
        if not local.is_file():
            status = 'incoming_only'
            current = b''
        else:
            current = local.read_bytes()
            status = ('identical' if current == other else
                'line_endings_only' if current.replace(b'\r\n',b'\n') == other.replace(b'\r\n',b'\n') else 'changed')
        comparisons.append(dict(path=relative.as_posix(), status=status,
            incoming_size=len(other), incoming_sha256=digest(path),
            current_sha256=digest(local) if local.is_file() else None))
        if status in ('changed','incoming_only') and b'\0' not in other and len(other)<500000:
            diffs.extend(difflib.unified_diff(current.decode('utf-8','replace').splitlines(True),
                other.decode('utf-8','replace').splitlines(True),
                fromfile='current/src/'+relative.as_posix(), tofile='incoming/src/'+relative.as_posix()))
    packages = {p.name for p in incoming.iterdir() if p.is_dir()}
    missing = []
    for package in sorted(packages):
        for path in sorted((workspace/'src'/package).rglob('*')):
            if path.is_file() and not (incoming/path.relative_to(workspace/'src')).is_file():
                missing.append(path.relative_to(workspace/'src').as_posix())
    report = dict(archives=archives, manifests=manifests, npu_checksums=checksums,
        comparisons=comparisons, local_only_in_covered_packages=missing,
        counts=dict(collections.Counter(r['status'] for r in comparisons)))
    (evidence/'archive-audit.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    (evidence/'source-differences.patch').write_text(''.join(diffs), encoding='utf-8')
    print(json.dumps(dict(archives=archives, comparison_counts=report['counts'],
        checksums_passed=all(r['passed'] for r in checksums),
        checksum_count=len(checksums), local_only_count=len(missing)), indent=2))
    print('Changed/new paths:')
    for row in comparisons:
        if row['status'] in ('changed','incoming_only'):
            print(row['status']+' '+row['path'])


if __name__ == '__main__':
    audit(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
