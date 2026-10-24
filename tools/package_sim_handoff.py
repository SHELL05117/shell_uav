#!/usr/bin/env python3
"""Package a frozen Git tree, handoff materials and original dataset. No installs."""
import argparse
from datetime import datetime
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import subprocess
import tarfile
import zipfile


ROOT_NAME = 'UAV_sim_handoff_1024'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ref', default='1024')
    ap.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[1])
    ap.add_argument('--dataset', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    repo = args.repo.resolve()
    output = args.output.resolve()
    if output.exists() or output.with_suffix(output.suffix + '.partial').exists():
        raise FileExistsError('Output or partial already exists; use a new filename')
    if not output.parent.is_dir() or not args.dataset.is_file():
        raise FileNotFoundError('Output parent or source dataset missing')
    commit = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', args.ref + '^{commit}']).decode().strip()
    tracked = subprocess.check_output(['git', '-C', str(repo), 'ls-tree', '-r', '-z', commit]).decode().split('\0')
    expected = set()
    for row in filter(None, tracked):
        attrs, name = row.split('\t', 1)
        if attrs.split()[1] != 'blob':
            raise ValueError('Unsupported Git tree entry: ' + name)
        expected.add(name)
    materials = repo / 'tools/sim_handoff'
    rows = []
    stamp = datetime.now().astimezone().isoformat()
    partial = output.with_suffix(output.suffix + '.partial')
    with zipfile.ZipFile(partial, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6, allowZip64=True) as z:
        def add(name, stream, role, mode=0o644, stored=False):
            p = PurePosixPath(name)
            if p.is_absolute() or '..' in p.parts or '\\' in name:
                raise ValueError('Unsafe member: ' + name)
            info = zipfile.ZipInfo(ROOT_NAME + '/' + name)
            info.create_system = 3
            info.external_attr = (0o100000 | mode) << 16
            info.compress_type = zipfile.ZIP_STORED if stored else zipfile.ZIP_DEFLATED
            h = hashlib.sha256()
            size = 0
            with z.open(info, 'w', force_zip64=True) as dest:
                while True:
                    block = stream.read(4 * 1024 * 1024)
                    if not block:
                        break
                    dest.write(block)
                    h.update(block)
                    size += len(block)
            rows.append(dict(path=name, bytes=size, sha256=h.hexdigest(), role=role))

        print('Packaging frozen project tree ' + commit, flush=True)
        process = subprocess.Popen(['git', '-C', str(repo), 'archive', '--format=tar', commit], stdout=subprocess.PIPE)
        seen = set()
        with tarfile.open(fileobj=process.stdout, mode='r|') as archive:
            for member in archive:
                if member.isdir():
                    continue
                if not member.isfile():
                    raise ValueError('Unsupported archive member: ' + member.name)
                stream = archive.extractfile(member)
                add('project/' + member.name, stream, 'git_snapshot', mode=member.mode)
                stream.close()
                seen.add(member.name)
        process.stdout.close()
        if process.wait() != 0 or seen != expected:
            raise RuntimeError('Git archive differs from frozen tree')
        print('Project files: ' + str(len(seen)), flush=True)
        for path in sorted(p for p in materials.rglob('*') if p.is_file() and '__pycache__' not in p.parts):
            with path.open('rb') as stream:
                add('handoff/' + path.relative_to(materials).as_posix(), stream, 'handoff_material',
                    mode=0o755 if path.suffix in ('.sh', '.py') else 0o644)
        print('Packaging original dataset (stored without recompressing)', flush=True)
        with args.dataset.open('rb') as stream:
            add('inputs/' + args.dataset.name, stream, 'original_dataset', stored=True)
        baseline = dict(source_tag=args.ref, source_commit=commit, built_at=stamp,
                        engineering_date='2026-10-24', project_file_count=len(seen),
                        project_snapshot_byte_preserving=True, git_history_included=False,
                        original_dataset_included=True, offline_runtime_image_included=False,
                        third_party_downloads_required=['ROS Docker image and packages', 'Gazebo Classic',
                                                        'PX4 v1.13.3 and recursive submodules',
                                                        'Cartographer and Foxglove bridge', 'PC ONNX dependencies'],
                        execution_status='Packaging only; no simulation/environment/hardware acceptance claimed')
        add('BASELINE.json', io.BytesIO(json.dumps(baseline, ensure_ascii=False, indent=2).encode()), 'baseline')
        manifest = dict(schema_version=1, source_commit=commit, source_tag=args.ref, files=rows,
                        excluded_from_self_hash=['MANIFEST.json', 'SHA256SUMS.txt'])
        z.writestr(ROOT_NAME + '/MANIFEST.json', json.dumps(manifest, ensure_ascii=False, indent=2).encode())
        z.writestr(ROOT_NAME + '/SHA256SUMS.txt', ''.join(r['sha256'] + '  ' + r['path'] + '\n' for r in rows).encode())
    partial.replace(output)
    h = hashlib.sha256()
    with output.open('rb') as f:
        while True:
            b = f.read(8 * 1024 * 1024)
            if not b:
                break
            h.update(b)
    output.with_suffix(output.suffix + '.sha256').write_text(h.hexdigest() + '  ' + output.name + '\n', encoding='ascii')
    print(json.dumps(dict(output=str(output), bytes=output.stat().st_size, sha256=h.hexdigest(),
                          source_commit=commit, project_files=len(seen), payload_files=len(rows)), indent=2), flush=True)


if __name__ == '__main__':
    main()
