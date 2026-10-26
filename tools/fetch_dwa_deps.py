"""Fetch exactly the apt-resolved ARM64 DWA closure on the Windows computer."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import re
import sys
import urllib.request

source = Path(sys.argv[1]).read_text(encoding='utf-8-sig')
output = Path(sys.argv[2])
output.mkdir(parents=True, exist_ok=True)
rows = re.findall(r"^'([^']+)' (\S+) (\d+) MD5Sum:([0-9a-f]+)", source, re.M)
if len(rows) != 13:
    raise RuntimeError('Expected the inspected 13-package closure')

def fetch(row):
    url, name, size, expected = row
    if Path(name).name != name:
        raise RuntimeError('Invalid package basename')
    target = output/name
    data = target.read_bytes() if target.exists() else urllib.request.urlopen(url, timeout=25).read()
    if len(data) != int(size) or hashlib.md5(data).hexdigest() != expected:
        raise RuntimeError('Apt size/hash mismatch: '+name)
    target.write_bytes(data)
    print('verified '+name, flush=True)
    return dict(url=url, name=name, size=len(data), md5=expected,
                sha256=hashlib.sha256(data).hexdigest())

with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
    results = list(pool.map(fetch, rows))
(output/'manifest.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
