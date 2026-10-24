"""Download only the needed member(s) of a remote ZIP via HTTP Range requests.

The RKNN-Toolkit 1.7.5 GitHub release ships a 678 MB archive of per-platform
wheels.  Full downloads through the available mirrors are throttled to a few
hundred KB/s, so this helper reads the ZIP central directory with ranged GETs and
extracts just the requested members (the Windows wheel for the offline converter
environment).

Usage:
  python remote_zip_extract.py <url> <out_dir> [--match win_amd64] [--list]
"""
from __future__ import annotations

import argparse
import io
import sys
import urllib.request
import zipfile
from pathlib import Path


class HttpRangeFile(io.RawIOBase):
    def __init__(self, url: str, block: int = 1 << 20):
        self.url = url
        req = urllib.request.Request(url, method="HEAD", headers={"User-Agent": "dsh/1.0"})
        with urllib.request.urlopen(req, timeout=60) as r:
            self.size = int(r.headers["Content-Length"])
            self.final_url = r.geturl()
        self.block = block
        self.pos = 0
        self.cache = b""
        self.cache_start = 0

    def readable(self) -> bool:
        return True

    def seekable(self) -> bool:
        return True

    def tell(self) -> int:
        return self.pos

    def seek(self, offset: int, whence: int = io.SEEK_SET) -> int:
        if whence == io.SEEK_SET:
            self.pos = offset
        elif whence == io.SEEK_CUR:
            self.pos += offset
        elif whence == io.SEEK_END:
            self.pos = self.size + offset
        return self.pos

    def _fetch(self, start: int, length: int) -> bytes:
        end = min(start + length, self.size) - 1
        req = urllib.request.Request(
            self.final_url, headers={"Range": f"bytes={start}-{end}", "User-Agent": "dsh/1.0"}
        )
        with urllib.request.urlopen(req, timeout=120) as r:
            data = r.read()
        if len(data) != end - start + 1:
            raise IOError(f"short range read {start}-{end}: got {len(data)}")
        return data

    def readinto(self, b) -> int:
        want = len(b)
        if self.pos >= self.size:
            return 0
        if not (self.cache_start <= self.pos and self.pos + want <= self.cache_start + len(self.cache)):
            self.cache_start = self.pos
            self.cache = self._fetch(self.pos, max(self.block, want))
        off = self.pos - self.cache_start
        chunk = self.cache[off:off + want]
        b[:len(chunk)] = chunk
        self.pos += len(chunk)
        return len(chunk)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("url")
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--match", default="win_amd64")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    f = io.BufferedReader(HttpRangeFile(args.url), buffer_size=1 << 16)
    with zipfile.ZipFile(f) as zf:
        names = zf.namelist()
        print(f"archive members: {len(names)}  size={f.raw.size} bytes", file=sys.stderr)
        for n in names:
            if args.list:
                print(n)
        targets = [n for n in names if args.match in n] if args.match else names
        for n in targets:
            info = zf.getinfo(n)
            dest = args.out_dir / Path(n).name
            print(f"extracting {n} ({info.file_size/1e6:.1f} MB)", file=sys.stderr)
            with zf.open(n) as src, dest.open("wb") as out:
                while True:
                    chunk = src.read(1 << 20)
                    if not chunk:
                        break
                    out.write(chunk)
            print(f"  -> {dest} {dest.stat().st_size} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
