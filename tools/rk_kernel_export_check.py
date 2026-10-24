#!/usr/bin/env python3
"""Check a module's undefined symbols against an uncompressed ARM64 Image.

Use only after verifying the Image hash against the running board's boot file.
The seed is a verified kernel_symbol entry for usb_submit_urb. Kernel 4.4 uses
two native pointers per entry. This is deliberately specific to this BSP ABI.
"""
import argparse
import json
import pathlib
import re
import struct


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image')
    parser.add_argument('required_symbols')
    parser.add_argument('--base', type=lambda s: int(s, 0), default=0xffffff8008080000)
    parser.add_argument('--seed', type=lambda s: int(s, 0), default=0x10b2f80)
    args = parser.parse_args()
    data = pathlib.Path(args.image).read_bytes()
    if data[56:60] != b'ARM\x64':
        raise ValueError('Expected an uncompressed ARM64 Image')
    def entry(offset):
        if offset < 0 or offset+16 > len(data):
            return None
        value, pointer = struct.unpack_from('<QQ', data, offset)
        name_offset = pointer-args.base
        if not (args.base <= value < args.base+0x10000000 and 0 <= name_offset < len(data)):
            return None
        end = data.find(b'\0', name_offset, name_offset+200)
        if end < 0:
            return None
        name = data[name_offset:end]
        return name.decode() if re.fullmatch(rb'[A-Za-z_][A-Za-z_0-9]*', name) else None
    if entry(args.seed) != 'usb_submit_urb':
        raise ValueError('Kernel export seed does not match this Image')
    start = args.seed
    while entry(start-16):
        start -= 16
    end = args.seed+16
    while entry(end):
        end += 16
    exports = {entry(i) for i in range(start, end, 16)}
    required = set(pathlib.Path(args.required_symbols).read_text().split())
    missing = sorted(required-exports)
    result = dict(table_start=hex(start), table_end=hex(end), exports=len(exports),
                  required=len(required), missing=missing, passed=not missing)
    print(json.dumps(result, indent=2))
    raise SystemExit(1 if missing else 0)


if __name__ == '__main__':
    main()
