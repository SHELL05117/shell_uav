"""Read selected public metadata from SHELL's RKFW/ext4 images without mounting.

This is an offline inspection helper, not a firmware validator or flash tool.
Only ext-family block/extent maps and the observed RKAF layout are supported.
It never executes files from the images or reads account/network credentials.
"""

import argparse
import hashlib
import json
import posixpath
import struct
from datetime import datetime, timezone
from pathlib import Path


def u16(data, offset):
    return struct.unpack_from('<H', data, offset)[0]


def u32(data, offset):
    return struct.unpack_from('<I', data, offset)[0]


def cstr(data):
    return data.split(b'\0', 1)[0].decode('utf-8', 'replace')


class Region:
    def __init__(self, stream, offset, size):
        self.stream, self.offset, self.size = stream, offset, size

    def read(self, offset, size):
        if offset < 0 or size < 0 or offset + size > self.size:
            raise ValueError('read outside image region')
        self.stream.seek(self.offset + offset)
        data = self.stream.read(size)
        if len(data) != size:
            raise ValueError('short image read')
        return data


class Ext4:
    def __init__(self, region):
        self.region = region
        sb = region.read(1024, 1024)
        if u16(sb, 56) != 0xEF53:
            raise ValueError('not ext4')
        if u32(sb, 96) & 0x10:
            raise ValueError('meta_bg unsupported')
        self.block = 1024 << u32(sb, 24)
        self.ipg = u32(sb, 40)
        self.inode_size = u16(sb, 88)
        self.desc_size = max(32, u16(sb, 254))
        self.gdt = (u32(sb, 20) + 1) * self.block
        self.blocks = u32(sb, 4)
        if u32(sb, 96) & 0x80:
            self.blocks += u32(sb, 336) << 32
        self.label = cstr(sb[120:136])
        self.features = {'compat': u32(sb, 92), 'incompat': u32(sb, 96),
                         'ro_compat': u32(sb, 100)}

    def inode(self, number):
        group, index = divmod(number - 1, self.ipg)
        desc = self.region.read(self.gdt + group * self.desc_size, self.desc_size)
        table = u32(desc, 8)
        if self.desc_size >= 64:
            table += u32(desc, 40) << 32
        return self.region.read(table * self.block + index * self.inode_size,
                                self.inode_size)

    def extents(self, data, level=0):
        if level > 6 or u16(data, 0) != 0xF30A:
            raise ValueError('invalid/unsupported extent tree')
        entries, depth = u16(data, 2), u16(data, 6)
        if 12 + entries * 12 > len(data):
            raise ValueError('invalid extent count')
        for i in range(entries):
            offset = 12 + 12 * i
            logical = u32(data, offset)
            if depth:
                child = u32(data, offset + 4) + (u16(data, offset + 8) << 32)
                yield from self.extents(self.region.read(child * self.block,
                                                        self.block), level + 1)
            else:
                length = u16(data, offset + 4)
                unwritten = length > 32768
                if unwritten:
                    length -= 32768
                physical = u32(data, offset + 8) + (u16(data, offset + 6) << 32)
                yield logical, physical, length, unwritten

    def data(self, number, limit=32 * 1024 * 1024):
        inode = self.inode(number)
        size = u32(inode, 4) + (u32(inode, 108) << 32)
        if size > limit:
            raise ValueError('file exceeds inspection limit')
        if u16(inode, 0) & 0xF000 == 0xA000 and size <= 60:
            return inode[40:40 + size]
        result = bytearray(size)
        if not u32(inode, 32) & 0x80000:
            pointers = struct.unpack_from('<15I', inode, 40)

            def block_at(index):
                if index < 12:
                    return pointers[index]
                index -= 12
                fanout = self.block // 4
                for depth in range(1, 4):
                    if index < fanout ** depth:
                        pointer = pointers[11 + depth]
                        for remaining in range(depth - 1, -1, -1):
                            if not pointer:
                                return 0
                            slot, index = divmod(index, fanout ** remaining)
                            pointer = u32(self.region.read(pointer * self.block,
                                                           self.block), slot * 4)
                        return pointer
                    index -= fanout ** depth
                raise ValueError('block index outside inode map')

            for index, start in enumerate(range(0, size, self.block)):
                physical = block_at(index)
                count = min(self.block, size - start)
                if physical:
                    result[start:start + count] = self.region.read(physical * self.block, count)
            return bytes(result)
        for logical, physical, length, unwritten in self.extents(inode[40:100]):
            start = logical * self.block
            count = min(length * self.block, size - start)
            if count > 0 and not unwritten:
                result[start:start + count] = self.region.read(physical * self.block, count)
        return bytes(result)

    def directory(self, number):
        data = self.data(number)
        offset = 0
        result = {}
        while offset + 8 <= len(data):
            ino, length = u32(data, offset), u16(data, offset + 4)
            if length < 8 or offset + length > len(data):
                raise ValueError('invalid directory entry')
            name_length = data[offset + 6]
            if name_length > length - 8:
                raise ValueError('invalid directory name')
            name = data[offset + 8:offset + 8 + name_length].decode('utf-8', 'replace')
            if ino and name not in ('.', '..'):
                result[name] = ino
            offset += length
        return result

    def resolve(self, path, links=0):
        if links > 16:
            raise ValueError('symlink limit')
        number, parent = 2, '/'
        parts = [part for part in posixpath.normpath(path).split('/') if part]
        for i, part in enumerate(parts):
            number = self.directory(number)[part]
            if u16(self.inode(number), 0) & 0xF000 == 0xA000:
                target = self.data(number).decode('utf-8')
                return self.resolve(posixpath.join(parent, target, *parts[i + 1:]),
                                    links + 1)
            parent = posixpath.join(parent, part)
        return number

    def read_path(self, path):
        return self.data(self.resolve(path))

    def list_path(self, path):
        try:
            return sorted(self.directory(self.resolve(path)))
        except KeyError:
            return None


def inspect_fs(region):
    fs = Ext4(region)
    result = {'block_size': fs.block, 'filesystem_bytes': fs.blocks * fs.block,
              'label': fs.label, 'features': fs.features}
    result['selected_file_sha256'] = {}
    for path in ('/', '/boot', '/extlinux', '/opt', '/opt/ros', '/etc/netplan',
                 '/etc/apt/sources.list.d', '/usr/lib/aarch64-linux-gnu'):
        names = fs.list_path(path)
        if path == '/usr/lib/aarch64-linux-gnu' and names is not None:
            names = [name for name in names if 'rknn' in name.lower()]
        result[path] = names
    for path in ('/etc/os-release', '/etc/lsb-release', '/etc/debian_version'):
        try:
            data = fs.read_path(path)
            result[path] = data.decode('utf-8', 'replace')
            result['selected_file_sha256'][path] = hashlib.sha256(data).hexdigest()
        except KeyError:
            pass
    for path in ('/bin/bash', '/usr/bin/python3'):
        try:
            data = fs.read_path(path)
            result[path] = {'elf_magic': data[:4].hex(),
                            'elf_machine': u16(data, 18) if data[:4] == b'\x7fELF' else None}
            result['selected_file_sha256'][path] = hashlib.sha256(data).hexdigest()
        except KeyError:
            pass
    for path in ('/extlinux/extlinux.conf', '/boot/extlinux/extlinux.conf'):
        try:
            text = fs.read_path(path).decode('utf-8', 'replace')
            result[path] = [line for line in text.splitlines() if line.strip()
                            and line.split()[0].lower() in
                            ('default', 'label', 'linux', 'kernel', 'fdt', 'initrd')]
        except KeyError:
            pass
    try:
        data = fs.read_path('/var/lib/dpkg/status')
        result['selected_file_sha256']['/var/lib/dpkg/status'] = hashlib.sha256(data).hexdigest()
        status = data.decode('utf-8', 'replace')
        packages = []
        for record in status.split('\n\n'):
            fields = dict(line.split(': ', 1) for line in record.splitlines()
                          if ': ' in line and not line.startswith(' '))
            name = fields.get('Package', '')
            if name.startswith('ros-') or name in ('netplan.io', 'network-manager',
                    'openssh-server', 'python3', 'libopencv-dev', 'geographiclib-tools') \
                    or 'rknn' in name.lower():
                packages.append({key: fields.get(key) for key in
                                 ('Package', 'Status', 'Version', 'Architecture')})
        result['selected_dpkg_packages'] = packages
    except KeyError:
        pass
    return result


def inspect_image(path):
    result = {'name': path.name, 'size': path.stat().st_size}
    with path.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        result['sha256'] = digest
        region = Region(stream, 0, result['size'])
        header = region.read(0, 102)
        if header[:4] != b'RKFW':
            result['ext4'] = inspect_fs(region)
            return result
        loader_offset, loader_size, af_offset, af_size = struct.unpack_from('<4I', header, 25)
        af = Region(stream, af_offset, af_size)
        metadata = af.read(0, 2048)
        if metadata[:4] != b'RKAF':
            raise ValueError('RKAF missing')
        count = u32(metadata, 136)
        if count > 16 or 140 + 112 * count > len(metadata):
            raise ValueError('unsupported RKAF table')
        result['rkfw'] = {'loader_offset': loader_offset, 'loader_size': loader_size,
                          'rkaf_offset': af_offset, 'rkaf_size': af_size}
        result['parts'] = []
        for i in range(count):
            entry = metadata[140 + i * 112:140 + (i + 1) * 112]
            offset, flash_offset, padded_sectors, size = struct.unpack_from('<4I', entry, 96)
            part = {'name': cstr(entry[:32]), 'path': cstr(entry[32:96]),
                    'offset_in_rkaf': offset, 'flash_offset_field': flash_offset,
                    'padded_sectors': padded_sectors, 'size': size}
            inner = Region(stream, af_offset + offset, size)
            af.read(offset, min(size, 16))  # Validate inner bounds first.
            if offset + size > af.size:
                raise ValueError('partition outside RKAF')
            if part['name'] == 'package-file':
                part['text'] = inner.read(0, size).decode('ascii')
            if part['name'] == 'parameter':
                raw = inner.read(0, size)
                if raw[:4] == b'PARM':
                    part['text'] = raw[8:8 + u32(raw, 4)].decode('ascii')
            if part['name'] in ('boot_linux', 'rootfs'):
                digest = hashlib.sha256()
                for start in range(0, size, 4 * 1024 * 1024):
                    digest.update(inner.read(start, min(size - start, 4 * 1024 * 1024)))
                part['sha256'] = digest.hexdigest()
                part['ext4'] = inspect_fs(inner)
            result['parts'].append(part)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_dir', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    images = [inspect_image(path) for path in sorted(args.input_dir.glob('*.img'))]
    result = {'inspected_at_utc': datetime.now(timezone.utc).isoformat(),
              'method': 'read-only RKFW/RKAF and ext4 metadata; no mount or execution',
              'images': images}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n',
                           encoding='utf-8')
    print(json.dumps({'images': len(images), 'output': str(args.output)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
