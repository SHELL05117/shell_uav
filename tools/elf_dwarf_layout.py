#!/usr/bin/env python3
"""Report selected C structure sizes/offsets from an ELF's DWARF information.

Requires pyelftools. --library-dir may point to an independently verified,
unpacked wheel; this avoids installing a Python package on the target board.
"""
import argparse
import json
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf')
    parser.add_argument('--library-dir')
    parser.add_argument('--names', nargs='+', default=[
        'module', 'usb_device', 'usb_interface', 'usb_host_interface',
        'usb_device_id', 'urb', 'usb_driver', 'tty_struct', 'tty_driver',
        'tty_port', 'file_operations', 'work_struct', 'delayed_work',
        'timer_list', 'mutex', 'device', 'list_head', 'wait_queue_head'])
    args = parser.parse_args()
    if args.library_dir:
        sys.path.insert(0, args.library_dir)
    from elftools.elf.elffile import ELFFile
    output = {}
    with open(args.elf, 'rb') as stream:
        elf = ELFFile(stream)
        dwarf = elf.get_dwarf_info()
        for unit in dwarf.iter_CUs():
            for die in unit.iter_DIEs():
                attributes = die.attributes
                if die.tag != 'DW_TAG_structure_type' or 'DW_AT_name' not in attributes or 'DW_AT_byte_size' not in attributes:
                    continue
                name = attributes['DW_AT_name'].value.decode('utf-8', 'replace')
                if name not in args.names:
                    continue
                members = {}
                for member in die.iter_children():
                    if member.tag != 'DW_TAG_member' or 'DW_AT_name' not in member.attributes:
                        continue
                    ma = member.attributes
                    key = ma['DW_AT_name'].value.decode('utf-8', 'replace')
                    offset = ma.get('DW_AT_data_member_location')
                    members[key] = offset.value if offset else None
                entry = dict(size=attributes['DW_AT_byte_size'].value, members=members)
                if name in output and output[name] != entry:
                    raise RuntimeError('Conflicting layouts for '+name)
                output[name] = entry
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
