#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Bounded PX4 NSH diagnostics over a verified CDC-ACM serial device.

Only heartbeats and SERIAL_CONTROL are sent. Commands are fixed per mode.
gps-persist sets only GPS_1_CONFIG=201; fusion/preflight parameters are read.
reboot restarts a disarmed, USB-powered flight controller for persistence checks.
The preflight mode only queries checks, estimator, battery and receiver state.
Requires no pyserial/pymavlink package. Refuses an armed controller.
"""
from __future__ import print_function
import argparse
import errno
import os
import select
import struct
import sys
import termios
import time
import tty

CRC_EXTRA = {0: 50, 126: 220}


def checksum(data):
    crc = 0xffff
    for value in bytearray(data):
        tmp = value ^ (crc & 0xff)
        tmp = (tmp ^ (tmp << 4)) & 0xff
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff
    return crc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00')
    parser.add_argument('--mode', choices=['status', 'preflight', 'buzzer', 'gps-persist', 'reboot'], default='status')
    args = parser.parse_args()
    commands = {
        'buzzer': ['\n', 'listener tune_control -n 1\n', 'commander check\n',
            'listener battery_status -n 1\n', 'listener vehicle_status -n 1\n',
            'listener vehicle_status_flags -n 1\n', 'listener input_rc -n 1\n',
            'gps status\n', 'listener sensor_gps -n 1\n', 'param show CBRK_BUZZER\n'],
        'status': ['\n', 'gps status\n', 'param show GPS_1_CONFIG\n', 'param show SYS_HAS_GPS\n', 'param show EKF2_AID_MASK\n', 'listener sensor_gps -n 1\n'],
        'preflight': ['\n', 'commander check\n', 'ekf2 status\n',
            'listener vehicle_status -n 1\n', 'listener battery_status -n 1\n',
            'listener input_rc -n 1\n', 'param show EKF2_AID_MASK\n',
            'param show EKF2_HGT_MODE\n'],
        'gps-persist': ['\n', 'gps status\n', 'param set GPS_1_CONFIG 201\n', 'param show GPS_1_CONFIG\n', 'param show SYS_HAS_GPS\n', 'param show EKF2_AID_MASK\n', 'gps start -d /dev/ttyS0 -b 0\n', 'gps status\n', 'listener sensor_gps -n 1\n'],
        'reboot': ['\n', 'reboot\n'],
    }[args.mode]
    fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attributes = termios.tcgetattr(fd)
    attributes[4] = attributes[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attributes)
    seq = [0]
    def send(message_id, payload):
        header = struct.pack('<BBBBBB', len(payload), 0, 0, seq[0], 255, 190) + struct.pack('<I', message_id)[:3]
        packet = b'\xfd' + header + payload + struct.pack('<H', checksum(header+payload+struct.pack('B', CRC_EXTRA[message_id])))
        seq[0] = (seq[0]+1) & 255
        offset = 0
        while offset < len(packet):
            select.select([], [fd], [], 1)
            offset += os.write(fd, packet[offset:])
    def shell(command, flags=6):
        raw = command.encode('ascii')
        send(126, struct.pack('<IHBBB70s', 0, 0, 10, flags, len(raw), raw))
    buffer = bytearray()
    disarmed = False
    count = 0
    next_heartbeat = 0
    next_command = 2
    started = time.time()
    try:
        while time.time()-started < 7+len(commands)*3:
            elapsed = time.time()-started
            if elapsed >= next_heartbeat:
                send(0, struct.pack('<IBBBBB', 0, 6, 8, 0, 4, 3))
                next_heartbeat = elapsed+1
            if disarmed and count < len(commands) and elapsed >= next_command:
                print('SHELL_COMMAND '+repr(commands[count]))
                shell(commands[count])
                count += 1
                next_command = elapsed+3
            readable, _, _ = select.select([fd], [], [], .1)
            if not readable:
                continue
            data = os.read(fd, 8192)
            if not data:
                if args.mode == 'reboot' and count == len(commands):
                    print('Expected firmware USB restart')
                    break
                raise RuntimeError('Serial device disconnected')
            buffer.extend(data)
            while buffer:
                if buffer[0] not in (0xfe, 0xfd):
                    del buffer[0]
                    continue
                version2 = buffer[0] == 0xfd
                head_size = 10 if version2 else 6
                if len(buffer) < head_size:
                    break
                length = buffer[1]
                signature = 13 if version2 and buffer[2] & 1 else 0
                total = head_size+length+2+signature
                if len(buffer) < total:
                    break
                packet = bytes(buffer[:total]) if sys.version_info[0] >= 3 else str(buffer[:total])
                del buffer[:total]
                values = bytearray(packet)
                message_id = values[7] | values[8]<<8 | values[9]<<16 if version2 else values[5]
                system_id = values[5] if version2 else values[3]
                if message_id not in CRC_EXTRA or system_id != 1:
                    continue
                crc = checksum(packet[1:head_size+length]+struct.pack('B', CRC_EXTRA[message_id]))
                if crc != struct.unpack('<H', packet[head_size+length:head_size+length+2])[0]:
                    raise RuntimeError('MAVLink checksum mismatch')
                payload = bytearray(packet[head_size:head_size+length])
                if message_id == 0:
                    payload.extend(bytearray(max(0, 9-len(payload))))
                    if payload[6] & 128:
                        raise RuntimeError('Controller is armed; refusing shell operation')
                    if not disarmed:
                        print('Verified PX4 heartbeat: disarmed')
                    disarmed = True
                elif message_id == 126 and len(payload) >= 9:
                    text = payload[9:9+payload[8]].decode('utf-8', 'replace')
                    sys.stdout.write(text)
                    sys.stdout.flush()
        if not disarmed:
            raise RuntimeError('No verified controller heartbeat')
    except OSError as error:
        if args.mode != 'reboot' or count != len(commands) or error.errno not in (errno.EIO, errno.ENODEV):
            raise
        print('Expected firmware USB restart')
    finally:
        try:
            shell('', flags=0)
        except OSError:
            pass
        os.close(fd)


if __name__ == '__main__':
    main()
