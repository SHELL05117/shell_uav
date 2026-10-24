#!/usr/bin/env python3
"""Manual NPU power control for SHELL's Toybrick ProD vendor 4.4.194 BSP.

Python >= 3.6, standard library only. Default action is read-only check.
Profile prod-20181217 uses the historical ProD schematic and Rockchip's
npu_powerctrl implementation. It does not certify a PCB revision or load
firmware. See docs/项目资料/RK_NPU上电脚本使用说明_2026-10-24.md.
"""
import argparse
import contextlib
import json
import mmap
import os
import re
import signal
import struct
import sys
import time
from pathlib import Path

PROFILE = 'prod-20181217'
COMPATIBLE = 'rockchip,rk3399pro-toybrick-prod-linux'
POWER = (4, 10, 11, 54, 55, 56)
RESET, SLEEP, IRQ = 32, 35, 36
OUTPUTS = POWER + (RESET, IRQ)
GPIOS = OUTPUTS + (SLEEP,)
CLOCK24, CLOCK32 = 'clk_wifi_pmu', 'rk808-clkout2'
STATE = Path('/run/shell-rk-npu-power.json')
LOCK = Path('/run/lock/shell-rk-npu-power.lock')
BUSY_PROGRAMS = {'npu_powerctrl', 'npu_powerctrl_combine', 'npu_upgrade',
                 'npu_transfer_proxy', 'rknn_server'}


class PowerError(RuntimeError):
    pass


class System:
    """The only hardware backend. /dev/mem is opened read-only, never written."""

    def read(self, path):
        return Path(path).read_text().strip()

    def write(self, path, value):
        with Path(path).open('w') as stream:
            stream.write(str(value) + '\n')

    def sleep(self, seconds):
        time.sleep(seconds)

    def boot_id(self):
        return self.read('/proc/sys/kernel/random/boot_id')

    def state(self):
        return json.loads(STATE.read_text()) if STATE.exists() else None

    def save(self, state):
        temporary = STATE.with_suffix('.tmp')
        with temporary.open('w') as stream:
            os.chmod(str(temporary), 0o600)
            json.dump(state, stream, indent=2, sort_keys=True)
            stream.write('\n')
        os.replace(str(temporary), str(STATE))

    def forget(self):
        STATE.unlink()

    def gpio_path(self, number, attribute):
        return Path('/sys/class/gpio/gpio%d/%s' % (number, attribute))

    def export(self, number):
        # Kernel export fails with EBUSY if another client acquired this GPIO.
        # gpiolib creates sysfs attributes synchronously before this returns;
        # journal ownership immediately, before inspecting those attributes.
        self.write('/sys/class/gpio/export', number)

    def unexport(self, number):
        self.write('/sys/class/gpio/unexport', number)

    def direction(self, number, direction):
        # "low" configures output and its initial value atomically.
        self.write(self.gpio_path(number, 'direction'), direction)
        expected = 'out' if direction == 'low' else direction
        if self.read(self.gpio_path(number, 'direction')) != expected:
            raise PowerError('gpio%d direction readback mismatch' % number)
        if direction == 'low' and int(self.read(self.gpio_path(number, 'value'))) != 0:
            raise PowerError('gpio%d initial low readback mismatch' % number)

    def gpio(self, number, value):
        if self.read(self.gpio_path(number, 'direction')) != 'out':
            raise PowerError('gpio%d is no longer an output' % number)
        self.write(self.gpio_path(number, 'value'), value)
        if int(self.read(self.gpio_path(number, 'value'))) != value:
            raise PowerError('gpio%d readback differs from %d' % (number, value))

    def clock_count(self, name):
        return int(self.read('/sys/kernel/debug/clk/%s/clk_enable_count' % name))

    def clock(self, name, enable):
        # Vendor debugfs is a refcount operation, not an assignment to a count.
        self.write('/sys/kernel/debug/clk/%s/clk_enable_count' % name, int(enable))

    def busy(self):
        found = []
        for path in Path('/proc').glob('[0-9]*/exe'):
            try:
                name = Path(os.readlink(str(path))).name
                if name in BUSY_PROGRAMS:
                    found.append('%s:%s' % (path.parent.name, name))
            except OSError:
                pass  # Process may have exited while /proc was being read.
        return found

    def usb(self):
        devices = {}
        for path in Path('/sys/bus/usb/devices').iterdir():
            try:
                devices[path.name] = (self.read(path / 'idVendor').lower(),
                                      self.read(path / 'idProduct').lower())
            except OSError:
                continue  # Interface node or a device disappearing mid-scan.
        return devices

    def npu_usb(self):
        devices = self.usb()
        hubs = [name for name, ids in devices.items()
                if ids in {('05e3', '0610'), ('05e3', '0616')}]
        return {name: ':'.join(ids) for name, ids in devices.items()
                # Historical ProD schematic connects NPU to GL3520 port 1.
                if ids[0] == '2207' and any(name == hub + '.1' for hub in hubs)}

    def registers(self):
        values = {}
        # Verify live DT addresses before reading fixed register offsets.
        for bank, base in [(0, 0xff720000), (1, 0xff730000)]:
            path = Path('/proc/device-tree/pinctrl/gpio%d@%x/reg' % (bank, base))
            address, size = struct.unpack('>QQ', path.read_bytes())
            if address != base or size < 0x54:
                raise PowerError('Unexpected live GPIO register layout')
        fd = os.open('/dev/mem', os.O_RDONLY | os.O_SYNC)
        try:
            for bank, base in [(0, 0xff720000), (1, 0xff730000)]:
                with mmap.mmap(fd, 4096, flags=mmap.MAP_SHARED,
                               prot=mmap.PROT_READ, offset=base) as mapped:
                    dr, ddr, level = [struct.unpack_from('<I', mapped, offset)[0]
                                      for offset in (0, 4, 0x50)]
                    for number in GPIOS:
                        if number // 32 == bank:
                            bit = number % 32
                            values[number] = {'output': bool((ddr >> bit) & 1),
                                              'latch': (dr >> bit) & 1,
                                              'level': (level >> bit) & 1}
            with mmap.mmap(fd, 4096, flags=mmap.MAP_SHARED,
                           prot=mmap.PROT_READ, offset=0xff750000) as mapped:
                select = struct.unpack_from('<I', mapped, 0x84)[0]
                gate = struct.unpack_from('<I', mapped, 0x100)[0]
            return values, select, gate
        finally:
            os.close(fd)

    def snapshot(self):
        compat = Path('/proc/device-tree/compatible').read_bytes().split(b'\0')
        pins = '\n'.join(path.read_text() for path in
                         Path('/sys/kernel/debug/pinctrl').glob('*/pinmux-pins'))
        gpio, select, gate = self.registers()
        return {'compatible': [s.decode('ascii') for s in compat if s],
                'machine': os.uname().machine, 'kernel': os.uname().release,
                'boot_id': self.boot_id(), 'pins': pins, 'gpio': gpio,
                'exported': [n for n in GPIOS if self.gpio_path(n, 'direction').exists()],
                'clock24_count': self.clock_count(CLOCK24),
                'clock32_count': self.clock_count(CLOCK32),
                'clock24_rate': int(self.read('/sys/kernel/debug/clk/%s/clk_rate' % CLOCK24)),
                'clock24_prepare': int(self.read('/sys/kernel/debug/clk/%s/clk_prepare_count' % CLOCK24)),
                'clock_select': select, 'clock_gate': gate,
                'npu_usb': self.npu_usb(), 'busy': self.busy()}


def cold_errors(snapshot):
    errors = []
    if COMPATIBLE not in snapshot['compatible']:
        errors.append('Live DT is not Toybrick ProD Linux')
    if snapshot['machine'] != 'aarch64' or snapshot['kernel'] != '4.4.194':
        errors.append('This profile requires aarch64 vendor kernel 4.4.194')
    if snapshot['exported']:
        errors.append('Target GPIOs already exported: %s' % snapshot['exported'])
    for number in GPIOS:
        match = re.search(r'^pin %d \([^\n]*$' % number, snapshot['pins'], re.M)
        if not match or '(MUX UNCLAIMED)' not in match.group(0) or '(GPIO UNCLAIMED)' not in match.group(0):
            errors.append('GPIO%d pinmux/consumer is occupied or unknown' % number)
        entry = snapshot['gpio'][number]
        # gpiolib unexport releases ownership but leaves a previous output-low
        # direction in hardware. That is also a safe inactive state; rejecting
        # it would make a normal off/on cycle impossible. Occupancy is checked
        # independently above, and any high level or latch still blocks start.
        if entry['level'] or entry['latch']:
            errors.append('GPIO%d is not in the expected inactive low state' % number)
    if not re.search(r'^pin 2 .*function npu_clk group npu-ref-clk$', snapshot['pins'], re.M):
        errors.append('GPIO0_A2 is not routed to npu-ref-clk')
    if snapshot['clock24_count'] or snapshot['clock24_prepare']:
        errors.append('24MHz clock already has an owner; cold-start refused')
    if snapshot['clock24_rate'] != 24000000:
        errors.append('NPU reference clock is not configured for 24MHz')
    if snapshot['clock_select'] & 0x6000 != 0x2000 or not snapshot['clock_gate'] & 0x100:
        errors.append('Unexpected reference-clock mux or active hardware gate')
    if snapshot['npu_usb']:
        errors.append('Rockchip device already present behind internal hub')
    if snapshot['busy']:
        errors.append('NPU programs are running: %s' % snapshot['busy'])
    return errors


class Controller:
    def __init__(self, system):
        self.io = system

    def validate_state(self, state):
        if (state.get('schema') != 1 or state.get('profile') != PROFILE or
                state.get('boot_id') != self.io.boot_id() or
                not set(state.get('exported', ())).issubset(GPIOS) or
                not set(state.get('outputs', ())).issubset(OUTPUTS) or
                not set(state.get('outputs', ())).issubset(state.get('exported', ())) or
                not set(state.get('clocks', {})).issubset((CLOCK24, CLOCK32))):
            raise PowerError('Unrecognized ownership journal; inspect %s' % STATE)

    def healthy(self, state):
        self.validate_state(state)
        if set(state['exported']) != set(GPIOS) or set(state['outputs']) != set(OUTPUTS):
            raise PowerError('Incomplete GPIO journal; use off to clean owned resources')
        for number in OUTPUTS:
            expected = 0 if number == IRQ else 1
            if (self.io.read(self.io.gpio_path(number, 'direction')) != 'out' or
                    int(self.io.read(self.io.gpio_path(number, 'value'))) != expected):
                raise PowerError('Powered-state GPIO%d changed; refusing a reset' % number)
        if self.io.read(self.io.gpio_path(SLEEP, 'direction')) != 'in':
            raise PowerError('NPU sleep-status GPIO direction changed')
        if self.io.clock_count(CLOCK24) < 1:
            raise PowerError('Owned 24MHz clock is no longer enabled')

    def acquire_clock(self, state, name):
        baseline = self.io.clock_count(name)
        if baseline:
            if name == CLOCK24:
                raise PowerError('24MHz clock became busy during initialization')
            return  # Reuse an existing 32k clock without changing its refcount.
        self.io.clock(name, True)
        state['clocks'][name] = baseline
        self.io.save(state)
        if self.io.clock_count(name) != baseline + 1:
            raise PowerError('Unexpected refcount after enabling %s' % name)

    def cleanup(self, state):
        """Vendor power-off order, then release only resources in our journal."""
        errors = []
        for number in (10, 56, 55, 54, 11, 4, IRQ, RESET):
            if number in state['outputs']:
                try:
                    self.io.gpio(number, 0)
                    if number == 10:
                        self.io.sleep(0.002)
                except Exception as exc:
                    errors.append('gpio%d: %s' % (number, exc))
        # If any rail/RESET could not be lowered, retain all ownership so a
        # failed off can be retried; do not release a potentially powered GPIO.
        if errors:
            state['phase'], state['cleanup_errors'] = 'cleanup_failed', errors
            self.io.save(state)
            return errors
        for name in (CLOCK24, CLOCK32):
            if name in state['clocks']:
                try:
                    before = self.io.clock_count(name)
                    if before <= state['clocks'][name]:
                        raise PowerError('Clock ownership refcount changed externally')
                    self.io.clock(name, False)
                    del state['clocks'][name]
                    self.io.save(state)
                    if self.io.clock_count(name) != before - 1:
                        raise PowerError('Clock disable readback mismatch')
                except Exception as exc:
                    errors.append('%s: %s' % (name, exc))
        for number in list(reversed(state['exported'])):
            try:
                self.io.unexport(number)
                state['exported'].remove(number)
                if number in state['outputs']:
                    state['outputs'].remove(number)
                self.io.save(state)
            except Exception as exc:
                errors.append('unexport gpio%d: %s' % (number, exc))
        if errors:
            state['phase'], state['cleanup_errors'] = 'cleanup_failed', errors
            self.io.save(state)
        else:
            self.io.forget()
        return errors

    def on(self, timeout):
        state = self.io.state()
        if state:
            if state.get('phase') != 'powered':
                raise PowerError('Previous transaction incomplete; run off first')
            self.healthy(state)
            return {'result': 'already_powered_no_reset', 'npu_usb': self.io.npu_usb()}
        errors = cold_errors(self.io.snapshot())
        if errors:
            raise PowerError('; '.join(errors))
        state = {'schema': 1, 'profile': PROFILE, 'boot_id': self.io.boot_id(),
                 'phase': 'initializing', 'exported': [], 'outputs': [], 'clocks': {}}
        self.io.save(state)
        try:
            for number in GPIOS:
                self.io.export(number)
                state['exported'].append(number)
                self.io.save(state)
                self.io.direction(number, 'in' if number == SLEEP else 'low')
                if number != SLEEP:
                    state['outputs'].append(number)
                    self.io.save(state)
            self.acquire_clock(state, CLOCK32)
            self.io.sleep(0.002)
            for number in POWER:
                if number == 54:
                    self.acquire_clock(state, CLOCK24)
                self.io.gpio(number, 1)
                self.io.sleep(0.025 if number == 56 else 0.002)
            self.io.gpio(RESET, 1)
            self.healthy(state)
            # Do not confuse BootROM/Loader enumeration with usable inference.
            deadline = time.monotonic() + timeout
            while True:
                devices = self.io.npu_usb()
                if devices:
                    break
                if time.monotonic() >= deadline:
                    raise PowerError('No Rockchip USB device after power-on; rolling back')
                self.io.sleep(0.1)
            state['phase'], state['usb_at_power_on'] = 'powered', devices
            self.io.save(state)
            return {'result': 'power_on_usb_visible', 'npu_usb': devices,
                    'firmware_loaded_by_script': False}
        except BaseException as original:
            # Ignore another interrupt while executing the bounded cleanup.
            with ignore_interrupts():
                cleanup_errors = self.cleanup(state)
            if cleanup_errors:
                raise PowerError('%s; cleanup incomplete: %s' % (original, cleanup_errors)) from original
            raise

    def off(self):
        state = self.io.state()
        if not state:
            return {'result': 'no_owned_resources_no_changes'}
        self.validate_state(state)
        busy = self.io.busy()
        if busy:
            raise PowerError('Stop NPU programs before off: %s' % busy)
        with ignore_interrupts():
            errors = self.cleanup(state)
        if errors:
            raise PowerError('Cleanup incomplete; journal retained: %s' % errors)
        return {'result': 'owned_resources_powered_off_and_released'}


def interrupted(signum, frame):
    raise PowerError('Interrupted by signal %d' % signum)


@contextlib.contextmanager
def ignore_interrupts():
    previous = {sig: signal.signal(sig, signal.SIG_IGN) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', nargs='?', default='check', choices=('check', 'on', 'off'))
    parser.add_argument('--profile', choices=(PROFILE,), help='Explicit board wiring profile for on/off')
    parser.add_argument('--usb-timeout', type=float, default=5, help='Seconds to wait for USB, 0..30 (default 5)')
    args = parser.parse_args()
    if not 0 <= args.usb_timeout <= 30:
        parser.error('--usb-timeout must be between 0 and 30')
    if not sys.platform.startswith('linux'):
        parser.error('Hardware access is supported only on the target Linux board')
    if os.geteuid() != 0:
        parser.error('Run with sudo; check is read-only but needs debugfs and /dev/mem access')
    io = System()
    try:
        if args.action == 'check':
            snapshot = io.snapshot()
            state = io.state()
            report = {'action': 'read_only_check', 'profile': PROFILE,
                      'profile_pcb_revision_verified': False,
                      'cold_start_blockers': cold_errors(snapshot),
                      'ownership_journal': state, 'hardware': snapshot}
            report['hardware'].pop('pins')
            print(json.dumps(report, indent=2, sort_keys=True))
            return 0
        if args.profile != PROFILE:
            parser.error('on/off requires --profile %s; see the wiring/version limits in the guide' % PROFILE)
        import fcntl
        LOCK.parent.mkdir(parents=True, exist_ok=True)
        with LOCK.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            for sig in (signal.SIGINT, signal.SIGTERM):
                signal.signal(sig, interrupted)
            controller = Controller(io)
            result = controller.on(args.usb_timeout) if args.action == 'on' else controller.off()
            print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except (OSError, ValueError, PowerError, KeyboardInterrupt) as exc:
        print('ERROR: %s' % exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
