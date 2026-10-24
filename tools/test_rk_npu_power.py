#!/usr/bin/env python3
"""Fault-injection tests; no GPIO, clock, /dev/mem or /run access."""
import copy
import importlib.util
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location('rk_npu_power',
                                            str(Path(__file__).with_name('rk_npu_power.py')))
power = importlib.util.module_from_spec(spec)
spec.loader.exec_module(power)


class FakeSystem:
    def __init__(self, fail_at=None, usb_visible=True, clock32_count=0):
        self.journal = None
        self.exported = set()
        self.directions = {}
        self.levels = {}
        self.counts = {power.CLOCK24: 0, power.CLOCK32: clock32_count}
        self.initial_counts = dict(self.counts)
        self.events = []
        self.now = 0
        self.fail_at = fail_at
        self.writes = 0
        self.usb_visible = usb_visible
        self.busy_programs = []
        self.blocker = None

    def mutation(self, *event):
        self.writes += 1
        if self.writes == self.fail_at:
            raise OSError('Injected hardware failure at %r' % (event,))
        self.events.append((self.now,) + event)

    def boot_id(self):
        return 'fake-boot'

    def state(self):
        return copy.deepcopy(self.journal)

    def save(self, state):
        self.journal = copy.deepcopy(state)

    def forget(self):
        self.journal = None

    def sleep(self, seconds):
        self.now += seconds

    def snapshot(self):
        pins = '\n'.join('pin %d (gpio): (MUX UNCLAIMED) (GPIO UNCLAIMED)' % n
                         for n in power.GPIOS)
        pins += '\npin 2 (gpio0-2): pinctrl (GPIO UNCLAIMED) function npu_clk group npu-ref-clk'
        result = {'compatible': [power.COMPATIBLE, 'rockchip,rk3399pro'],
                  'machine': 'aarch64', 'kernel': '4.4.194', 'boot_id': self.boot_id(),
                  'pins': pins, 'gpio': {n: {'output': False, 'latch': 0, 'level': 0}
                                        for n in power.GPIOS},
                  'exported': sorted(self.exported), 'clock24_count': 0,
                  'clock32_count': self.counts[power.CLOCK32], 'clock24_rate': 24000000,
                  'clock24_prepare': 0, 'clock_select': 0x2086, 'clock_gate': 0xffc,
                  'npu_usb': self.npu_usb(), 'busy': self.busy()}
        if self.blocker:
            self.blocker(result)
        return result

    def export(self, number):
        self.mutation('export', number)
        if number in self.exported:
            raise OSError('already owned')
        self.exported.add(number)

    def unexport(self, number):
        self.mutation('unexport', number)
        self.exported.remove(number)

    def direction(self, number, direction):
        self.mutation('direction', number, direction)
        self.directions[number] = 'out' if direction == 'low' else direction
        if direction == 'low':
            self.levels[number] = 0

    def gpio(self, number, value):
        self.mutation('gpio', number, value)
        if self.directions[number] != 'out':
            raise OSError('not output')
        self.levels[number] = value

    def gpio_path(self, number, attribute):
        return number, attribute

    def read(self, path):
        number, attribute = path
        return self.directions[number] if attribute == 'direction' else str(self.levels[number])

    def clock_count(self, name):
        return self.counts[name]

    def clock(self, name, enable):
        self.mutation('clock', name, enable)
        self.counts[name] += 1 if enable else -1

    def npu_usb(self):
        if self.usb_visible and self.levels.get(power.RESET) == 1:
            return {'3-1.1': '2207:180a'}
        return {}

    def busy(self):
        return list(self.busy_programs)


class PowerTests(unittest.TestCase):
    def assert_clean(self, io):
        self.assertIsNone(io.journal)
        self.assertFalse(io.exported)
        self.assertFalse(any(io.levels.values()))
        self.assertEqual(io.counts, io.initial_counts)

    def test_usb_success_requires_internal_hub_port_one(self):
        io = power.System()
        io.usb = lambda: {'3-1': ('05e3', '0610'),
                          '3-1.2': ('2207', '180a'),
                          '8-1': ('2207', '180a')}
        self.assertEqual(io.npu_usb(), {})
        io.usb = lambda: {'3-1': ('05e3', '0610'), '3-1.1': ('2207', '180a')}
        self.assertEqual(io.npu_usb(), {'3-1.1': '2207:180a'})

    def test_electrical_order_and_minimum_delays(self):
        io = FakeSystem()
        result = power.Controller(io).on(0)
        self.assertEqual(result['result'], 'power_on_usb_visible')
        raised = {e[2]: e[0] for e in io.events if e[1] == 'gpio' and e[3] == 1}
        for first, second in zip(power.POWER, power.POWER[1:]):
            self.assertGreaterEqual(raised[second] - raised[first] + 1e-9, 0.002)
        self.assertGreaterEqual(raised[power.RESET] - raised[56] + 1e-9, 0.025)
        clock_index = next(i for i, e in enumerate(io.events)
                           if e[1:] == ('clock', power.CLOCK24, True))
        cpu_index = next(i for i, e in enumerate(io.events) if e[1:] == ('gpio', 54, 1))
        self.assertLess(clock_index, cpu_index)
        self.assertEqual(io.directions[power.SLEEP], 'in')
        self.assertEqual(io.levels[power.IRQ], 0)
        self.assertTrue(set(io.exported).issubset(power.GPIOS))

    def test_repeat_on_never_resets_or_increments_clocks(self):
        io = FakeSystem()
        controller = power.Controller(io)
        controller.on(0)
        before = io.writes
        self.assertEqual(controller.on(0)['result'], 'already_powered_no_reset')
        self.assertEqual(io.writes, before)
        self.assertEqual(io.counts[power.CLOCK24], 1)

    def test_off_releases_only_owned_resources_and_preserves_shared_32k(self):
        io = FakeSystem(clock32_count=3)
        controller = power.Controller(io)
        controller.on(0)
        self.assertEqual(io.counts[power.CLOCK32], 3)
        controller.off()
        self.assert_clean(io)
        before = io.writes
        controller.off()
        self.assertEqual(io.writes, before)

    def test_failure_at_every_startup_hardware_write_rolls_back(self):
        successful = FakeSystem()
        power.Controller(successful).on(0)
        for index in range(1, successful.writes + 1):
            with self.subTest(write=index):
                io = FakeSystem(fail_at=index)
                with self.assertRaises(OSError):
                    power.Controller(io).on(0)
                self.assert_clean(io)

    def test_missing_usb_rolls_back_instead_of_reporting_npu_success(self):
        io = FakeSystem(usb_visible=False)
        with self.assertRaisesRegex(power.PowerError, 'No Rockchip USB device'):
            power.Controller(io).on(0)
        self.assert_clean(io)

    def test_foreign_gpio_active_clock_wrong_board_and_busy_process_refuse_without_writes(self):
        blockers = [lambda s: s.update(exported=[4]),
                    lambda s: s.update(compatible=['rockchip,rk3399']),
                    lambda s: s.update(clock24_count=1),
                    lambda s: s.update(clock_gate=0),
                    lambda s: s.update(clock24_rate=26000000),
                    lambda s: s.update(busy=['42:npu_transfer_proxy']),
                    lambda s: s['gpio'][54].update(output=True, level=1),
                    lambda s: s.update(pins=s['pins'].replace('(MUX UNCLAIMED)', 'another-driver'))]
        for blocker in blockers:
            with self.subTest(blocker=blocker):
                io = FakeSystem()
                io.blocker = blocker
                with self.assertRaises(power.PowerError):
                    power.Controller(io).on(0)
                self.assertEqual(io.writes, 0)
                self.assertIsNone(io.journal)

    def test_active_inference_blocks_off_without_changing_power(self):
        io = FakeSystem()
        controller = power.Controller(io)
        controller.on(0)
        io.busy_programs = ['42:npu_transfer_proxy']
        before = io.writes
        with self.assertRaises(power.PowerError):
            controller.off()
        self.assertEqual(io.writes, before)
        self.assertEqual(io.levels[power.RESET], 1)

    def test_failed_power_down_retains_gpio_ownership_for_retry(self):
        io = FakeSystem()
        controller = power.Controller(io)
        controller.on(0)
        io.fail_at = io.writes + 1  # First rail cannot be lowered.
        with self.assertRaisesRegex(power.PowerError, 'journal retained'):
            controller.off()
        self.assertEqual(io.journal['phase'], 'cleanup_failed')
        self.assertEqual(set(io.journal['exported']), set(power.GPIOS))
        self.assertFalse(any(e[1] == 'unexport' for e in io.events))
        controller.off()
        self.assert_clean(io)

    def test_changed_running_state_is_never_silently_reinitialized(self):
        io = FakeSystem()
        controller = power.Controller(io)
        controller.on(0)
        io.levels[power.RESET] = 0
        before = io.writes
        with self.assertRaises(power.PowerError):
            controller.on(0)
        self.assertEqual(io.writes, before)

    def test_gpio_acquired_by_another_client_during_start_is_never_released(self):
        io = FakeSystem()
        original_export = io.export

        def racing_export(number):
            if number == 10:
                io.exported.add(10)
                raise OSError('Another client won export race')
            original_export(number)

        io.export = racing_export
        with self.assertRaises(OSError):
            power.Controller(io).on(0)
        self.assertEqual(io.exported, {10})
        self.assertFalse(any(e[1:] == ('unexport', 10) for e in io.events))
        self.assertIsNone(io.journal)

    def test_clock_refcount_race_releases_one_reference_and_preserves_the_other(self):
        io = FakeSystem()
        original_clock = io.clock

        def racing_clock(name, enable):
            original_clock(name, enable)
            if name == power.CLOCK24 and enable:
                io.counts[name] += 1  # Another client acquired its own reference.

        io.clock = racing_clock
        with self.assertRaisesRegex(power.PowerError, 'Unexpected refcount'):
            power.Controller(io).on(0)
        self.assertEqual(io.counts[power.CLOCK24], 1)
        self.assertEqual(io.counts[power.CLOCK32], 0)
        self.assertFalse(io.exported)
        self.assertFalse(any(io.levels.values()))
        self.assertIsNone(io.journal)


if __name__ == '__main__':
    unittest.main(verbosity=2)
