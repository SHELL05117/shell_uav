"""Read RK3399Pro NPU-related registers; never writes GPIO, clocks or firmware.

Register offsets: Toybrick kernel a80be5749ac552821967eff313df53f9e0cd1e01,
drivers/pinctrl/pinctrl-rockchip.c and drivers/clk/rockchip/clk-rk3399.c.
Board mapping: ProD 20181217 schematic, cross-checked against Rockchip's
npu_powerctrl source. This mapping is evidence for diagnosis, not authority
to drive an unverified PCB revision.
"""
import json
import mmap
import os
import struct
from pathlib import Path


def read_registers(fd, base, offsets):
    with mmap.mmap(fd, 4096, flags=mmap.MAP_SHARED,
                   prot=mmap.PROT_READ, offset=base) as mapped:
        return {name: struct.unpack_from('<I', mapped, offset)[0]
                for name, offset in offsets.items()}


def main():
    for name, base in [('gpio0', 0xff720000), ('gpio1', 0xff730000)]:
        reg = Path('/proc/device-tree/pinctrl') / ('%s@%x' % (name, base)) / 'reg'
        raw = reg.read_bytes()
        address, size = struct.unpack('>QQ', raw)
        if address != base or size < 0x54:
            raise RuntimeError('Unexpected GPIO register map: %s' % reg)
        print('DT', str(reg), raw.hex())
    fd = os.open('/dev/mem', os.O_RDONLY | os.O_SYNC)
    try:
        banks = {}
        for bank, base in [(0, 0xff720000), (1, 0xff730000)]:
            values = read_registers(fd, base, {'DR': 0, 'DDR': 4, 'EXT_PORT': 0x50})
            banks[bank] = values
            print('gpio%d' % bank, json.dumps({k: hex(v) for k, v in values.items()}))
        signals = [(4, 'NPU_PWREN_1_0V8'), (10, 'NPU_PWREN_2_LOG'),
                   (11, 'NPU_PWREN_3_1V8_DDR'), (54, 'NPU_PWREN_4_CPU'),
                   (55, 'NPU_PWREN_5_3V3'), (56, 'NPU_PWREN_6_CORE'),
                   (32, 'CPU_RST_NPU'), (35, 'NPU_INT_CPU'), (36, 'CPU_INT_NPU')]
        for number, label in signals:
            bank, bit = divmod(number, 32)
            values = banks[bank]
            print(number, label,
                  'output' if (values['DDR'] >> bit) & 1 else 'input',
                  'latch', (values['DR'] >> bit) & 1,
                  'level', (values['EXT_PORT'] >> bit) & 1)
        clocks = read_registers(fd, 0xff750000,
                               {'CLKSEL_CON1': 0x84, 'CLKGATE_CON0': 0x100})
        print('PMUCRU', json.dumps({k: hex(v) for k, v in clocks.items()}))
        print('clk_wifi_div_gate_bit8', (clocks['CLKGATE_CON0'] >> 8) & 1,
              '(1 means disabled, CLK_GATE_SET_TO_DISABLE)')
        print('clk_wifi_source_bit13', (clocks['CLKSEL_CON1'] >> 13) & 1,
              '(1 selects xin24m)')
        print('clk_wifi_pmu_mux_bit14', (clocks['CLKSEL_CON1'] >> 14) & 1,
              '(0 selects clk_wifi_div)')
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
