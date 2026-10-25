"""Physical-target and vendor-listing regressions; temporary files only."""
import tempfile
import unittest
from pathlib import Path

from rk_npu_usb import internal_target, listing_location


class USBTargetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)

    def device(self, bus=2, controller='fe900000.dwc3', port='1.1',
               hub_product='0610', product='180a'):
        hub = Path(self.temp.name)/controller/'xhci-hcd.10.auto'/('usb%d' % bus)/('%d-1' % bus)
        path = hub/('%d-%s' % (bus, port))
        path.mkdir(parents=True)
        for name, value in [('idVendor','05e3'),('idProduct',hub_product)]:
            (hub/name).write_text(value)
        for name, value in [('idVendor','2207'),('idProduct',product),
                            ('busnum',str(bus)),('devpath',port)]:
            (path/name).write_text(value)
        return hub, path

    def test_bus_renumbering_keeps_same_physical_target(self):
        for bus in (1, 2, 3, 4):
            with self.subTest(bus=bus):
                hub, path = self.device(bus=bus)
                self.assertEqual(internal_target(hub)['port'], path.name)

    def test_superspeed_runtime_hub_is_accepted(self):
        hub, _ = self.device(hub_product='0616', product='0019')
        self.assertEqual(internal_target(hub)['product'], '0019')

    def test_wrong_controller_is_rejected(self):
        hub, _ = self.device(controller='fe800000.dwc3')
        with self.assertRaises(RuntimeError): internal_target(hub)

    def test_wrong_hub_port_is_rejected(self):
        hub, _ = self.device(port='1.2')
        with self.assertRaises(RuntimeError): internal_target(hub)

    def test_wrong_hub_is_rejected(self):
        hub, _ = self.device(hub_product='9999')
        with self.assertRaises(RuntimeError): internal_target(hub)

    def test_second_rockchip_device_is_rejected(self):
        hub, _ = self.device()
        other = hub/'2-1.2'
        other.mkdir()
        (other/'idVendor').write_text('2207')
        with self.assertRaises(RuntimeError): internal_target(hub)

    def test_bus_attribute_mismatch_is_rejected(self):
        hub, path = self.device()
        (path/'busnum').write_text('3')
        with self.assertRaises(RuntimeError): internal_target(hub)

    def test_unexpected_product_is_rejected(self):
        hub, _ = self.device(product='9999')
        with self.assertRaises(RuntimeError): internal_target(hub)

    def test_absent_device_is_only_allowed_during_readiness_wait(self):
        root = Path(self.temp.name)
        self.assertIsNone(internal_target(root, allow_absent=True))
        with self.assertRaises(RuntimeError): internal_target(root)


class ToolListingTests(unittest.TestCase):
    def row(self, location='20101', mode='Maskrom'):
        return 'DevNo=1\tVid=0x2207,Pid=0x180a,LocationID=%s\tMode=%s\n' % (location,mode)

    def test_location_is_bound_from_unique_observed_device(self):
        self.assertEqual(listing_location(self.row(), 'Maskrom'), '20101')
        self.assertEqual(listing_location(self.row(mode='Loader'), 'Loader', '20101'), '20101')

    def test_location_change_aborts(self):
        with self.assertRaises(RuntimeError): listing_location(self.row(), 'Maskrom', '30101')

    def test_loader_optional_serial_field_is_accepted(self):
        listing = self.row(mode='Loader').rstrip()+'\tSerialNo=rockchip\n'
        self.assertEqual(listing_location(listing, 'Loader', '20101'), '20101')

    def test_wrong_mode_aborts(self):
        with self.assertRaises(RuntimeError): listing_location(self.row(), 'Loader')

    def test_multiple_or_unknown_tool_devices_abort(self):
        for listing in (self.row()*2, self.row().replace('0x180a','0x350a'),
                        self.row().rstrip()+' Extra=unknown', ''):
            with self.subTest(listing=listing), self.assertRaises(RuntimeError):
                listing_location(listing, 'Maskrom')


if __name__ == '__main__':
    unittest.main(verbosity=2)
