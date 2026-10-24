"""Startup-state tests: no hardware or firmware writes."""
import subprocess
import unittest
from unittest.mock import MagicMock, patch

import rk_npu_autostart as boot


class StartupTests(unittest.TestCase):
    def test_unknown_usb_never_starts_loader(self):
        with patch.object(boot, 'owned_runtime', return_value={'3-1.2': '2207:180a'}), patch.object(boot.subprocess, 'run') as run:
            with self.assertRaises(RuntimeError):
                boot.firmware(MagicMock())
            run.assert_not_called()

    def test_existing_runtime_is_not_reloaded(self):
        with patch.object(boot, 'owned_runtime', return_value={'1-1.1': '2207:0019'}), patch.object(boot, 'wait_runtime'), patch.object(boot.time, 'sleep'), patch.object(boot.subprocess, 'run') as run:
            boot.firmware(MagicMock())
            run.assert_not_called()

    def test_maskrom_uses_guarded_helper_once(self):
        with patch.object(boot, 'owned_runtime', return_value={'3-1.1': '2207:180a'}), patch.object(boot, 'wait_runtime'), patch.object(boot.time, 'sleep'), patch.object(boot.subprocess, 'run') as run:
            boot.firmware(MagicMock())
            run.assert_called_once_with(['/bin/bash', str(boot.ROOT/'rk_npu_boot.sh'), 'start'], check=True, timeout=190)

    def test_failed_transfer_never_proceeds_to_readiness(self):
        with patch.object(boot, 'owned_runtime', return_value={'3-1.1': '2207:180a'}), patch.object(boot, 'wait_runtime') as wait, patch.object(boot.subprocess, 'run', side_effect=subprocess.CalledProcessError(1, 'loader')):
            with self.assertRaises(subprocess.CalledProcessError):
                boot.firmware(MagicMock())
            wait.assert_not_called()

    def test_placeholder_serial_is_not_runtime_ready(self):
        with patch.object(boot, 'owned_runtime', return_value={'1-1.1': '2207:0019'}), patch.object(boot, 'Path') as path:
            (path.return_value.__truediv__.return_value.__truediv__.return_value.read_text.return_value) = '0123456789ABCDEF'
            with self.assertRaises(RuntimeError):
                boot.wait_runtime(MagicMock(), timeout=0)

    def test_proxy_must_match_internal_device_serial(self):
        io=MagicMock(); io.npu_usb.return_value={'1-1.1':'2207:0019'}
        with patch.object(boot, 'wait_runtime'), patch.object(boot, 'owned_runtime'), patch.object(boot, 'Path') as path, patch.object(boot.subprocess, 'run', return_value=MagicMock(returncode=0,stdout='other-id a0096b1e USB_DEVICE\n')):
            path.return_value.__truediv__.return_value.__truediv__.return_value.read_text.return_value='expected-id'
            with self.assertRaises(RuntimeError):
                boot.proxy(io, timeout=0)

    def test_proxy_valid_device_is_ready(self):
        io=MagicMock(); io.npu_usb.return_value={'1-1.1':'2207:0019'}
        with patch.object(boot, 'wait_runtime'), patch.object(boot, 'owned_runtime'), patch.object(boot, 'Path') as path, patch.object(boot.subprocess, 'run', return_value=MagicMock(returncode=0,stdout='expected-id a0096b1e USB_DEVICE\n')) as run:
            path.return_value.__truediv__.return_value.__truediv__.return_value.read_text.return_value='expected-id'
            boot.proxy(io, timeout=0)
            self.assertEqual(run.call_args[1]['timeout'],5)


if __name__=='__main__':
    unittest.main(verbosity=2)
