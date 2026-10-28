#!/usr/bin/env python3
"""Safety boundaries of the independent lightweight recorder; no hardware."""
import json
import os
import tempfile
import unittest
from unittest import mock
if os.name == 'nt':
    import sys
    sys.modules['fcntl'] = mock.Mock()
import uav_log_lite as lite


class LiteSafety(unittest.TestCase):
    def test_insufficient_space_never_creates_a_session_or_deletes_old_logs(self):
        with tempfile.TemporaryDirectory() as root:
            old = os.path.join(root, 'old.bag')
            with open(old, 'w') as stream:
                stream.write('preserve')
            with mock.patch.dict(os.environ, {'UAV_LOG_ROOT': root}), \
                 mock.patch.object(lite, 'active', return_value=None), \
                 mock.patch.object(lite, 'free_bytes', return_value=2*1024**3):
                with self.assertRaises(RuntimeError):
                    lite.prepare('a')
            self.assertEqual(os.listdir(root), ['old.bag'])

    def test_pid_reuse_invalidates_active_state(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, 'active.json')
            with open(path, 'w') as stream:
                json.dump(dict(process=dict(pid=123, start='1', boot='a')), stream)
            with mock.patch.object(lite, 'ACTIVE', path), \
                 mock.patch.object(lite, 'identity', return_value=dict(pid=123, start='2', boot='a')):
                self.assertIsNone(lite.active())

    def test_stop_never_signals_reused_pid(self):
        old = dict(pid=123, start='1', boot='a')
        with mock.patch.object(lite, 'active', return_value=dict(process=old, directory='/unused')), \
             mock.patch.object(lite, 'identity', return_value=dict(pid=123, start='2', boot='a')), \
             mock.patch.object(lite.os, 'kill') as kill:
            with self.assertRaises(RuntimeError):
                lite.stop_active()
            kill.assert_not_called()

    def test_report_refuses_an_active_session(self):
        with tempfile.TemporaryDirectory() as root, \
             mock.patch.dict('sys.modules', {'rosbag': mock.Mock()}), \
             mock.patch.object(lite, 'active', return_value=dict(directory=root)):
            with self.assertRaises(RuntimeError):
                lite.report(root)

    def test_attach_refuses_another_master_without_writing(self):
        with tempfile.TemporaryDirectory() as root:
            with open(root+'/session.json', 'w') as stream:
                json.dump(dict(master='http://old:11311'), stream)
            with mock.patch.object(lite, 'active', return_value=dict(directory=root)), \
                 mock.patch.dict(os.environ, {'ROS_MASTER_URI': 'http://new:11311'}):
                with self.assertRaises(RuntimeError):
                    lite.attach('mission', {})
            self.assertEqual(os.listdir(root), ['session.json'])


if __name__ == '__main__':
    unittest.main()
