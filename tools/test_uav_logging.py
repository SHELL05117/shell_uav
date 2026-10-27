#!/usr/bin/env python3
"""Recorder safety regressions; no ROS or hardware required."""
import importlib
import json
import os
import shutil
import tempfile
import unittest
from unittest import mock
if os.name=='nt':
    import sys
    sys.modules['fcntl']=mock.Mock()  # Only Linux file locking is platform specific.
import uav_log
import uav_log_report


class RecorderSafety(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.mkdtemp(prefix='uav-log-test-')

    def tearDown(self):
        shutil.rmtree(self.directory)

    def test_disk_reserve_refuses_before_creating_session(self):
        with mock.patch.dict(os.environ,{'UAV_LOG_ROOT':self.directory}), \
             mock.patch.object(uav_log,'free_bytes',return_value=2*1024**3):
            with self.assertRaises(RuntimeError): uav_log.prepare('check')
        self.assertEqual(os.listdir(self.directory),[])

    def test_pid_reuse_is_not_an_active_recorder(self):
        path=os.path.join(self.directory,'active.json')
        with open(path,'w') as stream:
            json.dump(dict(process=dict(pid=123,start='4',boot='a'),directory=self.directory),stream)
        with mock.patch.object(uav_log,'ACTIVE',path), \
             mock.patch.object(uav_log,'identity',return_value=dict(pid=123,start='5',boot='a')):
            self.assertIsNone(uav_log.active())

    def test_stop_refuses_changed_identity(self):
        original=dict(pid=123,start='4',boot='a')
        with mock.patch.object(uav_log,'active',return_value=dict(process=original,directory=self.directory)), \
             mock.patch.object(uav_log,'identity',return_value=dict(pid=123,start='5',boot='a')), \
             mock.patch.object(uav_log.os,'kill') as kill:
            with self.assertRaises(RuntimeError): uav_log.stop_active()
            kill.assert_not_called()

    def test_attach_does_not_reuse_inactive_session(self):
        with mock.patch.object(uav_log,'active',return_value=None):
            self.assertIsNone(uav_log.attach('mission',{}))

    def test_report_refuses_unclosed_bag(self):
        open(os.path.join(self.directory,'core.bag.active'),'w').close()
        with mock.patch.dict('sys.modules',{'rosbag':mock.Mock()}):
            with self.assertRaises(RuntimeError): uav_log_report.generate(self.directory)

    def test_percentiles_do_not_treat_missing_values_as_zero(self):
        result=uav_log_report.stats([None,1.,2.,3.])
        self.assertEqual(result['count'],3)
        self.assertEqual(result['mean'],2.)


if __name__=='__main__': unittest.main()
