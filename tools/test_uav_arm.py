#!/usr/bin/env python3
"""Check arming gates without any ROS service or hardware access."""
from types import SimpleNamespace
import unittest
import uav_arm


class ArmGate(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 10.)))
        self.values = dict(state=(10., SimpleNamespace(connected=True, armed=False, mode='OFFBOARD')),
            readiness=(10., dict(model_loaded=True, frame_fresh=True, inference_ready=True,
                pose_fresh=True, fcu_fresh=True, tf_fresh=True, actuation_enabled=True,
                mission_aborted=False, phase=0)), location=(5.,'READY'),
            pose=(10.,self.message), vision=(10.,self.message),
            setpoints=[9.+i*.02 for i in range(51)])
        self.params = dict(enable_actuation=True, auto_start=False, target_hazard='flammable')

    def check(self):
        return uav_arm.reasons(self.values, self.now, self.now, self.params)

    def test_fresh_offboard_waiting_state_allows_a_request(self):
        self.assertEqual(self.check(), [])

    def test_auto_takeoff_and_manual_modes_are_refused(self):
        for mode in ['AUTO.TAKEOFF','POSCTL','MANUAL','AUTO.LAND']:
            self.values['state'][1].mode = mode
            self.assertTrue(self.check())

    def test_already_armed_is_never_rearmed(self):
        self.values['state'][1].armed = True
        self.assertTrue(self.check())

    def test_aborted_or_started_mission_is_refused(self):
        self.values['readiness'][1]['mission_aborted'] = True
        self.assertTrue(self.check())
        self.values['readiness'][1]['mission_aborted'] = False
        self.values['readiness'][1]['phase'] = 1
        self.assertTrue(self.check())

    def test_stale_or_unavailable_position_is_refused(self):
        self.values['vision'] = (8.,self.message)
        self.assertTrue(self.check())

    def test_setpoint_gap_is_refused(self):
        self.values['setpoints'] = [9.+i*.01 for i in range(35)]+[10.]
        self.assertTrue(self.check())

    def test_dry_run_or_unknown_target_is_refused(self):
        self.params['enable_actuation'] = False
        self.assertTrue(self.check())
        self.params['enable_actuation'] = True
        self.params['target_hazard'] = ''
        self.assertTrue(self.check())

    def test_bad_input_health_cannot_pass(self):
        for key in uav_arm.GATES:
            self.values['readiness'][1][key] = False
            self.assertTrue(self.check())
            self.values['readiness'][1][key] = True


if __name__ == '__main__':
    unittest.main()
