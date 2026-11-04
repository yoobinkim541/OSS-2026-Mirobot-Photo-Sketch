import unittest
import numpy as np
from mirobot_sketch import mirobot_sim as ms

class ControllerOrientationTest(unittest.TestCase):
    def test_measured_controller_angles_match_joint_rotation(self):
        samples = [([0, 0, -3, 0, -127.992, -2.906], [-3.339, -131.076, 4.425]),
                   ([0, 10, -12, 0, -127.992, -2.994], [-3.563, -130.085, 4.653])]
        for joints, abc in samples:
            actual, _ = ms.fk(np.radians(joints))
            np.testing.assert_allclose(ms.controller_rotation(abc), actual[:3, :3], atol=0.00003)

    def test_simulation_parameters_use_requested_orientation_and_contact_seed(self):
        cfg = {'orientation_deg': {'a': -3.563, 'b': -130.085, 'c': 4.653},
               'simulation_start_joints_deg': [0, 10, -12, 0, -128, -3]}
        rotation, q0 = ms.simulation_pose(cfg)
        np.testing.assert_allclose(rotation, ms.controller_rotation([-3.563, -130.085, 4.653]))
        np.testing.assert_allclose(q0, np.radians(cfg['simulation_start_joints_deg']))

    def test_home_defaults_remain_unchanged(self):
        rotation, q0 = ms.simulation_pose({'orientation_deg': {'a': 0, 'b': 0, 'c': 0}})
        np.testing.assert_allclose(rotation, ms.HOME_ROT)
        np.testing.assert_allclose(q0, np.zeros(6))
