"""Keyboard jogging maps WASD to the existing bounded start-pose controls."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from mirobot_sketch.draw_window import DrawWindow


class JogKeysTest(unittest.TestCase):
    def test_wasd_and_arrow_keys_send_expected_steps_only_during_setup(self):
        view = SimpleNamespace(job=Mock(), jog_frame=Mock(), jog_step_var=Mock())
        view.jog_frame.winfo_ismapped.return_value = True
        view.jog_step_var.get.return_value = "0.5"
        for key, expected in (("w", (0.5, 0, 0)), ("s", (-0.5, 0, 0)),
                              ("a", (0, 0.5, 0)), ("D", (0, -0.5, 0)),
                              ("Up", (0, 0, 0.5))):
            DrawWindow._on_jog_key(view, SimpleNamespace(keysym=key))
            view.job.jog.assert_called_with(*expected)
        view.job.reset_mock()
        view.jog_frame.winfo_ismapped.return_value = False
        DrawWindow._on_jog_key(view, SimpleNamespace(keysym="w"))
        view.job.jog.assert_not_called()
