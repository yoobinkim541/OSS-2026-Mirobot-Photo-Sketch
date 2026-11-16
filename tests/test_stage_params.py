"""조절 칸 표시가 스텝 값으로 되읽히는지."""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from mirobot_sketch import stages  # noqa: E402
from mirobot_sketch.stage_view import ParamControls  # noqa: E402


class FloatFormatRoundTripTest(unittest.TestCase):
    def test_display_then_parse_keeps_every_float_step(self):
        for spec in stages.PARAM_SPECS.values():
            if spec.kind != "float":
                continue
            n = int(round((spec.hi - spec.lo) / spec.step))
            for i in range(n + 1):
                v = spec.clamp(spec.lo + i * spec.step)
                shown = ParamControls._fmt(spec, v)
                back = spec.clamp(float(shown))
                self.assertEqual(back, v, (spec.key, v, shown, back))


if __name__ == "__main__":
    unittest.main()
