"""같은 종이의 빈 칸(지난번 −5.5 칸) 8곳에 −5…−1 mm를 긋는다. 깊이 0 칸은 지난번 실행으로 같은 종이에 있다."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compare_depth as cd  # noqa: E402

CELLS = [(-5.0, -20.0, 40.0), (-1.0, 30.0, 40.0), (-3.0, -45.0, 10.0), (-4.0, 5.0, 10.0),
         (-3.0, -20.0, -20.0), (-2.0, 30.0, -20.0), (-2.0, -45.0, -50.0), (-1.0, 5.0, -50.0)]
cd.OUT = os.path.join(os.environ["TEMP"], "comparedepth2")
os.makedirs(cd.OUT, exist_ok=True)
cd.tl.OUT = cd.OUT
cd.cells = lambda: CELLS

if __name__ == "__main__":
    cd.main()
