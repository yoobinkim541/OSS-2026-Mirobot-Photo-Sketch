"""관절 하나씩 실물 확인(펜이 종이에서 떨어진 대기 자세에서 관절 이동 M21만 사용).
대기 자세 → 시험 관절을 정해진 각도들로 차례로 옮기고, 각 자세에서 사용자가 홀더 각도(휴대폰 기울기 앱)를 읽는다.
다음 자세로 넘어가려면 %TEMP%/jointcheck/next 파일을 만든다(이 스크립트가 지운다). 15분 동안 신호가 없으면 대기 자세로 돌아가 끝낸다.

  python joint_check.py J5 -10 10      (대기 자세 기준 J5 +0 → -10 → 0 → +10 → 0)"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plane_ladder as pl  # noqa: E402

tl, dex, spm, ms = pl.tl, pl.dex, pl.spm, None
from mirobot_sketch import mirobot_sim as ms  # noqa: E402

FLAG_DIR = os.path.join(os.environ["TEMP"], "jointcheck")
os.makedirs(FLAG_DIR, exist_ok=True)
FLAG = os.path.join(FLAG_DIR, "next")
MIN_GAP_MM = 6.0


def pitch_deg(qd):
    """모델의 플랜지 z축(펜 방향)이 수평에서 위로 이루는 각."""
    t, _ = ms.fk(np.radians(qd))
    z = t[:3, 2]
    return float(np.degrees(np.arctan2(z[2], np.hypot(z[0], z[1]))))


def wait_flag(timeout_s=900):
    t = time.time()
    while time.time() - t < timeout_s:
        if os.path.exists(FLAG):
            os.remove(FLAG)
            return True
        time.sleep(0.5)
    return False


def main():
    joint = int(sys.argv[1].upper().lstrip("J")) - 1
    deltas = [float(v) for v in sys.argv[2:]]
    cfg = tl.cfg
    req = cfg["requested_start_pose"]
    wait_q = list(map(float, req["joints_deg"]))
    tool = np.asarray(req["tool_offset_mm"], float)
    contact_x = float(spm._tip(np.radians(req["contact_joints_deg"]), float(req["pen_length_mm"]), tool)[0])
    seq = [0.0]
    for d in deltas:
        seq += [d, 0.0]
    for d in seq:                                # 모든 자세에서 펜 끝이 종이(접촉 자세의 펜 끝 X)에서 충분히 떨어지는지
        q = list(wait_q)
        q[joint] += d
        gap = contact_x - float(spm._tip(np.radians(q), float(req["pen_length_mm"]), tool)[0])
        if gap < MIN_GAP_MM:
            raise SystemExit(f"J{joint + 1} {d:+g}°에서 펜 끝과 종이 간격이 {gap:.1f} mm라 멈춤")
    if os.path.exists(FLAG):
        os.remove(FLAG)
    link = dex.open_link(cfg)
    try:
        pl.check_config(link)
        print("호밍 자세 확인", tl.wait_home(link)["joints_deg"], flush=True)
        spm.execute_requested_start(link, cfg)
        for n, d in enumerate(seq):
            q = list(wait_q)
            q[joint] += d
            if n:
                link.send_and_ack(spm.joint_line(q, 300), cfg["ack_timeout_s"])
                link.wait_idle(cfg["idle_timeout_s"])
            st = dex.parse_status(link.query_status()[2])
            print(f"[{n + 1}/{len(seq)}] J{joint + 1} {d:+g}° → 보고 관절 {st.get('joints_deg')}, "
                  f"모델 펜 기울기(수평 기준 위로) {pitch_deg(q):.1f}°. 읽은 뒤 신호를 주세요.", flush=True)
            if not wait_flag():
                print("15분 동안 신호가 없어 대기 자세로 돌아갑니다.", flush=True)
                break
        if seq and dex.parse_status(link.query_status()[2]).get("joints_deg") != wait_q:
            link.send_and_ack(spm.joint_line(wait_q, 300), cfg["ack_timeout_s"])
            link.wait_idle(cfg["idle_timeout_s"])
        st = spm.execute_requested_return(link, cfg)
        print("복귀", st["joints_deg"], flush=True)
    finally:
        link.close()
        print("포트 닫음", flush=True)


if __name__ == "__main__":
    main()
