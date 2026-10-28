"""종이 모서리 X와 펜 끝이 화면에 보이는지만 확인합니다.

픽셀을 mm로 바꾸거나 종이 중심을 옮기지 않습니다. 접촉 깊이도 판정하지 않습니다.
X가 4개로 안정되지 않으면 종이 위치를 추정하지 않습니다.
"""

import cv2
import numpy as np


def _cross_like(ink, box):
    """어두운 덩어리가 X자인지. 대각선에는 잉크가 있고 네 변의 가운데는 비어 있어야 합니다."""
    x, y, w, h = box
    if w < 12 or h < 12 or not (0.6 <= w / h <= 1.7):
        return False
    roi = ink[y:y + h, x:x + w]
    if roi.size == 0:
        return False
    fill = float(roi.mean())
    if not 0.06 <= fill <= 0.42:
        return False
    yy, xx = np.mgrid[0:h, 0:w]
    band = max(2, int(0.12 * min(w, h)))
    diag = (np.abs(xx / max(w - 1, 1) - yy / max(h - 1, 1)) * max(w, h) <= band) | (
        np.abs(xx / max(w - 1, 1) + yy / max(h - 1, 1) - 1) * max(w, h) <= band)
    # X의 끝은 귀퉁이에 닿으므로, 비어 있어야 하는 곳은 네 변의 가운데다.
    m = 0.35
    edges = ((yy < h * 0.18) & (xx > w * m) & (xx < w * (1 - m))) | (
        (yy > h * 0.82) & (xx > w * m) & (xx < w * (1 - m))) | (
        (xx < w * 0.18) & (yy > h * m) & (yy < h * (1 - m))) | (
        (xx > w * 0.82) & (yy > h * m) & (yy < h * (1 - m)))
    if edges.sum() == 0 or float(roi[diag].mean()) < 0.35 or float(roi[edges].mean()) > 0.15:
        return False
    return True


def _pen_like(ink, box):
    x, y, w, h = box
    if w < 4 or h < 4 or w > 28 or h > 28 or not (0.5 <= w / h <= 2.0):
        return False
    roi = ink[y:y + h, x:x + w]
    return float(roi.mean()) >= 0.45


def _stable_quad(boxes):
    if len(boxes) != 4:
        return False
    centers = np.array([[x + w / 2, y + h / 2] for x, y, w, h in boxes], np.float32)
    area = abs(cv2.contourArea(cv2.convexHull(centers)))
    if area < 80 * 80:
        return False
    sizes = np.array([w * h for *_, w, h in boxes], np.float32)
    return float(sizes.max() / max(sizes.min(), 1)) <= 2.5


def observe_setup(frame_bgr):
    """프레임에서 X 4개와 펜 끝 후보를 봅니다. 접촉·이동량은 반환하지 않습니다."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    ink = gray < 90
    n, labels, stats, _ = cv2.connectedComponentsWithStats(ink.astype(np.uint8), 8)
    crosses, pens = [], []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < 20:
            continue
        box = (int(x), int(y), int(w), int(h))
        if _cross_like(ink, box):
            crosses.append(box)
        elif _pen_like(ink, box):
            pens.append(box)
    stable = _stable_quad(crosses)
    return {
        "crosses": len(crosses),
        "crosses_stable": stable,
        "pen_tip_seen": len(pens) == 1,
        "contact_checked": False,
        "center_moved": False,
    }


def attach_camera_check(guide, port_choice, observation):
    """robot_guide 결과에 USB·카메라 관찰만 얹습니다. guide의 ready를 카메라로 바꾸지 않습니다."""
    out = dict(guide)
    out["usb"] = {"port": port_choice.port, "source": port_choice.source}
    out["camera"] = dict(observation)
    out["contact_checked"] = False
    out["center_moved"] = False
    notes = list(out.get("notes") or [])
    if port_choice.port is None:
        from . import robot_status
        notes.append(robot_status.port_choice_problem(port_choice) + ". 포트를 열지 않았습니다.")
    if not observation["crosses_stable"]:
        notes.append("종이 모서리 X 4개가 안정적으로 보이지 않습니다. 종이 위치는 옮기지 않습니다.")
    elif not observation["pen_tip_seen"]:
        notes.append("펜 끝을 하나로 구분하지 못했습니다. 접촉은 확인하지 않습니다.")
    else:
        notes.append("X 4개와 펜 끝이 화면에 보입니다. 접촉 깊이와 종이 기울기는 확인하지 않았습니다.")
    out["notes"] = notes
    return out


def ready_button_visible(camera_index, frame):
    """웹캠이 프레임을 줄 때만 준비 확인 버튼을 보입니다."""
    return camera_index is not None and frame is not None


def ready_text(report):
    """준비 확인 버튼이 보여줄 짧은 문구."""
    cam = report.get("camera") or {}
    usb = report.get("usb") or {}
    lines = []
    if report.get("ready"):
        lines.append("그림·시뮬레이션: 확인됨")
    else:
        issues = report.get("issues") or []
        lines.append(issues[0] if issues else report.get("next", "그림 준비가 되지 않았습니다."))
    port = usb.get("port")
    lines.append(f"USB: {port}" if port else "USB: 로봇 포트 없음")
    if cam.get("crosses_stable") and cam.get("pen_tip_seen"):
        lines.append("카메라: X 4개·펜 끝 보임")
    else:
        lines.append(f"카메라: X {cam.get('crosses', 0)}개, 펜 끝 {'보임' if cam.get('pen_tip_seen') else '구분 못 함'}")
    lines.append("접촉은 확인하지 않음")
    return "\n".join(lines)
