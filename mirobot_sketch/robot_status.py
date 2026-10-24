"""순수한 로봇 연결 상태 보조 함수. 포트 열기나 하드웨어 명령은 하지 않습니다."""

from dataclasses import dataclass

# Mirobot USB 케이블의 USB-시리얼 칩 제조사(WCH: CH340/CH341/CH9102). 블루투스 직렬 포트 등은 제외.
ROBOT_USB_VID = 0x1A86
AUTO_PORT_VALUES = ("", "auto")


def _device(port):
    return str(getattr(port, "device", port)).strip()


def port_is_present(configured_port, ports):
    """pyserial 포트 목록에 설정된 장치가 있는지 확인합니다 (대소문자 무시)."""
    wanted = str(configured_port or "").strip().casefold()
    if not wanted:
        return False
    return any(_device(port).casefold() == wanted for port in ports)


def is_robot_usb(port):
    """로봇 USB 케이블(WCH USB-시리얼)로 보이는 포트인지. 포트 정보만 보고 열지는 않습니다."""
    if getattr(port, "vid", None) == ROBOT_USB_VID:
        return True
    text = f"{getattr(port, 'description', '') or ''} {getattr(port, 'hwid', '') or ''}".upper()
    return "CH340" in text or "CH341" in text or "VID:PID=1A86:" in text


@dataclass(frozen=True)
class PortChoice:
    """port: 연결할 포트(없으면 None). source: configured | auto | ambiguous | missing."""
    port: "str | None"
    source: str
    candidates: tuple = ()


def resolve_robot_port(configured_port, ports):
    """연결할 로봇 포트를 고릅니다.

    1) 설정된 포트가 로봇 USB로 보이면 그 포트, 2) 로봇 USB 포트가 하나뿐이면 그 포트(자동),
    3) 로봇 USB가 없고 설정된 포트가 있으면 그 포트(다른 USB-시리얼 어댑터).
    로봇 USB가 여러 개면 잘못된 장치를 리셋하지 않도록 고르지 않습니다."""
    ports = list(ports)
    configured = str(configured_port or "").strip()
    if configured.casefold() in AUTO_PORT_VALUES:
        configured = ""
    candidates = tuple(sorted({_device(p) for p in ports if is_robot_usb(p)}, key=str.casefold))
    if configured and any(c.casefold() == configured.casefold() for c in candidates):
        return PortChoice(next(c for c in candidates if c.casefold() == configured.casefold()), "configured", candidates)
    if len(candidates) == 1:
        return PortChoice(candidates[0], "auto", candidates)
    if not candidates and configured and port_is_present(configured, ports):
        return PortChoice(next(_device(p) for p in ports if _device(p).casefold() == configured.casefold()),
                          "configured", candidates)
    return PortChoice(None, "ambiguous" if candidates else "missing", candidates)


def detect_robot_port(configured_port):
    """현재 연결된 포트 목록에서 resolve_robot_port. 시리얼 포트는 열지 않습니다."""
    from serial.tools import list_ports
    return resolve_robot_port(configured_port, list_ports.comports())


def port_choice_problem(choice):
    """포트를 고르지 못한 이유 (화면·오류 문구)."""
    if choice.source == "ambiguous":
        return f"로봇 USB 포트가 여러 개입니다 ({', '.join(choice.candidates)})"
    return "로봇 USB 포트를 찾지 못했습니다"


def controller_state_from_progress(message):
    """MirobotLink 호밍 진행 문구에서 컨트롤러 상태만 가져옵니다."""
    prefix = "  컨트롤러 상태: "
    if not isinstance(message, str) or not message.startswith(prefix):
        return None
    state = message[len(prefix):].strip().split(maxsplit=1)
    return state[0] if state else None


def controller_status_text(state=None, recent=False):
    """제어기 상태와 현재 연결 중인지 여부를 구분하는 화면 문구."""
    if not state:
        return "제어기 상태: 미확인"
    label = "최근 제어기 상태" if recent else "제어기 상태"
    return f"{label}: {state}"


class ControllerStatusReporter:
    """실제 DrawJob의 열린 링크에서 얻은 상태를 GUI 이벤트로 전달합니다."""

    def __init__(self, emit):
        self.emit = emit
        self.attempted = False
        self.phase = None
        self.last_state = None

    def connection_started(self):
        self.attempted = True
        self.phase = "connect"
        self.last_state = None
        self.emit(state="연결 중")

    def progress(self, message):
        state = controller_state_from_progress(message)
        if state is not None:
            self.last_state = state
            self.emit(state=state)

    def drawing_started(self):
        self.phase = "drawing"
        self.emit(state="그리는 중")

    def finished(self, result, link_open):
        if not self.attempted:
            return
        if result == "completed":
            state = "Idle"
        elif not link_open:
            state = "연결 실패"
        elif self.phase == "connect":
            state = self.last_state or "응답 없음"
        else:
            state = None
        self.emit(state=state, recent=True)
