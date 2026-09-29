"""순수한 로봇 연결 상태 보조 함수. 포트 열기나 하드웨어 명령은 하지 않습니다."""


def port_is_present(configured_port, ports):
    """pyserial 포트 목록에 설정된 장치가 있는지 확인합니다 (대소문자 무시)."""
    wanted = str(configured_port or "").strip().casefold()
    if not wanted:
        return False
    return any(str(getattr(port, "device", port)).strip().casefold() == wanted for port in ports)


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
