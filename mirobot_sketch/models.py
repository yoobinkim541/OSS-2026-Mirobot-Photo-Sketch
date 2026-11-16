"""저장소에 넣지 않는 모델 파일을 처음 쓸 때 내려받아 SHA256을 확인한 뒤 캐시 폴더(paths.cache_dir()/models)에 둡니다."""

import hashlib
import os
import urllib.request

DOWNLOAD_TIMEOUT_S = 60


class ModelUnavailable(RuntimeError):
    """모델 파일을 쓸 수 없음 (내려받기 실패, 파일 손상)."""


def cache_path(name):
    from . import paths
    return paths.cache_dir() / "models" / name


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def fetch(name, url, sha256, what, timeout=DOWNLOAD_TIMEOUT_S):
    """캐시의 모델 파일 경로. 없으면 url에서 내려받고, SHA256이 다르면 버리고 ModelUnavailable(what: 안내용 이름)."""
    path = cache_path(name)
    if path.exists():
        return path
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_suffix(".part")
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp, open(part, "wb") as out:
            for block in iter(lambda: resp.read(1 << 20), b""):
                out.write(block)
        if sha256_of(part) != sha256:
            raise ModelUnavailable(f"내려받은 {what}이 손상됐습니다 (SHA256 불일치).")
        os.replace(part, path)
    except OSError as e:          # 인터넷 없음, 시간 초과, 디스크 오류 (URLError도 OSError)
        raise ModelUnavailable(f"{what}을 내려받지 못했습니다: {e}") from None
    finally:
        if part.exists():
            part.unlink()
    return path
