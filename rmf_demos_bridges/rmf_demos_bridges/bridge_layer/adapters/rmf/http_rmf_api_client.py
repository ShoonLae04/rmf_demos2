import json
from urllib import request, error


class HttpRmfApiClient:
    def __init__(self, base_url: str, token: str | None = None, timeout_sec: float = 10.0) -> None:
        self._base_url = base_url.rstrip("/")
        self._token = token
        self._timeout_sec = timeout_sec

    def dispatch_task(self, payload: dict[str, object]) -> dict[str, object]:
        return self._post_json("/tasks/dispatch_task", payload)

    def robot_task(self, payload: dict[str, object]) -> dict[str, object]:
        return self._post_json("/tasks/robot_task", payload)

    def get_task_state(self, task_id: str) -> dict[str, object]:
        return self._get_json(f"/tasks/{task_id}/state")

    def _post_json(self, path: str, payload: dict[str, object]) -> dict[str, object]:
        body = json.dumps(payload).encode("utf-8")
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
        }
        if self._token:
            headers["Authorization"] = f"Bearer {self._token}"

        req = request.Request(
            url=self._base_url + path,
            data=body,
            headers=headers,
            method="POST",
        )
        try:
            with request.urlopen(req, timeout=self._timeout_sec) as resp:
                data = resp.read().decode("utf-8")
                return json.loads(data)
        except error.HTTPError as err:
            payload = err.read().decode("utf-8") if err.fp else ""
            raise RuntimeError(f"RMF API POST {path} failed: {err.code} {payload}") from err

    def _get_json(self, path: str) -> dict[str, object]:
        headers = {
            "Accept": "application/json",
        }
        if self._token:
            headers["Authorization"] = f"Bearer {self._token}"
        req = request.Request(
            url=self._base_url + path,
            headers=headers,
            method="GET",
        )
        try:
            with request.urlopen(req, timeout=self._timeout_sec) as resp:
                data = resp.read().decode("utf-8")
                return json.loads(data)
        except error.HTTPError as err:
            payload = err.read().decode("utf-8") if err.fp else ""
            raise RuntimeError(f"RMF API GET {path} failed: {err.code} {payload}") from err
