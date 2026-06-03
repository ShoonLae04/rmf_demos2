import asyncio
import json
import time 
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

    def cancel_task(self, task_id: str) -> dict[str, object]:
        payload = {
            "type": "cancel_task_request",
            "task_id": task_id,
            "labels": ["critical_preemption"]
        }

        return self._post_json(
            "/tasks/cancel_task",
            payload
        )
    def wait_until_idle(self, robot_name: str, timeout: float = 30.0) -> None:
   

        start = time.time()

        while time.time() - start < timeout:
            state = self._get_json(f"/robots/{robot_name}/state")

            status = (
                state.get("status")
                or state.get("state")
                or state.get("mode")
                or state.get("activity")
            )

            print(f"[DEBUG] robot state = {state}")

            if status in {"idle", "stopped", "ready"}:
                return

            time.sleep(0.5)

        raise TimeoutError(f"Robot {robot_name} did not become idle: last state={state}")
    
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
