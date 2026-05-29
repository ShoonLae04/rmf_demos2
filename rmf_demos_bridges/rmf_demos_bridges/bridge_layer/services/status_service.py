import json
from threading import Event, Thread
from time import sleep

from ..contracts.ports import (
    BridgeRepositoryPort,
    MqttPort,
    RmfApiPort,
    RmfToWorkOrderMapperPort,
)


class StatusService:
    def __init__(
        self,
        mqtt: MqttPort,
        rmf_api: RmfApiPort,
        repository: BridgeRepositoryPort,
        mapper: RmfToWorkOrderMapperPort,
        topic_update: str,
        qos: int,
        poll_interval_sec: float,
    ) -> None:
        self._mqtt = mqtt
        self._rmf_api = rmf_api
        self._repository = repository
        self._mapper = mapper
        self._topic_update = topic_update
        self._qos = qos
        self._poll_interval_sec = poll_interval_sec
        self._stop = Event()
        self._thread: Thread | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            mappings = self._repository.list_mappings()
            for work_order_id, rmf_task_id, _tenant_id in mappings:
                try:
                    state = self._rmf_api.get_task_state(rmf_task_id)
                    self.on_rmf_task_state(state)
                except Exception:
                    # Best effort polling. Errors are retried in the next cycle.
                    continue
            sleep(self._poll_interval_sec)

    def on_rmf_task_state(self, task_state: dict[str, object]) -> None:
        update = self._mapper.to_update_event(task_state)
        previous = self._repository.get_last_status(update.work_order_id)
        if previous == update.status:
            return

        self._mqtt.publish(
            topic=self._topic_update,
            payload=json.dumps(update.__dict__, default=lambda o: o.__dict__),
            qos=self._qos,
            retain=False,
        )
        self._repository.save_last_status(update.work_order_id, update.status)
