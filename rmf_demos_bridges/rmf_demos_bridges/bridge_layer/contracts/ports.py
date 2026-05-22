from typing import Any, Callable, Protocol

from .inbound import WorkOrderCreateEvent
from .outbound import WorkOrderUpdateEvent

MqttHandler = Callable[[str, bytes], None]


class MqttPort(Protocol):
    def connect(self) -> None:
        ...

    def disconnect(self) -> None:
        ...

    def subscribe(self, topic: str, qos: int, handler: MqttHandler) -> None:
        ...

    def publish(self, topic: str, payload: str, qos: int, retain: bool = False) -> None:
        ...


class RmfApiPort(Protocol):
    def dispatch_task(self, payload: dict[str, Any]) -> dict[str, Any]:
        ...

    def robot_task(self, payload: dict[str, Any]) -> dict[str, Any]:
        ...

    def get_task_state(self, task_id: str) -> dict[str, Any]:
        ...


class BridgeRepositoryPort(Protocol):
    def seen_event(self, event_id: str) -> bool:
        ...

    def mark_event_seen(self, event_id: str) -> None:
        ...

    def save_mapping(self, work_order_id: str, rmf_task_id: str, tenant_id: str) -> None:
        ...

    def get_rmf_task_id(self, work_order_id: str) -> str | None:
        ...

    def save_last_status(self, work_order_id: str, status: str) -> None:
        ...

    def get_last_status(self, work_order_id: str) -> str | None:
        ...

    def list_mappings(self) -> list[tuple[str, str, str]]:
        ...


class WorkOrderToRmfMapperPort(Protocol):
    def requires_robot_dispatch(self, event: WorkOrderCreateEvent) -> bool:
        ...

    def to_dispatch_payload(self, event: WorkOrderCreateEvent) -> dict[str, Any]:
        ...

    def to_robot_payload(self, event: WorkOrderCreateEvent) -> dict[str, Any]:
        ...


class RmfToWorkOrderMapperPort(Protocol):
    def to_update_event(self, rmf_task_state: dict[str, Any]) -> WorkOrderUpdateEvent:
        ...


class DeadLetterPublisherPort(Protocol):
    def publish_invalid(self, raw_payload: bytes, reason: str) -> None:
        ...

    def publish_failed_dispatch(self, event_id: str, reason: str, context: dict[str, Any]) -> None:
        ...
