from dataclasses import dataclass
from typing import Any

from ..contracts.inbound import WorkOrderCreateEvent


@dataclass
class WorkOrderToRmfRequestMapper:
    def requires_robot_dispatch(self, event: WorkOrderCreateEvent) -> bool:
        return event.work_order.robot_target is not None

    def to_dispatch_payload(self, event: WorkOrderCreateEvent) -> dict[str, Any]:
        request = self._build_task_request(event)
        return {
            "type": "dispatch_task_request",
            "request": request,
        }

    def to_robot_payload(self, event: WorkOrderCreateEvent) -> dict[str, Any]:
        if event.work_order.robot_target is None:
            raise ValueError("robot_target is required for robot dispatch")

        request = self._build_task_request(event)
        return {
            "type": "robot_task_request",
            "fleet": event.work_order.robot_target.fleet,
            "robot": event.work_order.robot_target.robot,
            "request": request,
        }

    def _build_task_request(self, event: WorkOrderCreateEvent) -> dict[str, Any]:
        labels = [
            f"event_id={event.event_id}",
            f"tenant_id={event.tenant_id}",
            f"source_system={event.source_system}",
            f"work_order_id={event.work_order.work_order_id}",
        ]
        for key, value in event.work_order.metadata.items():
            labels.append(f"{key}={value}")

        request: dict[str, Any] = {
            "unix_millis_request_time": event.occurred_at_unix_ms,
            "category": event.work_order.task.category,
            "description": event.work_order.task.description,
            "labels": labels,
        }
        if event.work_order.requester:
            request["requester"] = event.work_order.requester
        if event.work_order.earliest_start_unix_ms is not None:
            request["unix_millis_earliest_start_time"] = event.work_order.earliest_start_unix_ms
        if event.work_order.priority is not None:
            request["priority"] = {"type": "binary", "value": event.work_order.priority}
        if event.work_order.fleet_name:
            request["fleet_name"] = event.work_order.fleet_name
        return request
