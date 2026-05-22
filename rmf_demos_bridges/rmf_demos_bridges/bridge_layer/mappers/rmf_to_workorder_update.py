from dataclasses import dataclass
from time import time
from uuid import uuid4

from ..contracts.outbound import RobotRef, WorkOrderUpdateEvent


@dataclass
class RmfToWorkOrderUpdateMapper:
    status_map: dict[str, str] | None = None

    def __post_init__(self) -> None:
        if self.status_map is None:
            self.status_map = {
                "queued": "ACCEPTED",
                "standby": "ACCEPTED",
                "underway": "IN_PROGRESS",
                "blocked": "IN_PROGRESS",
                "delayed": "IN_PROGRESS",
                "completed": "COMPLETED",
                "failed": "FAILED",
                "error": "FAILED",
                "canceled": "CANCELED",
                "killed": "CANCELED",
                "uninitialized": "PENDING",
            }

    def to_update_event(self, rmf_task_state: dict[str, object]) -> WorkOrderUpdateEvent:
        booking = rmf_task_state.get("booking", {})
        booking_id = booking.get("id") if isinstance(booking, dict) else None
        labels = booking.get("labels", []) if isinstance(booking, dict) else []

        work_order_id = self._extract_label(labels, "work_order_id") or "unknown"
        tenant_id = self._extract_label(labels, "tenant_id") or "unknown"

        raw_status = rmf_task_state.get("status")
        if not isinstance(raw_status, str):
            raw_status = "uninitialized"

        assigned_to = rmf_task_state.get("assigned_to")
        robot = None
        if isinstance(assigned_to, dict):
            robot = RobotRef(
                fleet=assigned_to.get("group") if isinstance(assigned_to.get("group"), str) else None,
                name=assigned_to.get("name") if isinstance(assigned_to.get("name"), str) else None,
            )

        return WorkOrderUpdateEvent(
            schema_version="1.0",
            event_type="workorder.update",
            event_id=str(uuid4()),
            occurred_at_unix_ms=int(time() * 1000),
            tenant_id=tenant_id,
            source_system="rmf_bridge",
            work_order_id=work_order_id,
            rmf_task_id=booking_id if isinstance(booking_id, str) else None,
            status=self.status_map.get(raw_status, "PENDING"),
            status_reason=raw_status,
            robot=robot,
        )

    @staticmethod
    def _extract_label(labels: object, key: str) -> str | None:
        if not isinstance(labels, list):
            return None
        prefix = f"{key}="
        for item in labels:
            if isinstance(item, str) and item.startswith(prefix):
                return item[len(prefix):]
        return None
