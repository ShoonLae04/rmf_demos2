from dataclasses import dataclass


@dataclass(frozen=True)
class RobotRef:
    fleet: str | None = None
    name: str | None = None


@dataclass(frozen=True)
class WorkOrderUpdateEvent:
    schema_version: str
    event_type: str
    event_id: str
    occurred_at_unix_ms: int
    tenant_id: str
    source_system: str
    work_order_id: str
    status: str
    rmf_task_id: str | None = None
    status_reason: str | None = None
    robot: RobotRef | None = None
    progress_percent: int | None = None
