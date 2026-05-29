from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class RobotTarget:
    fleet: str
    robot: str


@dataclass(frozen=True)
class WorkOrderTask:
    category: str
    description: Any


@dataclass(frozen=True)
class PatrolTaskDescription:
    places: list[str]
    rounds: int


@dataclass(frozen=True)
class DeliveryTaskDescription:
    pickup_place_name: str
    dropoff_place_name: str


@dataclass(frozen=True)
class CleanTaskDescription:
    cleaning_zone: str


@dataclass(frozen=True)
class WorkOrder:
    work_order_id: str
    task: WorkOrderTask
    requester: str | None = None
    priority: int | None = None
    earliest_start_unix_ms: int | None = None
    fleet_name: str | None = None
    robot_target: RobotTarget | None = None
    metadata: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class WorkOrderCreateEvent:
    schema_version: str
    event_type: str
    event_id: str
    occurred_at_unix_ms: int
    tenant_id: str
    source_system: str
    work_order: WorkOrder
