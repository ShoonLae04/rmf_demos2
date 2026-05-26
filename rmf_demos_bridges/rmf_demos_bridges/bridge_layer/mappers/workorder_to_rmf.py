import os
from dataclasses import dataclass, field
from typing import Any

from ..contracts.inbound import WorkOrderCreateEvent


DEFAULT_KATONG_OFFICE_PLACES: tuple[str, ...] = (
    "meeting_room1",
    "meeting_room2",
    "meeting_room3",
    "meeting_room4",
    "meeting_room5",
    "pantry",
    "conference_room",
    "inno_room",
    "clean_inno_room",
    "clean_conference_room",
    "tinyRobot1_charger",
    "cleanerbot_charger1",
    "DeliveryRobot_charger",
)


class PayloadValidationError(ValueError):
    pass


@dataclass
class WorkOrderToRmfRequestMapper:
    include_optional_rmf_fields: bool = field(
        default_factory=lambda: os.getenv(
            "BRIDGE_INCLUDE_OPTIONAL_RMF_FIELDS", "false"
        ).lower()
        in ("1", "true", "yes")
    )
    allowed_places: set[str] = field(
        default_factory=lambda: set(DEFAULT_KATONG_OFFICE_PLACES)
    )

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
        category = event.work_order.task.category
        description = event.work_order.task.description

        if category == "patrol":
            request = self._build_patrol_request(description)
        elif category == "delivery":
            request = self._build_delivery_request(description)
        elif category == "clean":
            request = self._build_clean_request(description)
        else:
            raise PayloadValidationError(
                f"unsupported category '{category}', expected patrol|delivery|clean"
            )

        if self.include_optional_rmf_fields:
            request.update(self._build_optional_request_fields(event))

        return request

    def _build_patrol_request(self, description: Any) -> dict[str, Any]:
        details = self._as_description_obj(description, "patrol")
        places = details.get("places")
        rounds = details.get("rounds")

        if not isinstance(places, list) or not places:
            raise PayloadValidationError("patrol.places must be a non-empty list")
        normalized_places: list[str] = []
        for place in places:
            if not isinstance(place, str) or not place.strip():
                raise PayloadValidationError("patrol.places must contain non-empty strings")
            normalized_places.append(place.strip())

        if not isinstance(rounds, int) or rounds < 1:
            raise PayloadValidationError("patrol.rounds must be an integer >= 1")

        self._validate_places_exist(normalized_places)
        return {
            "category": "patrol",
            "description": {
                "places": normalized_places,
                "rounds": rounds,
            },
        }

    def _build_delivery_request(self, description: Any) -> dict[str, Any]:
        details = self._as_description_obj(description, "delivery")
        pickup = details.get("pickup_place_name")
        dropoff = details.get("dropoff_place_name")

        if not isinstance(pickup, str) or not pickup.strip():
            raise PayloadValidationError("delivery.pickup_place_name is required")
        if not isinstance(dropoff, str) or not dropoff.strip():
            raise PayloadValidationError("delivery.dropoff_place_name is required")

        pickup_name = pickup.strip()
        dropoff_name = dropoff.strip()
        self._validate_places_exist([pickup_name, dropoff_name])

        return {
            "category": "delivery",
            "description": {
                "pickup_place_name": pickup_name,
                "dropoff_place_name": dropoff_name,
            },
        }

    def _build_clean_request(self, description: Any) -> dict[str, Any]:
        details = self._as_description_obj(description, "clean")
        zone = details.get("cleaning_zone")

        if not isinstance(zone, str) or not zone.strip():
            raise PayloadValidationError("clean.cleaning_zone is required")

        zone_name = zone.strip()
        self._validate_places_exist([zone_name])
        return {
            "category": "clean",
            "description": {
                "cleaning_zone": zone_name,
            },
        }

    @staticmethod
    def _as_description_obj(description: Any, category: str) -> dict[str, Any]:
        if not isinstance(description, dict):
            raise PayloadValidationError(
                f"{category}.description must be an object"
            )
        return description

    def _validate_places_exist(self, places: list[str]) -> None:
        unknown = [place for place in places if place not in self.allowed_places]
        if unknown:
            raise PayloadValidationError(
                "unknown katong_office place(s): " + ", ".join(unknown)
            )

    def _build_optional_request_fields(self, event: WorkOrderCreateEvent) -> dict[str, Any]:
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
