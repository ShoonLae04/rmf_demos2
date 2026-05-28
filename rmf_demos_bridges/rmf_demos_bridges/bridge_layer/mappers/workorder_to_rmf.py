import logging
import os
import re
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
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

_PLACE_NAME_PATTERN = re.compile(r"\bname:\s*(?:\[\s*\d+\s*,\s*)?([A-Za-z0-9_]+)")


class PayloadValidationError(ValueError):
    pass


def get_valid_places(tenant_id: str) -> set[str]:
    valid_places = _load_valid_places_from_rmf_sources(tenant_id)
    if valid_places:
        return valid_places
    return set(DEFAULT_KATONG_OFFICE_PLACES)


@lru_cache(maxsize=16)
def _load_valid_places_from_rmf_sources(tenant_id: str) -> set[str]:
    candidate_paths: list[Path] = []

    for env_name in (
        "BRIDGE_RMF_MAP_PATH",
        "BRIDGE_RMF_MAP_DIR",
        "BRIDGE_RMF_BUILDING_YAML",
        "BRIDGE_RMF_NAV_GRAPH_PATH",
    ):
        env_value = os.getenv(env_name)
        if env_value:
            candidate_paths.append(Path(env_value).expanduser())

    workspace_root = Path(__file__).resolve().parents[6]
    candidate_paths.extend(
        [
            workspace_root / "install" / "rmf_demos_maps" / "share" / "rmf_demos_maps" / "maps" / tenant_id,
            workspace_root / "src" / "rmf_demos" / "rmf_demos_maps" / "maps" / tenant_id,
            workspace_root / "install" / "rmf_demos_maps" / "share" / "rmf_demos_maps" / tenant_id,
            workspace_root / "src" / "rmf_demos" / "rmf_demos_maps" / "maps" / tenant_id / f"{tenant_id}.building.yaml",
        ]
    )

    valid_places: set[str] = set()
    for candidate in candidate_paths:
        valid_places.update(_extract_place_names_from_path(candidate))

    return valid_places


def _extract_place_names_from_path(path: Path) -> set[str]:
    if not path.exists():
        return set()

    if path.is_file():
        files = [path]
    else:
        files = [candidate for candidate in path.rglob("*.yaml") if candidate.is_file()]

    places: set[str] = set()
    for file_path in files:
        try:
            content = file_path.read_text(encoding="utf-8")
        except OSError:
            continue

        for line in content.splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            for match in _PLACE_NAME_PATTERN.finditer(stripped):
                places.add(match.group(1))

    return places


@dataclass
class WorkOrderToRmfRequestMapper:
    include_optional_rmf_fields: bool = field(
        default_factory=lambda: os.getenv(
            "BRIDGE_INCLUDE_OPTIONAL_RMF_FIELDS", "false"
        ).lower()
        in ("1", "true", "yes")
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

        self._warn_for_unknown_places(event.tenant_id, request)

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

        return {
            "category": "patrol",
            "description": {
                "places": normalized_places,
                "rounds": rounds,
            },
        }

    def _build_delivery_request(self, description: Any) -> dict[str, Any]:
        details = self._as_description_obj(description, "delivery")
        pickup_name = self._extract_delivery_place_name(
            details,
            primary_key="pickup",
            legacy_key="pickup_place_name",
            error_message="delivery.pickup_place_name or delivery.pickup.place is required",
        )
        dropoff_name = self._extract_delivery_place_name(
            details,
            primary_key="dropoff",
            legacy_key="dropoff_place_name",
            error_message="delivery.dropoff_place_name or delivery.dropoff.place is required",
        )

        return {
            "category": "delivery",
            "description": {
                "pickup": {
                    "place": pickup_name,
                    "payload": [],
                },
                "dropoff": {
                    "place": dropoff_name,
                    "payload": [],
                },
            },
        }

    @staticmethod
    def _extract_delivery_place_name(
        details: dict[str, Any],
        *,
        primary_key: str,
        legacy_key: str,
        error_message: str,
    ) -> str:
        primary_value = details.get(primary_key)
        if isinstance(primary_value, dict):
            place = primary_value.get("place")
            if isinstance(place, str) and place.strip():
                return place.strip()

        legacy_value = details.get(legacy_key)
        if isinstance(legacy_value, str) and legacy_value.strip():
            return legacy_value.strip()

        raise PayloadValidationError(error_message)

    def _build_clean_request(self, description: Any) -> dict[str, Any]:
        details = self._as_description_obj(description, "clean")
        zone = details.get("zone")
        if zone is None:
            zone = details.get("cleaning_zone")

        if not isinstance(zone, str) or not zone.strip():
            raise PayloadValidationError("clean.zone is required")

        zone_name = zone.strip()
        return {
            "category": "clean",
            "description": {
                "zone": zone_name,
            },
        }

    @staticmethod
    def _as_description_obj(description: Any, category: str) -> dict[str, Any]:
        if not isinstance(description, dict):
            raise PayloadValidationError(
                f"{category}.description must be an object"
            )
        return description

    def _warn_for_unknown_places(self, tenant_id: str, request: dict[str, Any]) -> None:
        valid_places = get_valid_places(tenant_id)

        places_to_check: list[str] = []
        category = request.get("category")
        description = request.get("description")
        if category == "patrol" and isinstance(description, dict):
            places = description.get("places")
            if isinstance(places, list):
                places_to_check.extend([place for place in places if isinstance(place, str)])
        elif category == "delivery" and isinstance(description, dict):
            pickup = description.get("pickup")
            dropoff = description.get("dropoff")
            if isinstance(pickup, dict):
                pickup_place = pickup.get("place")
                if isinstance(pickup_place, str):
                    places_to_check.append(pickup_place)
            elif isinstance(pickup, str):
                places_to_check.append(pickup)

            if isinstance(dropoff, dict):
                dropoff_place = dropoff.get("place")
                if isinstance(dropoff_place, str):
                    places_to_check.append(dropoff_place)
            elif isinstance(dropoff, str):
                places_to_check.append(dropoff)

            legacy_pickup = description.get("pickup_place_name")
            legacy_dropoff = description.get("dropoff_place_name")
            if isinstance(legacy_pickup, str):
                places_to_check.append(legacy_pickup)
            if isinstance(legacy_dropoff, str):
                places_to_check.append(legacy_dropoff)
        elif category == "clean" and isinstance(description, dict):
            zone = description.get("cleaning_zone")
            if isinstance(zone, str):
                places_to_check.append(zone)

        unknown_places = sorted({place for place in places_to_check if place not in valid_places})
        for place in unknown_places:
            logging.warning(
                "PLACE_VALIDATION_WARNING: place not in RMF map but forwarding anyway: %s",
                place,
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
