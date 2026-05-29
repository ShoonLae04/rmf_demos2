import json
import logging
from typing import Any

from ..contracts.inbound import RobotTarget, WorkOrder, WorkOrderCreateEvent, WorkOrderTask
from ..contracts.ports import DeadLetterPublisherPort
from .dispatch_service import DispatchService


class IntakeService:
    def __init__(
        self,
        dispatch_service: DispatchService,
        dead_letter: DeadLetterPublisherPort,
    ) -> None:
        self._dispatch_service = dispatch_service
        self._dead_letter = dead_letter

    def on_mqtt_create(self, topic: str, payload: bytes) -> None:
        logger = logging.getLogger("bridge_layer.intake")
        logger.info("Received MQTT message on %s", topic)
        try:
            obj = json.loads(payload.decode("utf-8"))
            event = self._parse_event(obj)
            logger.info("Parsed WorkOrderCreateEvent event_id=%s work_order_id=%s", event.event_id, event.work_order.work_order_id)
            self._dispatch_service.handle_create(event)
        except Exception as err:
            self._dead_letter.publish_invalid(payload, f"{topic}: {err}")

    @staticmethod
    def _parse_event(obj: dict) -> WorkOrderCreateEvent:
        work_order_obj = obj["work_order"]
        robot_target_obj = work_order_obj.get("robot_target")
        robot_target = None
        if robot_target_obj is not None:
            robot_target = RobotTarget(
                fleet=robot_target_obj["fleet"],
                robot=robot_target_obj["robot"],
            )

        task_obj = work_order_obj["task"]
        category = str(task_obj["category"]).strip().lower()

        work_order = WorkOrder(
            work_order_id=work_order_obj["work_order_id"],
            task=WorkOrderTask(
                category=category,
                description=IntakeService._normalize_task_description(category, task_obj),
            ),
            requester=work_order_obj.get("requester"),
            priority=work_order_obj.get("priority"),
            earliest_start_unix_ms=work_order_obj.get("earliest_start_unix_ms"),
            fleet_name=work_order_obj.get("fleet_name"),
            robot_target=robot_target,
            metadata={
                str(k): str(v)
                for k, v in work_order_obj.get("metadata", {}).items()
            },
        )
        return WorkOrderCreateEvent(
            schema_version=obj["schema_version"],
            event_type=obj["event_type"],
            event_id=obj["event_id"],
            occurred_at_unix_ms=obj["occurred_at_unix_ms"],
            tenant_id=obj["tenant_id"],
            source_system=obj["source_system"],
            work_order=work_order,
        )

    @staticmethod
    def _normalize_task_description(category: str, task_obj: dict[str, Any]) -> dict[str, Any]:
        # Accept both legacy `description` and business-friendly category blocks.
        if category == "patrol":
            details = IntakeService._task_details(task_obj, "patrol")
            return {
                "places": details.get("places"),
                "rounds": details.get("rounds"),
            }

        if category == "delivery":
            details = IntakeService._task_details(task_obj, "delivery")
            return details

        if category == "clean":
            details = IntakeService._task_details(task_obj, "clean")
            zone = details.get("zone")
            if zone is None:
                zone = details.get("cleaning_zone")
            return {
                "zone": zone,
            }

        # Leave unknown categories untouched; mapper validation will reject them.
        description = task_obj.get("description")
        if isinstance(description, dict):
            return description
        raise ValueError(f"task.description must be an object for category '{category}'")

    @staticmethod
    def _task_details(task_obj: dict[str, Any], key: str) -> dict[str, Any]:
        details = task_obj.get(key)
        if details is None:
            details = task_obj.get("description")
        if not isinstance(details, dict):
            raise ValueError(f"task.{key} or task.description must be an object")
        return details
