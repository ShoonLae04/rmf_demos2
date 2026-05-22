import json
import logging

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

        work_order = WorkOrder(
            work_order_id=work_order_obj["work_order_id"],
            task=WorkOrderTask(
                category=work_order_obj["task"]["category"],
                description=work_order_obj["task"]["description"],
            ),
            requester=work_order_obj.get("requester"),
            priority=work_order_obj.get("priority"),
            earliest_start_unix_ms=work_order_obj.get("earliest_start_unix_ms"),
            fleet_name=work_order_obj.get("fleet_name"),
            robot_target=robot_target,
            metadata=work_order_obj.get("metadata", {}),
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
