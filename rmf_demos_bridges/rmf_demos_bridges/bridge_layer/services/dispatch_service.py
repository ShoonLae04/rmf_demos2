from ..contracts.inbound import WorkOrderCreateEvent
from ..contracts.ports import (
    BridgeRepositoryPort,
    DeadLetterPublisherPort,
    RmfApiPort,
    WorkOrderToRmfMapperPort,
)
import logging


class DispatchService:
    def __init__(
        self,
        rmf_api: RmfApiPort,
        repository: BridgeRepositoryPort,
        dead_letter: DeadLetterPublisherPort,
        mapper: WorkOrderToRmfMapperPort,
    ) -> None:
        self._rmf_api = rmf_api
        self._repository = repository
        self._dead_letter = dead_letter
        self._mapper = mapper

    def handle_create(self, event: WorkOrderCreateEvent) -> None:
        logger = logging.getLogger("bridge_layer.dispatch")
        if self._repository.seen_event(event.event_id):
            logger.info("Event %s already seen, ignoring", event.event_id)
            return

        try:
            if self._mapper.requires_robot_dispatch(event):
                payload = self._mapper.to_robot_payload(event)
                logger.info("Dispatching robot_task for work_order=%s", event.work_order.work_order_id)
                response = self._rmf_api.robot_task(payload)
            else:
                payload = self._mapper.to_dispatch_payload(event)
                logger.info("Dispatching dispatch_task for work_order=%s", event.work_order.work_order_id)
                response = self._rmf_api.dispatch_task(payload)

            logger.info("RMF response: %s", response)
            rmf_task_id = self._extract_task_id(response)
            self._repository.mark_event_seen(event.event_id)
            self._repository.save_mapping(
                work_order_id=event.work_order.work_order_id,
                rmf_task_id=rmf_task_id,
                tenant_id=event.tenant_id,
            )
            logger.info("Saved mapping work_order=%s -> rmf_task=%s", event.work_order.work_order_id, rmf_task_id)
        except Exception as err:
            logger.exception("Dispatch failed for event %s: %s", event.event_id, err)
            self._dead_letter.publish_failed_dispatch(
                event_id=event.event_id,
                reason=str(err),
                context={
                    "work_order_id": event.work_order.work_order_id,
                    "tenant_id": event.tenant_id,
                },
            )

    @staticmethod
    def _extract_task_id(response: dict) -> str:
        state = response.get("state")
        if not isinstance(state, dict):
            raise ValueError("missing state in RMF response")
        booking = state.get("booking")
        if not isinstance(booking, dict):
            raise ValueError("missing booking in RMF response")
        booking_id = booking.get("id")
        if not isinstance(booking_id, str) or not booking_id:
            raise ValueError("missing booking.id in RMF response")
        return booking_id
