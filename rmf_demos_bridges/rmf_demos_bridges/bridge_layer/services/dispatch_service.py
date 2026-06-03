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
        print("DISPATCH SERVICE VERSION: DISPATCH_TASK_ONLY v2")
        self._rmf_api = rmf_api
        self._repository = repository
        self._dead_letter = dead_letter
        self._mapper = mapper
        self._active_tasks = {} 

    def get_active_task(self, robot):
        return self._active_tasks.get(robot)


    def set_active_task(self, robot, task_id, priority):
        self._active_tasks[robot] = {
            "rmf_task_id": task_id,
            "priority": priority
        }    


    def handle_create(self, event: WorkOrderCreateEvent) -> None:
        logger = logging.getLogger("bridge_layer.dispatch")
        logger.info(" HANDLE_CREATE ENTERED: event_id=%s", event.event_id)
        if self._repository.seen_event(event.event_id):
            logger.warning("SKIPPING EVENT (already seen): %s", event.event_id)
            return

        try:
            robot = event.work_order.robot_target.robot if event.work_order.robot_target else None
            priority = int(event.work_order.priority or 0)
            
            
            if event.work_order.robot_target:
                robot = event.work_order.robot_target.robot

            current_priority = 0
            active_task_id = None

            logger.info(
                "PREEMPT CHECK: robot=%s priority=%s current_priority=%s active_task_id=%s",
                robot,
                priority,
                current_priority,
                active_task_id,
            )

            existing = self.get_active_task(robot)
            if existing:
                current_priority = existing.get("priority", 0)
                active_task_id = existing.get("rmf_task_id")

            is_critical = priority > current_priority
            logger.info(
                    "IS_CRITICAL=%s",
                    is_critical,
                )

            if is_critical and robot and active_task_id:
                logger.warning(
                    "CRITICAL TASK: preempting active task %s for robot %s",
                    active_task_id,
                    robot,
                )
                

                try:
                    # cancel RMF task
                    self._rmf_api.cancel_task(active_task_id)

                    # stop robot motion (triggers RobotCommandHandle.stop)
                    self._rmf_api.stop_robot(robot)

                    # wait until robot fully idle
                    self._rmf_api.wait_until_idle(robot)

                except Exception as e:
                    logger.error("Failed during preemption: %s", e)

            payload = self._mapper.to_dispatch_payload(event)
            logger.info("RMF PAYLOAD: %s", payload)
            logger.info("Dispatching dispatch_task for work_order=%s", event.work_order.work_order_id)

            response = self._rmf_api.dispatch_task(payload)

            logger.info("RMF response: %s", response)
            # STEP 6: extract RMF task id
           
            rmf_task_id = self._extract_task_id(response)
            # STEP 7: persist mapping
            self._repository.mark_event_seen(event.event_id)

            self._repository.save_mapping(
                work_order_id=event.work_order.work_order_id,
                rmf_task_id=rmf_task_id,
                tenant_id=event.tenant_id,
            )
            if robot:
                self.set_active_task(robot, rmf_task_id, priority)

            logger.info(
                "Saved mapping work_order=%s -> rmf_task=%s",
                event.work_order.work_order_id,
                rmf_task_id,
            )


           
        except Exception as err:
            print("ERROR:", err)
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
