from dataclasses import dataclass

from .config import BridgeConfig
from .contracts.ports import (
    BridgeRepositoryPort,
    DeadLetterPublisherPort,
    MqttPort,
    RmfApiPort,
)
from .mappers.rmf_to_workorder_update import RmfToWorkOrderUpdateMapper
from .mappers.workorder_to_rmf import WorkOrderToRmfRequestMapper
from .services.dispatch_service import DispatchService
from .services.intake_service import IntakeService
from .services.status_service import StatusService


@dataclass
class BridgeApp:
    config: BridgeConfig
    mqtt: MqttPort
    rmf_api: RmfApiPort
    repository: BridgeRepositoryPort
    dead_letter: DeadLetterPublisherPort
    wo_mapper: WorkOrderToRmfRequestMapper
    status_mapper: RmfToWorkOrderUpdateMapper

    def __post_init__(self) -> None:
        self._status_service: StatusService | None = None

    def start(self) -> None:
        import logging
        logger = logging.getLogger("bridge_layer.app")
        dispatch_service = DispatchService(
            rmf_api=self.rmf_api,
            repository=self.repository,
            dead_letter=self.dead_letter,
            mapper=self.wo_mapper,
        )
        intake_service = IntakeService(
            dispatch_service=dispatch_service,
            dead_letter=self.dead_letter,
        )
        status_service = StatusService(
            mqtt=self.mqtt,
            rmf_api=self.rmf_api,
            repository=self.repository,
            mapper=self.status_mapper,
            topic_update=self.config.mqtt_topic_update,
            qos=self.config.mqtt_qos,
            poll_interval_sec=self.config.rmf_poll_interval_sec,
        )
        self._status_service = status_service

        self.mqtt.connect()
        self.mqtt.subscribe(
            topic=self.config.mqtt_topic_create,
            qos=self.config.mqtt_qos,
            handler=intake_service.on_mqtt_create,
        )

        # Status tracking strategy (poll/subscription) is intentionally left
        # implementation-specific behind StatusService.
        status_service.start()
        logger.info("BridgeApp started: subscribed to %s", self.config.mqtt_topic_create)

    def stop(self) -> None:
        import logging
        logger = logging.getLogger("bridge_layer.app")
        if self._status_service:
            self._status_service.stop()
        self.mqtt.disconnect()
        logger.info("BridgeApp stopped")
