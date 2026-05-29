from time import sleep
import logging

from .adapters.mqtt.dead_letter_publisher import MqttDeadLetterPublisher
from .adapters.mqtt.paho_mqtt_adapter import PahoMqttAdapter
from .adapters.rmf.http_rmf_api_client import HttpRmfApiClient
from .adapters.storage.sqlite_repo import SqliteBridgeRepository
from .app import BridgeApp
from .config import BridgeConfig
from .mappers.rmf_to_workorder_update import RmfToWorkOrderUpdateMapper
from .mappers.workorder_to_rmf import WorkOrderToRmfRequestMapper


def main() -> None:
    # configure simple logging for visibility in terminal
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    logger = logging.getLogger("bridge_layer")

    config = BridgeConfig.from_env()

    mqtt = PahoMqttAdapter(
        host=config.mqtt_host,
        port=config.mqtt_port,
        username=config.mqtt_username,
        password=config.mqtt_password,
        tls_enabled=config.mqtt_tls_enabled,
        tls_ca_path=config.mqtt_tls_ca_path,
        tls_insecure=config.mqtt_tls_insecure,
    )
    rmf_api = HttpRmfApiClient(
        base_url=config.rmf_api_base_url,
        token=config.rmf_api_token,
    )
    repository = SqliteBridgeRepository(config.sqlite_path)
    dead_letter = MqttDeadLetterPublisher(
        mqtt=mqtt,
        topic_dead_letter=config.mqtt_topic_dead_letter,
        qos=config.mqtt_qos,
    )
    app = BridgeApp(
        config=config,
        mqtt=mqtt,
        rmf_api=rmf_api,
        repository=repository,
        dead_letter=dead_letter,
        wo_mapper=WorkOrderToRmfRequestMapper(),
        status_mapper=RmfToWorkOrderUpdateMapper(),
    )

    app.start()
    logger.info("Bridge started; MQTT connected and subscribed to %s", config.mqtt_topic_create)
    try:
        while True:
            sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()


if __name__ == "__main__":
    main()
