from dataclasses import dataclass
import os


@dataclass(frozen=True)
class BridgeConfig:
    mqtt_host: str
    mqtt_port: int
    mqtt_username: str | None
    mqtt_password: str | None
    mqtt_tls_enabled: bool
    mqtt_tls_ca_path: str | None
    mqtt_tls_insecure: bool
    mqtt_topic_create: str
    mqtt_topic_update: str
    mqtt_topic_dead_letter: str
    mqtt_qos: int
    rmf_api_base_url: str
    rmf_api_token: str | None
    rmf_poll_interval_sec: float
    sqlite_path: str

    @staticmethod
    def from_env() -> "BridgeConfig":
        return BridgeConfig(
            mqtt_host=os.getenv("BRIDGE_MQTT_HOST", "localhost"),
            mqtt_port=int(os.getenv("BRIDGE_MQTT_PORT", "8883")),
            mqtt_username=os.getenv("BRIDGE_MQTT_USERNAME") or None,
            mqtt_password=os.getenv("BRIDGE_MQTT_PASSWORD") or None,
            mqtt_tls_enabled=os.getenv("BRIDGE_MQTT_TLS", "true").lower() in ("1", "true", "yes"),
            mqtt_tls_ca_path=os.getenv("BRIDGE_MQTT_CA_PATH") or None,
            mqtt_tls_insecure=os.getenv("BRIDGE_MQTT_TLS_INSECURE", "true").lower() in ("1", "true", "yes"),
            mqtt_topic_create=os.getenv("BRIDGE_TOPIC_CREATE", "digibase/workorder/v1/create"),
            mqtt_topic_update=os.getenv("BRIDGE_TOPIC_UPDATE", "digibase/workorder/v1/update"),
            mqtt_topic_dead_letter=os.getenv("BRIDGE_TOPIC_DLQ", "digibase/workorder/v1/dlq"),
            mqtt_qos=int(os.getenv("BRIDGE_MQTT_QOS", "1")),
            rmf_api_base_url=os.getenv("BRIDGE_RMF_API_BASE_URL", "http://localhost:8000"),
            rmf_api_token=os.getenv("BRIDGE_RMF_API_TOKEN") or None,
            rmf_poll_interval_sec=float(os.getenv("BRIDGE_RMF_POLL_INTERVAL_SEC", "2.0")),
            sqlite_path=os.getenv("BRIDGE_SQLITE_PATH", "./bridge_layer.sqlite3"),
        )
