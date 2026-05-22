import ssl
from threading import Lock

from paho.mqtt import client as mqtt_client

from ...contracts.ports import MqttHandler


class PahoMqttAdapter:
    def __init__(
        self,
        host: str,
        port: int,
        username: str | None,
        password: str | None,
        tls_enabled: bool,
        tls_ca_path: str | None = None,
        tls_insecure: bool = False,
        client_id: str = "rmf_bridge_layer",
    ) -> None:
        self._host = host
        self._port = port
        self._client = mqtt_client.Client(client_id=client_id)
        self._handlers: dict[str, MqttHandler] = {}
        self._lock = Lock()

        if username and password:
            self._client.username_pw_set(username, password)
        if tls_enabled:
            context = ssl.create_default_context()
            if tls_ca_path:
                context.load_verify_locations(cafile=tls_ca_path)
            self._client.tls_set_context(context)
            self._client.tls_insecure_set(tls_insecure)

        self._client.on_message = self._on_message

    def connect(self) -> None:
        self._client.connect(self._host, self._port, 60)
        self._client.loop_start()

    def disconnect(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()

    def subscribe(self, topic: str, qos: int, handler: MqttHandler) -> None:
        with self._lock:
            self._handlers[topic] = handler
        self._client.subscribe(topic, qos)

    def publish(self, topic: str, payload: str, qos: int, retain: bool = False) -> None:
        info = self._client.publish(topic, payload, qos=qos, retain=retain)
        if info.rc != mqtt_client.MQTT_ERR_SUCCESS:
            raise RuntimeError(f"mqtt publish failed rc={info.rc} topic={topic}")

    def _on_message(self, _client: mqtt_client.Client, _userdata, msg) -> None:
        handler = None
        with self._lock:
            handler = self._handlers.get(msg.topic)
        if handler:
            handler(msg.topic, msg.payload)
