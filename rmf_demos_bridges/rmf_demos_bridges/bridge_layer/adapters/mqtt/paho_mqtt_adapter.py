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
        # store handler + qos so we can re-subscribe with the same qos on reconnect
        self._handlers: dict[str, tuple[MqttHandler, int]] = {}
        self._lock = Lock()

        # wire lifecycle callbacks for visibility
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_subscribe = self._on_subscribe
        self._client.on_disconnect = self._on_disconnect
        self._client.on_log = self._on_log

        if username and password:
            self._client.username_pw_set(username, password)

        if tls_enabled:
            context = ssl.create_default_context()
            if tls_ca_path:
                context.load_verify_locations(cafile=tls_ca_path)

            context.check_hostname = not tls_insecure
            context.verify_mode = ssl.CERT_REQUIRED if not tls_insecure else ssl.CERT_NONE
            self._client.tls_set_context(context)

    def connect(self) -> None:
        self._client.connect(self._host, self._port, 60)
        self._client.loop_start()

    def disconnect(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()

    def subscribe(self, topic: str, qos: int, handler: MqttHandler) -> None:
        with self._lock:
            self._handlers[topic] = (handler, qos)
        # request subscribe now; Paho will queue subscribe if not yet connected
        self._client.subscribe(topic, qos)

    def publish(self, topic: str, payload: str, qos: int, retain: bool = False) -> None:
        info = self._client.publish(topic, payload, qos=qos, retain=retain)
        if info.rc != mqtt_client.MQTT_ERR_SUCCESS:
            raise RuntimeError(f"mqtt publish failed rc={info.rc} topic={topic}")

    def _on_message(self, _client: mqtt_client.Client, _userdata, msg) -> None:
        print("🔥 RAW MQTT MESSAGE RECEIVED")
        print("Topic:", msg.topic)
        print("Payload:", msg.payload.decode())
        with self._lock:
            handlers = list(self._handlers.items())

        for topic_filter, (handler, _qos) in handlers:
            if mqtt_client.topic_matches_sub(topic_filter, msg.topic):
                try:
                    handler(msg.topic, msg.payload)
                except Exception as e:
                    print("Handler error:", e)

    def _on_connect(self, client, userdata, flags, rc):
        print(" MQTT CONNECTED, rc =", rc)
        with self._lock:
            items = list(self._handlers.items())

        # re-subscribe with original requested QoS
        for topic, (_handler, qos) in items:
            print(" Subscribing to:", topic, "qos=", qos)
            client.subscribe([(topic, qos)])

    def _on_subscribe(self, client, userdata, mid, granted_qos):
        print(" SUBACK received mid=", mid, "granted_qos=", granted_qos)

    def _on_disconnect(self, client, userdata, rc):
        print(" DISCONNECTED rc=", rc)

    def _on_log(self, client, userdata, level, buf):
        print(" MQTT LOG:", level, buf)
