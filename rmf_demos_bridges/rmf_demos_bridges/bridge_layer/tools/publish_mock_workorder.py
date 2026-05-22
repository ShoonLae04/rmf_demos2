import json
import ssl
import time
import uuid

from paho.mqtt import client as mqtt_client

from ..config import BridgeConfig


def build_mock_payload() -> dict[str, object]:
    now_ms = int(time.time() * 1000)
    event_id = str(uuid.uuid4())
    work_order_id = f"WO-MOCK-{now_ms}"
    return {
        "schema_version": "1.0",
        "event_type": "workorder.create",
        "event_id": event_id,
        "occurred_at_unix_ms": now_ms,
        "tenant_id": "site_a",
        "source_system": "mock_publisher",
        "work_order": {
            "work_order_id": work_order_id,
            "requester": "test_operator",
            "priority": 1,
            "earliest_start_unix_ms": now_ms,
            "fleet_name": "TinyRobot",
            "robot_target": {
                "fleet": "TinyRobot",
                "robot": "TinyRobot1",
            },
            "task": {
                "category": "patrol",
                "description": {
                    # list of waypoint/place names present in katong_office nav graph
                    "places": ["reception_desk", "desk_person_a"],
                    # number of rounds to patrol the route
                    "rounds": 1,
                },
            },
            "metadata": {
                "site": "katong_office",
                "department": "ops",
            },
        },
    }


def publish_once() -> None:
    config = BridgeConfig.from_env()
    payload = build_mock_payload()

    client = mqtt_client.Client(client_id=f"mock_pub_{uuid.uuid4()}")
    if config.mqtt_username and config.mqtt_password:
        client.username_pw_set(config.mqtt_username, config.mqtt_password)
    if config.mqtt_tls_enabled:
        context = ssl.create_default_context()
        if config.mqtt_tls_ca_path:
            context.load_verify_locations(cafile=config.mqtt_tls_ca_path)
        client.tls_set_context(context)
        client.tls_insecure_set(config.mqtt_tls_insecure)

    client.connect(config.mqtt_host, config.mqtt_port, 60)
    client.loop_start()

    json_payload = json.dumps(payload)
    info = client.publish(
        topic=config.mqtt_topic_create,
        payload=json_payload,
        qos=config.mqtt_qos,
        retain=False,
    )
    # paho-mqtt versions differ: some accept a timeout arg, others don't.
    try:
        info.wait_for_publish(timeout=5)
    except TypeError:
        info.wait_for_publish()
    if info.rc != mqtt_client.MQTT_ERR_SUCCESS:
        raise RuntimeError(f"Failed to publish mock work order rc={info.rc}")

    print("Published mock work order")
    print(f"Topic: {config.mqtt_topic_create}")
    print(f"Event ID: {payload['event_id']}")
    print(f"Work Order ID: {payload['work_order']['work_order_id']}")
    print(f"Payload: {json_payload}")

    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    publish_once()
