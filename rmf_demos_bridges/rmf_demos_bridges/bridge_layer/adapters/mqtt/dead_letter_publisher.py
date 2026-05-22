import base64
import json
from dataclasses import dataclass
from time import time
from typing import Any

from ...contracts.ports import DeadLetterPublisherPort, MqttPort


@dataclass
class MqttDeadLetterPublisher(DeadLetterPublisherPort):
    mqtt: MqttPort
    topic_dead_letter: str
    qos: int

    def publish_invalid(self, raw_payload: bytes, reason: str) -> None:
        msg = {
            "type": "invalid_payload",
            "reason": reason,
            "occurred_at_unix_ms": int(time() * 1000),
            "payload_base64": base64.b64encode(raw_payload).decode("ascii"),
        }
        self.mqtt.publish(
            topic=self.topic_dead_letter,
            payload=json.dumps(msg),
            qos=self.qos,
            retain=False,
        )

    def publish_failed_dispatch(self, event_id: str, reason: str, context: dict[str, Any]) -> None:
        msg = {
            "type": "dispatch_failed",
            "event_id": event_id,
            "reason": reason,
            "context": context,
            "occurred_at_unix_ms": int(time() * 1000),
        }
        self.mqtt.publish(
            topic=self.topic_dead_letter,
            payload=json.dumps(msg),
            qos=self.qos,
            retain=False,
        )
