#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from paho.mqtt import client as mqtt_client


class MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')

        self.mqtt_host = 'cd5921ff.ala.asia-southeast1.emqxsl.com'
        self.mqtt_port = 8883
        self.mqtt_topic = 'rmf/demos/status'

        self.mqtt_user = None
        self.mqtt_password = None
        self.client.tls_set()

        self.client = mqtt_client.Client(client_id='rmf_bridge_node')

        try:
            if self.mqtt_user and self.mqtt_password:
                self.client.username_pw_set(CBM, Katong@123456)

            self.client.connect(self.mqtt_host, self.mqtt_port, 60)
            self.client.loop_start()
            self.get_logger().info(
                f'Connected to MQTT at {self.mqtt_host}:{self.mqtt_port}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT: {e}')
            raise

        self.subscription = self.create_subscription(
            String,
            '/test/topic',
            self.callback,
            10,
        )

    def callback(self, msg: String):
        try:
            payload_data = {
                'source': 'RMF_Bridge',
                'timestamp': time.time(),
                'data': msg.data,
            }
            payload_json = json.dumps(payload_data)

            result = self.client.publish(self.mqtt_topic, payload_json)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                self.get_logger().info(
                    f'Published to {self.mqtt_topic}: {payload_json}'
                )
            else:
                self.get_logger().warn(
                    f'Failed to publish to {self.mqtt_topic}: rc={result.rc}'
                )

        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()