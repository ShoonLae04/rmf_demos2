import argparse
import json
import math
import sys
import time
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import RobotMode, RobotState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import requests


BLOCKING_MODES = {
    RobotMode.MODE_WAITING,
    RobotMode.MODE_ADAPTER_ERROR,
}


class PerceptionBridge(Node):
    _semantic_severity_map = {
        'stain': 'info',
        'obstacle': 'info',
        'unattended_bag': 'urgent',
        'fire': 'critical',
    }

    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('-r', '--robot_state_topic',
                            default='/robot_state',
                            help='Robot state topic to monitor')
        parser.add_argument('-e', '--entities_topic',
                            default='/sim_injected_entities',
                            help='Injected entity metadata topic')
        parser.add_argument('--semantic_alerts_topic',
                    default='/semantic_obstacle_alerts',
                    help='Semantic obstacle alerts topic')
        parser.add_argument('--enable-semantic-patrol-reactions', action='store_true',
                help='Enable TinyRobot semantic patrol reaction logging')
        parser.add_argument('--enable-fire-critical-response', action='store_true',
                help='Enable fire-specific critical response for TinyRobot')
        parser.add_argument('--fire-safe-distance-m',
                            type=float,
                            default=2.0,
                            help='Safe distance threshold for fire (meters)')
        parser.add_argument('--fire-alert-interval-seconds',
                            type=float,
                            default=2.0,
                            help='Interval for periodic fire critical alerts')
        parser.add_argument('-a', '--alert_topic',
                            default='/rmf_demo_alerts',
                            help='Alert topic to publish')
        parser.add_argument('-d', '--alert_distance_threshold',
                            type=float,
                            default=1.0,
                            help='Distance threshold for alerting')
        parser.add_argument('-c', '--alert_cooldown_seconds',
                            type=float,
                            default=2.0,
                            help='Debounce interval for repeated alerts')
        parser.add_argument('--trigger-on-any-nearby', action='store_true',
                    help='Emit alerts for nearby injected entities even when '
                     'the robot is not in a blocking mode. Useful for '
                     'manual validation.')

        self.args, _ = parser.parse_known_args(argv[1:])
        super().__init__('perception_bridge')

        self._robots: Dict[str, Dict[str, object]] = {}
        self._entities: Dict[str, Dict[str, object]] = {}
        self._last_alert_time: Dict[Tuple[str, str, str], float] = {}
        self._semantic_last_alert_time: Dict[Tuple[str, str, float, float], float] = {}
        self._semantic_patrol_reaction_last_time: Dict[Tuple[str, str], float] = {}
        # TinyRobot1-only fire safety state and publisher map
        self.robot_state = {'TinyRobot1': 'PATROL'}
        self.robot_fire_state: Dict[str, Dict[str, object]] = {}
        self._cmd_vel_publishers: Dict[str, object] = {}
        self._fire_alert_timer = self.create_timer(
            self.args.fire_alert_interval_seconds,
            self._fire_periodic_alert_timer_callback)

        self._alert_pub = self.create_publisher(String, self.args.alert_topic, 10)
        # Publisher for semantic alerts (used for fire critical alerts)
        self._semantic_pub = self.create_publisher(String, self.args.semantic_alerts_topic, 10)
        # Generic cmd_vel publisher kept for non-fire legacy paths
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._robot_state_sub = self.create_subscription(
            RobotState,
            self.args.robot_state_topic,
            self._robot_state_callback,
            10)
        self._entities_sub = self.create_subscription(
            String,
            self.args.entities_topic,
            self._entities_callback,
            10)
        self._semantic_alerts_sub = self.create_subscription(
            String,
            self.args.semantic_alerts_topic,
            self._semantic_alerts_callback,
            10)

    # Helper: compute distance between two objects/dicts with x,y
    def compute_distance(self, a, b):
        try:
            ax = getattr(a, 'x', None)
            ay = getattr(a, 'y', None)
            if ax is None or ay is None:
                ax = a.get('x') if isinstance(a, dict) else None
                ay = a.get('y') if isinstance(a, dict) else None

            bx = getattr(b, 'x', None)
            by = getattr(b, 'y', None)
            if bx is None or by is None:
                bx = b.get('x') if isinstance(b, dict) else None
                by = b.get('y') if isinstance(b, dict) else None

            if ax is None or ay is None or bx is None or by is None:
                return float('inf')

            return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)
        except Exception:
            return float('inf')

    def _robot_state_callback(self, msg: RobotState):
        self._robots[msg.name] = {
            'x': msg.location.x,
            'y': msg.location.y,
            'level_name': msg.location.level_name,
            'mode': msg.mode.mode,
            'mode_name': self._mode_name(msg.mode.mode),
        }
        # Ensure we have a per-robot cmd_vel publisher for continuous STOP publishing
        if msg.name not in self._cmd_vel_publishers:
            try:
                topic = f'/{msg.name}/cmd_vel'
                if msg.name == 'TinyRobot1':
                    self._cmd_vel_publishers[msg.name] = self.create_publisher(Twist, topic, 10)
                    self.get_logger().info(f'Created cmd_vel publisher for {msg.name}: {topic}')
            except Exception:
                # fallback remains global cmd_vel
                pass

        # TinyRobot1-only: re-evaluate cached fire entities whenever TinyRobot1 moves.
        # This is the missing step when the fire was injected earlier and the robot later
        # drives into range without receiving a new entity message.
        if msg.name == 'TinyRobot1' and self._entities:
            for entity in self._entities.values():
                if str(entity.get('classification', '')).strip().lower() != 'fire':
                    continue
                try:
                    self.handle_fire_tinyrobot1(self._robots[msg.name], entity)
                except Exception as exc:
                    self.get_logger().warn(f'Error re-evaluating TinyRobot1 fire on pose update: {exc}')
        self._evaluate_robot(msg.name)

    def _entities_callback(self, msg: String):
        entities = self._parse_entities_payload(msg.data)
        if not entities:
            return

        for entity in entities:
            self._entities[entity['name']] = entity

        # Evaluate proximity alerts and fire detection for all robots
        for robot_name in list(self._robots.keys()):
            self._evaluate_robot(robot_name)

        # Additional event-driven fire detection: TinyRobot1 only
        for entity in entities:
            try:
                ent_type = str(entity.get('classification', '')).strip().lower()
            except Exception:
                continue

            if ent_type != 'fire':
                continue

            robot_name = 'TinyRobot1'
            robot = self._robots.get(robot_name)
            if robot is None:
                continue

            try:
                self.handle_fire_tinyrobot1(robot, entity)
            except Exception as exc:
                self.get_logger().warn(f'Error processing fire event for {robot_name}: {exc}')

    def _semantic_alerts_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'Unable to parse semantic obstacle alert payload: {exc}')
            return

        if not isinstance(payload, dict):
            self.get_logger().warn('Semantic obstacle alert payload must be a dict')
            return

        robot_name = str(payload.get('robot_name', '')).strip()
        object_type = self._semantic_object_type(payload.get('object_type', 'obstacle'))
        severity = self._semantic_severity(object_type)

        try:
            x = float(payload.get('x', 0.0))
            y = float(payload.get('y', 0.0))
        except (TypeError, ValueError):
            self.get_logger().warn('Semantic obstacle alert payload has invalid coordinates')
            return

        timestamp = float(payload.get('timestamp', time.time()))
        alert_key = (self._robot_key(robot_name), object_type, round(x, 3), round(y, 3))
        last_alert = self._semantic_last_alert_time.get(alert_key)
        if last_alert is not None and timestamp - last_alert < self.args.alert_cooldown_seconds:
            return

        self._semantic_last_alert_time[alert_key] = timestamp
        # If this is a fire semantic alert, use TinyRobot1-only processing
        if object_type == 'fire':
            if robot_name != 'TinyRobot1':
                return
            robot = self._robots.get('TinyRobot1')
            if robot is None:
                return
            try:
                fire_entity = {
                    'x': x,
                    'y': y,
                    'name': '',
                    'classification': 'fire',
                    'active': True,
                    'timestamp': timestamp
                }

                self.handle_fire_tinyrobot1(robot, fire_entity)
            except Exception as exc:
                self.get_logger().warn(f'Error processing semantic fire alert: {exc}')
            return

        # Non-fire semantic alerts keep the legacy behavior
        self._publish_semantic_alert(robot_name, object_type, severity, x, y, timestamp)
        self._maybe_log_semantic_patrol_reaction(robot_name, object_type, x, y, timestamp)

    def _semantic_object_type(self, object_type: object) -> str:
        semantic_type = str(object_type).strip().lower() if object_type else 'obstacle'
        if semantic_type in self._semantic_severity_map:
            return semantic_type
        return 'obstacle'

    def _semantic_severity(self, object_type: str) -> str:
        return self._semantic_severity_map.get(object_type, 'info')

    def _maybe_log_semantic_patrol_reaction(self, robot_name: str, object_type: str,
                                            x: float, y: float, timestamp: float):
        if not self.args.enable_semantic_patrol_reactions:
            return

        normalized_robot_name = self._robot_key(robot_name)
        if 'tinyrobot' not in normalized_robot_name:
            return

        if object_type not in ('stain', 'obstacle'):
            return

        reaction_key = (normalized_robot_name, object_type)
        last_reaction_time = self._semantic_patrol_reaction_last_time.get(reaction_key, 0.0)
        if timestamp - last_reaction_time < self.args.alert_cooldown_seconds:
            return

        self._semantic_patrol_reaction_last_time[reaction_key] = timestamp
        self.get_logger().info(
            f'[{robot_name}] Avoiding {object_type} and continuing patrol '
            f'at ({x:.2f},{y:.2f})')

    def _handle_fire_critical_response(self, robot_name: str, object_type: str,
                                       x: float, y: float, timestamp: float,
                                       entity_name: str = '', entity_active: bool = True):
        """State machine for fire-specific critical response with reversible pause."""
        if not self.args.enable_fire_critical_response:
            return

        if object_type != 'fire':
            return

        normalized_robot_name = self._robot_key(robot_name)
        if 'tinyrobot' not in normalized_robot_name:
            return

        # Ensure fire state dict exists for this robot
        if robot_name not in self._fire_state:
            self._fire_state[robot_name] = {
                'fire_active': False,
                'fire_x': 0.0,
                'fire_y': 0.0,
                'fire_entity_name': '',
                'fire_entity_active': True,
                'detection_time': timestamp,
                'last_alert_time': timestamp,
                'fire_last_seen': timestamp,
            }

        fire_state = self._fire_state[robot_name]
        if entity_name:
            fire_state['fire_entity_name'] = entity_name
        fire_state['fire_entity_active'] = bool(entity_active)
        fire_state['fire_last_seen'] = timestamp

        # Fire transition: inactive -> active
        if not fire_state['fire_active']:
            fire_state['fire_active'] = True
            fire_state['detection_time'] = timestamp
            fire_state['last_alert_time'] = timestamp
            self.get_logger().warn(f'[{robot_name}] Fire detected. Patrol paused.')
            # Stop the robot: publish zero velocity (best-effort) and attempt HTTP stop
            try:
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
                # publish global cmd_vel
                try:
                    self._cmd_vel_pub.publish(twist)
                    self.get_logger().info(f'[{robot_name}] Published zero Twist to /cmd_vel')
                except Exception:
                    self.get_logger().warn(f'[{robot_name}] Failed to publish zero Twist to /cmd_vel')

                # Best-effort HTTP stop to fleet manager API
                try:
                    # Use default local RMF demo fleet manager endpoint
                    stop_url = f'http://localhost:8000/open-rmf/rmf_demos_fm/stop_robot?robot_name={robot_name}&cmd_id={int(time.time())}'
                    resp = requests.get(stop_url, timeout=2.0)
                    if resp.status_code == 200:
                        self.get_logger().info(f'[{robot_name}] Requested stop via fleet manager API')
                    else:
                        self.get_logger().warn(f'[{robot_name}] Fleet manager stop request returned {resp.status_code}')
                except Exception as exc:
                    self.get_logger().warn(f'[{robot_name}] Fleet manager stop request failed: {exc}')
            except Exception as exc:
                self.get_logger().warn(f'[{robot_name}] Error issuing stop commands: {exc}')

        # Update fire location
        fire_state['fire_x'] = x
        fire_state['fire_y'] = y

        # Check safe distance to fire
        robot = self._robots.get(robot_name, {})
        if robot:
            robot_x = float(robot.get('x', 0.0))
            robot_y = float(robot.get('y', 0.0))
            distance_to_fire = self._distance(robot_x, robot_y, x, y)
            if distance_to_fire > self.args.fire_safe_distance_m:
                self.get_logger().info(
                    f'[{robot_name}] Fire distance {distance_to_fire:.2f}m > '
                    f'safe distance {self.args.fire_safe_distance_m:.2f}m; '
                    f'paused patrol may resume when clear')
        else:
            self.get_logger().info(
                f'[{robot_name}] Fire at ({x:.2f},{y:.2f}); waiting for safe distance {self.args.fire_safe_distance_m:.2f}m')

    # --- TinyRobot1-only fire processing ---
    def handle_fire_tinyrobot1(self, robot_pose: Dict[str, object], fire_pose: Dict[str, object]):
        robot_name = 'TinyRobot1'
        if not self.args.enable_fire_critical_response:
            return

        distance = self.compute_distance(robot_pose, fire_pose)

        if robot_name not in self.robot_fire_state:
            self.robot_fire_state[robot_name] = {
                'fire_active': False,
                'last_distance': None,
            }

        state = self.robot_fire_state[robot_name]
        state['last_distance'] = distance

        if distance > 2.0:
            state['fire_active'] = False
            self.robot_state[robot_name] = 'PATROL'
            self.publish_resume(robot_name)
            return

        elif 1.5 < distance <= 2.0:
            state['fire_active'] = True
            self.robot_state[robot_name] = 'WARNING'
            self.publish_alert(robot_name, 'FIRE_WARNING', distance)
            return

        elif distance <= 1.5:
            state['fire_active'] = True
            self.robot_state[robot_name] = 'STOPPED'
            self.publish_alert(robot_name, 'FIRE_CRITICAL', distance)
            self.publish_stop(robot_name)

    def publish_alert(self, robot_name: str, alert_type: str, distance: float):
        if robot_name != 'TinyRobot1':
            return

        severity = 'critical' if alert_type == 'FIRE_CRITICAL' else 'warning'
        payload = {
            'robot_name': robot_name,
            'type': 'fire',
            'severity': severity,
            'distance': round(float(distance), 3),
        }

        self._alert_pub.publish(String(data=json.dumps(payload)))

    def publish_stop(self, robot_name: str):
        if robot_name != 'TinyRobot1':
            return

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        pub = self._cmd_vel_publishers.get('TinyRobot1')
        try:
            if pub is not None:
                pub.publish(msg)
            else:
                self._cmd_vel_pub.publish(msg)
            self.get_logger().warn(f'[{robot_name}] FIRE STOP - within 1.5m')
        except Exception as exc:
            self.get_logger().warn(f'[{robot_name}] Failed to publish stop Twist: {exc}')

    def publish_resume(self, robot_name: str):
        if robot_name != 'TinyRobot1':
            return

        self.get_logger().info(f'[{robot_name}] Fire cleared - resume patrol')
        resume_msg = {
            'robot_name': robot_name,
            'action': 'RESUME_PATROL'
        }
        try:
            self._alert_pub.publish(String(data=json.dumps(resume_msg)))
        except Exception as exc:
            self.get_logger().warn(f'[{robot_name}] Failed publishing resume message: {exc}')

    def publish_fire_alert(self, robot_name: str, severity: str, distance: float):
        # severity: 'critical' | 'warning' | 'FIRE_CLEARED'
        payload = {
            'robot_name': robot_name,
            'type': 'fire' if severity in ('critical', 'warning') else severity,
            'severity': severity if severity in ('critical', 'warning') else 'cleared',
            'distance': round(float(distance), 3),
        }
        try:
            self._alert_pub.publish(String(data=json.dumps(payload)))
        except Exception as exc:
            self.get_logger().warn(f'[{robot_name}] Failed to publish fire alert: {exc}')

    def _fire_entity_still_present(self, fire_state: Dict[str, object]) -> bool:
        entity_name = str(fire_state.get('fire_entity_name', '')).strip()
        if not entity_name:
            return True

        entity = self._entities.get(entity_name)
        if entity is None:
            return False

        if not entity.get('active', True):
            return False

        entity_type = str(entity.get('classification', '')).strip().lower()
        if not entity_type:
            entity_type = str(entity.get('object_type', '')).strip().lower()
        return entity_type == 'fire'

    def _resume_robot_after_fire(self, robot_name: str):
        try:
            stop_url = (
                'http://localhost:8000/open-rmf/rmf_demos_fm/toggle_action?'
                f'robot_name={robot_name}'
            )
            resp = requests.post(stop_url, timeout=2.0, json={'toggle': False})
            if resp.status_code == 200:
                self.get_logger().info(f'[{robot_name}] Cleared stop latch via fleet manager API')
            else:
                self.get_logger().warn(
                    f'[{robot_name}] Fleet manager resume request returned {resp.status_code}')
        except Exception as exc:
            self.get_logger().warn(f'[{robot_name}] Fleet manager resume request failed: {exc}')

    def _fire_periodic_alert_timer_callback(self):
        """Publish periodic critical fire alerts for all active fire states."""
        if not self.args.enable_fire_critical_response:
            return
        # Use robot-centric state to periodically enforce stops and alerts
        robots_to_clear = []
        for robot_name, state in list(self.robot_fire_state.items()):
            if not state.get('fire_active'):
                continue

            robot = self._robots.get(robot_name)
            last_distance = state.get('last_distance')

            # If we don't have a recent distance or robot pose, skip enforcement
            if last_distance is None or robot is None:
                continue

            # If distance > 2.0, clear
            if last_distance > 2.0:
                robots_to_clear.append(robot_name)
                continue

            # If critical zone, continuously publish stop
            if last_distance <= 1.5:
                self.publish_stop(robot_name)
                # Also publish periodic critical alert
                self.publish_fire_alert(robot_name, 'critical', last_distance)
                continue

            # Warning zone: periodic warning alert
            if 1.5 < last_distance <= 2.0:
                self.publish_fire_alert(robot_name, 'warning', last_distance)

        # Clear states for robots where fire is far away
        for robot_name in robots_to_clear:
            st = self.robot_fire_state.get(robot_name)
            if not st:
                continue
            self.get_logger().info(f'[{robot_name}] Fire cleared (distance {st.get("last_distance"):.2f}m). Patrol may resume.')
            st['fire_active'] = False
            # publish cleared alert and resume
            self.publish_fire_alert(robot_name, 'FIRE_CLEARED', st.get('last_distance', 999.0))
            self.publish_resume(robot_name)

    def _parse_entities_payload(self, payload: str):
        if not payload:
            return []

        try:
            decoded = json.loads(payload)
        except Exception as exc:
            self.get_logger().warn(f'Unable to parse injected entity payload: {exc}')
            return []

        if isinstance(decoded, dict):
            if 'entities' in decoded:
                decoded = decoded['entities']
            else:
                decoded = [decoded]

        if not isinstance(decoded, list):
            self.get_logger().warn('Injected entity payload must be a dict or list')
            return []

        normalized = []
        for entry in decoded:
            entity = self._normalize_entity(entry)
            if entity is not None:
                normalized.append(entity)
        return normalized

    def _normalize_entity(self, entry: object):
        if not isinstance(entry, dict):
            return None

        name = str(entry.get('name', '')).strip()
        if not name:
            return None

        try:
            x = float(entry.get('x'))
            y = float(entry.get('y'))
        except (TypeError, ValueError):
            return None

        level_name = str(entry.get('level_name', '')).strip()
        classification = self._classify_entity(name, entry.get('classification'))

        return {
            'name': name,
            'x': x,
            'y': y,
            'level_name': level_name,
            'classification': classification,
            'active': bool(entry.get('active', True)),
        }

    def _classify_entity(self, name: str, classification: object):
        classification_text = str(classification).strip().lower() if classification else ''
        if classification_text:
            return classification_text

        lowered = name.lower()
        if lowered.startswith('water_puddle'):
            return 'water_puddle'
        if lowered.startswith('intruder') or lowered.startswith('actor_human'):
            return 'intruder'
        if lowered.startswith('barrel'):
            return 'barrel'
        return 'obstacle'

    def _evaluate_robot(self, robot_name: str):
        robot = self._robots.get(robot_name)
        if robot is None:
            return

        nearest = self._find_nearest_entity(robot)
        if nearest is None:
            return

        entity, distance = nearest
        if distance > self.args.alert_distance_threshold:
            return

        is_blocking = int(robot['mode']) in BLOCKING_MODES
        if not is_blocking and not self.args.trigger_on_any_nearby:
            return

        self._publish_alert(robot_name, robot, entity, distance)

    def _find_nearest_entity(self, robot: Dict[str, object]):
        robot_level = str(robot.get('level_name', '')).strip()
        candidates = []
        for entity in self._entities.values():
            if not entity.get('active', True):
                continue

            entity_level = str(entity.get('level_name', '')).strip()
            if robot_level and entity_level and robot_level != entity_level:
                continue

            distance = self._distance(
                float(robot['x']),
                float(robot['y']),
                float(entity['x']),
                float(entity['y']))
            candidates.append((entity, distance))

        if not candidates:
            return None

        return min(candidates, key=lambda item: item[1])

    def _publish_alert(self, robot_name: str, robot: Dict[str, object],
                       entity: Dict[str, object], distance: float):
        alert_type = str(entity.get('classification', 'obstacle'))
        alert_key = (robot_name, entity['name'], alert_type)
        now = time.time()
        last_alert = self._last_alert_time.get(alert_key)
        if last_alert is not None and now - last_alert < self.args.alert_cooldown_seconds:
            return

        self._last_alert_time[alert_key] = now

        payload = {
            'timestamp': now,
            'robot_name': robot_name,
            'level_name': robot.get('level_name', ''),
            'obstacle_type': alert_type,
            'obstacle_name': entity['name'],
            'distance_estimate': round(distance, 3),
            'obstacle_position': {
                'x': round(float(entity['x']), 3),
                'y': round(float(entity['y']), 3),
            },
            'robot_position': {
                'x': round(float(robot['x']), 3),
                'y': round(float(robot['y']), 3),
            },
            'recommended_action': self._recommended_action(alert_type),
            'robot_mode': robot.get('mode_name', 'unknown'),
            'source': 'perception_bridge',
        }

        self.get_logger().warn(
            f"Alerting on {alert_type} '{entity['name']}' for {robot_name} at {distance:.2f} m")
        self._alert_pub.publish(String(data=json.dumps(payload)))

    def _publish_semantic_alert(self, robot_name: str, object_type: str, severity: str,
                                x: float, y: float, timestamp: float):
        robot = self._robots.get(robot_name, {})
        payload = {
            'timestamp': timestamp,
            'robot_name': robot_name,
            'level_name': robot.get('level_name', ''),
            'obstacle_type': object_type,
            'obstacle_name': '',
            'distance_estimate': 0.0,
            'obstacle_position': {
                'x': round(x, 3),
                'y': round(y, 3),
            },
            'robot_position': {
                'x': round(float(robot.get('x', 0.0)), 3),
                'y': round(float(robot.get('y', 0.0)), 3),
            },
            'severity': severity,
            'alert_message': self._semantic_alert_message(object_type),
            'recommended_action': 'Investigate obstacle and replan',
            'robot_mode': robot.get('mode_name', 'unknown'),
            'source': 'perception_bridge',
            'semantic_alert': True,
        }

        self.get_logger().info(
            f"Semantic alert processed for {robot_name or 'unknown_robot'}: "
            f'{object_type} severity={severity} at ({x:.2f},{y:.2f})')
        self._alert_pub.publish(String(data=json.dumps(payload)))

    def _semantic_alert_message(self, object_type: str) -> str:
        if object_type == 'stain':
            return 'Stain detected near patrol route'
        if object_type == 'obstacle':
            return 'Obstacle detected'
        if object_type == 'unattended_bag':
            return 'Unattended bag detected'
        if object_type == 'fire':
            return 'FIRE DETECTED'
        return 'Obstacle detected'

    def _recommended_action(self, alert_type: str):
        if alert_type in ('water_puddle', 'puddle'):
            return 'Work Order: Mop required'
        if alert_type in ('spill', 'spills', 'stain', 'stains'):
            return 'Work Order: Scrub required'
        if alert_type == 'litter':
            return 'Work Order: Vacuum required'
        if alert_type in ('leaf', 'leaves', 'hair'):
            return 'Work Order: Sweep or vacuum required'
        if alert_type == 'intruder':
            return 'Security Alert: investigate immediately'
        if alert_type == 'barrel':
            return 'Dynamic blockage: replan or clear the path'
        return 'Investigate obstacle and replan'

    def _distance(self, x0: float, y0: float, x1: float, y1: float):
        return math.hypot(x1 - x0, y1 - y0)

    def _robot_key(self, robot_name: str) -> str:
        return str(robot_name).strip().lower()

    def _mode_name(self, mode: int):
        names = {
            RobotMode.MODE_IDLE: 'idle',
            RobotMode.MODE_CHARGING: 'charging',
            RobotMode.MODE_MOVING: 'moving',
            RobotMode.MODE_PAUSED: 'paused',
            RobotMode.MODE_WAITING: 'waiting',
            RobotMode.MODE_EMERGENCY: 'emergency',
            RobotMode.MODE_GOING_HOME: 'going_home',
            RobotMode.MODE_DOCKING: 'docking',
            RobotMode.MODE_ADAPTER_ERROR: 'adapter_error',
        }
        return names.get(mode, f'unknown_{mode}')


def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = PerceptionBridge(argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
