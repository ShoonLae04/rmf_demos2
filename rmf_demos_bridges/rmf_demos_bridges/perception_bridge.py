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
    FIRE_NORMAL_PATROL = 'NORMAL_PATROL'
    FIRE_WARNING = 'FIRE_WARNING'
    FIRE_CRITICAL_STOPPED = 'FIRE_CRITICAL_STOPPED'

    def __init__(self, argv=sys.argv):
        # Pre-process argv to handle arguments passed as single space-separated strings
        # This occurs when ROS2 launch concatenates arguments without proper tokenization
        processed_argv = []
        for i, arg in enumerate(argv[1:], start=1):  # Skip program name
            # If argument starts with -- and contains spaces, it's likely a concatenated string
            if arg.startswith('--') and ' ' in arg and i < len(argv) - 1:
                # Check if next arg is --ros-args or another flag
                next_arg = argv[i + 1] if i + 1 < len(argv) else ''
                if next_arg == '--ros-args' or next_arg.startswith('--'):
                    # Split the concatenated string
                    processed_argv.extend(arg.split())
                    continue
            processed_argv.append(arg)
        
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
        parser.add_argument('--fire-warning-distance-m',
                    type=float,
                    default=3.0,
                    help='Distance threshold for FIRE_WARNING alerts (meters)')
        parser.add_argument('--fire-critical-distance-m',
                    type=float,
                    default=1.5,
                    help='Distance threshold for FIRE_CRITICAL_STOPPED alerts (meters)')
        parser.add_argument('--fire-safe-distance-m',
                            type=float,
                            default=2.0,
                    help='Legacy alias for fire-warning distance (meters)')
        parser.add_argument('--fire-alert-interval-seconds',
                            type=float,
                            default=2.0,
                            help='Interval for periodic fire critical alerts')
        parser.add_argument('-a', '--alert_topic',
                            default='/rmf_demo_alerts',
                            help='Alert topic to publish')
        parser.add_argument('-d', '--alert_distance_threshold',
                            type=float,
                            default=3.0,
                            help='Distance threshold for alerting')
        parser.add_argument('-c', '--alert_cooldown_seconds',
                            type=float,
                            default=2.0,
                            help='Debounce interval for repeated alerts')
        parser.add_argument('--trigger-on-any-nearby', action='store_true',
                    help='Emit alerts for nearby injected entities even when '
                     'the robot is not in a blocking mode. Useful for '
                     'manual validation.')

        self.args, _ = parser.parse_known_args(processed_argv)
        super().__init__('perception_bridge')
        self.get_logger().warn(f"[PERCEPTION_BRIDGE_INIT] argv={argv}")
        self.get_logger().warn(f"[PERCEPTION_BRIDGE_INIT] enable_fire_critical_response={self.args.enable_fire_critical_response}")

        self._robots: Dict[str, Dict[str, object]] = {}
        self._entities: Dict[str, Dict[str, object]] = {}
        self._last_alert_time: Dict[Tuple[str, str, str], float] = {}
        self._semantic_last_alert_time: Dict[Tuple[str, str, float, float], float] = {}
        self._semantic_patrol_reaction_last_time: Dict[Tuple[str, str], float] = {}
        # TinyRobot1-only fire safety state and publisher map
        self.robot_state = {'TinyRobot1': 'PATROL'}
        self.robot_fire_state: Dict[str, Dict[str, object]] = {}
        self._cmd_vel_publishers: Dict[str, object] = {}
        self._fire_warning_distance_m = float(
            self.args.fire_warning_distance_m or self.args.fire_safe_distance_m
        )
        self._fire_critical_distance_m = float(self.args.fire_critical_distance_m)
        self._fire_clear_timeout_sec = max(4.0, self.args.fire_alert_interval_seconds * 3.0)
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
        self.get_logger().warn(
            f"ROBOT UPDATE: {msg.name} x={msg.location.x:.2f} y={msg.location.y:.2f}"
        )
        self.get_logger().warn(f"[DEBUG] Robot {msg.name} updated, total robots tracked: {len(self._robots) + 1}")
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
            self.get_logger().warn(
                f"ENTITY STORED: {entity}"
            )

        # Evaluate proximity alerts and fire detection for all robots
        for robot_name in list(self._robots.keys()):
            self._evaluate_robot(robot_name)

        # Additional event-driven fire detection: TinyRobot1 only
        for entity in entities:
            try:
                ent_type = str(entity.get('classification', '')).strip().lower()
            except Exception:
                continue

            self.get_logger().warn(f"[DEBUG] Entity: {entity.get('name')}, classification={ent_type}")
            
            if ent_type != 'fire':
                self.get_logger().info(f"[DEBUG] Skipping {ent_type}, not fire")
                continue

            self.get_logger().warn(f"[DEBUG] ✓ FIRE ENTITY DETECTED: {entity.get('name')}")
            robot_name = 'TinyRobot1'
            
            self.get_logger().warn(f"[DEBUG] Looking for robot: {robot_name}")
            self.get_logger().warn(f"[DEBUG] Available robots: {list(self._robots.keys())}")
            
            robot = self._robots.get(robot_name)
            if robot is None:
                self.get_logger().warn(f"[DEBUG] ✗ Robot {robot_name} NOT FOUND in tracked robots")
                continue

            self.get_logger().warn(f"[DEBUG] ✓ Robot {robot_name} found at ({robot['x']:.2f}, {robot['y']:.2f})")
            distance = self.compute_distance(robot, entity)
            self.get_logger().warn(f"[DEBUG] Fire distance: {distance:.2f}m, Critical: {self._fire_critical_distance_m}m, Warning: {self._fire_warning_distance_m}m")
            
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

        # TinyRobot1-specific obstacle type routing (fire, puddle, stain, etc.)
        if robot_name == 'TinyRobot1':
            robot = self._robots.get('TinyRobot1')
            if robot is not None:
                alert_key = (self._robot_key(robot_name), object_type, round(x, 3), round(y, 3))
                last_alert = self._semantic_last_alert_time.get(alert_key)
                if last_alert is not None and timestamp - last_alert < 0.2:
                    return

                self._semantic_last_alert_time[alert_key] = timestamp

                obstacle_entity = {
                    'x': x,
                    'y': y,
                    'name': '',
                    'classification': object_type,
                    'active': True,
                    'timestamp': timestamp
                }

                try:
                    if object_type == 'fire':
                        self.handle_fire_tinyrobot1(robot, obstacle_entity)
                    elif object_type in ('puddle', 'water_puddle'):
                        self.handle_puddle_tinyrobot1(robot, obstacle_entity)
                    elif object_type == 'stain':
                        self.handle_stain_tinyrobot1(robot, obstacle_entity)
                    else:
                        # Generic obstacle handler for other types
                        self.handle_obstacle_tinyrobot1(robot, obstacle_entity, object_type)
                except Exception as exc:
                    self.get_logger().warn(f'Error processing semantic {object_type} alert for TinyRobot1: {exc}')
                return

        alert_key = (self._robot_key(robot_name), object_type, round(x, 3), round(y, 3))
        last_alert = self._semantic_last_alert_time.get(alert_key)
        if last_alert is not None and timestamp - last_alert < self.args.alert_cooldown_seconds:
            return

        self._semantic_last_alert_time[alert_key] = timestamp

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

    # --- TinyRobot1-only fire processing ---
    def handle_fire_tinyrobot1(self, robot_pose: Dict[str, object], fire_pose: Dict[str, object]):
        robot_name = 'TinyRobot1'
        self.get_logger().warn(f"[DEBUG] handle_fire_tinyrobot1 called")
        
        if not self.args.enable_fire_critical_response:
            self.get_logger().warn(f"[DEBUG] ✗ Fire critical response DISABLED")
            return

        self.get_logger().warn(f"[DEBUG] ✓ Fire critical response ENABLED")
        
        if not robot_pose:
            self.get_logger().warn(f"[DEBUG] ✗ No robot pose provided")
            return

        now = time.time()
        state = self._fire_state_for_robot(robot_name)
        fire_name = str(fire_pose.get('name', '')).strip()
        fire_level_name = str(fire_pose.get('level_name', '')).strip()
        fire_active = bool(fire_pose.get('active', True))

        state['fire_entity_name'] = fire_name
        state['fire_level_name'] = fire_level_name
        state['fire_x'] = float(fire_pose.get('x', 0.0))
        state['fire_y'] = float(fire_pose.get('y', 0.0))
        state['entity_active'] = fire_active
        state['last_seen'] = now

        if not fire_active:
            self._handle_fire_cleared(robot_name, state, reason='fire entity became inactive')
            return

        distance = self.compute_distance(robot_pose, fire_pose)
        state['last_distance'] = distance
        self.get_logger().warn(f"[DEBUG] Computed distance: {distance:.2f}m")

        if state['state'] == self.FIRE_CRITICAL_STOPPED:
            self.get_logger().warn(f"[DEBUG] Already in FIRE_CRITICAL_STOPPED state")
            self._handle_fire_critical(robot_name, state, distance, fire_pose, now)
            return

        if distance <= self._fire_critical_distance_m:
            self.get_logger().warn(f"[DEBUG] ✓ ENTERING FIRE_CRITICAL: {distance:.2f}m <= {self._fire_critical_distance_m}m")
            self._handle_fire_critical(robot_name, state, distance, fire_pose, now)
            return

        if distance <= self._fire_warning_distance_m:
            self.get_logger().warn(f"[DEBUG] ✓ ENTERING FIRE_WARNING: {distance:.2f}m <= {self._fire_warning_distance_m}m")
            self._handle_fire_warning(robot_name, state, distance, fire_pose, now)
            return

        if state['state'] == self.FIRE_WARNING:
            self._handle_fire_cleared(robot_name, state, reason='fire moved beyond warning distance')
            return

        state['state'] = self.FIRE_NORMAL_PATROL
        state['fire_active'] = False
        self.robot_state[robot_name] = 'PATROL'

    def handle_puddle_tinyrobot1(self, robot_pose: Dict[str, object], puddle_pose: Dict[str, object]):
        """Handle puddle/water detection for TinyRobot1"""
        robot_name = 'TinyRobot1'
        self.get_logger().warn(f"[DEBUG] handle_puddle_tinyrobot1 called")

        if not robot_pose:
            self.get_logger().warn(f"[DEBUG] ✗ No robot pose provided")
            return

        distance = self.compute_distance(robot_pose, puddle_pose)
        self.get_logger().warn(f"[DEBUG] Puddle detected at distance: {distance:.2f}m")

        # Puddles: stop robot to avoid
        if distance <= 2.0:  # Stop within 2m of puddle
            self.get_logger().warn(f"[DEBUG] ✓ PUDDLE ALERT: {distance:.2f}m from puddle - STOPPING ROBOT")
            self._publish_robot_stop('TinyRobot1')
        else:
            self.get_logger().warn(f"[DEBUG] Puddle at safe distance: {distance:.2f}m")

    def handle_stain_tinyrobot1(self, robot_pose: Dict[str, object], stain_pose: Dict[str, object]):
        """Handle stain detection for TinyRobot1"""
        robot_name = 'TinyRobot1'
        self.get_logger().warn(f"[DEBUG] handle_stain_tinyrobot1 called")

        if not robot_pose:
            self.get_logger().warn(f"[DEBUG] ✗ No robot pose provided")
            return

        distance = self.compute_distance(robot_pose, stain_pose)
        self.get_logger().warn(f"[DEBUG] Stain detected at distance: {distance:.2f}m")

        # Stains: stop robot to avoid
        if distance <= 1.5:  # Stop within 1.5m of stain
            self.get_logger().warn(f"[DEBUG] ✓ STAIN ALERT: {distance:.2f}m from stain - STOPPING ROBOT")
            self._publish_robot_stop('TinyRobot1')
        else:
            self.get_logger().warn(f"[DEBUG] Stain at safe distance: {distance:.2f}m")

    def handle_obstacle_tinyrobot1(self, robot_pose: Dict[str, object], obstacle_pose: Dict[str, object], obstacle_type: str):
        """Handle generic obstacle detection for TinyRobot1"""
        robot_name = 'TinyRobot1'
        self.get_logger().warn(f"[DEBUG] handle_obstacle_tinyrobot1 called for type={obstacle_type}")

        if not robot_pose:
            self.get_logger().warn(f"[DEBUG] ✗ No robot pose provided")
            return

        distance = self.compute_distance(robot_pose, obstacle_pose)
        self.get_logger().warn(f"[DEBUG] Obstacle ({obstacle_type}) detected at distance: {distance:.2f}m")

        # Generic obstacles: stop robot if too close
        if distance <= 1.5:  # Stop within 1.5m
            self.get_logger().warn(f"[DEBUG] ✓ OBSTACLE ALERT ({obstacle_type}): {distance:.2f}m - STOPPING ROBOT")
            self._publish_robot_stop('TinyRobot1')
        else:
            self.get_logger().warn(f"[DEBUG] Obstacle ({obstacle_type}) at safe distance: {distance:.2f}m")

    def _fire_state_for_robot(self, robot_name: str) -> Dict[str, object]:
        state = self.robot_fire_state.get(robot_name)
        if state is None:
            state = {
                'state': self.FIRE_NORMAL_PATROL,
                'fire_active': False,
                'fire_entity_name': '',
                'fire_level_name': '',
                'fire_x': 0.0,
                'fire_y': 0.0,
                'entity_active': True,
                'last_seen': 0.0,
                'last_distance': None,
                'last_warning_alert_at': 0.0,
                'last_critical_alert_at': 0.0,
                'last_stop_enforced_at': 0.0,
            }
            self.robot_fire_state[robot_name] = state
        return state

    def _handle_fire_warning(
        self,
        robot_name: str,
        state: Dict[str, object],
        distance: float,
        fire_pose: Dict[str, object],
        now: float,
    ) -> None:
        state['state'] = self.FIRE_WARNING
        state['fire_active'] = True
        state['last_distance'] = distance
        state['last_seen'] = now
        state['fire_entity_name'] = str(fire_pose.get('name', '')).strip()
        state['fire_level_name'] = str(fire_pose.get('level_name', '')).strip()
        state['fire_x'] = float(fire_pose.get('x', 0.0))
        state['fire_y'] = float(fire_pose.get('y', 0.0))
        self.robot_state[robot_name] = 'WARNING'

        last_warning = float(state.get('last_warning_alert_at', 0.0))
        if now - last_warning < self.args.fire_alert_interval_seconds:
            return

        state['last_warning_alert_at'] = now
        self.publish_alert(robot_name, 'warning', distance, now)
        self.get_logger().warn(f'[{robot_name}] Fire warning at {distance:.2f}m; patrol continues')

    def _handle_fire_critical(
        self,
        robot_name: str,
        state: Dict[str, object],
        distance: float,
        fire_pose: Dict[str, object],
        now: float,
    ) -> None:
        state['state'] = self.FIRE_CRITICAL_STOPPED
        state['fire_active'] = True
        state['last_distance'] = distance
        state['last_seen'] = now
        state['fire_entity_name'] = str(fire_pose.get('name', '')).strip()
        state['fire_level_name'] = str(fire_pose.get('level_name', '')).strip()
        state['fire_x'] = float(fire_pose.get('x', 0.0))
        state['fire_y'] = float(fire_pose.get('y', 0.0))
        self.robot_state[robot_name] = 'STOPPED'

        last_critical = float(state.get('last_critical_alert_at', 0.0))
        last_stop = float(state.get('last_stop_enforced_at', 0.0))
        if now - last_critical < self.args.fire_alert_interval_seconds and \
                now - last_stop < self.args.fire_alert_interval_seconds:
            return

        state['last_critical_alert_at'] = now
        state['last_stop_enforced_at'] = now
        self.publish_alert(robot_name, 'critical', distance, now)
        self.publish_stop(robot_name)
        self.get_logger().warn(f'[{robot_name}] Fire critical at {distance:.2f}m; stop enforced')

    def _handle_fire_cleared(
        self,
        robot_name: str,
        state: Dict[str, object],
        reason: str = '',
    ) -> None:
        previous_state = str(state.get('state', self.FIRE_NORMAL_PATROL))
        state['state'] = self.FIRE_NORMAL_PATROL
        state['fire_active'] = False
        state['fire_entity_name'] = ''
        state['fire_level_name'] = ''
        state['fire_x'] = 0.0
        state['fire_y'] = 0.0
        state['last_distance'] = None
        state['last_warning_alert_at'] = 0.0
        state['last_critical_alert_at'] = 0.0
        state['last_stop_enforced_at'] = 0.0
        self.robot_state[robot_name] = 'PATROL'

        if previous_state != self.FIRE_NORMAL_PATROL:
            if reason:
                self.get_logger().info(f'[{robot_name}] Fire cleared ({reason}); resuming patrol')
            else:
                self.get_logger().info(f'[{robot_name}] Fire cleared; resuming patrol')
            self._resume_robot(robot_name)

    def _resume_robot(self, robot_name: str) -> None:
        self.publish_resume(robot_name)

    def publish_alert(self, robot_name: str, severity: str, distance: float, timestamp: float):
        if robot_name != 'TinyRobot1':
            return

        payload = {
            'robot_name': robot_name,
            'severity': severity,
            'distance': round(float(distance), 3),
            'type': 'fire',
            'source': 'perception_bridge',
            'timestamp': timestamp,
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
            self.get_logger().warn(
                f'[{robot_name}] FIRE STOP - within {self._fire_critical_distance_m:.1f}m (cmd_vel published)'
            )
        except Exception as exc:
            self.get_logger().warn(f'[{robot_name}] Failed to publish stop Twist: {exc}')

        # CRITICAL: Call fleet manager API to actually stop the robot's task
        # cmd_vel alone is not sufficient - must pause the task execution
        try:
            stop_url = f'http://localhost:8000/open-rmf/rmf_demos_fm/stop_robot?robot_name={robot_name}&cmd_id={int(time.time())}'
            resp = requests.get(stop_url, timeout=2.0)
            if resp.status_code == 200:
                self.get_logger().warn(f'[{robot_name}] ✓ Fleet manager STOP command successful')
            else:
                self.get_logger().warn(f'[{robot_name}] Fleet manager stop returned status {resp.status_code}')
        except Exception as exc:
            self.get_logger().warn(f'[{robot_name}] Fleet manager stop request failed: {exc}')

    def publish_resume(self, robot_name: str):
        if robot_name != 'TinyRobot1':
            return

        self.get_logger().info(f'[{robot_name}] Fire cleared - attempting to resume patrol')
        
        # First, publish resume alert through the alert topic
        resume_msg = {
            'robot_name': robot_name,
            'action': 'RESUME_PATROL'
        }
        try:
            self._alert_pub.publish(String(data=json.dumps(resume_msg)))
            self.get_logger().info(f'[{robot_name}] Published RESUME_PATROL alert')
        except Exception as exc:
            self.get_logger().warn(f'[{robot_name}] Failed publishing resume alert: {exc}')

        # Also try to resume via fleet manager API if robot was stopped there
        try:
            # Use toggle action to clear any stop latch
            resume_url = (
                f'http://localhost:8000/open-rmf/rmf_demos_fm/toggle_action?'
                f'robot_name={robot_name}'
            )
            resp = requests.post(resume_url, timeout=2.0, json={'toggle': False})
            if resp.status_code == 200:
                self.get_logger().info(f'[{robot_name}] ✓ Fleet manager RESUME command successful')
            else:
                self.get_logger().warn(f'[{robot_name}] Fleet manager resume returned status {resp.status_code}')
        except Exception as exc:
            self.get_logger().warn(f'[{robot_name}] Fleet manager resume request failed: {exc}')

    def _fire_entity_still_present(self, fire_state: Dict[str, object]) -> bool:
        entity_name = str(fire_state.get('fire_entity_name', '')).strip()
        if not entity_name:
            last_seen = float(fire_state.get('last_seen', 0.0))
            return (time.time() - last_seen) <= self._fire_clear_timeout_sec

        entity = self._entities.get(entity_name)
        if entity is None:
            last_seen = float(fire_state.get('last_seen', 0.0))
            return (time.time() - last_seen) <= self._fire_clear_timeout_sec

        if not entity.get('active', True):
            return False

        entity_type = str(entity.get('classification', '')).strip().lower()
        if not entity_type:
            entity_type = str(entity.get('object_type', '')).strip().lower()
        return entity_type == 'fire'

    def _fire_periodic_alert_timer_callback(self):
        """Publish periodic fire alerts and enforce stop while fire remains active."""
        if not self.args.enable_fire_critical_response:
            return
        state = self.robot_fire_state.get('TinyRobot1')
        if not state:
            return

        if not self._fire_entity_still_present(state):
            self._handle_fire_cleared('TinyRobot1', state, reason='fire entity no longer present')
            return

        robot = self._robots.get('TinyRobot1')
        if robot is None:
            return

        fire_pose = {
            'x': float(state.get('fire_x', 0.0)),
            'y': float(state.get('fire_y', 0.0)),
            'name': str(state.get('fire_entity_name', '')).strip(),
            'level_name': str(state.get('fire_level_name', '')).strip(),
            'active': bool(state.get('entity_active', True)),
        }
        distance = self.compute_distance(robot, fire_pose)
        state['last_distance'] = distance
        now = time.time()

        if state.get('state') == self.FIRE_CRITICAL_STOPPED:
            self._handle_fire_critical('TinyRobot1', state, distance, fire_pose, now)
            return

        if distance <= self._fire_critical_distance_m:
            self._handle_fire_critical('TinyRobot1', state, distance, fire_pose, now)
            return

        if distance <= self._fire_warning_distance_m:
            self._handle_fire_warning('TinyRobot1', state, distance, fire_pose, now)
            return

        if state.get('state') == self.FIRE_WARNING:
            self._handle_fire_cleared('TinyRobot1', state, reason='fire moved beyond warning distance')
            return

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
        if lowered.startswith('fire'):
            return 'fire'
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

        alert_type = str(entity.get('classification', 'obstacle')).strip().lower()
        # Allow all robots to detect fire (removed intentional fire block)

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
        # Allow all robots to publish fire alerts (removed fire-only restriction)
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
        if alert_type == 'fire':
            return 'FIRE ALERT: Evacuate area and notify authorities'
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
