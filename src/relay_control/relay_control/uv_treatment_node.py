#!/usr/bin/env python3
"""Coordinate prescription-driven UV treatment with navigation and GPIO."""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64

from relay_control.treatment_logic import load_waypoints, UVTreatmentSequence
from relay_control.safety_logic import navigation_gate_reason
from relay_control.progress_store import (
    load_progress,
    resumable_index,
    save_progress,
    waypoint_file_fingerprint,
)


class MockOutputDevice:
    """GPIO substitute for development without Raspberry Pi hardware."""

    def __init__(self):
        self.value = 0

    def on(self):
        self.value = 1

    def off(self):
        self.value = 0

    def close(self):
        self.off()


class UVTreatmentNode(Node):
    """Apply ordered UV lamp ON/OFF treatment boundaries."""

    def __init__(self):
        super().__init__('uv_treatment_node')
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('trigger_distance_m', 0.5)
        self.declare_parameter('hysteresis_distance_m', 0.3)
        self.declare_parameter('missed_waypoint_enabled', True)
        self.declare_parameter('approach_confirmation_m', 0.5)
        self.declare_parameter('pass_confirmation_m', 0.3)
        self.declare_parameter('max_closest_approach_m', 1.5)
        self.declare_parameter('away_confirmation_samples', 3)
        self.declare_parameter('trend_epsilon_m', 0.03)
        self.declare_parameter('start_waypoint_index', 0)
        self.declare_parameter(
            'progress_file',
            os.path.expanduser('~/.ros/uv_treatment/progress.json'))
        self.declare_parameter('recovery_mode', 'prompt')
        self.declare_parameter('gpio_pin', 23)
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('gps_timeout_sec', 1.0)
        self.declare_parameter('uv_on_speed_mps', 0.1)
        self.declare_parameter('uv_off_speed_mps', 1.0)
        self.declare_parameter('failsafe_speed_mps', 0.1)
        self.declare_parameter('speed_topic', '/uv_treatment/target_speed')
        self.declare_parameter('require_speed_ack', True)
        self.declare_parameter(
            'speed_ack_topic', '/navigation/active_target_speed')
        self.declare_parameter('speed_ack_timeout_sec', 1.0)
        self.declare_parameter('speed_ack_tolerance_mps', 0.02)
        self.declare_parameter('navigation_active_required', True)
        self.declare_parameter(
            'navigation_state_topic', '/navigation/uv_treatment_enable')
        self.declare_parameter('navigation_state_timeout_sec', 0.5)
        self.declare_parameter('require_cmd_vel_match', True)
        self.declare_parameter('navigation_cmd_topic', '/cmd_vel_nav')
        self.declare_parameter('robot_cmd_topic', '/cmd_vel_out')
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('linear_cmd_tolerance_mps', 0.03)
        self.declare_parameter('angular_cmd_tolerance_rps', 0.05)
        self.declare_parameter('mock_gpio', False)

        waypoint_file = str(self.get_parameter('waypoint_file').value)
        self._trigger_distance_m = float(
            self.get_parameter('trigger_distance_m').value)
        hysteresis_distance_m = float(
            self.get_parameter('hysteresis_distance_m').value)
        missed_waypoint_enabled = bool(
            self.get_parameter('missed_waypoint_enabled').value)
        approach_confirmation_m = float(
            self.get_parameter('approach_confirmation_m').value)
        pass_confirmation_m = float(
            self.get_parameter('pass_confirmation_m').value)
        max_closest_approach_m = float(
            self.get_parameter('max_closest_approach_m').value)
        away_confirmation_samples = int(
            self.get_parameter('away_confirmation_samples').value)
        trend_epsilon_m = float(
            self.get_parameter('trend_epsilon_m').value)
        start_waypoint_index = int(
            self.get_parameter('start_waypoint_index').value)
        progress_file = str(self.get_parameter('progress_file').value)
        recovery_mode = str(
            self.get_parameter('recovery_mode').value).strip().lower()
        gpio_pin = int(self.get_parameter('gpio_pin').value)
        gps_topic = str(self.get_parameter('gps_topic').value)
        self._gps_timeout_sec = float(
            self.get_parameter('gps_timeout_sec').value)
        self._uv_on_speed_mps = float(
            self.get_parameter('uv_on_speed_mps').value)
        self._uv_off_speed_mps = float(
            self.get_parameter('uv_off_speed_mps').value)
        self._failsafe_speed_mps = float(
            self.get_parameter('failsafe_speed_mps').value)
        speed_topic = str(self.get_parameter('speed_topic').value)
        self._require_speed_ack = bool(
            self.get_parameter('require_speed_ack').value)
        speed_ack_topic = str(
            self.get_parameter('speed_ack_topic').value)
        self._speed_ack_timeout_sec = float(
            self.get_parameter('speed_ack_timeout_sec').value)
        self._speed_ack_tolerance_mps = float(
            self.get_parameter('speed_ack_tolerance_mps').value)
        self._navigation_active_required = bool(
            self.get_parameter('navigation_active_required').value)
        navigation_state_topic = str(
            self.get_parameter('navigation_state_topic').value)
        self._navigation_state_timeout_sec = float(
            self.get_parameter('navigation_state_timeout_sec').value)
        self._require_cmd_vel_match = bool(
            self.get_parameter('require_cmd_vel_match').value)
        navigation_cmd_topic = str(
            self.get_parameter('navigation_cmd_topic').value)
        robot_cmd_topic = str(self.get_parameter('robot_cmd_topic').value)
        self._cmd_vel_timeout_sec = float(
            self.get_parameter('cmd_vel_timeout_sec').value)
        self._linear_cmd_tolerance_mps = float(
            self.get_parameter('linear_cmd_tolerance_mps').value)
        self._angular_cmd_tolerance_rps = float(
            self.get_parameter('angular_cmd_tolerance_rps').value)
        mock_gpio = bool(self.get_parameter('mock_gpio').value)

        if not waypoint_file:
            raise ValueError(
                'waypoint_file is required; use --ros-args '
                '-p waypoint_file:=/absolute/path/to/uv_treatment_waypoints.csv')
        if self._gps_timeout_sec <= 0.0:
            raise ValueError('gps_timeout_sec must be greater than zero')
        if min(self._uv_on_speed_mps, self._uv_off_speed_mps,
               self._failsafe_speed_mps) <= 0.0:
            raise ValueError('all configured speeds must be greater than zero')
        if self._speed_ack_timeout_sec <= 0.0:
            raise ValueError('speed_ack_timeout_sec must be greater than zero')
        if self._speed_ack_tolerance_mps < 0.0:
            raise ValueError('speed_ack_tolerance_mps cannot be negative')
        if self._navigation_state_timeout_sec <= 0.0:
            raise ValueError(
                'navigation_state_timeout_sec must be greater than zero')
        if self._cmd_vel_timeout_sec <= 0.0:
            raise ValueError('cmd_vel_timeout_sec must be greater than zero')
        if min(self._linear_cmd_tolerance_mps,
               self._angular_cmd_tolerance_rps) < 0.0:
            raise ValueError('command tolerances cannot be negative')
        if recovery_mode not in ('prompt', 'resume', 'restart'):
            raise ValueError(
                'recovery_mode must be prompt, resume, or restart')

        self._waypoints = load_waypoints(waypoint_file)
        self._waypoint_file = waypoint_file
        self._progress_file = progress_file
        self._waypoint_fingerprint = waypoint_file_fingerprint(waypoint_file)
        start_waypoint_index = self._choose_start_index(
            start_waypoint_index, recovery_mode)
        self._sequence = UVTreatmentSequence(
            self._waypoints,
            trigger_distance_m=self._trigger_distance_m,
            hysteresis_distance_m=hysteresis_distance_m,
            start_index=start_waypoint_index,
            missed_waypoint_enabled=missed_waypoint_enabled,
            approach_confirmation_m=approach_confirmation_m,
            pass_confirmation_m=pass_confirmation_m,
            max_closest_approach_m=max_closest_approach_m,
            away_confirmation_samples=away_confirmation_samples,
            trend_epsilon_m=trend_epsilon_m,
        )
        self._uv_power = None
        self._uv_lamps_on = False
        self._last_gps_monotonic = None
        self._gps_timed_out = False
        self._closed = False
        self._last_trigger_reason = None
        self._last_trigger_distance_m = None
        self._last_closest_distance_m = None

        self._save_progress()

        if mock_gpio:
            self._uv_power = MockOutputDevice()
            self.get_logger().warning('Mock GPIO enabled; no physical pin is used')
        else:
            try:
                from gpiozero import OutputDevice
                self._uv_power = OutputDevice(
                    gpio_pin, active_high=True, initial_value=False)
            except Exception as exc:
                raise RuntimeError(
                    f'Could not initialize BCM GPIO {gpio_pin}: {exc}. '
                    'Check /dev/gpiochip permissions and python3-lgpio.') from exc

        speed_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._speed_pub = self.create_publisher(Float64, speed_topic, speed_qos)
        self._last_published_speed = None
        self._last_speed_ack = None
        self._last_speed_ack_monotonic = None
        self._last_speed_ack_warning = None
        self._navigation_active = False
        self._last_navigation_state_monotonic = None
        self._navigation_cmd = None
        self._last_navigation_cmd_monotonic = None
        self._robot_cmd = None
        self._last_robot_cmd_monotonic = None
        self._last_navigation_gate_warning = None
        self._last_navigation_gate_reason = None
        self.create_subscription(
            Float64, speed_ack_topic, self._speed_ack_callback, speed_qos)

        self.create_subscription(
            Bool, navigation_state_topic,
            self._navigation_state_callback, 10)
        self.create_subscription(
            Twist, navigation_cmd_topic,
            self._navigation_cmd_callback, 10)
        self.create_subscription(
            Twist, robot_cmd_topic, self._robot_cmd_callback, 10)

        self.create_subscription(NavSatFix, gps_topic, self._gps_callback, 10)
        self._watchdog_timer = self.create_timer(0.1, self._gps_watchdog)

        self.get_logger().info(
            f'Loaded {len(self._waypoints)} UV treatment waypoint(s) from '
            f'{waypoint_file}; trigger distance={self._trigger_distance_m:.2f} m; '
            f'hysteresis distance={hysteresis_distance_m:.2f} m; '
            f'start index={start_waypoint_index}; '
            f'GPS topic={gps_topic}; BCM GPIO={gpio_pin}')
        self.get_logger().info(
            'Prescription order is ON, OFF, ON, OFF; UV lamps turn OFF on GPS timeout')
        self.get_logger().info(f'Progress file: {self._progress_file}')
        self.get_logger().info(
            f'Variable speed: UV ON={self._uv_on_speed_mps:.2f} m/s, '
            f'UV OFF={self._uv_off_speed_mps:.2f} m/s, topic={speed_topic}')
        if self._require_speed_ack:
            self.get_logger().info(
                f'UV ON requires speed acknowledgement on {speed_ack_topic}')
        if self._navigation_active_required:
            detail = (
                f' and command ownership {navigation_cmd_topic} -> '
                f'{robot_cmd_topic}'
                if self._require_cmd_vel_match else '')
            self.get_logger().info(
                'UV waypoint processing requires active navigation heartbeat '
                f'on {navigation_state_topic}{detail}')
        self._publish_speed(
            self._failsafe_speed_mps, 'startup fail-safe', force=True)

    def _choose_start_index(self, configured_index, recovery_mode):
        try:
            progress = load_progress(self._progress_file)
        except Exception as exc:
            self.get_logger().warning(
                f'Cannot read progress file; starting a new task: {exc}')
            return configured_index

        if progress is not None and progress.get('completed') is True:
            self.get_logger().info(
                'Previous UV treatment was complete; starting a new mission')
            return configured_index

        saved_index = resumable_index(
            progress,
            self._waypoint_file,
            self._waypoint_fingerprint,
            len(self._waypoints))
        if saved_index is None:
            if progress is not None:
                self.get_logger().warning(
                    'Previous progress does not match this waypoint file; '
                    'starting a new task')
            return configured_index

        if recovery_mode == 'restart':
            self.get_logger().info('Recovery mode is restart; starting new task')
            return configured_index
        if recovery_mode == 'resume':
            self.get_logger().info(
                f'Resuming unfinished task at waypoint index {saved_index}')
            return saved_index

        if not sys.stdin.isatty():
            self.get_logger().warning(
                'Unfinished task found but stdin is not interactive; '
                f'automatically resuming at waypoint index {saved_index}')
            return saved_index

        answer = input(
            f'Unfinished UV treatment found at waypoint index {saved_index}. '
            'Resume? [Y/n]: ').strip().lower()
        if answer in ('', 'y', 'yes'):
            self.get_logger().info(
                f'Resuming at waypoint index {saved_index}')
            return saved_index
        self.get_logger().info(
            'Starting a new UV treatment mission from configured index')
        return configured_index

    def _save_progress(self):
        save_progress(
            self._progress_file,
            self._waypoint_file,
            self._waypoint_fingerprint,
            len(self._waypoints),
            self._sequence.current_index,
            self._sequence.uv_lamps_should_be_on,
            self._sequence.complete,
            self._last_trigger_reason,
            self._last_trigger_distance_m,
            self._last_closest_distance_m,
        )

    def _gps_callback(self, msg):
        if not self._valid_fix(msg):
            self._force_safe_output('invalid GPS fix')
            return

        self._last_gps_monotonic = time.monotonic()
        if self._gps_timed_out:
            self.get_logger().info('Valid GPS updates resumed')
            self._gps_timed_out = False

        gate_reason = self._navigation_gate_reason()
        if gate_reason is not None:
            self._force_safe_output(gate_reason)
            self._warn_navigation_gate(gate_reason)
            return

        event = self._sequence.update(msg.latitude, msg.longitude)
        if event is not None:
            waypoint_index, should_be_on, distance_m, reason, minimum_distance_m = event
            self._last_trigger_reason = reason
            self._last_trigger_distance_m = distance_m
            self._last_closest_distance_m = minimum_distance_m
            action = 'ON' if should_be_on else 'OFF'
            self.get_logger().info(
                f'Reached waypoint #{waypoint_index + 1}: requested {action}; '
                f'distance={distance_m:.3f} m; trigger={reason}; '
                f'closest={minimum_distance_m:.3f} m')
            try:
                self._save_progress()
            except Exception as exc:
                self.get_logger().error(
                    f'Failed to save UV treatment progress: {exc}')
            if self._sequence.complete:
                self.get_logger().info(
                    'UV treatment sequence complete; lamps are OFF')

        self._set_uv_lamps(
            self._sequence.uv_lamps_should_be_on,
            'current UV treatment sequence state')

    @staticmethod
    def _valid_fix(msg):
        return (
            msg.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and -90.0 <= msg.latitude <= 90.0
            and -180.0 <= msg.longitude <= 180.0
        )

    def _gps_watchdog(self):
        gate_reason = self._navigation_gate_reason()
        if gate_reason is not None:
            self._force_safe_output(gate_reason)
            self._warn_navigation_gate(gate_reason)

        now = time.monotonic()
        timed_out = (
            self._last_gps_monotonic is None
            or now - self._last_gps_monotonic > self._gps_timeout_sec)
        if not timed_out:
            return
        self._force_safe_output('GPS data timeout')
        if not self._gps_timed_out:
            self.get_logger().warning(
                f'No valid GPS update for {self._gps_timeout_sec:.2f} s; '
                'UV lamps forced OFF')
            self._gps_timed_out = True

    def _force_safe_output(self, reason):
        self._publish_speed(self._failsafe_speed_mps, reason)
        if self._uv_lamps_on:
            self._uv_power.off()
            self._uv_lamps_on = False
            self.get_logger().warning(f'UV lamps forced OFF: {reason}')

    def _set_uv_lamps(self, turn_on, reason):
        if turn_on:
            # Apply the UV speed limit before energizing the lamps.
            gate_reason = self._navigation_gate_reason()
            if gate_reason is not None:
                self._force_safe_output(gate_reason)
                self._warn_navigation_gate(gate_reason)
                return
            self._publish_speed(self._uv_on_speed_mps, reason)
            if self._require_speed_ack and not self._speed_ack_ready():
                if self._uv_lamps_on:
                    self._uv_power.off()
                    self._uv_lamps_on = False
                now = time.monotonic()
                if (
                    self._last_speed_ack_warning is None
                    or now - self._last_speed_ack_warning >= 2.0
                ):
                    self.get_logger().warning(
                        'Waiting for navigation to acknowledge UV ON speed; '
                        'UV lamps remain OFF')
                    self._last_speed_ack_warning = now
                return
            if not self._uv_lamps_on:
                self._uv_power.on()
                self._uv_lamps_on = True
                self.get_logger().info(f'UV lamps ON: {reason}')
            return

        if self._uv_lamps_on:
            self._uv_power.off()
            self._uv_lamps_on = False
            self.get_logger().info(f'UV lamps OFF: {reason}')
        self._publish_speed(self._uv_off_speed_mps, reason)

    def _speed_ack_callback(self, msg):
        if not math.isfinite(msg.data):
            return
        self._last_speed_ack = float(msg.data)
        self._last_speed_ack_monotonic = time.monotonic()

    def _speed_ack_ready(self):
        if self._last_speed_ack_monotonic is None:
            return False
        if (
            time.monotonic() - self._last_speed_ack_monotonic
            > self._speed_ack_timeout_sec
        ):
            return False
        return math.isclose(
            self._last_speed_ack, self._uv_on_speed_mps,
            abs_tol=self._speed_ack_tolerance_mps)

    def _navigation_state_callback(self, msg):
        self._navigation_active = bool(msg.data)
        self._last_navigation_state_monotonic = time.monotonic()
        if not self._navigation_active:
            self._force_safe_output('navigation inactive')

    def _navigation_cmd_callback(self, msg):
        self._navigation_cmd = (float(msg.linear.x), float(msg.angular.z))
        self._last_navigation_cmd_monotonic = time.monotonic()

    def _robot_cmd_callback(self, msg):
        self._robot_cmd = (float(msg.linear.x), float(msg.angular.z))
        self._last_robot_cmd_monotonic = time.monotonic()

    def _navigation_gate_reason(self):
        if not self._navigation_active_required:
            return None
        now = time.monotonic()

        def age(last_time):
            return None if last_time is None else now - last_time

        return navigation_gate_reason(
            self._navigation_active,
            age(self._last_navigation_state_monotonic),
            self._navigation_state_timeout_sec,
            self._require_cmd_vel_match,
            self._navigation_cmd,
            age(self._last_navigation_cmd_monotonic),
            self._robot_cmd,
            age(self._last_robot_cmd_monotonic),
            self._cmd_vel_timeout_sec,
            self._linear_cmd_tolerance_mps,
            self._angular_cmd_tolerance_rps,
        )

    def _warn_navigation_gate(self, reason):
        now = time.monotonic()
        if (
            reason != self._last_navigation_gate_reason
            or self._last_navigation_gate_warning is None
            or now - self._last_navigation_gate_warning >= 2.0
        ):
            self.get_logger().warning(
                f'Navigation safety gate closed ({reason}); UV lamps forced OFF '
                'and treatment progress frozen')
            self._last_navigation_gate_reason = reason
            self._last_navigation_gate_warning = now

    def _publish_speed(self, speed_mps, reason, force=False):
        if (
            not force and self._last_published_speed is not None
            and math.isclose(speed_mps, self._last_published_speed, abs_tol=1e-6)
        ):
            return
        self._speed_pub.publish(Float64(data=float(speed_mps)))
        self._last_published_speed = float(speed_mps)
        self.get_logger().info(
            f'Target speed -> {speed_mps:.3f} m/s: {reason}')

    def close(self):
        """Fail safe: turn UV lamps off and release GPIO exactly once."""
        if self._closed:
            return
        self._closed = True
        if hasattr(self, '_speed_pub'):
            self._publish_speed(
                self._failsafe_speed_mps, 'node shutdown', force=True)
        if self._uv_power is not None:
            self._uv_power.off()
            self._uv_lamps_on = False
            self._uv_power.close()
        self.get_logger().info('UV lamps OFF; GPIO released')

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UVTreatmentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f'uv_treatment_node: {exc}', file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
