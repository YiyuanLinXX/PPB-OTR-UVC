#!/usr/bin/env python3
"""Waypoint follower node with parameterized controller tuning and debug output."""

from argparse import ArgumentParser
import csv
import json
import math
from pathlib import Path
import shutil
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from pyproj import Transformer

from amiga_navigation.utils.pid_line_controller import (
    LineTrackingConfig,
)
from amiga_navigation.utils.tracking_geometry import (
    TrackingCommand,
    compute_segment_metrics,
    is_goal_reached,
)
from amiga_navigation.utils.tracking_controller_factory import (
    FormalMPCConfig,
    MPCRolloutConfig,
    PurePursuitConfig,
    RowHybridConfig,
    build_tracking_controller,
)
from amiga_navigation.utils.alignment_turn_controller import (
    TurnConfig,
    TurnCommand,
    compute_turn_command,
)


DEFAULT_WAYPOINTS_PATH = 'waypoints.csv'

CONTROLLER_CHOICES = [
    'pid_line',
    'pure_pursuit',
    'mpc_rollout',
    'mpc_formal',
    'row_hybrid',
]

DEFAULT_PARAMS_FILE_CANDIDATES = [
    Path(__file__).resolve().parents[1] / 'config' / 'waypoint_follower_params.yaml',
    Path(__file__).resolve().parents[4] / 'share' / 'amiga_navigation' / 'config' / 'waypoint_follower_params.yaml',
]


class WaypointFollower(Node):
    def __init__(self, csv_path, resume_mode='ask', controller_override=None):
        super().__init__('waypoint_follower')

        self._declare_parameters()

        self.control_frequency = self._get_float('control_frequency')
        self.max_odom_age_sec = self._get_float('max_odom_age_sec')
        self.enable_csv_logging = self._get_bool('enable_csv_logging')
        self.publish_debug = self._get_bool('publish_debug')
        self.debug_publish_period_sec = self._get_float('debug_publish_period_sec')
        self.terminal_status_enabled = self._get_bool('terminal_status_enabled')
        self.terminal_status_level = self._get_str('terminal_status_level').lower()
        self.terminal_status_period_sec = self._get_float(
            'terminal_status_period_sec'
        )
        self.terminal_show_alignment_details = self._get_bool(
            'terminal_show_alignment_details'
        )
        self.terminal_show_tracking_details = self._get_bool(
            'terminal_show_tracking_details'
        )

        # Dynamic speed configuration.
        self.dynamic_speed_enabled = self._get_bool('dynamic_speed_enabled')
        self.dynamic_speed_topic = self._get_str('dynamic_speed_topic')
        self.dynamic_speed_ack_topic = self._get_str(
            'dynamic_speed_ack_topic'
        )
        self.dynamic_speed_min_mps = self._get_float(
            'dynamic_speed_min_mps'
        )
        self.dynamic_speed_max_mps = self._get_float(
            'dynamic_speed_max_mps'
        )
        self.dynamic_speed_enforce_output_limit = self._get_bool(
            'dynamic_speed_enforce_output_limit'
        )

        # IMPORTANT:
        # Controller configs are frozen dataclasses. Therefore the runtime
        # target speed is stored separately and configs are recreated when
        # the speed changes.
        self.active_target_speed = self._get_float('target_speed')

        self.uv_treatment_navigation_state_topic = self._get_str(
            'uv_treatment_navigation_state_topic'
        )
        self.uv_treatment_navigation_state_period_sec = self._get_float(
            'uv_treatment_navigation_state_period_sec'
        )

        self.log_directory = Path(self._get_str('log_directory')).expanduser()
        self.log_directory.mkdir(parents=True, exist_ok=True)

        self.status_path = Path(self._get_str('status_path')).expanduser()
        self.last_wp_path = Path(self._get_str('last_waypoints_path')).expanduser()

        self.requested_csv_path = Path(csv_path)
        self.resume_mode = resume_mode

        self.csv_path, self.current_index = self._select_navigation_file()

        self.selected_controller_type = (
            controller_override or self._get_str('controller_type')
        )

        # Build all controller configurations using the current runtime speed.
        self._build_controller_configs()

        self.turn_config = TurnConfig(
            alignment_threshold=self._get_float('alignment_threshold'),
            gain=self._get_float('turn_gain'),
            min_turn_speed=self._get_float('turn_min_speed'),
            max_turn_speed=self._get_float('turn_max_speed'),
            enable_slowdown_near_target=self._get_bool(
                'turn_enable_slowdown_near_target'
            ),
            slowdown_angle=self._get_float('turn_slowdown_angle'),
            near_target_min_speed_ratio=self._get_float(
                'turn_near_target_min_speed_ratio'
            ),
        )

        self.controller = self._build_controller()
        self._log_controller_configuration()

        # ROS publishers.
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_nav',
            10,
        )

        self.debug_pub = self.create_publisher(
            String,
            '/nav/controller_debug',
            10,
        )

        speed_ack_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.speed_ack_pub = self.create_publisher(
            Float64,
            self.dynamic_speed_ack_topic,
            speed_ack_qos,
        )

        self.uv_treatment_navigation_state_pub = self.create_publisher(
            Bool,
            self.uv_treatment_navigation_state_topic,
            10,
        )

        # ROS subscriptions.
        self.create_subscription(
            Odometry,
            '/robot/odom',
            self.odom_callback,
            qos_profile_sensor_data,
        )

        datum_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Float64MultiArray,
            '/gps/datum',
            self.datum_callback,
            datum_qos,
        )

        speed_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Float64,
            self.dynamic_speed_topic,
            self.dynamic_speed_callback,
            speed_qos,
        )

        # Timers.
        self.create_timer(
            1.0 / max(self.control_frequency, 1.0),
            self.control_loop,
        )

        self.create_timer(
            1.0,
            self.flush_log_file,
        )

        self.create_timer(
            0.2,
            self.publish_speed_ack,
        )

        self.create_timer(
            self.uv_treatment_navigation_state_period_sec,
            self.publish_uv_treatment_navigation_state,
        )

        # Runtime state.
        self.pose = None
        self.last_odom_time = None
        self.last_odom_warn_time = None
        self.last_debug_publish_time = None
        self.last_terminal_status_time = None

        self.transformer = None
        self.reached_final = False
        self.phase = 'waiting_for_pose'
        self.last_phase = None
        self.init_pose_inserted = False
        self.segment_aligned = False
        self.current_route = None
        self.last_segment_key = None
        self.active_controller_name = self.selected_controller_type

        self.waypoints_gps = self.load_csv_waypoints(self.csv_path)
        self.waypoints_enu = []

        self._prepare_waypoint_state()
        self._setup_log_file()
        self._log_startup_summary()

        self.get_logger().info('WaypointFollower initialized')

    # -------------------------------------------------------------------------
    # Controller configuration
    # -------------------------------------------------------------------------

    def _build_controller_configs(self):
        """
        Build all frozen controller configuration objects.

        The controller configuration dataclasses are immutable/frozen.
        Dynamic target speed is therefore handled by recreating these
        configuration objects rather than modifying them in place.
        """

        speed = self.active_target_speed

        self.tracking_config = LineTrackingConfig(
            target_speed=speed,
            max_lateral_speed=self._get_float('max_lateral_speed'),
            epsilon=self._get_float('epsilon'),
            pid_kp=self._get_float('pid_kp'),
            pid_ki=self._get_float('pid_ki'),
            pid_kd=self._get_float('pid_kd'),
            heading_gain=self._get_float('heading_gain'),
            max_angular_speed=self._get_float('max_angular_speed'),
            min_forward_ratio=self._get_float('min_forward_ratio'),
            max_heading_for_full_speed=self._get_float(
                'max_heading_for_full_speed'
            ),
            max_cross_track_error=self._get_float(
                'max_cross_track_error'
            ),
            goal_threshold=self._get_float('goal_threshold'),
            alignment_threshold=self._get_float(
                'alignment_threshold'
            ),
            dist_start_threshold=self._get_float(
                'dist_start_threshold'
            ),
            dist_stop_threshold=self._get_float(
                'dist_stop_threshold'
            ),
            initial_speed_ratio=self._get_float(
                'initial_speed_ratio'
            ),
            stop_speed_ratio=self._get_float(
                'stop_speed_ratio'
            ),
            enable_start_slowdown=self._get_bool(
                'enable_start_slowdown'
            ),
            enable_goal_slowdown=self._get_bool(
                'enable_goal_slowdown'
            ),
            regulate_target_speed=self._get_bool(
                'regulate_target_speed'
            ),
        )

        self.pure_pursuit_config = PurePursuitConfig(
            target_speed=speed,
            min_lookahead=self._get_float(
                'pure_pursuit_min_lookahead'
            ),
            max_lookahead=self._get_float(
                'pure_pursuit_max_lookahead'
            ),
            lookahead_gain=self._get_float(
                'pure_pursuit_lookahead_gain'
            ),
            slowdown_distance=self._get_float(
                'pure_pursuit_slowdown_distance'
            ),
            max_angular_speed=self._get_float(
                'max_angular_speed'
            ),
            min_forward_ratio=self._get_float(
                'min_forward_ratio'
            ),
            start_slowdown_distance=self._get_float(
                'dist_start_threshold'
            ),
            initial_speed_ratio=self._get_float(
                'initial_speed_ratio'
            ),
            enable_start_slowdown=self._get_bool(
                'enable_start_slowdown'
            ),
            enable_goal_slowdown=self._get_bool(
                'enable_goal_slowdown'
            ),
            max_cross_track_error=self._get_float(
                'max_cross_track_error'
            ),
            goal_threshold=self._get_float(
                'goal_threshold'
            ),
        )

        self.mpc_rollout_config = MPCRolloutConfig(
            target_speed=speed,
            horizon_steps=self._get_int(
                'mpc_horizon_steps'
            ),
            step_time=self._get_float(
                'mpc_step_time'
            ),
            candidate_count=self._get_int(
                'mpc_candidate_count'
            ),
            max_angular_speed=self._get_float(
                'max_angular_speed'
            ),
            min_forward_ratio=self._get_float(
                'min_forward_ratio'
            ),
            slowdown_distance=self._get_float(
                'mpc_slowdown_distance'
            ),
            heading_weight=self._get_float(
                'mpc_heading_weight'
            ),
            cross_track_weight=self._get_float(
                'mpc_cross_track_weight'
            ),
            goal_distance_weight=self._get_float(
                'mpc_goal_distance_weight'
            ),
            effort_weight=self._get_float(
                'mpc_effort_weight'
            ),
            progress_weight=self._get_float(
                'mpc_progress_weight'
            ),
            start_slowdown_distance=self._get_float(
                'dist_start_threshold'
            ),
            initial_speed_ratio=self._get_float(
                'initial_speed_ratio'
            ),
            enable_start_slowdown=self._get_bool(
                'enable_start_slowdown'
            ),
            enable_goal_slowdown=self._get_bool(
                'enable_goal_slowdown'
            ),
            max_cross_track_error=self._get_float(
                'max_cross_track_error'
            ),
            goal_threshold=self._get_float(
                'goal_threshold'
            ),
        )

        self.formal_mpc_config = FormalMPCConfig(
            target_speed=speed,
            horizon_steps=self._get_int(
                'formal_mpc_horizon_steps'
            ),
            step_time=self._get_float(
                'formal_mpc_step_time'
            ),
            min_forward_speed=self._get_float(
                'formal_mpc_min_forward_speed'
            ),
            max_angular_speed=self._get_float(
                'max_angular_speed'
            ),
            slowdown_distance=self._get_float(
                'formal_mpc_slowdown_distance'
            ),
            min_forward_ratio=self._get_float(
                'min_forward_ratio'
            ),
            cross_track_weight=self._get_float(
                'formal_mpc_cross_track_weight'
            ),
            heading_weight=self._get_float(
                'formal_mpc_heading_weight'
            ),
            goal_distance_weight=self._get_float(
                'formal_mpc_goal_distance_weight'
            ),
            terminal_cross_track_weight=self._get_float(
                'formal_mpc_terminal_cross_track_weight'
            ),
            terminal_heading_weight=self._get_float(
                'formal_mpc_terminal_heading_weight'
            ),
            terminal_goal_distance_weight=self._get_float(
                'formal_mpc_terminal_goal_distance_weight'
            ),
            linear_effort_weight=self._get_float(
                'formal_mpc_linear_effort_weight'
            ),
            angular_effort_weight=self._get_float(
                'formal_mpc_angular_effort_weight'
            ),
            linear_smooth_weight=self._get_float(
                'formal_mpc_linear_smooth_weight'
            ),
            angular_smooth_weight=self._get_float(
                'formal_mpc_angular_smooth_weight'
            ),
            progress_weight=self._get_float(
                'formal_mpc_progress_weight'
            ),
            solver_maxiter=self._get_int(
                'formal_mpc_solver_maxiter'
            ),
            solver_ftol=self._get_float(
                'formal_mpc_solver_ftol'
            ),
            start_slowdown_distance=self._get_float(
                'dist_start_threshold'
            ),
            initial_speed_ratio=self._get_float(
                'initial_speed_ratio'
            ),
            enable_start_slowdown=self._get_bool(
                'enable_start_slowdown'
            ),
            enable_goal_slowdown=self._get_bool(
                'enable_goal_slowdown'
            ),
            max_cross_track_error=self._get_float(
                'max_cross_track_error'
            ),
            goal_threshold=self._get_float(
                'goal_threshold'
            ),
        )

        self.row_hybrid_config = RowHybridConfig(
            connector_length_threshold=self._get_float(
                'row_connector_length_threshold'
            ),
            row_length_threshold=self._get_float(
                'row_length_threshold'
            ),
        )

    def _build_controller(self):
        return build_tracking_controller(
            controller_type=self.selected_controller_type,
            line_config=self.tracking_config,
            pure_pursuit_config=self.pure_pursuit_config,
            mpc_rollout_config=self.mpc_rollout_config,
            formal_mpc_config=self.formal_mpc_config,
            row_hybrid_config=self.row_hybrid_config,
        )

    # -------------------------------------------------------------------------
    # Parameters
    # -------------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('max_odom_age_sec', 0.25)
        self.declare_parameter('enable_csv_logging', True)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_publish_period_sec', 0.2)

        self.declare_parameter('terminal_status_enabled', True)
        self.declare_parameter('terminal_status_level', 'normal')
        self.declare_parameter('terminal_status_period_sec', 1.5)
        self.declare_parameter('terminal_show_alignment_details', False)
        self.declare_parameter('terminal_show_tracking_details', False)

        self.declare_parameter(
            'log_directory',
            '~/.ros/waypoint_follower',
        )

        self.declare_parameter(
            'status_path',
            '~/.ros/waypoint_follower/status.txt',
        )

        self.declare_parameter(
            'last_waypoints_path',
            '~/.ros/waypoint_follower/last_waypoints.csv',
        )

        self.declare_parameter('controller_type', 'pid_line')

        self.declare_parameter('dynamic_speed_enabled', True)
        self.declare_parameter(
            'dynamic_speed_topic',
            '/uv_treatment/target_speed',
        )
        self.declare_parameter(
            'dynamic_speed_ack_topic',
            '/navigation/active_target_speed',
        )
        self.declare_parameter('dynamic_speed_min_mps', 0.05)
        self.declare_parameter('dynamic_speed_max_mps', 1.2)
        self.declare_parameter(
            'dynamic_speed_enforce_output_limit',
            True,
        )

        self.declare_parameter(
            'uv_treatment_navigation_state_topic',
            '/navigation/uv_treatment_enable',
        )

        self.declare_parameter(
            'uv_treatment_navigation_state_period_sec',
            0.1,
        )

        self.declare_parameter('target_speed', 0.85)
        self.declare_parameter('max_lateral_speed', 0.4)
        self.declare_parameter('epsilon', 0.5)

        self.declare_parameter('pid_kp', 0.38)
        self.declare_parameter('pid_ki', 0.05)
        self.declare_parameter('pid_kd', 0.28)

        self.declare_parameter('heading_gain', 1.0)
        self.declare_parameter('max_angular_speed', 0.65)
        self.declare_parameter('min_forward_ratio', 0.2)
        self.declare_parameter('max_heading_for_full_speed', 0.26)
        self.declare_parameter('max_cross_track_error', 1.0)

        self.declare_parameter('goal_threshold', 0.30)
        self.declare_parameter('alignment_threshold', 0.18)

        self.declare_parameter('dist_start_threshold', 4.5)
        self.declare_parameter('dist_stop_threshold', 1.0)

        self.declare_parameter('initial_speed_ratio', 0.22)
        self.declare_parameter('stop_speed_ratio', 0.0)

        self.declare_parameter('enable_start_slowdown', True)
        self.declare_parameter('enable_goal_slowdown', True)
        self.declare_parameter('regulate_target_speed', True)

        self.declare_parameter('turn_gain', 0.7)
        self.declare_parameter('turn_min_speed', 0.14)
        self.declare_parameter('turn_max_speed', 0.4)
        self.declare_parameter(
            'turn_enable_slowdown_near_target',
            False,
        )
        self.declare_parameter('turn_slowdown_angle', 0.35)
        self.declare_parameter(
            'turn_near_target_min_speed_ratio',
            0.35,
        )

        self.declare_parameter(
            'pure_pursuit_min_lookahead',
            1.5,
        )
        self.declare_parameter(
            'pure_pursuit_max_lookahead',
            6.0,
        )
        self.declare_parameter(
            'pure_pursuit_lookahead_gain',
            2.5,
        )
        self.declare_parameter(
            'pure_pursuit_slowdown_distance',
            3.0,
        )

        self.declare_parameter('mpc_horizon_steps', 10)
        self.declare_parameter('mpc_step_time', 0.2)
        self.declare_parameter('mpc_candidate_count', 15)
        self.declare_parameter('mpc_slowdown_distance', 3.0)
        self.declare_parameter('mpc_heading_weight', 1.5)
        self.declare_parameter('mpc_cross_track_weight', 2.0)
        self.declare_parameter('mpc_goal_distance_weight', 4.0)
        self.declare_parameter('mpc_effort_weight', 0.3)
        self.declare_parameter('mpc_progress_weight', 1.0)

        self.declare_parameter('formal_mpc_horizon_steps', 8)
        self.declare_parameter('formal_mpc_step_time', 0.25)
        self.declare_parameter(
            'formal_mpc_min_forward_speed',
            0.0,
        )
        self.declare_parameter(
            'formal_mpc_slowdown_distance',
            3.0,
        )
        self.declare_parameter(
            'formal_mpc_cross_track_weight',
            8.0,
        )
        self.declare_parameter(
            'formal_mpc_heading_weight',
            4.0,
        )
        self.declare_parameter(
            'formal_mpc_goal_distance_weight',
            2.0,
        )
        self.declare_parameter(
            'formal_mpc_terminal_cross_track_weight',
            12.0,
        )
        self.declare_parameter(
            'formal_mpc_terminal_heading_weight',
            8.0,
        )
        self.declare_parameter(
            'formal_mpc_terminal_goal_distance_weight',
            6.0,
        )
        self.declare_parameter(
            'formal_mpc_linear_effort_weight',
            0.8,
        )
        self.declare_parameter(
            'formal_mpc_angular_effort_weight',
            0.4,
        )
        self.declare_parameter(
            'formal_mpc_linear_smooth_weight',
            0.5,
        )
        self.declare_parameter(
            'formal_mpc_angular_smooth_weight',
            1.2,
        )
        self.declare_parameter(
            'formal_mpc_progress_weight',
            1.0,
        )
        self.declare_parameter(
            'formal_mpc_solver_maxiter',
            40,
        )
        self.declare_parameter(
            'formal_mpc_solver_ftol',
            0.001,
        )

        self.declare_parameter(
            'row_connector_length_threshold',
            3.0,
        )
        self.declare_parameter(
            'row_length_threshold',
            15.0,
        )

    def _get_float(self, name):
        return float(self.get_parameter(name).value)

    def _get_bool(self, name):
        return bool(self.get_parameter(name).value)

    def _get_int(self, name):
        return int(self.get_parameter(name).value)

    def _get_str(self, name):
        return str(self.get_parameter(name).value)

    # -------------------------------------------------------------------------
    # Terminal status
    # -------------------------------------------------------------------------

    def _terminal_level_value(self, level: str) -> int:
        levels = {
            'silent': 0,
            'normal': 1,
            'verbose': 2,
            'debug': 3,
        }

        return levels.get(level, levels['normal'])

    def _terminal_enabled_for(self, level: str) -> bool:
        if not self.terminal_status_enabled:
            return False

        return (
            self._terminal_level_value(self.terminal_status_level)
            >= self._terminal_level_value(level)
        )

    def _terminal_info(
        self,
        message: str,
        level: str = 'normal',
    ) -> None:
        if self._terminal_enabled_for(level):
            self.get_logger().info(message)

    def _set_phase(
        self,
        phase: str,
        controller_name: str | None = None,
        detail: str | None = None,
    ) -> None:
        if controller_name is not None:
            self.active_controller_name = controller_name

        if phase != self.phase:
            self.last_phase = self.phase
            self.phase = phase

            message = (
                f'Phase change: '
                f'{self.last_phase} -> {self.phase}'
            )

            if detail:
                message += f' | {detail}'

            self._terminal_info(
                message,
                level='normal',
            )
        else:
            self.phase = phase

    # -------------------------------------------------------------------------
    # Logging
    # -------------------------------------------------------------------------

    def _format_route_summary(self, route) -> str:
        start_x, start_y = route[0]
        goal_x, goal_y = route[1]

        metrics = compute_segment_metrics(
            route,
            [start_x, start_y, 0.0],
        )

        return (
            f'segment {self.current_index}->{self.current_index + 1} | '
            f'start=({start_x:.2f}, {start_y:.2f}) | '
            f'goal=({goal_x:.2f}, {goal_y:.2f}) | '
            f'length={metrics.segment_length:.2f} m | '
            f'heading={math.degrees(metrics.path_angle):.2f} deg'
        )

    def _maybe_log_segment_start(self) -> None:
        if self.current_route is None:
            return

        segment_key = (
            self.current_index,
            round(self.current_route[0][0], 3),
            round(self.current_route[0][1], 3),
            round(self.current_route[1][0], 3),
            round(self.current_route[1][1], 3),
        )

        if segment_key == self.last_segment_key:
            return

        self.last_segment_key = segment_key

        self._terminal_info(
            f'Entering {self._format_route_summary(self.current_route)}',
            level='normal',
        )

    def _maybe_log_runtime_status(
        self,
        tracking_command: TrackingCommand | None = None,
        turn_command: TurnCommand | None = None,
    ) -> None:
        now = self.get_clock().now()

        if self.last_terminal_status_time is not None:
            age = (
                now - self.last_terminal_status_time
            ).nanoseconds / 1e9

            if age < self.terminal_status_period_sec:
                return

        if (
            turn_command is not None
            and self.terminal_show_alignment_details
        ):
            self.last_terminal_status_time = now

            self._terminal_info(
                'Aligning | '
                f'wp={self.current_index}->{self.current_index + 1} | '
                f'heading_error='
                f'{math.degrees(turn_command.heading_error):.2f} deg | '
                f'cmd_w={turn_command.angular_velocity:.3f} rad/s | '
                f'threshold='
                f'{math.degrees(self.turn_config.alignment_threshold):.2f} deg',
                level='verbose',
            )

            return

        if (
            tracking_command is not None
            and self.terminal_show_tracking_details
        ):
            self.last_terminal_status_time = now

            self._terminal_info(
                'Tracking | '
                f'wp={self.current_index}->{self.current_index + 1} | '
                f'heading_error='
                f'{math.degrees(tracking_command.heading_error):.2f} deg | '
                f'cross_track='
                f'{tracking_command.cross_track_error:.3f} m | '
                f'dist_to_goal='
                f'{tracking_command.dist_to_goal:.2f} m | '
                f'cmd_v={tracking_command.linear_velocity:.3f} m/s | '
                f'cmd_w={tracking_command.angular_velocity:.3f} rad/s',
                level='verbose',
            )

    def _log_startup_summary(self) -> None:
        self._terminal_info(
            'Startup summary | '
            f'waypoints={self.csv_path} | '
            f'requested={self.requested_csv_path} | '
            f'resume={self.resume_mode} | '
            f'controller={self.selected_controller_type} | '
            f'gps_points={len(self.waypoints_gps)} | '
            f'target_speed={self.active_target_speed:.3f} m/s | '
            f'terminal_level={self.terminal_status_level}',
            level='normal',
        )

        self._terminal_info(
            'Startup details | '
            f'csv_logging={self.enable_csv_logging} | '
            f'debug_topic={self.publish_debug} | '
            f'log_dir={self.log_directory} | '
            f'max_odom_age={self.max_odom_age_sec:.2f}s',
            level='normal',
        )

    def _log_controller_configuration(self):
        self.get_logger().info(
            'Controller selection: '
            f'requested={self.selected_controller_type}, '
            f'target_speed={self.tracking_config.target_speed:.2f}, '
            f'goal_threshold={self.tracking_config.goal_threshold:.2f}, '
            f'alignment_threshold='
            f'{self.tracking_config.alignment_threshold:.2f}'
        )

        self.get_logger().info(
            'Controller families: '
            f'pid=('
            f'{self.tracking_config.pid_kp:.3f}, '
            f'{self.tracking_config.pid_ki:.3f}, '
            f'{self.tracking_config.pid_kd:.3f}), '
            f'pure_pursuit_lookahead=['
            f'{self.pure_pursuit_config.min_lookahead:.2f}, '
            f'{self.pure_pursuit_config.max_lookahead:.2f}], '
            f'mpc=('
            f'steps={self.mpc_rollout_config.horizon_steps}, '
            f'dt={self.mpc_rollout_config.step_time:.2f}, '
            f'candidates={self.mpc_rollout_config.candidate_count}), '
            f'formal_mpc=('
            f'steps={self.formal_mpc_config.horizon_steps}, '
            f'dt={self.formal_mpc_config.step_time:.2f}, '
            f'maxiter={self.formal_mpc_config.solver_maxiter}), '
            f'row_hybrid=('
            f'connector<='
            f'{self.row_hybrid_config.connector_length_threshold:.2f}, '
            f'row>='
            f'{self.row_hybrid_config.row_length_threshold:.2f})'
        )

    # -------------------------------------------------------------------------
    # Navigation resume
    # -------------------------------------------------------------------------

    def _select_navigation_file(self):
        resume_info = self._get_unfinished_navigation_info()

        if resume_info is None:
            return self.requested_csv_path, 0

        resume_index, remaining_points = resume_info

        if self.resume_mode == 'yes':
            self.get_logger().info(
                f'Automatically resuming unfinished navigation from '
                f'{self.last_wp_path} at waypoint index {resume_index}.'
            )

            return self.last_wp_path, resume_index

        if self.resume_mode == 'no':
            self.get_logger().info(
                'Ignoring unfinished navigation and starting from '
                'the requested waypoint file.'
            )

            return self.requested_csv_path, 0

        if sys.stdin.isatty():
            prompt = (
                f'Detected unfinished navigation in '
                f'{self.last_wp_path} '
                f'(resume from waypoint index {resume_index}, '
                f'{remaining_points} points remaining). '
                'Continue it? [y/N]: '
            )

            answer = input(prompt).strip().lower()

            if answer in ('y', 'yes'):
                self.get_logger().info(
                    f'Resuming unfinished navigation from '
                    f'{self.last_wp_path} at waypoint index '
                    f'{resume_index}.'
                )

                return self.last_wp_path, resume_index

        self.get_logger().info(
            'Starting from the requested waypoint file instead of '
            'the unfinished navigation snapshot.'
        )

        return self.requested_csv_path, 0

    def _get_unfinished_navigation_info(self):
        if (
            not self.last_wp_path.exists()
            or not self.status_path.exists()
        ):
            return None

        try:
            saved_index = int(
                self.status_path.read_text()
                .splitlines()[0]
                .strip()
            )

            last_waypoints = self.load_csv_waypoints(
                self.last_wp_path
            )

        except Exception as exc:
            self.get_logger().warn(
                f'Could not inspect unfinished navigation state: {exc}'
            )

            return None

        if len(last_waypoints) < 2:
            return None

        if 0 <= saved_index < len(last_waypoints) - 1:
            remaining_points = (
                len(last_waypoints) - saved_index - 1
            )

            return saved_index, remaining_points

        return None

    def _prepare_waypoint_state(self):
        if self.csv_path != self.last_wp_path:
            shutil.copy(
                self.csv_path,
                self.last_wp_path,
            )

        if self.current_index > 0:
            self.get_logger().info(
                f'Resuming from waypoint index {self.current_index}'
            )
        else:
            self.get_logger().info(
                f'Starting navigation using waypoint file: '
                f'{self.csv_path}'
            )

        self.update_status_file()

    # -------------------------------------------------------------------------
    # CSV logging
    # -------------------------------------------------------------------------

    def _setup_log_file(self):
        self.csv_log_file = None
        self.csv_log_writer = None

        if not self.enable_csv_logging:
            return

        timestamp = int(time.time())

        csv_log_path = (
            self.log_directory
            / f'waypoint_control_log_{timestamp}.csv'
        )

        self.csv_log_file = csv_log_path.open(
            'w',
            newline='',
        )

        self.csv_log_writer = csv.writer(
            self.csv_log_file
        )

        self.csv_log_writer.writerow([
            'timestamp',
            'phase',
            'controller',
            'waypoint_index',
            'segment_start_x',
            'segment_start_y',
            'x',
            'y',
            'yaw_deg',
            'path_heading_deg',
            'segment_length_m',
            'target_x',
            'target_y',
            'heading_error_deg',
            'cross_track_error_m',
            'dist_to_goal_m',
            'dist_from_start_m',
            'target_speed_mps',
            'lateral_speed_mps',
            'cmd_v',
            'cmd_w',
        ])

        self.get_logger().info(
            f'Control CSV logging enabled: {csv_log_path}'
        )

    def load_csv_waypoints(self, path: Path):
        waypoints = []

        with path.open(
            'r',
            newline='',
        ) as waypoint_file:
            reader = csv.reader(waypoint_file)

            next(reader)

            for row in reader:
                if len(row) >= 2:
                    waypoints.append(
                        (
                            float(row[0]),
                            float(row[1]),
                        )
                    )

        return waypoints

    # -------------------------------------------------------------------------
    # GPS / odometry
    # -------------------------------------------------------------------------

    def datum_callback(self, msg):
        if self.transformer is not None:
            return

        lat0, lon0, _ = msg.data

        self.transformer = Transformer.from_crs(
            'epsg:4326',
            (
                f'+proj=tmerc '
                f'+lat_0={lat0} '
                f'+lon_0={lon0} '
                f'+k=1 '
                f'+x_0=0 '
                f'+y_0=0 '
                f'+datum=WGS84'
            ),
            always_xy=True,
        )

        self.waypoints_enu = []

        for lat, lon in self.waypoints_gps:
            x, y = self.transformer.transform(
                lon,
                lat,
            )

            self.waypoints_enu.append(
                [x, y]
            )

        self.get_logger().info(
            f'Converted {len(self.waypoints_enu)} '
            f'waypoints to ENU coordinates.'
        )

        if (
            self.pose is not None
            and not self.init_pose_inserted
        ):
            self._insert_initial_pose()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        yaw = self.quaternion_to_yaw(
            ori.x,
            ori.y,
            ori.z,
            ori.w,
        )

        self.pose = [
            pos.x,
            pos.y,
            yaw,
        ]

        self.last_odom_time = self.get_clock().now()

        if (
            self.transformer is not None
            and not self.init_pose_inserted
        ):
            self._insert_initial_pose()

    def _insert_initial_pose(self):
        self.waypoints_enu.insert(
            0,
            [
                self.pose[0],
                self.pose[1],
            ],
        )

        self.init_pose_inserted = True

        self.get_logger().info(
            f'Inserted current position as waypoint 0: '
            f'{[self.pose[0], self.pose[1]]}'
        )

        self._terminal_info(
            'Navigation start pose inserted as waypoint 0 '
            f'at ({self.pose[0]:.2f}, {self.pose[1]:.2f}).',
            level='normal',
        )

    # -------------------------------------------------------------------------
    # Dynamic speed
    # -------------------------------------------------------------------------

    def dynamic_speed_callback(self, msg):
        """
        Update navigation target speed.

        IMPORTANT:
        LineTrackingConfig, PurePursuitConfig, MPCRolloutConfig and
        FormalMPCConfig are frozen dataclasses. They cannot be modified
        in place.

        Therefore we:
          1. Validate the requested speed.
          2. Store it in self.active_target_speed.
          3. Recreate all controller configs.
          4. Rebuild the active controller.
          5. Reset the controller.
          6. Publish the speed acknowledgement.
        """

        if not self.dynamic_speed_enabled:
            return

        speed_mps = float(msg.data)

        if not math.isfinite(speed_mps):
            self.get_logger().warning(
                'Ignoring non-finite dynamic speed'
            )
            return

        if not (
            self.dynamic_speed_min_mps
            <= speed_mps
            <= self.dynamic_speed_max_mps
        ):
            self.get_logger().warning(
                f'Ignoring dynamic speed {speed_mps:.3f} m/s '
                f'outside '
                f'[{self.dynamic_speed_min_mps:.3f}, '
                f'{self.dynamic_speed_max_mps:.3f}] m/s'
            )
            return

        # Already at requested speed.
        if math.isclose(
            self.active_target_speed,
            speed_mps,
            abs_tol=1e-6,
        ):
            self.speed_ack_pub.publish(
                Float64(data=speed_mps)
            )
            return

        old_speed = self.active_target_speed

        # Store runtime speed separately from the frozen configs.
        self.active_target_speed = speed_mps

        try:
            # Recreate frozen controller configuration objects.
            self._build_controller_configs()

            # Rebuild the active controller with the new target speed.
            self.controller = self._build_controller()

            # Reset controller state so it starts cleanly with the
            # new speed configuration.
            self.controller.reset()

        except Exception as exc:
            # If rebuilding fails, restore the previous speed and configs.
            self.get_logger().error(
                f'Failed to apply dynamic target speed '
                f'{speed_mps:.3f} m/s: {exc}'
            )

            self.active_target_speed = old_speed

            try:
                self._build_controller_configs()
                self.controller = self._build_controller()
                self.controller.reset()
            except Exception as restore_exc:
                self.get_logger().error(
                    f'Failed to restore previous controller '
                    f'configuration: {restore_exc}'
                )

            return

        # Acknowledge only after the new configuration is active.
        self.speed_ack_pub.publish(
            Float64(data=speed_mps)
        )

        self.get_logger().info(
            f'Navigation target speed updated '
            f'{old_speed:.3f} -> {speed_mps:.3f} m/s '
            f'from {self.dynamic_speed_topic}'
        )

    def publish_speed_ack(self):
        if not self.dynamic_speed_enabled:
            return

        self.speed_ack_pub.publish(
            Float64(
                data=float(self.active_target_speed)
            )
        )

    # -------------------------------------------------------------------------
    # UV treatment safety heartbeat
    # -------------------------------------------------------------------------

    def publish_uv_treatment_navigation_state(self):
        odom_fresh = False

        if self.last_odom_time is not None:
            odom_age_sec = (
                self.get_clock().now()
                - self.last_odom_time
            ).nanoseconds / 1e9

            odom_fresh = (
                odom_age_sec <= self.max_odom_age_sec
            )

        active = (
            not self.reached_final
            and self.pose is not None
            and self.transformer is not None
            and self.init_pose_inserted
            and odom_fresh
            and self.phase in (
                'aligning',
                'tracking',
            )
        )

        self.uv_treatment_navigation_state_pub.publish(
            Bool(data=active)
        )

    # -------------------------------------------------------------------------
    # Main control loop
    # -------------------------------------------------------------------------

    def control_loop(self):
        if self.reached_final:
            return

        if (
            self.pose is None
            or self.transformer is None
            or not self.init_pose_inserted
        ):
            self._set_phase(
                'waiting_for_pose',
                controller_name='waiting_for_pose',
            )

            return

        if not self._odom_is_fresh():
            self._set_phase(
                'stale_pose',
                controller_name='stale_pose',
                detail='Waiting for fresh /robot/odom.',
            )

            self._publish_stop()

            self._publish_debug({
                'reason': 'stale_pose',
                **self._build_debug_snapshot(),
            })

            return

        if self.current_index >= len(self.waypoints_enu) - 1:
            self._set_phase(
                'complete',
                controller_name='complete',
            )

            self.reached_final = True

            self._publish_stop()
            self.update_status_file()

            self.get_logger().info(
                'Navigation complete.'
            )

            self._terminal_info(
                'Navigation complete | '
                f'reached final waypoint index '
                f'{self.current_index}',
                level='normal',
            )

            self._publish_debug({
                'reason': 'complete',
                **self._build_debug_snapshot(),
            })

            return

        self.current_route = [
            self.waypoints_enu[self.current_index],
            self.waypoints_enu[
                self.current_index + 1
            ],
        ]

        self._maybe_log_segment_start()

        if not self.segment_aligned:
            turn_command = compute_turn_command(
                self.current_route,
                self.pose,
                self.turn_config,
            )

            if not turn_command.aligned:
                self._set_phase(
                    'aligning',
                    controller_name='alignment_turn',
                    detail=(
                        f'Preparing segment '
                        f'{self.current_index}->'
                        f'{self.current_index + 1}'
                    ),
                )

                self._publish_twist(
                    0.0,
                    turn_command.angular_velocity,
                )

                self._publish_debug(
                    self._build_debug_snapshot(
                        turn_command=turn_command
                    )
                )

                self._log_control_sample(
                    turn_command=turn_command
                )

                self._maybe_log_runtime_status(
                    turn_command=turn_command
                )

                return

            self.segment_aligned = True

            self.controller.reset()

            self._set_phase(
                'tracking',
                controller_name=self.selected_controller_type,
            )

            self.get_logger().info(
                f'Alignment complete for waypoint segment '
                f'{self.current_index} -> '
                f'{self.current_index + 1}'
            )

            self._terminal_info(
                'Alignment complete | '
                f'wp={self.current_index}->'
                f'{self.current_index + 1} | '
                f'heading_error='
                f'{math.degrees(turn_command.heading_error):.2f} deg',
                level='normal',
            )

        try:
            tracking_command = (
                self.controller.compute_command(
                    self.current_route,
                    self.pose,
                )
            )

            self.active_controller_name = getattr(
                self.controller,
                'active_controller_name',
                self.selected_controller_type,
            )

        except ValueError as exc:
            self._set_phase(
                'control_error',
                controller_name='control_error',
            )

            self._publish_stop()

            self.get_logger().error(
                str(exc)
            )

            self.get_logger().warn(
                'Control error triggered a stop | '
                f'wp={self.current_index}->'
                f'{self.current_index + 1} | '
                f'pose=('
                f'{self.pose[0]:.2f}, '
                f'{self.pose[1]:.2f}, '
                f'{math.degrees(self.pose[2]):.2f} deg)'
            )

            self._publish_debug({
                'reason': 'control_error',
                'message': str(exc),
                **self._build_debug_snapshot(
                    error_message=str(exc)
                ),
            })

            self._log_control_sample(
                error_message=str(exc)
            )

            return

        self._publish_twist(
            tracking_command.linear_velocity,
            tracking_command.angular_velocity,
        )

        self._publish_debug(
            self._build_debug_snapshot(
                tracking_command=tracking_command
            )
        )

        self._log_control_sample(
            tracking_command=tracking_command
        )

        self._maybe_log_runtime_status(
            tracking_command=tracking_command
        )

        if is_goal_reached(
            self.current_route,
            self.pose,
            self.tracking_config.goal_threshold,
        ):
            self.get_logger().info(
                f'Reached waypoint {self.current_index + 1}'
            )

            self._terminal_info(
                'Reached waypoint | '
                f'wp={self.current_index + 1} | '
                f'pose=('
                f'{self.pose[0]:.2f}, '
                f'{self.pose[1]:.2f}) | '
                f'dist_to_goal='
                f'{tracking_command.dist_to_goal:.2f} m',
                level='normal',
            )

            self.current_index += 1
            self.segment_aligned = False

            self._set_phase(
                'aligning',
                controller_name='alignment_turn',
                detail='Switching to next segment.',
            )

            self.controller.reset()
            self.update_status_file()

    # -------------------------------------------------------------------------
    # Odometry safety
    # -------------------------------------------------------------------------

    def _odom_is_fresh(self):
        if self.last_odom_time is None:
            return False

        odom_age = (
            self.get_clock().now()
            - self.last_odom_time
        ).nanoseconds / 1e9

        if odom_age <= self.max_odom_age_sec:
            return True

        if (
            self.last_odom_warn_time is None
            or (
                self.get_clock().now()
                - self.last_odom_warn_time
            ).nanoseconds > int(2e9)
        ):
            self.get_logger().warn(
                f'/robot/odom is stale '
                f'({odom_age:.2f}s); publishing stop '
                f'until pose updates resume.'
            )

            self._terminal_info(
                f'Safety stop | stale /robot/odom '
                f'for {odom_age:.2f}s '
                f'(limit {self.max_odom_age_sec:.2f}s)',
                level='normal',
            )

            self.last_odom_warn_time = (
                self.get_clock().now()
            )

        return False

    # -------------------------------------------------------------------------
    # Command output
    # -------------------------------------------------------------------------

    def _publish_twist(
        self,
        linear_x,
        angular_z,
    ):
        if (
            self.dynamic_speed_enabled
            and self.dynamic_speed_enforce_output_limit
        ):
            # Use runtime target speed rather than attempting to modify
            # the frozen config.
            speed_limit = self.active_target_speed

            linear_x = max(
                -speed_limit,
                min(speed_limit, linear_x),
            )

        twist = Twist()

        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)

        self.cmd_pub.publish(twist)

    def _publish_stop(self):
        self._publish_twist(
            0.0,
            0.0,
        )

    # -------------------------------------------------------------------------
    # Debug output
    # -------------------------------------------------------------------------

    def _publish_debug(self, extra_fields):
        if not self.publish_debug:
            return

        now = self.get_clock().now()

        if self.last_debug_publish_time is not None:
            age = (
                now - self.last_debug_publish_time
            ).nanoseconds / 1e9

            if age < self.debug_publish_period_sec:
                return

        self.last_debug_publish_time = now

        payload = {
            'phase': self.phase,
            'controller': self.active_controller_name,
            'waypoint_index': self.current_index,
            'pose': (
                None
                if self.pose is None
                else {
                    'x': round(self.pose[0], 3),
                    'y': round(self.pose[1], 3),
                    'yaw_deg': round(
                        math.degrees(self.pose[2]),
                        2,
                    ),
                }
            ),
            'route': (
                None
                if self.current_route is None
                else {
                    'start': [
                        round(value, 3)
                        for value in self.current_route[0]
                    ],
                    'goal': [
                        round(value, 3)
                        for value in self.current_route[1]
                    ],
                }
            ),
        }

        for key, value in extra_fields.items():
            if isinstance(value, float):
                payload[key] = round(value, 4)
            else:
                payload[key] = value

        self.debug_pub.publish(
            String(
                data=json.dumps(
                    payload,
                    sort_keys=True,
                )
            )
        )

    def _build_debug_snapshot(
        self,
        tracking_command: TrackingCommand | None = None,
        turn_command: TurnCommand | None = None,
        error_message: str | None = None,
    ) -> dict:
        payload = {
            'phase': self.phase,
            'controller': (
                self.active_controller_name
                if error_message is None
                else (
                    f'{self.active_controller_name}:'
                    f'{error_message}'
                )
            ),
            'waypoint_index': self.current_index,
            'active_target_speed_mps': round(
                self.active_target_speed,
                3,
            ),
        }

        if self.pose is not None:
            payload.update({
                'x': round(self.pose[0], 3),
                'y': round(self.pose[1], 3),
                'yaw_deg': round(
                    math.degrees(self.pose[2]),
                    2,
                ),
            })

        if self.current_route is not None:
            metrics = compute_segment_metrics(
                self.current_route,
                (
                    self.pose
                    if self.pose is not None
                    else [0.0, 0.0, 0.0]
                ),
            )

            start_x, start_y = self.current_route[0]
            target_x, target_y = self.current_route[1]

            payload.update({
                'segment_start_x': round(start_x, 3),
                'segment_start_y': round(start_y, 3),
                'path_heading_deg': round(
                    math.degrees(metrics.path_angle),
                    2,
                ),
                'segment_length_m': round(
                    metrics.segment_length,
                    3,
                ),
                'target_x': round(target_x, 3),
                'target_y': round(target_y, 3),
            })

        heading_error = None
        cross_track_error = None
        dist_to_goal = None
        dist_from_start = None

        target_speed = self.active_target_speed
        lateral_speed = 0.0
        cmd_v = 0.0
        cmd_w = 0.0

        if (
            self.current_route is not None
            and self.pose is not None
        ):
            metrics = compute_segment_metrics(
                self.current_route,
                self.pose,
            )

            heading_error = metrics.heading_error
            cross_track_error = metrics.cross_track_error
            dist_to_goal = metrics.dist_to_goal
            dist_from_start = metrics.dist_from_start

        if tracking_command is not None:
            heading_error = tracking_command.heading_error
            cross_track_error = tracking_command.cross_track_error
            dist_to_goal = tracking_command.dist_to_goal
            dist_from_start = tracking_command.dist_from_start
            target_speed = tracking_command.target_speed
            lateral_speed = tracking_command.lateral_speed
            cmd_v = tracking_command.linear_velocity
            cmd_w = tracking_command.angular_velocity

        if turn_command is not None:
            heading_error = turn_command.heading_error
            cmd_w = turn_command.angular_velocity

        if heading_error is not None:
            payload['heading_error_deg'] = round(
                math.degrees(heading_error),
                2,
            )

        if cross_track_error is not None:
            payload['cross_track_error_m'] = round(
                cross_track_error,
                3,
            )

        if dist_to_goal is not None:
            payload['dist_to_goal_m'] = round(
                dist_to_goal,
                3,
            )

        if dist_from_start is not None:
            payload['dist_from_start_m'] = round(
                dist_from_start,
                3,
            )

        payload.update({
            'target_speed_mps': round(
                target_speed,
                3,
            ),
            'lateral_speed_mps': round(
                lateral_speed,
                3,
            ),
            'cmd_v': round(
                cmd_v,
                3,
            ),
            'cmd_w': round(
                cmd_w,
                3,
            ),
        })

        return payload

    # -------------------------------------------------------------------------
    # Control logging
    # -------------------------------------------------------------------------

    def _log_control_sample(
        self,
        tracking_command: TrackingCommand | None = None,
        turn_command: TurnCommand | None = None,
        error_message: str | None = None,
    ):
        if (
            self.csv_log_writer is None
            or self.current_route is None
            or self.pose is None
        ):
            return

        snapshot = self._build_debug_snapshot(
            tracking_command=tracking_command,
            turn_command=turn_command,
            error_message=error_message,
        )

        self.csv_log_writer.writerow([
            round(
                self.get_clock().now().nanoseconds / 1e9,
                3,
            ),
            snapshot['phase'],
            snapshot['controller'],
            snapshot['waypoint_index'],
            snapshot.get('segment_start_x', 0.0),
            snapshot.get('segment_start_y', 0.0),
            snapshot.get('x', 0.0),
            snapshot.get('y', 0.0),
            snapshot.get('yaw_deg', 0.0),
            snapshot.get('path_heading_deg', 0.0),
            snapshot.get('segment_length_m', 0.0),
            snapshot.get('target_x', 0.0),
            snapshot.get('target_y', 0.0),
            snapshot.get('heading_error_deg', 0.0),
            snapshot.get('cross_track_error_m', 0.0),
            snapshot.get('dist_to_goal_m', 0.0),
            snapshot.get('dist_from_start_m', 0.0),
            snapshot.get('target_speed_mps', 0.0),
            snapshot.get('lateral_speed_mps', 0.0),
            snapshot.get('cmd_v', 0.0),
            snapshot.get('cmd_w', 0.0),
        ])

    def flush_log_file(self):
        if self.csv_log_file is not None:
            self.csv_log_file.flush()

    def update_status_file(self):
        self.status_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.status_path.write_text(
            f'{self.current_index}\n'
        )

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------

    def destroy_node(self):
        if self.csv_log_file is not None:
            self.csv_log_file.flush()
            self.csv_log_file.close()

        super().destroy_node()

    @staticmethod
    def quaternion_to_yaw(
        x,
        y,
        z,
        w,
    ):
        siny = 2.0 * (
            w * z
            + x * y
        )

        cosy = 1.0 - 2.0 * (
            y * y
            + z * z
        )

        return math.atan2(
            siny,
            cosy,
        )


def main(args=None):
    raw_args = (
        list(args)
        if args is not None
        else list(sys.argv)
    )

    parser = ArgumentParser()

    parser.add_argument(
        '--waypoints',
        default=DEFAULT_WAYPOINTS_PATH,
        help=(
            'Path to CSV file containing '
            'latitude,longitude waypoints'
        ),
    )

    parser.add_argument(
        '--resume',
        choices=[
            'ask',
            'yes',
            'no',
        ],
        default='ask',
        help=(
            'Whether to resume an unfinished '
            'navigation snapshot from '
            'last_waypoints.csv'
        ),
    )

    parser.add_argument(
        '--controller',
        choices=CONTROLLER_CHOICES,
        help=(
            'Force the controller type for this run. '
            'This overrides the parameter file.'
        ),
    )

    parsed_args, ros_args = parser.parse_known_args(
        raw_args[1:]
    )

    if '--params-file' not in ros_args:
        for candidate in DEFAULT_PARAMS_FILE_CANDIDATES:
            if candidate.exists():
                ros_args = [
                    '--ros-args',
                    '--params-file',
                    str(candidate),
                    *ros_args,
                ]
                break

    rclpy.init(args=ros_args)

    node = WaypointFollower(
        parsed_args.waypoints,
        resume_mode=parsed_args.resume,
        controller_override=parsed_args.controller,
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
