"""Pure navigation safety checks shared by UV control and unit tests."""

import math


def navigation_gate_reason(
    navigation_active,
    navigation_state_age_sec,
    navigation_state_timeout_sec,
    require_cmd_vel_match,
    navigation_cmd,
    navigation_cmd_age_sec,
    robot_cmd,
    robot_cmd_age_sec,
    cmd_vel_timeout_sec,
    linear_tolerance_mps,
    angular_tolerance_rps,
):
    """Return ``None`` only while autonomous navigation owns motion output."""
    if navigation_state_age_sec is None:
        return 'no navigation heartbeat'
    if navigation_state_age_sec > navigation_state_timeout_sec:
        return 'navigation heartbeat stale'
    if not navigation_active:
        return 'navigation inactive'

    if not require_cmd_vel_match:
        return None
    if navigation_cmd is None or navigation_cmd_age_sec is None:
        return 'no navigation velocity command'
    if robot_cmd is None or robot_cmd_age_sec is None:
        return 'no robot output velocity command'
    if navigation_cmd_age_sec > cmd_vel_timeout_sec:
        return 'navigation velocity command stale'
    if not all(math.isfinite(value) for value in navigation_cmd):
        return 'invalid navigation velocity command'
    if not all(math.isfinite(value) for value in robot_cmd):
        return 'invalid robot output velocity command'
    if robot_cmd_age_sec > cmd_vel_timeout_sec:
        return 'robot output velocity command stale'

    if abs(navigation_cmd[0] - robot_cmd[0]) > linear_tolerance_mps:
        return 'manual or non-navigation linear command detected'
    if abs(navigation_cmd[1] - robot_cmd[1]) > angular_tolerance_rps:
        return 'manual or non-navigation angular command detected'
    return None
