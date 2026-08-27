from relay_control.safety_logic import navigation_gate_reason


BASE = dict(
    navigation_active=True,
    navigation_state_age_sec=0.1,
    navigation_state_timeout_sec=0.5,
    require_cmd_vel_match=True,
    navigation_cmd=(0.1, 0.0),
    navigation_cmd_age_sec=0.1,
    robot_cmd=(0.1, 0.0),
    robot_cmd_age_sec=0.1,
    cmd_vel_timeout_sec=0.5,
    linear_tolerance_mps=0.03,
    angular_tolerance_rps=0.05,
)


def reason(**changes):
    values = {**BASE, **changes}
    return navigation_gate_reason(**values)


def test_active_navigation_with_selected_command_opens_gate():
    assert reason() is None


def test_inactive_or_stale_navigation_closes_gate():
    assert reason(navigation_active=False) == 'navigation inactive'
    assert reason(navigation_state_age_sec=0.6) == 'navigation heartbeat stale'


def test_manual_velocity_override_closes_gate():
    assert reason(robot_cmd=(0.5, 0.0)) == (
        'manual or non-navigation linear command detected')
    assert reason(robot_cmd=(0.1, 0.2)) == (
        'manual or non-navigation angular command detected')




def test_missing_or_stale_selected_command_closes_gate():
    assert reason(robot_cmd=None) == 'no robot output velocity command'
    assert reason(robot_cmd_age_sec=0.6) == 'robot output velocity command stale'


def test_non_finite_command_closes_gate():
    assert reason(navigation_cmd=(float('nan'), 0.0)) == (
        'invalid navigation velocity command')
    assert reason(robot_cmd=(0.1, float('inf'))) == (
        'invalid robot output velocity command')


def test_command_match_can_be_disabled_explicitly():
    assert reason(
        require_cmd_vel_match=False,
        navigation_cmd=None,
        robot_cmd=None,
    ) is None
