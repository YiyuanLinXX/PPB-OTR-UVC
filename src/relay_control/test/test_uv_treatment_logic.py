import csv

import pytest

from relay_control.treatment_logic import (
    distance_metres,
    load_waypoints,
    nearest_waypoint,
    UVTreatmentSequence,
)


def test_distance_metres_for_small_latitude_change():
    distance = distance_metres(42.0, -76.0, 42.000009, -76.0)
    assert distance == pytest.approx(1.0008, abs=0.002)


def test_load_annotated_csv(tmp_path):
    path = tmp_path / 'uv_treatment_waypoints.csv'
    with path.open('w', newline='') as file_handle:
        writer = csv.writer(file_handle)
        writer.writerow(['latitude', 'longitude', 'action'])
        writer.writerow([42.0, -76.0, 'ON'])
        writer.writerow([42.1, -76.1, 'OFF'])
    assert load_waypoints(path) == [
        (42.0, -76.0, 'ON'), (42.1, -76.1, 'OFF')]


def test_legacy_two_column_csv_infers_actions(tmp_path):
    path = tmp_path / 'legacy.csv'
    path.write_text(
        'latitude,longitude\n42.0,-76.0\n42.1,-76.1\n')
    assert load_waypoints(path) == [
        (42.0, -76.0, 'ON'), (42.1, -76.1, 'OFF')]


def test_invalid_action_order_is_rejected(tmp_path):
    path = tmp_path / 'bad.csv'
    path.write_text(
        'latitude,longitude,action\n'
        '42.0,-76.0,OFF\n42.1,-76.1,ON\n')
    with pytest.raises(ValueError, match='expected ON, found OFF'):
        load_waypoints(path)


def test_invalid_header_is_rejected(tmp_path):
    path = tmp_path / 'bad_header.csv'
    path.write_text(
        'latitude,wrong,action\n'
        '42.0,-76.0,ON\n42.1,-76.1,OFF\n')
    with pytest.raises(ValueError, match='second header must be longitude'):
        load_waypoints(path)


def test_nearest_waypoint_accepts_actions():
    waypoints = [(42.0, -76.0, 'ON'), (43.0, -77.0, 'OFF')]
    index, distance = nearest_waypoint(42.000001, -76.0, waypoints)
    assert index == 0
    assert distance < 0.2


def test_empty_csv_is_rejected(tmp_path):
    path = tmp_path / 'empty.csv'
    path.write_text('latitude,longitude,action\n')
    with pytest.raises(ValueError, match='contains no waypoints'):
        load_waypoints(path)


def test_sequence_triggers_on_off_in_file_order():
    points = [(42.0, -76.0, 'ON'), (42.000020, -76.0, 'OFF')]
    sequence = UVTreatmentSequence(points, 0.5, 0.3)

    event = sequence.update(42.000004, -76.0)
    assert event[:2] == (0, True)
    assert sequence.uv_lamps_should_be_on
    assert sequence.update(42.000005, -76.0) is None

    event = sequence.update(42.000017, -76.0)
    assert event[:2] == (1, False)
    assert sequence.complete
    assert not sequence.uv_lamps_should_be_on


def test_close_off_and_next_on_wait_for_hysteresis_travel():
    metre_lat = 0.000009
    points = [
        (42.0, -76.0, 'ON'),
        (42.0 + 2 * metre_lat, -76.0, 'OFF'),
        (42.0 + 2.1 * metre_lat, -76.0, 'ON'),
        (42.0 + 4 * metre_lat, -76.0, 'OFF'),
    ]
    sequence = UVTreatmentSequence(points, 0.5, 0.3)

    assert sequence.update(42.0, -76.0)[:2] == (0, True)
    assert sequence.update(42.0 + 2 * metre_lat, -76.0)[:2] == (1, False)

    # The close ON point is seen, but less than 0.3 m has been travelled.
    assert sequence.update(42.0 + 2.1 * metre_lat, -76.0) is None
    assert not sequence.uv_lamps_should_be_on

    # ON occurs only after 0.3 m travel from the actual OFF event location.
    event = sequence.update(42.0 + 2.4 * metre_lat, -76.0)
    assert event[:2] == (2, True)


def test_odd_waypoint_count_is_rejected():
    with pytest.raises(ValueError, match='complete ON/OFF pairs'):
        UVTreatmentSequence([(42.0, -76.0, 'ON')], 0.5, 0.3)


def test_narrowly_missed_waypoint_triggers_after_confirmed_departure():
    metre_lat = 0.000009
    points = [
        (42.0, -76.0, 'ON'),
        (42.001, -76.0, 'OFF'),
    ]
    sequence = UVTreatmentSequence(
        points,
        trigger_distance_m=0.5,
        hysteresis_distance_m=0.3,
        approach_confirmation_m=0.5,
        pass_confirmation_m=0.3,
        max_closest_approach_m=1.5,
        away_confirmation_samples=3,
        trend_epsilon_m=0.03,
    )

    positions_m = [-2.0, -1.2, -0.8, -0.6, -0.7, -0.85, -1.0]
    event = None
    for offset_m in positions_m:
        event = sequence.update(42.0 + offset_m * metre_lat, -76.0)

    assert event is not None
    assert event[:2] == (0, True)
    assert event[3] == 'confirmed_pass'
    assert 0.5 < event[4] < 0.7


def test_gps_noise_does_not_look_like_a_confirmed_pass():
    metre_lat = 0.000009
    points = [
        (42.0, -76.0, 'ON'),
        (42.001, -76.0, 'OFF'),
    ]
    sequence = UVTreatmentSequence(points, 0.5, 0.3)

    for offset_m in [-1.2, -0.7, -0.62, -0.66, -0.61, -0.65, -0.63]:
        assert sequence.update(
            42.0 + offset_m * metre_lat, -76.0) is None
    assert sequence.current_index == 0


def test_passing_too_far_away_does_not_trigger():
    metre_lat = 0.000009
    points = [
        (42.0, -76.0, 'ON'),
        (42.001, -76.0, 'OFF'),
    ]
    sequence = UVTreatmentSequence(
        points, 0.5, 0.3, max_closest_approach_m=1.5)

    for offset_m in [-5.0, -3.0, -2.0, -2.2, -2.5, -3.0]:
        assert sequence.update(
            42.0 + offset_m * metre_lat, -76.0) is None
    assert sequence.current_index == 0


def test_slow_departure_accumulates_across_trend_deadband():
    metre_lat = 0.000009
    points = [
        (42.0, -76.0, 'ON'),
        (42.001, -76.0, 'OFF'),
    ]
    sequence = UVTreatmentSequence(
        points, 0.5, 0.3,
        pass_confirmation_m=0.3,
        away_confirmation_samples=3,
        trend_epsilon_m=0.03,
    )

    event = None
    # About 1 cm per sample, representative of 0.1 m/s at 10 Hz.
    for offset_cm in range(-60, -101, -1):
        event = sequence.update(
            42.0 + (offset_cm / 100.0) * metre_lat, -76.0)
        if event is not None:
            break

    assert event is not None
    assert event[3] == 'confirmed_pass'
