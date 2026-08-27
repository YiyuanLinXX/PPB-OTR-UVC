from relay_control.progress_store import (
    load_progress,
    resumable_index,
    save_progress,
    waypoint_file_fingerprint,
)


def test_progress_round_trip_and_resume(tmp_path):
    waypoint_file = tmp_path / 'waypoints.csv'
    waypoint_file.write_text(
        'latitude,longitude,action\n'
        '42.0,-76.0,ON\n42.1,-76.1,OFF\n')
    progress_file = tmp_path / 'progress.json'
    fingerprint = waypoint_file_fingerprint(waypoint_file)

    save_progress(
        progress_file, waypoint_file, fingerprint, 2, 1, True, False)
    progress = load_progress(progress_file)

    assert progress['next_waypoint_index'] == 1
    assert progress['triggered_waypoint_indices'] == [0]
    assert progress['uv_lamps_should_be_on'] is True
    assert resumable_index(
        progress, waypoint_file, fingerprint, 2) == 1


def test_completed_progress_is_not_resumed(tmp_path):
    waypoint_file = tmp_path / 'waypoints.csv'
    waypoint_file.write_text('data')
    progress_file = tmp_path / 'progress.json'
    fingerprint = waypoint_file_fingerprint(waypoint_file)
    save_progress(
        progress_file, waypoint_file, fingerprint, 2, 2, False, True)

    assert resumable_index(
        load_progress(progress_file), waypoint_file, fingerprint, 2) is None


def test_changed_waypoint_file_is_not_resumed(tmp_path):
    waypoint_file = tmp_path / 'waypoints.csv'
    waypoint_file.write_text('first')
    progress_file = tmp_path / 'progress.json'
    old_fingerprint = waypoint_file_fingerprint(waypoint_file)
    save_progress(
        progress_file, waypoint_file, old_fingerprint, 2, 1, True, False)

    waypoint_file.write_text('changed')
    new_fingerprint = waypoint_file_fingerprint(waypoint_file)
    assert resumable_index(
        load_progress(progress_file), waypoint_file, new_fingerprint, 2) is None
