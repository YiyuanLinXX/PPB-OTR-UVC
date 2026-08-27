"""Persistent recovery state for UV treatment missions."""

import hashlib
import json
from pathlib import Path
from datetime import datetime, timezone


def waypoint_file_fingerprint(path):
    """Return a SHA-256 fingerprint for the exact waypoint file contents."""
    return hashlib.sha256(Path(path).expanduser().read_bytes()).hexdigest()


def load_progress(path):
    """Load a progress dictionary, returning None when no file exists."""
    progress_path = Path(path).expanduser()
    if not progress_path.exists():
        return None
    with progress_path.open('r', encoding='utf-8') as progress_file:
        data = json.load(progress_file)
    if not isinstance(data, dict):
        raise ValueError('progress file root must be a JSON object')
    return data


def save_progress(
    path, waypoint_file, waypoint_fingerprint, waypoint_count,
    next_waypoint_index, uv_lamps_should_be_on, completed,
    last_trigger_reason=None, last_trigger_distance_m=None,
    last_closest_distance_m=None,
):
    """Atomically save mission progress so interruption cannot truncate it."""
    progress_path = Path(path).expanduser()
    progress_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = progress_path.with_name(progress_path.name + '.tmp')
    data = {
        'waypoint_file': str(Path(waypoint_file).expanduser().resolve()),
        'waypoint_fingerprint': waypoint_fingerprint,
        'waypoint_count': waypoint_count,
        'next_waypoint_index': next_waypoint_index,
        # Since processing is strictly sequential, every lower index has
        # already triggered. Keep the explicit list for easy human inspection.
        'triggered_waypoint_indices': list(range(next_waypoint_index)),
        'uv_lamps_should_be_on': bool(uv_lamps_should_be_on),
        'completed': bool(completed),
        'last_trigger_reason': last_trigger_reason,
        'last_trigger_distance_m': last_trigger_distance_m,
        'last_closest_distance_m': last_closest_distance_m,
        'updated_at_utc': datetime.now(timezone.utc).isoformat(),
    }
    with temporary_path.open('w', encoding='utf-8') as progress_file:
        json.dump(data, progress_file, indent=2, sort_keys=True)
        progress_file.write('\n')
        progress_file.flush()
    temporary_path.replace(progress_path)


def resumable_index(progress, waypoint_file, fingerprint, waypoint_count):
    """Return a valid unfinished next index, otherwise None."""
    if not progress or progress.get('completed') is True:
        return None
    if progress.get('waypoint_file') != str(
            Path(waypoint_file).expanduser().resolve()):
        return None
    if progress.get('waypoint_fingerprint') != fingerprint:
        return None
    if progress.get('waypoint_count') != waypoint_count:
        return None
    index = progress.get('next_waypoint_index')
    if isinstance(index, bool) or not isinstance(index, int):
        return None
    if not 0 <= index < waypoint_count:
        return None
    return index
