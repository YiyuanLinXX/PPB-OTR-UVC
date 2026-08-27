"""Compatibility exports for the historical relay sequencing module."""

from relay_control.treatment_logic import (
    distance_metres,
    load_waypoints,
    nearest_waypoint,
    RelayWaypointSequence,
    UVTreatmentSequence,
)


__all__ = [
    'distance_metres',
    'load_waypoints',
    'nearest_waypoint',
    'RelayWaypointSequence',
    'UVTreatmentSequence',
]
