"""ROS-independent parsing and sequencing for UV treatment boundaries."""

import csv
import math
from pathlib import Path


EARTH_RADIUS_M = 6371008.8


def load_waypoints(path):
    """Load (latitude, longitude, action) rows; infer actions if omitted."""
    csv_path = Path(path).expanduser()
    if not csv_path.is_file():
        raise FileNotFoundError(
            f'UV treatment prescription does not exist: {csv_path}')

    waypoints = []
    with csv_path.open('r', newline='', encoding='utf-8-sig') as waypoint_file:
        reader = csv.reader(waypoint_file)
        for line_number, row in enumerate(reader, start=1):
            if not row or all(not value.strip() for value in row):
                continue
            if len(row) < 2:
                raise ValueError(
                    f'{csv_path}:{line_number}: expected latitude,longitude')
            try:
                latitude = float(row[0].strip())
                longitude = float(row[1].strip())
            except ValueError:
                if line_number == 1 and row[0].strip().lower() == 'latitude':
                    if row[1].strip().lower() != 'longitude':
                        raise ValueError(
                            f'{csv_path}:1: second header must be longitude')
                    if (
                        len(row) >= 3 and row[2].strip()
                        and row[2].strip().lower() != 'action'
                    ):
                        raise ValueError(
                            f'{csv_path}:1: third header must be action')
                    continue
                raise ValueError(
                    f'{csv_path}:{line_number}: invalid latitude/longitude')
            if not -90.0 <= latitude <= 90.0:
                raise ValueError(f'{csv_path}:{line_number}: latitude out of range')
            if not -180.0 <= longitude <= 180.0:
                raise ValueError(f'{csv_path}:{line_number}: longitude out of range')

            action = None
            if len(row) >= 3 and row[2].strip():
                action = row[2].strip().upper()
                if action not in ('ON', 'OFF'):
                    raise ValueError(
                        f'{csv_path}:{line_number}: action must be ON or OFF')
            waypoints.append((latitude, longitude, action))

    if not waypoints:
        raise ValueError(
            f'UV treatment prescription contains no waypoints: {csv_path}')
    if len(waypoints) % 2 != 0:
        raise ValueError(
            'UV treatment waypoints must contain complete ON/OFF pairs')

    has_actions = [waypoint[2] is not None for waypoint in waypoints]
    if any(has_actions) and not all(has_actions):
        raise ValueError(
            'Action column must be present for every waypoint or omitted '
            'from every waypoint')
    if not any(has_actions):
        waypoints = [
            (latitude, longitude, 'ON' if index % 2 == 0 else 'OFF')
            for index, (latitude, longitude, _) in enumerate(waypoints)
        ]

    for index, (_, _, action) in enumerate(waypoints):
        expected = 'ON' if index % 2 == 0 else 'OFF'
        if action != expected:
            raise ValueError(
                f'UV treatment action error at waypoint {index + 1}: '
                f'expected {expected}, found {action}')
    return waypoints


def distance_metres(latitude_a, longitude_a, latitude_b, longitude_b):
    """Return the great-circle distance between two GPS coordinates."""
    lat_a = math.radians(latitude_a)
    lat_b = math.radians(latitude_b)
    delta_lat = lat_b - lat_a
    delta_lon = math.radians(longitude_b - longitude_a)
    haversine = (
        math.sin(delta_lat / 2.0) ** 2
        + math.cos(lat_a) * math.cos(lat_b)
        * math.sin(delta_lon / 2.0) ** 2
    )
    central_angle = 2.0 * math.atan2(
        math.sqrt(haversine), math.sqrt(max(0.0, 1.0 - haversine)))
    return EARTH_RADIUS_M * central_angle


def nearest_waypoint(latitude, longitude, waypoints):
    """Return the zero-based index and distance of the nearest waypoint."""
    distances = [
        distance_metres(latitude, longitude, waypoint[0], waypoint[1])
        for waypoint in waypoints
    ]
    index = min(range(len(distances)), key=distances.__getitem__)
    return index, distances[index]


class UVTreatmentSequence:
    """Process lamp ON/OFF boundaries and recover a narrowly missed target."""

    def __init__(
        self, waypoints, trigger_distance_m=0.5,
        hysteresis_distance_m=0.3, start_index=0,
        missed_waypoint_enabled=True,
        approach_confirmation_m=0.5,
        pass_confirmation_m=0.3,
        max_closest_approach_m=1.5,
        away_confirmation_samples=3,
        trend_epsilon_m=0.03,
    ):
        if not waypoints or len(waypoints) % 2 != 0:
            raise ValueError(
                'UV treatment waypoints must contain complete ON/OFF pairs')
        if trigger_distance_m <= 0.0:
            raise ValueError('trigger_distance_m must be greater than zero')
        if hysteresis_distance_m < 0.0:
            raise ValueError('hysteresis_distance_m cannot be negative')
        if approach_confirmation_m < 0.0:
            raise ValueError('approach_confirmation_m cannot be negative')
        if pass_confirmation_m <= 0.0:
            raise ValueError('pass_confirmation_m must be greater than zero')
        if max_closest_approach_m < trigger_distance_m:
            raise ValueError(
                'max_closest_approach_m cannot be smaller than '
                'trigger_distance_m')
        if away_confirmation_samples < 1:
            raise ValueError('away_confirmation_samples must be at least 1')
        if trend_epsilon_m < 0.0:
            raise ValueError('trend_epsilon_m cannot be negative')
        if not 0 <= start_index < len(waypoints):
            raise ValueError('start_waypoint_index is outside the waypoint file')

        self.waypoints = [
            (waypoint[0], waypoint[1],
             waypoint[2] if len(waypoint) >= 3
             else ('ON' if index % 2 == 0 else 'OFF'))
            for index, waypoint in enumerate(waypoints)
        ]
        self.trigger_distance_m = trigger_distance_m
        self.hysteresis_distance_m = hysteresis_distance_m
        self.missed_waypoint_enabled = missed_waypoint_enabled
        self.approach_confirmation_m = approach_confirmation_m
        self.pass_confirmation_m = pass_confirmation_m
        self.max_closest_approach_m = max_closest_approach_m
        self.away_confirmation_samples = away_confirmation_samples
        self.trend_epsilon_m = trend_epsilon_m
        self.current_index = start_index
        # Before an OFF row, its paired ON action has already occurred.
        self.uv_lamps_should_be_on = start_index % 2 == 1
        self.complete = False
        self._last_trigger_position = None
        self._next_seen_during_hysteresis = False
        self._reset_target_tracking()

    def update(self, latitude, longitude):
        """Return event details when the next ordered point is triggered."""
        if self.complete:
            return None

        target = self.waypoints[self.current_index]
        target_distance = distance_metres(
            latitude, longitude, target[0], target[1])

        if self._last_trigger_position is not None:
            if target_distance <= self.trigger_distance_m:
                self._next_seen_during_hysteresis = True
            travelled_distance = distance_metres(
                latitude, longitude,
                self._last_trigger_position[0], self._last_trigger_position[1])
            if travelled_distance < self.hysteresis_distance_m:
                return None
            self._last_trigger_position = None
            if not self._next_seen_during_hysteresis:
                return None
            self._next_seen_during_hysteresis = False
            return self._trigger_current(
                latitude, longitude, target_distance,
                'overlap_after_hysteresis')

        if target_distance <= self.trigger_distance_m:
            return self._trigger_current(
                latitude, longitude, target_distance, 'inside_radius')

        if self.missed_waypoint_enabled and self._passed_target(target_distance):
            return self._trigger_current(
                latitude, longitude, target_distance, 'confirmed_pass')
        return None

    def _passed_target(self, distance_m):
        """Confirm approach then departure while filtering GNSS noise."""
        if self._first_target_distance is None:
            self._first_target_distance = distance_m
            self._minimum_target_distance = distance_m
            self._away_reference_distance = distance_m
            return False

        if distance_m < self._minimum_target_distance:
            self._minimum_target_distance = distance_m
            self._away_sample_count = 0
            self._away_reference_distance = distance_m

        approach_amount = (
            self._first_target_distance - self._minimum_target_distance)
        close_enough_to_confirm_approach = (
            self._minimum_target_distance
            <= self.trigger_distance_m + self.approach_confirmation_m)
        if (
            approach_amount >= self.approach_confirmation_m
            or close_enough_to_confirm_approach
        ):
            self._approach_confirmed = True

        if (
            distance_m
            > self._away_reference_distance + self.trend_epsilon_m
        ):
            self._away_sample_count += 1
            self._away_reference_distance = distance_m
        elif (
            distance_m
            < self._away_reference_distance - self.trend_epsilon_m
        ):
            self._away_sample_count = 0
            self._away_reference_distance = distance_m

        return (
            self._approach_confirmed
            and self._minimum_target_distance <= self.max_closest_approach_m
            and distance_m - self._minimum_target_distance
            >= self.pass_confirmation_m
            and self._away_sample_count >= self.away_confirmation_samples
        )

    def _reset_target_tracking(self):
        self._first_target_distance = None
        self._minimum_target_distance = None
        self._away_reference_distance = None
        self._approach_confirmed = False
        self._away_sample_count = 0

    def _trigger_current(self, latitude, longitude, distance_m, reason):
        index = self.current_index
        minimum_distance = self._minimum_target_distance
        if minimum_distance is None:
            minimum_distance = distance_m
        self.uv_lamps_should_be_on = self.waypoints[index][2] == 'ON'
        self._last_trigger_position = (latitude, longitude)
        self._next_seen_during_hysteresis = False
        self.current_index += 1
        self._reset_target_tracking()
        if self.current_index >= len(self.waypoints):
            self.complete = True
            self._last_trigger_position = None
        return (
            index, self.uv_lamps_should_be_on, distance_m, reason,
            minimum_distance)


# Historical class alias retained for deployed code importing relay_control.
RelayWaypointSequence = UVTreatmentSequence
