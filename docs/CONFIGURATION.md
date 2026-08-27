# Configuration

The repository provides safe templates, not deployment-ready settings. Copy them outside the checkout and keep live prescriptions and credentials out of Git.

## UV treatment prescription

`waypoint_file` points to a CSV containing treatment-zone boundaries:

```csv
latitude,longitude,action
42.000000,-76.000000,ON
42.000010,-76.000000,OFF
```

The header is optional. If the `action` column is omitted, alternating `ON` and `OFF` actions are inferred. Explicit actions are recommended for reviewability. Every file must begin with `ON`, alternate actions, and end with `OFF`.

The UV prescription is distinct from the navigation route. Navigation waypoints describe where the robot drives; UV boundaries describe where treatment begins and ends. A prescription may be derived from a disease map produced by PPBv2, PPB-NG, or another verified mapping workflow.

## UV controller parameters

Start with `src/relay_control/config/uv_treatment.yaml`.

| Parameter | Meaning |
| --- | --- |
| `waypoint_file` | Absolute path to the UV prescription CSV |
| `gpio_pin` | BCM GPIO number controlling the lamp power stage |
| `uv_on_speed_mps` | Requested speed inside a treatment zone |
| `uv_off_speed_mps` | Requested transit speed outside treatment zones |
| `failsafe_speed_mps` | Conservative request while required data is invalid |
| `trigger_distance_m` | Radius within which the next boundary triggers |
| `hysteresis_distance_m` | Required travel between consecutive actions |
| `gps_timeout_sec` | Maximum age of a valid GNSS update |
| `require_speed_ack` | Require follower confirmation before UV power on |
| `navigation_active_required` | Require autonomous-treatment heartbeat |
| `require_cmd_vel_match` | Verify navigation owns the selected velocity output |
| `progress_file` | Local JSON mission checkpoint path |
| `recovery_mode` | `prompt`, `resume`, or `restart` |
| `mock_gpio` | Simulate the output without GPIO access |

The missed-boundary parameters tune approach/departure inference. Keep this feature conservative and validate it against recorded GNSS data before field use.

## Navigation and GNSS

`src/amiga_navigation/config/bringup.yaml` configures:

- UM982 and Amiga bridge serial-device paths;
- antenna baseline length, mounting direction, and heading correction;
- acceptable heading uncertainty, message ages, and satellite count;
- optional NTRIP caster connection and TLS selection.

NTRIP username and password values in the template are intentionally empty. Store deployed credentials in a private local configuration with restrictive file permissions. Prefer TLS whenever the caster supports it.

`src/amiga_navigation/config/waypoint_follower_params.yaml` configures the tracking controller, speed limits, route recovery, diagnostics, and the topics used by the UV controller. Any topic override must be applied consistently on both sides.

## GPIO validation

GPIO numbering is BCM, not physical-header numbering. The template uses BCM 23 (physical pin 16 on common Raspberry Pi headers) and assumes an active-high input. Confirm the electrical interface with lamp power isolated. A GPIO must drive a correctly rated, isolated relay or contactor interface; it must not directly switch a UV lamp load.

## Recovery behavior

`prompt` asks whether to continue an unfinished matching prescription when a terminal is available and otherwise resumes automatically. Use `restart` when a fresh run is always required, or `resume` for explicitly unattended recovery. Review the operational implications before enabling unattended behavior.
