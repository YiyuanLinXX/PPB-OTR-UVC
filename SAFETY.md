# UV-C safety

PPB-OTR-UVC is research software for an experimental agricultural robot. It is not a safety-rated controller and has not been certified for unattended UV-C operation.

UV-C exposure can cause serious eye and skin injury. High-voltage lamp power systems introduce additional electrical, thermal, fire, and stored-energy hazards. Robot motion adds collision, crushing, rollover, and loss-of-control hazards. The system owner is responsible for a documented risk assessment and for compliance with all rules that apply at the operating site.

## Required independent protections

Do not rely on a Raspberry Pi, ROS topic, GNSS receiver, network link, or the software in this repository as the only protective measure. A field system should include protections appropriate to its design, including:

- a physical emergency stop that removes hazardous motion and lamp power;
- a keyed hardware enable and a normally de-energized lamp power stage;
- contactor feedback or other independent verification where required;
- shielding, exclusion zones, signage, and visible/audible warnings;
- safe behavior after power loss, reboot, broken wiring, and controller faults;
- over-current, ground-fault, thermal, and enclosure protection;
- trained operators and written startup, shutdown, and emergency procedures.

Use qualified personnel for mains or high-voltage wiring. Verify the relay or contactor input polarity: the checked-in controller assumes an **active-high** GPIO command and initializes it low.

## Before every field run

1. Inspect lamp mounts, cables, connectors, guards, antennas, and the robot.
2. Confirm that the treatment prescription belongs to the intended field and that every `ON` boundary has a following `OFF` boundary.
3. Confirm RTK quality, antenna geometry, GPIO numbering, and speed limits.
4. Test emergency stop, keyed enable, warnings, and lamp-power removal without UV exposure.
5. Run once with lamp power physically isolated or `mock_gpio: true` and review the route, speed changes, and state transitions.
6. Establish and enforce the exclusion zone before enabling lamp power.

## Software fail-safe behavior

The UV treatment node commands its GPIO output off on startup, shutdown, invalid or stale GNSS, stale or inactive navigation, velocity-command takeover, and missing treatment-speed acknowledgement. These checks reduce risk from known failure modes; they do not make the overall system fail-safe.

Report a security issue through [SECURITY.md](SECURITY.md). Report a safety behavior defect publicly only when doing so will not expose sensitive details; otherwise use the same private channel.
