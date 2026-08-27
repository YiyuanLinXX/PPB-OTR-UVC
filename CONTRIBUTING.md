# Contributing

Contributions that improve reproducibility, safety behavior, documentation, or hardware-independent test coverage are welcome.

## Development workflow

1. Open an issue before a large behavior or interface change.
2. Create a focused branch from the default branch.
3. Keep hardware-dependent code behind configurable interfaces or mock modes.
4. Add tests for treatment sequencing, recovery, protocol parsing, or safety logic whenever behavior changes.
5. Run `python3 -m pytest` from the repository root.
6. Submit a pull request describing the use case, safety impact, and validation.

Do not commit real field coordinates, GNSS caster credentials, serial device IDs, operator information, logs, or machine-specific absolute paths. Use synthetic data in tests and examples.

## Design principles

- UV lamps default to off and return to off when required inputs are stale.
- Safety decisions should be testable without ROS or physical hardware.
- Configuration names should describe the UV-treatment intent; electrical relay terminology is reserved for the power-stage implementation.
- Backward compatibility matters for deployed robots. Document unavoidable interface changes in `CHANGELOG.md`.

By participating, you agree to follow the [Code of Conduct](CODE_OF_CONDUCT.md).
