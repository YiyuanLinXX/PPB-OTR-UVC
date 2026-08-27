"""Compatibility wrapper for the historical GPS relay executable."""

from relay_control.uv_treatment_node import main, UVTreatmentNode


GpsRelayNode = UVTreatmentNode


if __name__ == '__main__':
    main()
