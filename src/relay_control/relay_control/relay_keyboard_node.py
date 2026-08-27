"""Compatibility wrapper for the historical relay keyboard executable."""

from relay_control.uv_lamp_keyboard_node import main, UVLampKeyboardNode


RelayKeyboardNode = UVLampKeyboardNode


if __name__ == '__main__':
    main()
