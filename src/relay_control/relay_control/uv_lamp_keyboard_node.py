#!/usr/bin/env python3
"""Manually exercise the UV lamp GPIO power output from a keyboard."""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node


class MockOutputDevice:
    """GPIO substitute for development on a computer without GPIO hardware."""

    def __init__(self):
        self.value = 0

    def on(self):
        self.value = 1

    def off(self):
        self.value = 0

    def close(self):
        self.off()


class UVLampKeyboardNode(Node):
    """Poll stdin without blocking ROS and drive the UV lamp power output."""

    def __init__(self):
        super().__init__('uv_lamp_keyboard_node')
        self.declare_parameter('gpio_pin', 23)
        self.declare_parameter('mock_gpio', False)

        gpio_pin = self.get_parameter('gpio_pin').value
        mock_gpio = self.get_parameter('mock_gpio').value
        self._old_terminal_settings = None
        self._closed = False

        if mock_gpio:
            self._relay = MockOutputDevice()
            self.get_logger().warning('Mock GPIO enabled; no physical pin is used')
        else:
            try:
                from gpiozero import OutputDevice
                # active_high=True and initial_value=False provide a safe OFF state.
                self._relay = OutputDevice(
                    gpio_pin, active_high=True, initial_value=False)
            except Exception as exc:
                raise RuntimeError(
                    f'Could not initialize BCM GPIO {gpio_pin}: {exc}. '
                    'Check GPIO permissions, wiring, and python3-gpiozero.') from exc

        if not sys.stdin.isatty():
            self._relay.close()
            raise RuntimeError('stdin is not a terminal; run this node interactively')

        self._old_terminal_settings = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        self._timer = self.create_timer(0.05, self._poll_keyboard)
        self.get_logger().info(
            f'UV lamp output ready on BCM GPIO {gpio_pin}: '
            '[s] ON, [e] OFF, [q] quit')

    def _poll_keyboard(self):
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1).lower()
            if key == 's':
                self._relay.on()
                self.get_logger().warning('UV lamp power output ON')
            elif key == 'e':
                self._relay.off()
                self.get_logger().info('UV lamp power output OFF')
            elif key == 'q':
                self.get_logger().info('Quit requested')
                rclpy.shutdown()

    def close(self):
        """Fail safe: switch off and restore the terminal exactly once."""
        if self._closed:
            return
        self._closed = True
        if hasattr(self, '_relay'):
            self._relay.off()
            self._relay.close()
        if self._old_terminal_settings is not None:
            termios.tcsetattr(
                sys.stdin.fileno(), termios.TCSADRAIN,
                self._old_terminal_settings)
        self.get_logger().info('UV lamp power output OFF; GPIO released')

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UVLampKeyboardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f'uv_lamp_keyboard_node: {exc}', file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
