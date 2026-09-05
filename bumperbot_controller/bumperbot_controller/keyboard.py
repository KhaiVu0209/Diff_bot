#!/usr/bin/env python3
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        # ==========================================
        # Publisher
        # ==========================================
        self.publisher = self.create_publisher(
            Twist,
            '/key_vel',
            10
        )

        # ==========================================
        # Parameters
        # ==========================================

        self.declare_parameter('linear_speed', 0.1)
        self.declare_parameter('angular_speed', 0.5)

        self.linear_speed = self.get_parameter(
            'linear_speed'
        ).value

        self.angular_speed = self.get_parameter(
            'angular_speed'
        ).value

        # ==========================================
        # Các phím đang được giữ
        # ==========================================

        self.keys_pressed = set()
        self.keys_lock = threading.Lock()
        self.last_key_time = 0.0
        self.keyboard_stop = threading.Event()
        self.keyboard_thread = threading.Thread(
            target=self.read_keyboard,
            daemon=True
        )

        # ==========================================
        # Keyboard listener
        # ==========================================

        self.keyboard_thread.start()

        # ==========================================
        # Publish 20 Hz
        # ==========================================

        self.timer = self.create_timer(
            0.05,
            self.publish_cmd_vel
        )

        self.get_logger().info(
            'Keyboard teleop started'
        )

        print('')
        print('======================================')
        print('        KEYBOARD TELEOP')
        print('======================================')
        print('')
        print('              W')
        print('          A   S   D')
        print('')
        print('Hold W       : Forward')
        print('Hold S       : Backward')
        print('Hold A       : Turn left')
        print('Hold D       : Turn right')
        print('')
        print('W + A        : Forward + Left')
        print('W + D        : Forward + Right')
        print('S + A        : Backward + Left')
        print('S + D        : Backward + Right')
        print('')
        print('Release keys : Stop')
        print('Q            : Quit')
        print('')
        print('======================================')
        print('')

    # ==========================================
    # Khi nhấn phím
    # ==========================================

    def read_keyboard(self):

        if not sys.stdin.isatty():
            self.get_logger().error(
                'Keyboard input requires running this node in a terminal'
            )
            return

        original_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while not self.keyboard_stop.is_set() and rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)

                if not ready:
                    continue

                char = sys.stdin.read(1).lower()

                if char in ['w', 's', 'a', 'd']:
                    with self.keys_lock:
                        self.keys_pressed.add(char)
                        self.last_key_time = time.monotonic()
                elif char == 'q':
                    self.get_logger().info('Quit keyboard teleop')
                    self.keyboard_stop.set()
                    rclpy.shutdown()
        finally:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                original_terminal_settings
            )

    # ==========================================
    # Tạo Twist
    # ==========================================

    def publish_cmd_vel(self):

        msg = Twist()

        with self.keys_lock:
            if time.monotonic() - self.last_key_time > 0.5:
                self.keys_pressed.clear()
            keys_pressed = set(self.keys_pressed)

        # ==========================================
        # Linear velocity
        # ==========================================

        if 'w' in keys_pressed:
            msg.linear.x = self.linear_speed

        elif 's' in keys_pressed:
            msg.linear.x = -self.linear_speed

        else:
            msg.linear.x = 0.0

        # ==========================================
        # Angular velocity
        # ==========================================

        if 'a' in keys_pressed:
            msg.angular.z = self.angular_speed

        elif 'd' in keys_pressed:
            msg.angular.z = -self.angular_speed

        else:
            msg.angular.z = 0.0

        # ==========================================
        # Publish
        # ==========================================

        self.publisher.publish(msg)

    # ==========================================
    # Stop robot
    # ==========================================

    def stop_robot(self):

        msg = Twist()

        msg.linear.x = 0.0
        msg.angular.z = 0.0

        for _ in range(5):
            self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = KeyboardTeleop()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_robot()

        node.keyboard_stop.set()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
