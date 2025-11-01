#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtlesim_controller')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def _publish_twist(self, lin, ang, duration_s, rate_hz=50):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        dt = 1.0 / rate_hz
        end = time.time() + duration_s
        while rclpy.ok() and time.time() < end:
            self.pub.publish(msg)
            time.sleep(dt)

    def stop(self):
        self._publish_twist(0.0, 0.0, 0.05)

    def circle(self):
        v = 1.0
        w = 1.0
        self.get_logger().info('Mode A: circle (Ctrl+C to stop)')
        try:
            while rclpy.ok():
                self._publish_twist(v, w, 0.2)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def square(self):
        v = 1.0
        side_time = 2.0
        w = 1.57
        turn_time = (math.pi / 2.0) / w

        self.get_logger().info('Mode B: square')
        for _ in range(4):
            self._publish_twist(v, 0.0, side_time)
            self.stop()
            self._publish_twist(0.0, w, turn_time)
            self.stop()
        self.stop()
        self.get_logger().info('Square complete')

    def spiral(self):
        w = 0.6
        v = 0.2
        vmax = 2.5
        self.get_logger().info('Mode C: outward spiral (Ctrl+C to stop)')
        try:
            while rclpy.ok():
                self._publish_twist(v, w, 0.1)
                v = min(v + 0.02, vmax)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    print("\nChoose a motion and press Enter:\n  A = circle\n  B = square\n  C = outward spiral\n")
    choice = input("> ").strip().upper()

    if choice == 'A':
        node.circle()
    elif choice == 'B':
        node.square()
    elif choice == 'C':
        node.spiral()
    else:
        print("Invalid choice. Use A, B, or C.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
