#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import time

INSTRUCTIONS = """
Teleop clavier — touches:
  q -> gauche (angular + linear optionnel)
  s -> stop
  d -> droite
  z -> avant
  x -> arrière
  CTRL-C -> quit
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.02  # 50 Hz -> latence << 100ms
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.current_twist = Twist()
        self.get_logger().info('Teleop node started. Publish rate: {:.0f}Hz'.format(1/self.timer_period))
        self.get_logger().info(INSTRUCTIONS)
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def timer_callback(self):
        # Publish current command at high rate to guarantee latency <100ms on key events
        self.pub.publish(self.current_twist)

    def run(self):
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    ch = sys.stdin.read(1)
                    self.handle_key(ch)
                rclpy.spin_once(self, timeout_sec=0)
                time.sleep(0.005)
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.get_logger().info('Teleop node exiting')

    def handle_key(self, ch):
        t = Twist()
        if ch == 'q':
            t.angular.z = 0.8
            self.current_twist = t
            self.get_logger().info('Commande: GAUCHE')
        elif ch == 'd':
            t.angular.z = -0.8
            self.current_twist = t
            self.get_logger().info('Commande: DROITE')
        elif ch == 'z':
            t.linear.x = 0.3
            self.current_twist = t
            self.get_logger().info('Commande: AVANT')
        elif ch == 'x':
            t.linear.x = -0.2
            self.current_twist = t
            self.get_logger().info('Commande: ARRIERE')
        elif ch == 's':
            self.current_twist = Twist()
            self.get_logger().info('Commande: STOP')
        else:
            # ignore other keys
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
