#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pandas as pd
import time
import sys

class ReplayNode(Node):
    def __init__(self, csv_path, speed=1.0):
        super().__init__('trajectory_replay')
        self.pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.csv_path = csv_path
        self.speed = speed
        self.df = pd.read_csv(csv_path)
        if 't' not in self.df.columns:
            raise RuntimeError('CSV doit contenir colonne t')
        self.t0 = self.df['t'].iloc[0]
        self.get_logger().info(f'Loaded {csv_path} with {len(self.df)} samples. Speed factor: {self.speed}')

    def run(self):
        start_real = time.time()
        for idx, row in self.df.iterrows():
            target_t = (row['t'] - self.t0) / self.speed  # seconds from start
            while True:
                elapsed = time.time() - start_real
                if elapsed >= target_t:
                    break
                time.sleep(0.001)
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            # columns after 't' are joint positions
            names = [c for c in self.df.columns if c != 't']
            js.name = names
            js.position = [float(row[c]) for c in names]
            self.pub.publish(js)
        self.get_logger().info('Replay finished.')

def main(args=None):
    if len(sys.argv) < 2:
        print('Usage: replay.py <trajectory.csv> [speed]')
        sys.exit(1)
    csv_path = sys.argv[1]
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    rclpy.init()
    node = ReplayNode(csv_path, speed)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
