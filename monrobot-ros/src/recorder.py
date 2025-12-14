#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import csv
import os
import time
import json
import sys
import select
import tty
import termios

OUT_DIR = 'trajectories'  # relative to where node is launched

class Recorder(Node):
    def __init__(self):
        super().__init__('joint_recorder')
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb_joint, 50)
        self.control_sub = self.create_subscription(String, '/recorder_control', self.cb_control, 10)
        self.recording = False
        self.current_buffer = []  # list of (t, positions)
        self.joint_names = []
        os.makedirs(OUT_DIR, exist_ok=True)
        self.get_logger().info('Recorder started. Press r=start, s=stop, w=save (in this terminal). Or publish to /recorder_control.')

        # terminal settings for simple key commands
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def cb_joint(self, msg: JointState):
        if not self.recording:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if not self.joint_names:
            self.joint_names = msg.name
        self.current_buffer.append({'t': t, 'positions': msg.position})

    def cb_control(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'r' or cmd == 'record':
            self.start_record()
        elif cmd == 's' or cmd == 'stop':
            self.stop_record()
        elif cmd == 'save':
            self.save_record()
        else:
            self.get_logger().info(f'Control topic received unknown: {cmd}')

    def start_record(self):
        if self.recording:
            self.get_logger().warn('Already recording')
            return
        self.recording = True
        self.current_buffer = []
        self.get_logger().info('RECORDING started')

    def stop_record(self):
        if not self.recording:
            self.get_logger().warn('Not recording')
            return
        self.recording = False
        self.get_logger().info(f'RECORDING stopped. {len(self.current_buffer)} samples captured.')

    def save_record(self):
        if not self.current_buffer:
            self.get_logger().warn('Buffer empty, nothing to save')
            return
        ts = time.strftime('%Y%m%d_%H%M%S')
        filename_csv = os.path.join(OUT_DIR, f'traj_{ts}.csv')
        with open(filename_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['t'] + (self.joint_names if self.joint_names else [f'j{i}' for i in range(len(self.current_buffer[0]['positions']))])
            writer.writerow(header)
            for row in self.current_buffer:
                writer.writerow([row['t']] + list(row['positions']))
        # also save JSON
        filename_json = os.path.join(OUT_DIR, f'traj_{ts}.json')
        with open(filename_json, 'w') as f:
            json.dump(self.current_buffer, f, indent=2)
        self.get_logger().info(f'Saved CSV: {filename_csv} and JSON: {filename_json}')

    def run(self):
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    ch = sys.stdin.read(1)
                    if ch == 'r':
                        self.start_record()
                    elif ch == 's':
                        self.stop_record()
                    elif ch == 'w':  # save
                        self.save_record()
                rclpy.spin_once(self, timeout_sec=0.05)
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.get_logger().info('Recorder exiting')

def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
