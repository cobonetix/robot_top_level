#!/usr/bin/env python3
"""Reads joint positions from a CSV file and sends them to the arm_joint service."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from cobonetix_interfaces.srv import ArmJoint

JOINT_FIELDS = ['l_j1', 'l_j2', 'l_j3', 'l_j4', 'r_j1', 'r_j2', 'r_j3', 'r_j4']
DEG_TO_RAD_FIELDS = {'l_j2', 'l_j3', 'l_j4', 'r_j2', 'r_j3', 'r_j4'}


class JointCsvPlayer(Node):
    def __init__(self):
        super().__init__('joint_csv_player')
        self.client = self.create_client(ArmJoint, 'arm_joint')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arm_joint service...')
        self.get_logger().info('Connected to arm_joint service')

    def send_request(self, joints):
        request = ArmJoint.Request()
        request.l_j1 = joints['l_j1']
        request.l_j2 = joints['l_j2']
        request.l_j3 = joints['l_j3']
        request.l_j4 = joints['l_j4']
        request.r_j1 = joints['r_j1']
        request.r_j2 = joints['r_j2']
        request.r_j3 = joints['r_j3']
        request.r_j4 = joints['r_j4']
        future = self.client.call_async(request)
        return future


def load_csv(filepath):
    """Load joint positions from a CSV file, indexed by the last column value.

    Expected column order: l_j1, l_j2, l_j3, l_j4, r_j1, r_j2, r_j3, r_j4, <name>
    First row is skipped (header). Returns a dict mapping name -> list of joint dicts.
    """
    positions = {}
    with open(filepath) as f:
        content = f.read().replace('"', '')
    lines = content.strip().splitlines()
    for line_num, line in enumerate(lines[1:], start=2):
        vals = [v.strip() for v in line.split(',')]
        if len(vals) < len(JOINT_FIELDS) + 1:
            raise ValueError(f"Not enough columns on CSV line {line_num}: expected {len(JOINT_FIELDS) + 1}, got {len(vals)}")
        key = vals[-1]
        try:
            joints = {}
            for field, val in zip(JOINT_FIELDS, vals):
                v = float(val)
                if field in DEG_TO_RAD_FIELDS:
                    v = math.radians(v)
                joints[field] = v
        except (ValueError, TypeError) as e:
            raise ValueError(f"Invalid numeric value on CSV line {line_num}: {e}")
        positions.setdefault(key, []).append(joints)
    return positions


def main():
    parser = argparse.ArgumentParser(description='Play joint positions from a CSV file')
    parser.add_argument('csv_file', nargs='?', default='trajectory1.csv', help='Path to CSV file (default: trajectory1.csv)')
    parser.add_argument('--rate', type=float, default=2.0, help='Commands per second (default: 1.0)')
    parser.add_argument('--name', type=str, default=None, help='Play only the trajectory with this name (default: play all)')
    cli_args = parser.parse_args()

    period = 1.0 / cli_args.rate

    positions = load_csv(cli_args.csv_file)
    if not positions:
        print('CSV file contains no data rows.')
        return

    if cli_args.name is not None:
        if cli_args.name not in positions:
            print(f"Trajectory '{cli_args.name}' not found. Available: {list(positions.keys())}")
            return
        rows = positions[cli_args.name]
        label = cli_args.name
    else:
        rows = [joints for group in positions.values() for joints in group]
        label = 'all'

    print(f'Loaded {len(rows)} joint positions ({label}) from {cli_args.csv_file}')
    print(f'Rate: {cli_args.rate} Hz (one command every {period:.2f}s)')

    rclpy.init()
    player = JointCsvPlayer()

    try:
        for i, joints in enumerate(rows):
            left = [joints[f'l_j{j}'] for j in range(1, 5)]
            right = [joints[f'r_j{j}'] for j in range(1, 5)]
            player.get_logger().info(
                f'[{i + 1}/{len(rows)}] left={left}, right={right}')

            start = time.monotonic()
            future = player.send_request(joints)
            rclpy.spin_until_future_complete(player, future)

            result = future.result()
            player.get_logger().info(
                f'  Response: success={result.success}, message={result.message}')

            if not result.success:
                player.get_logger().warn('Service returned failure, stopping playback.')
                break

            # Wait for the remainder of the period if the service call was fast
            elapsed = time.monotonic() - start
            remaining = period - elapsed
            if remaining > 0 and i < len(rows) - 1:
                time.sleep(remaining)

    except KeyboardInterrupt:
        player.get_logger().info('Playback interrupted by user.')

    player.get_logger().info('Playback complete.')
    player.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
