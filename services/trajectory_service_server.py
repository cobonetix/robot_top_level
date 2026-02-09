#!/usr/bin/env python3
"""ROS2 service that loads named trajectories from a CSV file and sends
the selected trajectory's joint values to the arm_joint service."""

import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from cobonetix_interfaces.srv import ArmJoint, TrajectorySelect

JOINT_FIELDS = ['l_j1', 'l_j2', 'l_j3', 'l_j4', 'r_j1', 'r_j2', 'r_j3', 'r_j4']
DEG_TO_RAD_FIELDS = {'l_j2', 'l_j3', 'l_j4', 'r_j2', 'r_j3', 'r_j4'}

DATA_DIR = Path(__file__).resolve().parent.parent / 'data'
CSV_FILE = DATA_DIR / 'trajectories.csv'


def load_trajectories(filepath):
    """Load trajectories from a CSV file.

    Expected format — first column is the integer trajectory ID,
    remaining 8 columns are joint values:
        id, l_j1, l_j2, l_j3, l_j4, r_j1, r_j2, r_j3, r_j4
    First row is a header and is skipped.
    Multiple rows may share the same ID — they form the ordered steps
    of that trajectory.
    Degree-valued fields (l_j2-l_j4, r_j2-r_j4) are converted to radians.
    """
    trajectories = {}
    with open(filepath) as f:
        content = f.read().replace('"', '')
    lines = content.strip().splitlines()
    for line_num, line in enumerate(lines[1:], start=2):
        vals = line.split(',')
        if len(vals) < 9:
            raise ValueError(
                f"CSV line {line_num}: expected 9 columns (id + 8 joints), got {len(vals)}")
        try:
            traj_id = int(vals[0])
            joints = {}
            for field, val in zip(JOINT_FIELDS, vals[1:]):
                v = float(val)
                if field in DEG_TO_RAD_FIELDS:
                    v = math.radians(v)
                joints[field] = v
        except (ValueError, TypeError) as e:
            raise ValueError(f"Invalid value on CSV line {line_num}: {e}")
        trajectories.setdefault(traj_id, []).append(joints)
    return trajectories


class TrajectoryServiceServer(Node):
    def __init__(self):
        super().__init__('trajectory_service_server')

        # Load trajectories from CSV
        self.get_logger().info(f'Loading trajectories from {CSV_FILE}')
        self.trajectories = load_trajectories(CSV_FILE)
        for tid, steps in self.trajectories.items():
            self.get_logger().info(f'  Trajectory {tid}: {len(steps)} steps')
        self.get_logger().info(
            f'Loaded {len(self.trajectories)} trajectories: '
            f'{sorted(self.trajectories.keys())}')

        # Client to the arm_joint service
        self.joint_client = self.create_client(ArmJoint, 'arm_joint')
        while not self.joint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arm_joint service...')
        self.get_logger().info('Connected to arm_joint service')

        # Trajectory selection service
        self.srv = self.create_service(
            TrajectorySelect, 'trajectory_select', self.handle_trajectory_select)
        self.get_logger().info('trajectory_select service is ready')

    def handle_trajectory_select(self, request, response):
        traj_id = request.trajectory_id

        if traj_id not in self.trajectories:
            response.success = False
            response.message = (
                f'Trajectory {traj_id} not found. '
                f'Available: {sorted(self.trajectories.keys())}')
            self.get_logger().warn(response.message)
            return response

        steps = self.trajectories[traj_id]
        total = len(steps)
        delay = request.step_delay_sec
        self.get_logger().info(
            f'Playing trajectory {traj_id} ({total} steps, '
            f'step_delay={delay:.2f}s)')

        for i, joints in enumerate(steps):
            if delay > 0 and i > 0:
                self.get_logger().info(f'  Waiting {delay:.2f}s before step {i + 1}')
                time.sleep(delay)
            self.get_logger().info(
                f'  [{i + 1}/{total}] {joints}')

            arm_request = ArmJoint.Request()
            arm_request.l_j1 = joints['l_j1']
            arm_request.l_j2 = joints['l_j2']
            arm_request.l_j3 = joints['l_j3']
            arm_request.l_j4 = joints['l_j4']
            arm_request.r_j1 = joints['r_j1']
            arm_request.r_j2 = joints['r_j2']
            arm_request.r_j3 = joints['r_j3']
            arm_request.r_j4 = joints['r_j4']

            future = self.joint_client.call_async(arm_request)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()
            self.get_logger().info(
                f'  arm_joint response: success={result.success}, '
                f'message={result.message}')

            if not result.success:
                response.success = False
                response.message = (
                    f'Trajectory {traj_id} failed at step {i + 1}/{total}: '
                    f'{result.message}')
                self.get_logger().warn(response.message)
                return response

        response.success = True
        response.message = (
            f'Trajectory {traj_id} completed successfully ({total} steps)')
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    server = TrajectoryServiceServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Shutting down trajectory service server.')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
