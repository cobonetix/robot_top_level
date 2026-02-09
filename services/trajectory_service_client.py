#!/usr/bin/env python3
"""Test client for the trajectory_select service."""

import argparse

import rclpy
from rclpy.node import Node
from cobonetix_interfaces.srv import TrajectorySelect


class TrajectoryServiceClient(Node):
    def __init__(self):
        super().__init__('trajectory_service_client')
        self.client = self.create_client(TrajectorySelect, 'trajectory_select')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for trajectory_select service...')
        self.get_logger().info('Connected to trajectory_select service')

    def send_request(self, trajectory_id, step_delay_sec=0.0):
        request = TrajectorySelect.Request()
        request.trajectory_id = trajectory_id
        request.step_delay_sec = step_delay_sec
        future = self.client.call_async(request)
        return future


def main():
    parser = argparse.ArgumentParser(description='Test client for trajectory_select service')
    parser.add_argument('trajectory_id', nargs='?', type=int, default=None,
                        help='Trajectory ID to request (omit for interactive mode)')
    parser.add_argument('--delay', type=float, default=0.0,
                        help='Seconds to wait between each trajectory step (default: 0)')
    cli_args = parser.parse_args()

    rclpy.init()
    client = TrajectoryServiceClient()

    if cli_args.trajectory_id is not None:
        # Single-shot mode
        tid = cli_args.trajectory_id
        print(f'Requesting trajectory {tid} (step_delay={cli_args.delay}s)')
        future = client.send_request(tid, cli_args.delay)
        rclpy.spin_until_future_complete(client, future)
        result = future.result()
        print(f'Response: success={result.success}, message={result.message}')
    else:
        # Interactive mode
        while True:
            try:
                user_input = input('Enter trajectory ID (or q to quit): ').strip()
                if user_input.lower() == 'q':
                    break
                tid = int(user_input)
                print(f'Requesting trajectory {tid} (step_delay={cli_args.delay}s)')
                future = client.send_request(tid, cli_args.delay)
                rclpy.spin_until_future_complete(client, future)
                result = future.result()
                print(f'Response: success={result.success}, message={result.message}')
            except ValueError:
                print('Invalid input. Enter an integer trajectory ID.')
            except KeyboardInterrupt:
                break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
