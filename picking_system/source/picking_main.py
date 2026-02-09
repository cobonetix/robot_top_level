#!/usr/bin/env python3
"""
Main entry point for the picking system.

Initializes the ROS2 node and global context, runs the arm initialization
sequence, then processes orders.
"""

import sys

import ros_context
from arm_init_calibrate import run_init_sequence
from picking_utils import process_orders


def main():
    """
    Main entry point. Accepts one command line argument:
    1. Path to the orders list CSV file
    """
    orders_file = "orders.csv"  #sys.argv[1]

    ros_context.init()

    try:
        success = run_init_sequence()
        if not success:
            ros_context.node.get_logger().error('Initialization sequence failed')
            return 1

        process_orders(orders_file)
    except KeyboardInterrupt:
        ros_context.node.get_logger().info('Interrupted by user')
    finally:
        ros_context.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
