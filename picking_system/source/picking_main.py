#!/usr/bin/env python3
"""
Main entry point for the picking system.

Initializes the ROS2 node and global context, runs the arm initialization
sequence, then processes orders.
"""

import sys
from time import sleep

import ros_context
import torch

from arm_init_calibrate import run_init_sequence
from process_orders import do_pick_test, process_all_orders
from label_detect import decodeShelf
from shelf_distance import getShelfDistance, init_detector



shelf_analysis = True # Set to True to run shelf analysis test instead of processing orders
processOrders = True
pickUpc = "000000000000" # UPC of item to pick for test mode, not used if processOrders is True


def main():
    """
    Main entry point. Accepts one command line argument:
    1. Path to the orders list CSV file
    """
    orders_file = "orders.csv"  #sys.argv[1]  
 

    #ros_context.init()

    if shelf_analysis:
      while True:
          shelfYaw = decodeShelf()
          if len(shelfYaw) == 2:
            print(shelfYaw)

            if abs(shelfYaw[0]) < 0.01:
                print('Shelf detected with acceptable yaw angle')
                d = getShelfDistance()
                if d is not None:
                    print(f"Shelf distance: {d[0]['distance_m']:.3f}")
                else:
                    print('No shelf distance detected')

          else:
            print('No shelf detected')
          sleep(1.0)
          
    if processOrders:
        try:
            success = run_init_sequence()
            if not success:
                ros_context.node.get_logger().error('Initialization sequence failed')
                return 1

            if processOrders:
                process_all_orders(orders_file)
        except KeyboardInterrupt:
            ros_context.node.get_logger().info('Interrupted by user')
    else:
        pass
    
        # test_move()
        #est_shelf_analysis(pickUpc)
    ros_context.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
