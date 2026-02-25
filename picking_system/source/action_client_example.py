#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from api_interface.action import Navigate


class NavigateActionClient(Node):
    def __init__(self):
        super().__init__('navigate_action_client')
        self._action_client = ActionClient(self, Navigate, 'navigate')

    def send_goal(self, nav_command, target_upc):
        """Send a navigation goal to the action server"""
        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Navigate.Goal()
        goal_msg.nav_command = nav_command
        goal_msg.target_upc = target_upc

        self.get_logger().info(f'Sending goal: {nav_command} to UPC {target_upc}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle action result"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback - Status: {feedback.status}, Nav Request Count: {feedback.nav_request_count}'
        )


def main(args=None):
    rclpy.init(args=args)

    action_client = NavigateActionClient()

    # Get nav command and target value from command line or use defaults
    import sys
    # Commands that don't require a second argument
    no_value_commands = ['PICK']

    if len(sys.argv) > 2:
        nav_command = sys.argv[1]
        target_value = sys.argv[2]
    elif len(sys.argv) > 1 and sys.argv[1] in no_value_commands:
        nav_command = sys.argv[1]
        target_value = '1'
    elif len(sys.argv) > 1:
        nav_command = 'NAV_REQ'  # Default nav command
        target_value = sys.argv[1]
    else:
        # Print usage information
        print('Usage: ros2 run myaction_server action_client_example <command> <value>')
        print('Commands:')
        print('  NAV_REQ <upc>     - Navigate to UPC location (e.g., NAV_REQ 12345)')
        print('  NUDGE <distance>  - Nudge by decimal distance (e.g., NUDGE 0.5)')
        print('  ROTATE <angle>    - Rotate by decimal angle (e.g., ROTATE 90.0)')
        print('  UPC <upc>         - Verify UPC, get target height & distance')
        print('                      (e.g., UPC 078000053166_831)')
        print('                      Returns: VALID/INVALID UPC, TARGET HEIGHT, TARGET DISTANCE')
        print('  PICK              - Get UPCs and bounding boxes from pick camera')
        print('                      Returns: PICK_RES: sku1,xmin,ymin,xmax,ymax ...')
        print('')
        print('Using defaults: NAV_REQ 12345')
        nav_command = 'NAV_REQ'  # Default nav command
        target_value = '12345'  # Default target UPC

    action_client.send_goal(nav_command, target_value)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
