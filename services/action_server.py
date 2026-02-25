#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from api_interface.action import Navigate
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading
import subprocess
import math
import time
import json
import networkx as nx
from networkx.readwrite import json_graph


class NavigateActionServer(Node):
    def __init__(self):
        super().__init__('navigate_action_server')

        # Create a reentrant callback group to allow callbacks to run concurrently
        self.callback_group = ReentrantCallbackGroup()

        # Create the action server
        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            self.execute_callback,
            callback_group=self.callback_group
        )

        # Create publisher for pos_node topic
        self.pos_node_publisher = self.create_publisher(
            String,
            'pos_node',
            10
        )

        # Subscribe to nav_stat_pub to receive navigation status
        self.nav_status_subscription = self.create_subscription(
            String,
            'nav_stat_pub',
            self.nav_status_callback,
            10,
            callback_group=self.callback_group
        )

        # Create publisher for nav_man_pub topic (to request vision)
        self.nav_man_publisher = self.create_publisher(
            String,
            'nav_man_pub',
            10
        )

        # Create publisher for cmd_vel topic (teleop rotation)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Subscribe to vision_pub to receive vision responses
        self.vision_subscription = self.create_subscription(
            String,
            'vision_pub',
            self.vision_callback,
            10,
            callback_group=self.callback_group
        )

        # Status tracking
        self.current_status = None
        self.nav_request_count = 0
        self.navigation_complete_event = None
        self.goal_handle = None
        self.vision_response = None
        self.vision_response_event = None
        self.vis_nav_req_count = 0
        self.pick_response = None
        self.pick_response_event = None
        self.pick_req_count = 0

        self.get_logger().info('Navigate Action Server is ready.')
        self.get_logger().info('Publisher created for topic: pos_node')
        self.get_logger().info('Publisher created for topic: nav_man_pub')
        self.get_logger().info('Subscribed to topic: nav_stat_pub')
        self.get_logger().info('Subscribed to topic: vision_pub')

        # Download netx_lst.txt from remote server
        self.download_netx_lst()

    def download_netx_lst(self):
        """Download netx_list.txt and netx_nav_list.json files from remote server using scp"""
        local_path = "/home/stan/dev_ws/netx_database/"

        # Download netx_list.txt
        remote_path = "bob@bob-orin:/home/bob/cobo_lab_test_v2/training/netx_list.txt"
        cmd = ["sshpass", "-p", "bob", "scp", "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null", remote_path, local_path]

        self.get_logger().info(f'Downloading netx_list.txt from {remote_path}')
        try:
            subprocess.run(cmd, check=True)
            self.get_logger().info(f'Successfully downloaded netx_list.txt to {local_path}')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to download netx_list.txt: {e}')
        except FileNotFoundError:
            self.get_logger().error('sshpass command not found')

        # Download netx_nav_list.json
        remote_path_json = "bob@bob-orin:/home/bob/cobo_lab_test_v2/training/netx_nav_list.json"
        cmd_json = ["sshpass", "-p", "bob", "scp", "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null", remote_path_json, local_path]

        self.get_logger().info(f'Downloading netx_nav_list.json from {remote_path_json}')
        try:
            subprocess.run(cmd_json, check=True)
            self.get_logger().info(f'Successfully downloaded netx_nav_list.json to {local_path}')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to download netx_nav_list.json: {e}')
        except FileNotFoundError:
            self.get_logger().error('sshpass command not found')

        # Download netx_nav_only_graph.txt
        remote_path_nav_only = "bob@bob-orin:/home/bob/cobo_lab_test_v2/training/netx_nav_only_graph.txt"
        cmd_nav_only = ["sshpass", "-p", "bob", "scp", "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null", remote_path_nav_only, local_path]

        self.get_logger().info(f'Downloading netx_nav_only_graph.txt from {remote_path_nav_only}')
        try:
            subprocess.run(cmd_nav_only, check=True)
            self.get_logger().info(f'Successfully downloaded netx_nav_only_graph.txt to {local_path}')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to download netx_nav_only_graph.txt: {e}')
        except FileNotFoundError:
            self.get_logger().error('sshpass command not found')

    def vision_callback(self, msg):
        """Callback to receive vision server responses"""
        self.get_logger().info(f'Vision response received: {msg.data}')

        if msg.data.startswith('VIS_NAV_RES:'):
            self.vision_response = msg.data
            if self.vision_response_event is not None:
                self.vision_response_event.set()

        if msg.data.startswith('VIS_PICK_RES:'):
            self.pick_response = msg.data
            if self.pick_response_event is not None:
                self.pick_response_event.set()

    def search_netx_list_by_short_upc(self, short_upc):
        """Find all full UPCs in netx_list.txt whose prefix (before _) matches short_upc.
        Returns list of unique full UPCs."""
        netx_file_path = "/home/stan/dev_ws/netx_database/netx_list.txt"
        matches = []
        try:
            with open(netx_file_path, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        full_upc = parts[1].strip()
                        if not full_upc:
                            continue
                        prefix = full_upc.split('_')[0] if '_' in full_upc else full_upc
                        if prefix == short_upc and full_upc not in matches:
                            matches.append(full_upc)
            self.get_logger().info(f'Short UPC "{short_upc}" matched {len(matches)} entries: {matches}')
        except FileNotFoundError:
            self.get_logger().error(f'netx_list.txt not found at {netx_file_path}')
        except Exception as e:
            self.get_logger().error(f'Error searching netx_list.txt: {e}')
        return matches

    def match_partial_upc_to_full(self, partial_upc):
        """Match partial UPC (before underscore) to full UPC in netx_nav_only_graph.txt"""
        netx_file_path = "/home/stan/dev_ws/netx_database/netx_nav_only_graph.txt"
        try:
            with open(netx_file_path, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        full_upc = parts[1].strip()
                        # Get the part before the underscore from the full UPC
                        upc_prefix = full_upc.split('_')[0] if '_' in full_upc else full_upc
                        if upc_prefix == partial_upc:
                            self.get_logger().info(f'Matched partial UPC {partial_upc} to full UPC {full_upc}')
                            return full_upc
            self.get_logger().info(f'No match found for partial UPC {partial_upc}')
            return None
        except FileNotFoundError:
            self.get_logger().error(f'netx_nav_only_graph.txt not found at {netx_file_path}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error matching partial UPC: {e}')
            return None

    def get_current_upc(self):
        """Request current location UPC from vision server and wait for response"""
        import time

        # Create event to wait for vision response
        self.vision_response_event = threading.Event()
        self.vision_response = None

        # Increment request count and publish VIS_NAV_REQ
        # Retry up to 3 times with re-publish every 10 seconds if no response
        self.vis_nav_req_count += 1
        msg = String()
        msg.data = f'VIS_NAV_REQ: {self.vis_nav_req_count}'

        max_retries = 3
        retry_interval = 10.0
        timeout = max_retries * retry_interval

        for attempt in range(max_retries):
            self.nav_man_publisher.publish(msg)
            self.get_logger().info(f'Published "{msg.data}" to nav_man_pub (attempt {attempt + 1}/{max_retries})')

            # Wait for vision response
            wait_start = time.time()
            while (time.time() - wait_start) < retry_interval:
                if self.vision_response_event.is_set():
                    break
                time.sleep(0.5)

            if self.vision_response_event.is_set():
                break

            self.get_logger().warn(f'No vision response after attempt {attempt + 1}, retrying...')

        # Clean up
        self.vision_response_event = None

        if self.vision_response is None:
            self.get_logger().error(f'No vision response received after {max_retries} attempts')
            return None

        # Parse response: 'VIS_NAV_RES: sku1 sku2 sku3 sku4 sku5'
        parts = self.vision_response.split()
        if len(parts) >= 2:
            # Try each UPC from vision response (index 1 onwards) until a match is found
            for i in range(1, len(parts)):
                partial_upc = parts[i]
                self.get_logger().info(f'Trying partial UPC [{i}]: {partial_upc}')

                # Match partial UPC to full UPC in netx_list.txt
                full_upc = self.match_partial_upc_to_full(partial_upc)
                if full_upc is not None:
                    self.get_logger().info(f'Matched UPC [{i}]: {partial_upc} -> {full_upc}')
                    return full_upc
                else:
                    self.get_logger().info(f'No match for UPC [{i}]: {partial_upc}, trying next...')

            self.get_logger().warn(f'No match found for any vision UPC: {parts[1:]}')
            return None

        self.get_logger().error('Failed to parse vision response')
        return None

    def calculate_path_distance(self, start_upc, target_upc):
        """Calculate shortest path distance between two UPCs using networkx"""
        netx_json_path = "/home/stan/dev_ws/netx_database/netx_nav_list.json"

        try:
            with open(netx_json_path, 'r') as f:
                json_data = json.load(f)

            graph = json_graph.node_link_graph(json_data)

            # Try to find shortest path
            try:
                path = nx.shortest_path(graph, start_upc, target_upc, 'bfs')
                path_length = len(path)
                self.get_logger().info(f'Shortest path from {start_upc} to {target_upc}: {path}')
                self.get_logger().info(f'Path length: {path_length}')
                return path_length
            except nx.NetworkXNoPath:
                self.get_logger().info(f'No path found from {start_upc} to {target_upc}')
                return -1
            except nx.NodeNotFound as e:
                self.get_logger().error(f'Node not found in graph: {e}')
                return -1

        except FileNotFoundError:
            self.get_logger().error(f'netx_nav_list.json not found at {netx_json_path}')
            return -1
        except Exception as e:
            self.get_logger().error(f'Error calculating path: {e}')
            return -1

    def verify_upc(self, upc):
        """Verify UPC against netx_list.txt file (second comma-separated value)"""
        netx_file_path = "/home/stan/dev_ws/netx_database/netx_list.txt"
        try:
            with open(netx_file_path, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        file_upc = parts[1].strip()
                        if file_upc == upc:
                            self.get_logger().info(f'UPC {upc} found in netx_list.txt')
                            return True
            self.get_logger().info(f'UPC {upc} not found in netx_list.txt')
            return False
        except FileNotFoundError:
            self.get_logger().error(f'netx_list.txt not found at {netx_file_path}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error reading netx_list.txt: {e}')
            return False

    def get_bottom_upc(self, upc):
        """Get the bottom UPC (5th field) for a given UPC (2nd field) from netx_list.txt"""
        netx_file_path = "/home/stan/dev_ws/netx_database/netx_list.txt"
        try:
            with open(netx_file_path, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) >= 5:
                        file_upc = parts[1].strip()
                        if file_upc == upc:
                            bottom_upc = parts[4].strip()
                            return bottom_upc if bottom_upc else None
            return None
        except FileNotFoundError:
            self.get_logger().error(f'netx_list.txt not found at {netx_file_path}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error reading netx_list.txt: {e}')
            return None

    def calculate_target_height(self, target_upc):
        """Calculate target height by iteratively following bottom UPC chain in netx_list.txt.
        Start with target UPC, look up its bottom UPC (3rd field), then search for that
        bottom UPC as 2nd field, repeat until null. Count the iterations."""
        height = 0
        current_upc = target_upc
        visited = set()  # Prevent infinite loops

        while current_upc and current_upc not in visited:
            visited.add(current_upc)
            bottom_upc = self.get_bottom_upc(current_upc)
            self.get_logger().info(f'Height search [{height}]: {current_upc} -> bottom: {bottom_upc}')
            if bottom_upc is None:
                break
            height += 1
            current_upc = bottom_upc

        self.get_logger().info(f'Target height for {target_upc} = {height}')
        return height

    def get_image_number_from_upc(self, upc):
        """Get image number (first field) from UPC by looking up in netx_list.txt"""
        netx_file_path = "/home/stan/dev_ws/netx_database/netx_list.txt"
        try:
            with open(netx_file_path, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        file_upc = parts[1].strip()
                        if file_upc == upc:
                            image_number = parts[0].strip()
                            self.get_logger().info(f'Found image number {image_number} for UPC {upc}')
                            return image_number
            self.get_logger().info(f'No image number found for UPC {upc}')
            return None
        except FileNotFoundError:
            self.get_logger().error(f'netx_list.txt not found at {netx_file_path}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error reading netx_list.txt: {e}')
            return None

    def get_nav_upcs_from_image(self, image_number):
        """Get navigation UPCs (second field) from image number by looking up in netx_nav_only_graph.txt"""
        netx_nav_only_path = "/home/stan/dev_ws/netx_database/netx_nav_only_graph.txt"
        nav_upcs = []
        try:
            with open(netx_nav_only_path, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        file_image = parts[0].strip()
                        if file_image == image_number:
                            nav_upc = parts[1].strip()
                            nav_upcs.append(nav_upc)
            self.get_logger().info(f'Found {len(nav_upcs)} navigation UPCs for image {image_number}: {nav_upcs}')
            return nav_upcs
        except FileNotFoundError:
            self.get_logger().error(f'netx_nav_only_graph.txt not found at {netx_nav_only_path}')
            return []
        except Exception as e:
            self.get_logger().error(f'Error reading netx_nav_only_graph.txt: {e}')
            return []

    def nav_status_callback(self, msg):
        """Callback to receive navigation manager status updates"""
        self.get_logger().info(f'Nav status received: {msg.data}')
        self.get_logger().info(f'Event object is: {self.navigation_complete_event}')

        self.current_status = msg.data

        # Check if it's a vision request (VIS_NAV_REQ: N)
        if msg.data.startswith('STARTING NAVIGATION'):
            try:
                # Extract the request count
                parts = msg.data.split(':')
                if len(parts) > 1:
                    self.nav_request_count = int(parts[1].strip())
                    self.get_logger().info(f'Vision request count: {self.nav_request_count}')
            except ValueError:
                pass

        # Check if navigation reached destination
        if msg.data == 'REACHED DESTINATION':
            self.get_logger().info('REACHED DESTINATION detected!')
            self.get_logger().info(f'Event is None: {self.navigation_complete_event is None}')
            self.get_logger().info(f'Event value: {self.navigation_complete_event}')
            if self.navigation_complete_event is not None:
                self.get_logger().info('Setting completion event!')
                self.navigation_complete_event.set()
                self.get_logger().info(f'Event is_set after setting: {self.navigation_complete_event.is_set()}')
            else:
                self.get_logger().info('Completion event is None - no navigation waiting (likely ROTATE or completed command)')

    def execute_callback(self, goal_handle):
        """Execute the navigation action"""
        self.get_logger().info('Executing navigation goal...')
        self.goal_handle = goal_handle

        # Get the nav command and target UPC from the goal
        nav_command = goal_handle.request.nav_command
        target_upc = goal_handle.request.target_upc
        self.get_logger().info(f'Nav Command: {nav_command}')
        self.get_logger().info(f'Target UPC: {target_upc}')

        # Check if nav_command is valid
        if nav_command not in ["NAV_REQ", "NUDGE", "ROTATE", "UPC", "PICK"]:
            self.get_logger().error(f'Invalid nav command: {nav_command}. Only "NAV_REQ", "NUDGE", "ROTATE", "UPC", or "PICK" are accepted.')
            goal_handle.abort()
            result = Navigate.Result()
            result.result = f'Navigation rejected - Invalid command "{nav_command}". Only "NAV_REQ", "NUDGE", "ROTATE", "UPC", or "PICK" are accepted.'
            return result

        # Handle UPC verification command
        if nav_command == "UPC":
            result = Navigate.Result()

            # Detect short UPC: no underscore, or suffix after underscore is not a digit
            is_short_upc = ('_' not in target_upc) or (not target_upc.split('_')[-1].isdigit())

            if is_short_upc:
                # --- Short UPC: find all matching full UPCs in netx_list.txt ---
                matches = self.search_netx_list_by_short_upc(target_upc)
                if not matches:
                    goal_handle.succeed()
                    result.result = f"NO MATCHES FOUND FOR SHORT UPC {target_upc}"
                    self.get_logger().info(f'No matches found for short UPC {target_upc}')
                    return result

                # Get current location once for all distance calculations
                current_upc = self.get_current_upc()

                match_results = []
                for full_upc in matches:
                    height = self.calculate_target_height(full_upc)

                    path_distance = -1
                    if current_upc is not None:
                        image_number = self.get_image_number_from_upc(full_upc)
                        if image_number is not None:
                            nav_upcs = self.get_nav_upcs_from_image(image_number)
                            for nav_upc in nav_upcs:
                                d = self.calculate_path_distance(current_upc, nav_upc)
                                if d > 0:
                                    path_distance = d
                                    break

                    match_results.append(f"{full_upc}:H={height}:D={path_distance}")
                    self.get_logger().info(f'Match {full_upc}: height={height}, distance={path_distance}')

                goal_handle.succeed()
                result.result = "MATCHES: " + ", ".join(match_results)
                self.get_logger().info(f'Short UPC result: {result.result}')
                return result

            # --- Full UPC: existing exact-match logic ---

            # First verify if target UPC exists in database
            if not self.verify_upc(target_upc):
                goal_handle.succeed()
                result.result = "INVALID UPC"
                self.get_logger().info(f'UPC {target_upc} is invalid')
                return result

            # Calculate target height
            target_height = self.calculate_target_height(target_upc)

            # Get image number from target UPC
            image_number = self.get_image_number_from_upc(target_upc)
            if image_number is None:
                goal_handle.succeed()
                result.result = f"VALID UPC, TARGET HEIGHT = {target_height}, NO IMAGE FOUND"
                self.get_logger().info(f'UPC {target_upc} is valid but no image number found')
                return result

            # Get navigation UPCs from image number
            nav_upcs = self.get_nav_upcs_from_image(image_number)
            if not nav_upcs:
                goal_handle.succeed()
                result.result = f"VALID UPC, TARGET HEIGHT = {target_height}, NO NAV UPCS FOUND"
                self.get_logger().info(f'UPC {target_upc} is valid but no navigation UPCs found for image {image_number}')
                return result

            # Get current location UPC from vision server
            current_upc = self.get_current_upc()
            if current_upc is None:
                goal_handle.succeed()
                result.result = f"VALID UPC, TARGET HEIGHT = {target_height}, NO CURRENT LOCATION"
                self.get_logger().info(f'UPC {target_upc} is valid but could not get current location')
                return result

            # Try shortest path to each navigation UPC until one succeeds
            path_distance = -1
            successful_nav_upc = None
            for nav_upc in nav_upcs:
                self.get_logger().info(f'Trying shortest path from {current_upc} to {nav_upc}')
                distance = self.calculate_path_distance(current_upc, nav_upc)
                if distance > 0:
                    path_distance = distance
                    successful_nav_upc = nav_upc
                    self.get_logger().info(f'Found valid path to {nav_upc} with distance {distance}')
                    break

            if path_distance == -1:
                goal_handle.succeed()
                result.result = f"VALID UPC, TARGET HEIGHT = {target_height}, NO PATH FOUND"
                self.get_logger().info(f'UPC {target_upc} is valid but no path found from {current_upc} to any nav UPC')
            else:
                goal_handle.succeed()
                result.result = f"VALID UPC, TARGET HEIGHT = {target_height}, TARGET DISTANCE = {path_distance}"
                self.get_logger().info(f'UPC {target_upc} is valid, height={target_height}, distance from {current_upc} to {successful_nav_upc} = {path_distance}')

            return result

        # Handle PICK command
        if nav_command == "PICK":
            import time
            result = Navigate.Result()

            # Create event to wait for pick response
            self.pick_response_event = threading.Event()
            self.pick_response = None

            # Send PICK_REQ to vision server
            self.pick_req_count += 1
            msg = String()
            msg.data = f'PICK_REQ: {self.pick_req_count}'

            max_retries = 3
            retry_interval = 10.0

            for attempt in range(max_retries):
                self.nav_man_publisher.publish(msg)
                self.get_logger().info(f'Published "{msg.data}" to nav_man_pub (attempt {attempt + 1}/{max_retries})')

                # Wait for pick response
                wait_start = time.time()
                while (time.time() - wait_start) < retry_interval:
                    if self.pick_response_event.is_set():
                        break
                    time.sleep(0.5)

                if self.pick_response_event.is_set():
                    break

                self.get_logger().warn(f'No pick response after attempt {attempt + 1}, retrying...')

            # Clean up
            self.pick_response_event = None

            if self.pick_response is None:
                self.get_logger().error(f'No pick response received after {max_retries} attempts')
                goal_handle.succeed()
                result.result = "PICK FAILED - NO VISION RESPONSE"
                return result

            # Pass through the vision server response (strip VIS_PICK_RES: prefix)
            pick_data = self.pick_response.replace('VIS_PICK_RES: ', '', 1)
            goal_handle.succeed()
            result.result = f"PICK_RES: {pick_data}"
            self.get_logger().info(f'PICK result: {result.result}')
            return result

        # Publish message to pos_node topic
        msg = String()
        if nav_command == "NUDGE":
            # Validate that target_upc contains a valid decimal number
            try:
                nudge_value = float(target_upc)
                msg.data = f"NUDGE: {nudge_value}"
            except ValueError:
                self.get_logger().error(f'Invalid NUDGE value: {target_upc}. Must be a decimal number.')
                goal_handle.abort()
                result = Navigate.Result()
                result.result = f'NUDGE rejected - Invalid value "{target_upc}". Must be a decimal number.'
                return result
        elif nav_command == "ROTATE":
            # Validate that target_upc contains a valid decimal number (degrees)
            try:
                degrees = float(target_upc)
            except ValueError:
                self.get_logger().error(f'Invalid ROTATE value: {target_upc}. Must be a decimal number (degrees).')
                goal_handle.abort()
                result = Navigate.Result()
                result.result = f'ROTATE rejected - Invalid value "{target_upc}". Must be a decimal number (degrees).'
                return result

            # Execute rotation using a separate process with its own rclpy context
            import os
            script_path = os.path.join(os.path.dirname(__file__), 'rotate_publisher.py')
            self.get_logger().info(f'Executing ROTATE: {degrees} degrees via {script_path}')
            subprocess.run(['python3', script_path, str(degrees)])

            self.get_logger().info('ROTATE completed')
            goal_handle.succeed()
            result = Navigate.Result()
            result.result = f'ROTATE completed - {degrees} degrees'
            return result
        else:
            msg.data = f"{nav_command}: {target_upc}"
        self.pos_node_publisher.publish(msg)
        self.get_logger().info(f'Published "{msg.data}" to pos_node topic')

        # Create event to wait for navigation completion
        self.navigation_complete_event = threading.Event()
        self.get_logger().info(f'Created navigation_complete_event: {self.navigation_complete_event}')
        self.get_logger().info(f'Event is_set initially: {self.navigation_complete_event.is_set()}')
        self.current_status = None
        self.nav_request_count = 0

        # Feedback message
        feedback_msg = Navigate.Feedback()

        # Wait for REACHED DESTINATION message (with timeout)
        timeout_seconds = 300.0  # 5 minutes timeout
        import time
        start_time = time.time()
        completed = False

        loop_count = 0
        while (time.time() - start_time) < timeout_seconds:
            loop_count += 1

            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal cancelled by client')
                result = Navigate.Result()
                result.result = 'Navigation cancelled'
                return result

            # Check if navigation is complete (with None check)
            if self.navigation_complete_event is not None and self.navigation_complete_event.is_set():
                self.get_logger().info('Navigation complete - event is set!')
                completed = True
                break

            # Defensive check: if event became None, something went wrong
            if self.navigation_complete_event is None:
                self.get_logger().error('Event became None during execution - aborting')
                break

            # Log event status periodically
            if loop_count % 10 == 0:
                elapsed = time.time() - start_time
                event_status = self.navigation_complete_event.is_set() if self.navigation_complete_event else 'None'
                self.get_logger().info(f'Loop {loop_count}: Event is_set={event_status}, elapsed={elapsed:.1f}s')

            # Publish feedback
            feedback_msg.status = self.current_status if self.current_status else 'Navigating...'
            feedback_msg.nav_request_count = self.nav_request_count
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback - Status: {feedback_msg.status}, Count: {feedback_msg.nav_request_count}')

            # Sleep briefly to allow other callbacks to run
            time.sleep(1.0)

        # Clean up
        self.navigation_complete_event = None
        self.goal_handle = None

        # Set result
        result = Navigate.Result()
        if completed:
            goal_handle.succeed()
            result.result = 'Navigation completed - Destination reached'
            self.get_logger().info('Navigation succeeded')
        else:
            goal_handle.abort()
            result.result = f'Navigation timeout - Did not reach destination after {timeout_seconds}s'
            self.get_logger().warn(f'Navigation timed out')

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = NavigateActionServer()

    # Use MultiThreadedExecutor to allow callbacks during action execution
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
