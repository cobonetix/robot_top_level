#!/usr/bin/env python3
"""
Main entry point for the picking system.

Initializes the ROS2 node and global context, runs the arm initialization
sequence, then processes orders.
"""

import sys
import os
import csv
import math


import ros_context
from picking_utils import doPick, get_joint_poses, send_navigate_goal, send_trajectory_request, send_joint_request, wait_for_arm_idle
from cobonetix_interfaces.srv import ArmJoint, GpioStatus, TrajectorySelect
from api_interface.action import Navigate
from dataclasses import dataclass
from label_detect import decodeShelf


image_width = 1080
image_height = 1920

@dataclass
class QuantitySku:
    quantity: int
    sku: str
    nav: str = ""
    distance: float = float('inf')  # Default to infinity for sorting purposes
    height: float = 0
    valid: bool = True
    
class UpcMatch:
    """Holds a single UPC match returned by the action server."""
    def __init__(self, upc: str, height: int, distance: int):
        self.upc      = upc
        self.height   = height
        self.distance = distance

    def __repr__(self):
        return f"UpcMatch(upc='{self.upc}', height={self.height}, distance={self.distance})"

def _resolve_path(file_path: str) -> str:
    """Resolve a filename relative to DATA_DIR if it is not already absolute."""
    if os.path.isabs(file_path):
        return file_path
    return str(ros_context.DATA_DIR / file_path)


def get_items_to_order(file_path: str) -> list[QuantitySku]:
    """
    Read a CSV file with quantity and SKU columns into a list of objects.

    Args:
        file_path: Path or filename (resolved relative to DATA_DIR)

    Returns:
        List of QuantitySku objects
    """
    logger = ros_context.node.get_logger()
    result = []
    with open(_resolve_path(file_path), 'r', newline='') as f:
        logger.info(f'Opening order file: {f.name}')
        reader = csv.reader(f)
        next(reader, None)  # Skip header row
        for row in reader:
            obj = QuantitySku(
                quantity=int(row[0]),
                sku=row[1]
            )
            result.append(obj)
            logger.info(f'Parsed order item: quantity={obj.quantity}, sku={obj.sku}')
    return result
    

def load_all_orders(file_path: str) -> list[str]:
    """
    Read a CSV file with rows of strings into a list.

    Args:
        file_path: Path or filename (resolved relative to DATA_DIR)

    Returns:
        List of strings
    """
    result = []
    with open(_resolve_path(file_path), 'r', newline='') as f:
        reader = csv.reader(f)
        next(reader, None)  # Skip header row
        for row in reader:
            result.append(row[0])
    return result


def parse_upc_matches(result_str: str) -> list:
    """
    Parse the action server result string into a list of UpcMatch objects.

    Expected format:
        "MATCHES: 051651093866_0:H=2:D=5, 051651093866_3:H=0:D=8"

    Returns an empty list for any other result format.
    """
    matches = []
    if not result_str.startswith("MATCHES:"):
        return matches

    payload = result_str[len("MATCHES:"):].strip()
    for token in payload.split(","):
        token = token.strip()
        if not token:
            continue
        # token format:  <full_upc>:H=<n>:D=<n>
        try:
            parts = token.split(":")
            # parts[0] = full_upc, parts[1] = H=n, parts[2] = D=n
            upc      = parts[0].strip()
            height   = int(parts[1].split("=")[1])
            distance = int(parts[2].split("=")[1])
            matches.append(UpcMatch(upc, height, distance))
        except (IndexError, ValueError) as e:
            print(f"[WARN] Could not parse token '{token}': {e}")     
    return matches



def sort_items(items: list[QuantitySku]) -> list[QuantitySku]:
    """
    Sort a list of QuantitySku objects.

    Args:
        items: List of QuantitySku objects to sort

    Returns:
        Sorted list of QuantitySku objects
    """

    for item in items:
        result = send_navigate_goal('UPC', item.sku)
        if result is not None:
            try:
                matches = parse_upc_matches(result)
                item.valid = False  # Assume invalid until we find a match with distance > 0

                for m in matches:
                    if m.distance != int(-1) :   # Only consider matches that are reachable (distance > 0)
                        if m.distance < item.distance:
                            item.distance = m.distance
                            item.height = m.height
                            item.valid = True
                            item.nav = m.upc
                            ros_context.node.get_logger().info(f'Updated item {item.sku} with match {m.upc}, height={m.height}, distance={m.distance}')
                        else:
                            ros_context.node.get_logger().info(f'Ignoring match {m.upc} for item {item.sku} due to greater distance (height={m.height}, distance={m.distance})')
                    else:
                        ros_context.node.get_logger().info(f'Unreachable item {item.sku} with match {m.upc}, height={m.height}, distance={m.distance}')
            except ValueError:
                ros_context.node.get_logger().warn(f'Invalid distance result for SKU {item.sku}: {result}')
                item.distance = float('inf')
                item.valid = False
        else:
            ros_context.node.get_logger().warn(f'No distance result for SKU {item.sku}')
            item.distance = float('inf')
            item.valid = False

    items.sort(key=lambda item: item.distance)
    return items




def process_an_order(file_name: str) -> list[QuantitySku]:
    
    items_in_this_order = get_items_to_order(file_name)
    ordered_items = sort_items(items_in_this_order)
    
    
    for item in ordered_items:
        print(item)
        if item.valid:
            fetch_item(item)
        else:
            ros_context.node.get_logger().warn(f'Skipping invalid item SKU {item.sku} in order {file_name}')


def process_all_orders(product_list: str) -> list[QuantitySku]:
    """
    Process orders from order files.

    Args:
        product_list: Path to a CSV file containing file names to read

    Returns:
        List of QuantitySku objects from all files
    """
    file_names = load_all_orders(product_list)
    for file_name in file_names:
       result = process_an_order(file_name)

    return result

def test_move():

  while True:
    """ result = send_navigate_goal('NUDGE', str( 0.3))

    if result is None:
        ros_context.node.get_logger().info(f'No Nudge result ')
        return None
    
    ros_context.node.get_logger().info(f'Nudge1 result: {result}')

    """


    result = send_navigate_goal('ROTATE', str(45))

    if result is None:
        ros_context.node.get_logger().info(f'No Rotate result ')
        return None

    ros_context.node.get_logger().info(f'Rotate result1: {result}')

    result = send_navigate_goal('ROTATE', str(-45))

    if result is None:
        ros_context.node.get_logger().info(f'No Rotate2 result ')
        return None

    ros_context.node.get_logger().info(f'Rotate result2: {result}')

def test_shelf_analysis(sku: str):
    box = doPick(sku)
    if box is None:
        ros_context.node.get_logger().info(f'No Pick result for SKU {sku}')
        return None

    ros_context.node.get_logger().info(f'Pick result for SKU {sku}: {box}')

    bbx_center_x = (box[0] + box[2]) / 2
    bbx_center_y = (box[1] + box[3]) / 2

    ros_context.node.get_logger().info(f'Bounding box center for SKU {sku}: ({bbx_center_x}, {bbx_center_y})')
     
    # now calcuate the yaw relative to the shelf edge

    shelfYaw = shelf_detect()

    ros_context.node.get_logger().info(f'Shelf yaw detected: {shelfYaw}')

def navigate_to_item(item: QuantitySku):
    
    result = send_navigate_goal('NAV_REQ', item.nav)
    
    if result is None:
        ros_context.node.get_logger().info(f'No Navigation result for SKU {item.sku}')
        return False
    
    ros_context.node.get_logger().info(f'Navigation result for SKU {item.sku}: {result}')
    
    # the target is probably at the left edge of image since we recognize it as soon as we see it. 
    
    result = send_navigate_goal('NUDGE', str(0.3))

    if result is None:
        ros_context.node.get_logger().info(f'No Nudge result for SKU {item.sku}')
        return False
    
    ros_context.node.get_logger().info(f'Nudge result for SKU {item.sku}: {result}')
    
    # first run pick using the nav camera to get an idea of how much we have to nudge
    
    n = 2
    while (n != 0):
        n -= 1
        nudge_box = doPick(True, item.sku)
    
        if nudge_box is None:
          ros_context.node.get_logger().info(f'No Nudge Pick result for SKU {item.sku}')
          if n == 0:
              return False
        else:
            ros_context.node.get_logger().info(f'Nudge Pick result for SKU {item.sku}: {nudge_box}')
            break

    nudge_box_center_x = (nudge_box[0] + nudge_box[2]) / 2

    ros_context.node.get_logger().info(f'Nudge box center for SKU {item.sku}: ({nudge_box_center_x})')

    nudge_amt = 0.3

    if nudge_box_center_x < (image_width/3) or nudge_box_center_x > (2*image_width/3):
        
        # product is not in the middle 3rd of the image and so need to move the system a bit

        if nudge_box_center_x < (image_width / 3):
            ros_context.node.get_logger().info(f'Nudging left for SKU {item.sku}')
            nudge_amt += 0.1
        else:
            ros_context.node.get_logger().info(f'Nudging right for SKU {item.sku}')
            nudge_amt -= 0.1    
    
    result = send_navigate_goal('NUDGE', str(nudge_amt))

    if result is None:
        ros_context.node.get_logger().info(f'Nudge failed for SKU {item.sku}')
        return False
    
    ros_context.node.get_logger().info(f'Nudge result for SKU {item.sku}: {result}')

    # make sure arms are tucked before we rotate, otherwise we might have a collision with the shelf

    ros_context.node.get_logger().info('Retucking arms')
    if not send_trajectory_request("TUCK", 0.0):
        ros_context.error('Failed to tuck arms.')
        return False
    
    if not wait_for_arm_idle('tower', 't_s_moving'):
        ros_context.error('Tower did not become idle after tucking.')
        return False

    # rotate to face the shelf

    result = send_navigate_goal('ROTATE', str(-80))

    if result is None:
        ros_context.node.get_logger().info(f'No Rotate result for SKU {item.sku}')
        return False

    ros_context.node.get_logger().info(f'Rotate result for SKU {item.sku}: {result}')

    result = send_navigate_goal('NUDGE', str(-0.1))

    if result is None:
        ros_context.node.get_logger().info(f'Nudge failed for SKU {item.sku}')
        return False

    ros_context.node.get_logger().info(f'Nudge back a bit, result for SKU {item.sku}: {result}')
    return True


def position_for_pick(item):
                                     
    # now extend arm to do view shelf

    trajReqString  = "R_CAM_MID_HI" if item.height < 4 else "R_CAM_MID_LOW"

    ros_context.node.get_logger().info('Moving arm so the camera faces shelf with trajectory: ' + trajReqString)

    if not send_trajectory_request(trajReqString, 0.0):
        ros_context.node.get_logger().error('Failed position camera')
        return False

    if not wait_for_arm_idle('right_arm', 'r_s_moving'):
        ros_context.node.get_logger().error('Right arm did not become idle after locking.')
        return False

    ros_context.node.get_logger().info('Arm positioned for picking')
    return True                 


def adjust_for_shelf_yaw():
    
    # now calcuate the yaw relative to the shelf edge

    shelfYaw = decodeShelf()

    ros_context.node.get_logger().info(f'Shelf yaw detected: {shelfYaw}')

    if shelfYaw is not None:
        ros_context.node.get_logger().info(f'Rotating to align with shelf with item')
        
        # some how we need to convert the shelf yaw into a joint angle for the arm. This is a simplification that assumes a direct mapping, but in reality it may require some kinematic calculations based on the current arm pose and the geometry of the shelf.
        
        result = send_joint_request((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, shelfYaw))
        return True
    else:
        ros_context.node.get_logger().warn(f'Could not detect shelf yaw, skipping rotation')
        return False


def calculate_ppi(bbx_center_x, item: QuantitySku):

    # first move the camera a known amount
     
    trajReqString  = "R_CAM_MID_LEFT" if bbx_center_x < image_width / 2 else "R_CAM_MID_RIGHT"

    ros_context.node.get_logger().info('Moving arm to calc PPI: ' + trajReqString)

    if not send_trajectory_request(trajReqString, 0.0):
        ros_context.node.get_logger().error('Failed camera position for PPI calculation')
        return None

    if not wait_for_arm_idle('right_arm', 'r_s_moving'):
        ros_context.node.get_logger().error('Right arm did not become idle after locking.')
        return None

    # now do second pick to get final bounding box for picking. use the arm camera


    box2 = doPick(False,item.sku)

    if box2 is None:
        ros_context.node.get_logger().info(f'No Pick2 result for SKU {item.sku} on second attempt')
        return None 
    
    #calcuate the center of that bbox

    bb2x_center_x = (box2[0] + box2[2]) / 2
    bb2x_center_y = (box2[1] + box2[3]) / 2

    ros_context.node.get_logger().info(f'Bounding box center for SKU {item.sku}: ({bb2x_center_x}, {bb2x_center_y})')

    ppi = abs(bb2x_center_x - bbx_center_x)/PPI_MOVE

    return [ppi, bb2x_center_x, bb2x_center_y]

     
def fetch_item(item: QuantitySku):
    """
    Fetch an item using the navigation system.
 
    Args:
        item: QuantitySku object containing SKU and quantity

    Returns:
        None
    """
    # first navigate to the item location

    ros_context.node.get_logger().info(f'Fetching {item.quantity} of SKU {item.sku} (nav: {item.nav}, distance: {item.distance}, height: {item.height})')

    # find the product we are looking for
    if not navigate_to_item(item):
        ros_context.node.get_logger().error(f'Failed to navigate to item SKU {item.sku}')
        return False
    
    # now rotate and nudge to the shelf and position the arm for picking

    if not position_for_pick(item):
        ros_context.node.get_logger().error(f'Failed to position for pick SKU {item.sku}')
        return False

    # run pick to locate the item and get an initial bounding box, which we will use to calculate the shelf yaw and PPI adjustments
    #  use the arm camera

    if doPick(False, item.sku) is None:
        ros_context.node.get_logger().info(f'No Pick result for SKU {item.sku}')
        return False

    #now adjust for shelf yaw so that the arm is aligned with the shelf, which should improve the bounding box accuracy on the second pick attempt and make it more likely to succeed
         
    if not adjust_for_shelf_yaw():
        ros_context.node.get_logger().error(f'Failed to adjust for shelf yaw for SKU {item.sku}')
        return False

    #now redo PICK action to get good bounding box with arm aligned to shelf. again using the arm camera    

    box = doPick(False,item.sku)

    if box is None:
        ros_context.node.get_logger().info(f'No Pick result for SKU {item.sku} on second attempt')
        return False
    
    #calcuate the center of the bounding box

    bbx_center_x = (box[0] + box[2]) / 2
    bbx_center_y = (box[1] + box[3]) / 2

    ros_context.node.get_logger().info(f'Bounding box center for SKU {item.sku}: ({bbx_center_x}, {bbx_center_y})')

    # now calculate the ppi and return a second box
    
    ppi_and_box2 = calculate_ppi(bbx_center_x, item)

    if ppi_and_box2 is None:
        ros_context.node.get_logger().error(f'Failed to calculate PPI for SKU {item.sku}')
        return False
    
    # now we adjust the pick position based on the PPI so that the camera points at the center of the bounding box

    #calculate the offsets in pixels between the center of the bounding box and the center of the image
    
    offset_x_pixels = bbx_center_x - image_width / 2
    offset_y_pixels = bbx_center_y - image_height / 2

    # now convert those pixel offsets to physical offsets using the PPI

    offset_x_physical = offset_x_pixels / ppi_and_box2[0]
    offset_y_physical = offset_y_pixels / ppi_and_box2[1]

    #move the camera by those physical offsets to point at the center of the bounding box

    pose = get_joint_poses()

    print (f'Current joint poses: {pose}')

    if pose is None:
        ros_context.node.get_logger().error(f'Failed to get joint poses for SKU {item.sku}')
        return False

    # x is left/right and z is up/down. Y in this case is up and down in the image
    pose.position.x += offset_x_physical
    pose.position.z += offset_y_physical

    print (f'Updated joint poses: {pose}')

    if not set_joint_poses(pose):
        ros_context.node.get_logger().error(f'Failed to set joint poses for SKU {item.sku}')
        return False
    

    return True


def do_pick_test(pickUpc: str):
    ros_context.node.get_logger().info(f'doing picking test')

    ros_context.node.get_logger().info('Moving arm so the camera faces shelf')
    if not send_trajectory_request("R_CAM_SIDE", 0.0):
        ros_context.node.get_logger().error('Failed position camera')
        return False

    if not wait_for_arm_idle('right_arm', 'r_s_moving'):
        ros_context.node.get_logger().error('Right arm did not become idle after locking.')
        return False

    ros_context.node.get_logger().info('execute pick action for UPC ' + pickUpc)

    result = send_navigate_goal('PICK', str( 0.6))

    if result is None:
        ros_context.node.get_logger().info(f'No Pick result ')
        return False

    ros_context.node.get_logger().info(f'Pick result: {result}')

    bounding_boxes = result.split(";")
    for box in bounding_boxes:
        ros_context.node.get_logger().info(f'Bounding box: {box}')

        upc, x1, y1, x2, y2 = box.split(",")
        ros_context.node.get_logger().info(f'UPC: {upc}, x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}')

        if upc == pickUpc:
            ros_context.node.get_logger().info(f'Found target UPC {pickUpc} in bounding box: {box}')
            # Here you would add code to command the robot to pick the item based on the bounding box coordinates
            


