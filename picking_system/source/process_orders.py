#!/usr/bin/env python3
"""
Main entry point for the picking system.

Initializes the ROS2 node and global context, runs the arm initialization
sequence, then processes orders.
"""

import sys
import os
import csv


import ros_context
from picking_utils import send_navigate_goal
from cobonetix_interfaces.srv import ArmJoint, GpioStatus, TrajectorySelect
from api_interface.action import Navigate
from dataclasses import dataclass


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

                for m in matches:
                    if m.distance < item.distance:
                            item.distance = m.distance
                            item.height = m.height
                            item.valid = True
                            item.nav = m.upc
                            ros_context.node.get_logger().info(f'Updated item {item.sku} with match {m.upc}, \
                                                                height={m.height}, distance={m.distance}')
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
    result = send_navigate_goal('NAV_REQ', item.nav)
    if result is None:
        ros_context.node.get_logger().info(f'No Navigation result for SKU {item.sku}')
        return None
    
    ros_context.node.get_logger().info(f'Navigation result for SKU {item.sku}: {result}')

    # now we do a nudge and rotate to the shelf

    ros_context.node.get_logger().info(f'Nudging and rotating arm for SKU {item.sku}')

    result = send_navigate_goal('NUDGE', 0.3)

    if result is None:
        ros_context.node.get_logger().info(f'No Nudge result for SKU {item.sku}')
        return None
    
    ros_context.node.get_logger().info(f'Nudge result for SKU {item.sku}: {result}')

    result = send_navigate_goal('ROTATE', -90)

    if result is None:
        ros_context.node.get_logger().info(f'No Rotate result for SKU {item.sku}')
        return None
    
    ros_context.node.get_logger().info(f'Nudge result for SKU {item.sku}: {result}')

