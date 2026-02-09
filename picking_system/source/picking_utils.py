import csv
import os
import time
from dataclasses import dataclass

import rclpy
from cobonetix_interfaces.srv import ArmJoint, GpioStatus

import ros_context


@dataclass
class QuantitySku:
    quantity: int
    sku: str


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
    result = []
    with open(_resolve_path(file_path), 'r', newline='') as f:
        reader = csv.reader(f)
        next(reader, None)  # Skip header row
        for row in reader:
            obj = QuantitySku(
                quantity=int(row[0]),
                sku=row[1]
            )
            result.append(obj)
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


def wait_for_arm_idle(group: str = 'tower', field: str = 't_s_moving',
                       timeout: float = 30.0, poll_interval: float = 0.5) -> bool:
    """Poll the gpio_status service until the specified field reports idle (0.0).

    Args:
        group: GPIO group to query (e.g. 'tower', 'right_arm', 'left_arm')
        field: Status field to check (e.g. 't_s_moving')
        timeout: Maximum time to wait in seconds
        poll_interval: Time between polls in seconds

    Returns:
        True if idle was reached, False on timeout or error
    """
    ros_context.node.get_logger().info(f'Waiting for {group}.{field} to become idle...')
    start = time.time()

    while (time.time() - start) < timeout:
        request = GpioStatus.Request()
        request.group = group
        request.field = field

        future = ros_context.arm_status_client.call_async(request)
        rclpy.spin_until_future_complete(ros_context.node, future)

        result = future.result()
        if result is None:
            ros_context.node.get_logger().warn('gpio_status call returned None')
        elif result.value == 0.0:
            ros_context.node.get_logger().info(f'{group}.{field} is idle')
            return True
        else:
            ros_context.node.get_logger().debug(f'{group}.{field} = {result.value}, still busy...')

        time.sleep(poll_interval)

    ros_context.node.get_logger().error(f'Timeout waiting for {group}.{field} to become idle')
    return False


def send_joint_request(left: tuple[float, float, float, float],
                       right: tuple[float, float, float, float]) -> bool:
    """Send joint positions to both arms, wait for the response and for
    all arms to stop moving before returning.

    Args:
        left:  (l_j1, l_j2, l_j3, l_j4) positions
        right: (r_j1, r_j2, r_j3, r_j4) positions

    Returns:
        True if the service call succeeded and all arms reached idle.
    """
    logger = ros_context.node.get_logger()

    request = ArmJoint.Request()
    request.l_j1, request.l_j2, request.l_j3, request.l_j4 = left
    request.r_j1, request.r_j2, request.r_j3, request.r_j4 = right

    logger.info(f'Sending joint request: left={left}, right={right}')

    future = ros_context.joint_client.call_async(request)
    rclpy.spin_until_future_complete(ros_context.node, future)

    result = future.result()
    if not result.success:
        logger.error(f'Joint request failed: {result.message}')
        return False
    logger.info(f'Joint request accepted: {result.message}')

    if not wait_for_arm_idle('right_arm', 'r_s_moving'):
        logger.error('Right arm did not become idle')
        return False
    if not wait_for_arm_idle('left_arm', 'l_s_moving'):
        logger.error('Left arm did not become idle')
        return False

    logger.info('Both arms idle')
    return True


def sort_items(items: list[QuantitySku]) -> list[QuantitySku]:
    """
    Sort a list of QuantitySku objects.

    Args:
        items: List of QuantitySku objects to sort

    Returns:
        Sorted list of QuantitySku objects
    """
    # TODO: Implement sorting algorithm
    return items


def process_orders(product_list: str) -> list[QuantitySku]:
    """
    Process orders from order files.

    Args:
        product_list: Path to a CSV file containing file names to read

    Returns:
        List of QuantitySku objects from all files
    """
    file_names = load_all_orders(product_list)
    order_list = []
    for file_name in file_names:
        order_list.extend(sort_items(get_items_to_order(file_name)))
    from pick_item import fetch_order
    for item in order_list:
        fetch_order(item)
    return order_list


