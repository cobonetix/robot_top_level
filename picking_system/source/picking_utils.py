import csv
from dataclasses import dataclass

from pick_item import fetch_order


@dataclass
class QuantitySku:
    quantity: int
    sku: str


def get_items_to_order(file_path: str) -> list[QuantitySku]:
    """
    Read a CSV file with quantity and SKU columns into a list of objects.

    Args:
        file_path: Path to the CSV file

    Returns:
        List of QuantitySku objects
    """
    result = []
    with open(file_path, 'r', newline='') as f:
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
        file_path: Path to the CSV file

    Returns:
        List of strings
    """
    result = []
    with open(file_path, 'r', newline='') as f:
        reader = csv.reader(f)
        next(reader, None)  # Skip header row
        for row in reader:
            result.append(row[0])
    return result


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
    for item in order_list:
        fetch_order(item)
    return order_list


