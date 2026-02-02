import csv
from dataclasses import dataclass

from picking_system.pick_item import fetch_order


@dataclass
class QuantitySku:
    quantity: int
    sku: str


@dataclass
class Item:
    sku: str
    upc: str
    item_name: str
    weight: float


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


def load_product_database(file_path: str) -> dict[str, Item]:
    """
    Read a CSV file with SKU, UPC, item name, and weight columns into a map.

    Args:
        file_path: Path to the CSV file

    Returns:
        Dictionary mapping SKU to Item objects
    """
    result = {}
    with open(file_path, 'r', newline='') as f:
        reader = csv.reader(f)
        next(reader, None)  # Skip header row
        for row in reader:
            item = Item(
                sku=row[0],
                upc=row[1],
                item_name=row[2],
                weight=float(row[3])
            )
            result[item.sku] = item
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


def process_orders(all_products: dict[str, Item], product_list: str) -> list[QuantitySku]:
    """
    Process orders using the product database and order files.

    Args:
        all_products: Dictionary mapping SKU to Item objects
        product_list: Path to a CSV file containing file names to read

    Returns:
        List of QuantitySku objects from all files
    """
    file_names = load_all_orders(product_list)
    order_list = []
    for file_name in file_names:
        order_list.extend(sort_items(get_items_to_order(file_name)))
    for item in order_list:
        fetch_order(item, all_products)
    return order_list


