from picking_utils import QuantitySku, get_items_to_order, load_all_orders, sort_items


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
