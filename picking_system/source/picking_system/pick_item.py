from picking_system.picking import QuantitySku, Item


def fetch_order(item: QuantitySku, items_map: dict[str, Item]) -> None:
    """
    Fetch an order item.

    Args:
        item: QuantitySku object to fetch
        items_map: Dictionary mapping SKU to Item objects
    """
    product = items_map.get(item.sku)
    if product:
        upc = product.upc
