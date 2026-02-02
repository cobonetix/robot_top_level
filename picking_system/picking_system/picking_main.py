import sys

from picking_system.picking_utils import load_product_database, process_orders
from picking_system.hardware_initialization import hardware_setup


def main():
    """
    Main entry point. Accepts two command line arguments:
    1. Path to the product database CSV file
    2. Path to the orders list CSV file
    """
    product_db_file = sys.argv[1]
    orders_file = sys.argv[2]
    hardware_setup()
    all_products = load_product_database(product_db_file)
    process_orders(all_products, orders_file)


if __name__ == "__main__":
    main()
