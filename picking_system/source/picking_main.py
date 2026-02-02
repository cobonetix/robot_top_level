import sys

from picking_utils import process_orders
from arm_init_calibrate import main as run_init_sequence
from service_clients import create_service_clients


def main():
    """
    Main entry point. Accepts one command line argument:
    1. Path to the orders list CSV file
    """
    orders_file = sys.argv[1]
    create_service_clients()
    run_init_sequence()
    process_orders(orders_file)


if __name__ == "__main__":
    main()
