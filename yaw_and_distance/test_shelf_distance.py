#!/usr/bin/env python3

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from shelf_distance import test


def main():
    results = test()

    if not results:
        print("No results returned.")
        return None

    print("\n=== Shelf Distance Results ===")
    for r in results:
        img  = os.path.basename(r['image'])
        dist = r['distance_m']
        h_px = r['avg_height_px']
        if dist is not None:
            print(f"  {img}  ->  {dist:.3f} m  ({dist*100:.1f} cm)  avg bbox h: {h_px:.0f} px")
        else:
            print(f"  {img}  ->  no shelf detected")

    # Return distance from the last image processed
    last_distance = results[-1]['distance_m']
    return last_distance


if __name__ == '__main__':
    distance = main()
    if distance is not None:
        print(f"\nShelf distance: {distance:.3f} m")
