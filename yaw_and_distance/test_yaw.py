#!/usr/bin/env python3

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from label_detect import decodeShelf

def main():
    yaw = decodeShelf()

    if yaw is None:
        print("Yaw: None (insufficient labels or rows detected)")
    else:
        print(f"Yaw (diff): {yaw:.2f} px")

    return yaw

if __name__ == '__main__':
    main()
