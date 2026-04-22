#!/usr/bin/env python

#!/usr/bin/env python

import rclpy
from sensor_msgs.msg import LaserScan
import math
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
import ctolcvsdk, argparse, cv2, random, sys, os
from ctolcvsdk.utils.modelregistry import ModelRegistry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
import time
from multiprocessing import Process
import multiprocessing
import threading
import queue
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import numpy as np
import pdb


#Pixel 6a phone
#Pixel pitch 1.4 μm
pixel_pitch = 0.0000014


#Label width 60 mm
W = 0.06

#Focal Length 4.38 mm
F = 0.00236

#w = pixel_pitch * label_x_in_pixels

#Distance:
#d = F*(W+w)/w

dist_array = []
x_pixel_array = []
x_pixel_array_abs = []
x_label_array = []
angle_array = []
X_array = [0]
x_dist_left = -5.0
y_dist_left = -5.0
x_dist_right = -5.0
y_dist_right = -5.0
#angle_increment = 0.0
#time_increment = 0.0
#range_min = 0.1 # Minimum range value
#range_max = 10.0 # Maximum range value

num_ranges = 0
ranges = []
angle_min = 0.0
angle_max = 0.0
angle_array = []
angle_increment = 0.0

map_data = np.full((200, 200), -1, dtype=np.int8)

odom_x_old = -1.0
image_sequence = 0
img1 = 0
img2 = 0

def fetchModel(modelId, forceDownload):
    atomToken = os.environ.get('ATOM_TOKEN', '')
    if not atomToken:
        #print('ATOM_TOKEN not set. Make sure that ATOM_TOKEN environment variable is set and it holds valid token')
        exit(-1)

    modelRegistry = ModelRegistry(atomToken)
    return modelRegistry.fetchModel(modelId, forceDownload)



_MODEL_DIR = os.path.dirname(os.path.abspath(__file__))

# Localizer is initialised lazily on first call to decodeShelf()
localizer = None


def _get_localizer():
    global localizer
    if localizer is None:
        model_filename = 'localizer'
        modelFilepath = os.path.join(_MODEL_DIR, '6812074e3763fc3a47d42091.tar.crypt')
        options = ctolcvsdk.Localizer.Options(modelFilepath, model_filename)
        options.padPixel  = [114, 114, 114]
        options.centrePad = False
        localizer = ctolcvsdk.Localizer(options)
    return localizer

clock_sec = 0
clock_nsec = 0
test_variable = 0 
publish_count = 0

# Travel threshold for new picture
travel_threshold = 0.5
    
def decodeShelf():
                global num_ranges
                global ranges
                global angle_min
                global angle_max
                global angle_array
                global angle_increment
                global X_array
                global y_dist_left
                global x_dist_left
                global y_dist_right
                global x_dist_right
                global odom_x_old
                global image_sequence
                global img1
                global img2               
                
                print("Running decode shelf")

                img_raw = cv2.imread("/home/bob/dev_ws/label_detect/image/image.jpg")

                img_height = img_raw.shape[0]
                img_width  = img_raw.shape[1]

                # ---- Lens correction for 24mm equivalent focal length ----
                # 24mm equivalent (full-frame) -> mild barrel distortion
                # Focal length in pixels: f = (f_equiv / 36mm) * image_width
                f_px = (12.0 / 36.0) * img_width

                # Camera intrinsic matrix
                cx = img_width  / 2.0
                cy = img_height / 2.0
                camera_matrix = np.array([[f_px, 0,    cx],
                                          [0,    f_px, cy],
                                          [0,    0,    1.0]], dtype=np.float64)

                # Radial distortion coefficients for 12mm equivalent wide-angle lens
                # k1, k2 (barrel distortion), p1, p2 (tangential), k3
                dist_coeffs = np.array([0.35, -0.12, 0.0, 0.0, 0.0], dtype=np.float64)

                # Compute optimal new camera matrix to retain full image area
                new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                    camera_matrix, dist_coeffs, (img_width, img_height), 1,
                    (img_width, img_height))

                # Apply undistortion
                #img = cv2.undistort(img_raw, camera_matrix, dist_coeffs,None, new_camera_matrix)
                                    
                img = img_raw                    

                # Crop to valid ROI
                rx, ry, rw, rh = roi
                if rw > 0 and rh > 0:
                    img = img[ry:ry+rh, rx:rx+rw]
                    img_height = img.shape[0]
                    img_width  = img.shape[1]

                cv2.imwrite('/home/bob/dev_ws/yaw_detect/undistorted_image.jpg', img)
                # ---------------------------------------------------------

                y1 = int(img_height/2) - 300
                y2 = int(img_height/2) + 300
                x1 = int(img_width/2) - 300
                x2 = int(img_width/2) + 300
                x = 0



                #=============== Detect shelf
                det = _get_localizer().Detect(img)
                if not det:
                    print("No img products found")
                else:
                    print("Img found {} products".format(len(det)))
                x_left_count = 0
                x_right_count = 0
                x_center_count = 0
                xave_left_sum = 0
                xave_right_sum = 0
                xave_center_sum = 0       
            
                # Show results 
                x_pixel_array_abs.clear()
                x_pixel_array.clear()
                x_label_array.clear()
                
                label_centers = []

                for i, e in enumerate(det):
                    if(e.cls == 2):
                        x_min = e.r.xmin
                        x_max = e.r.xmax
                        y_min = e.r.ymin
                        y_max = e.r.ymax
                        x_label_dim = x_max - x_min

                        x_pixel =  ((x_min + x_max)/2) - (img_width/2)
                        y_pixel = ((y_min + y_max)/2) - (img_height/2)

                        # Label distance
                        x_pixel_array_abs.insert(i,abs(x_pixel))
                        x_pixel_array.insert(i,x_pixel)
                        x_label_array.insert(i,x_label_dim)
                        label_image = cv2.rectangle(img, (int(x_min), int(y_min)),(int(x_max), int(y_max)),(0,0,255),thickness=4)

                        cx = (x_min + x_max) / 2
                        cy = (y_min + y_max) / 2
                        label_centers.append((cx, cy))

                # Group label centers into rows by y-coordinate proximity
                if len(label_centers) >= 2:
                    # Sort centers by y-coordinate
                    label_centers_sorted = sorted(label_centers, key=lambda c: c[1])

                    # Group into rows: labels within row_threshold pixels in y belong to same row
                    row_threshold = img_height * 0.05
                    rows = []
                    current_row = [label_centers_sorted[0]]
                    for center in label_centers_sorted[1:]:
                        if abs(center[1] - current_row[-1][1]) <= row_threshold:
                            current_row.append(center)
                        else:
                            rows.append(current_row)
                            current_row = [center]
                    rows.append(current_row)

                    print(f"Detected {len(rows)} row(s) of labels")

                    left_ys  = []
                    right_ys = []

                    for row_idx, row in enumerate(rows):
                        if len(row) < 2:
                            # Single label in row: draw horizontal line through its center
                            cy = int(row[0][1])
                            slope, y_left, y_right = 0.0, cy, cy
                        else:
                            xs = np.array([c[0] for c in row])
                            ys = np.array([c[1] for c in row])
                            slope, intercept = np.polyfit(xs, ys, 1)
                            y_left  = int(intercept)
                            y_right = int(slope * img_width + intercept)

                        left_ys.append(y_left)
                        right_ys.append(y_right)

                        # Draw the fitted line for this row
                        cv2.line(label_image, (0, y_left), (img_width, y_right), (0, 255, 0), 2)
                        cv2.circle(label_image, (0, y_left), 8, (255, 0, 0), -1)
                        cv2.circle(label_image, (img_width - 1, y_right), 8, (255, 0, 0), -1)

                        print(f"  Row {row_idx+1}: slope={slope:.4f}  left_y={y_left}  right_y={y_right}")

                    # Compute average spacing between consecutive row lines at each edge
                    left_ys_sorted  = sorted(left_ys)
                    right_ys_sorted = sorted(right_ys)

                    if len(rows) >= 2:
                        left_spacings  = [left_ys_sorted[i+1]  - left_ys_sorted[i]  for i in range(len(left_ys_sorted)  - 1)]
                        right_spacings = [right_ys_sorted[i+1] - right_ys_sorted[i] for i in range(len(right_ys_sorted) - 1)]
                        avg_left  = np.mean(left_spacings)
                        avg_right = np.mean(right_spacings)
                        diff      = avg_left - avg_right

                        print(f"Left  edge avg spacing:  {avg_left:.2f} px")
                        print(f"Right edge avg spacing:  {avg_right:.2f} px")
                        print(f"Left - Right difference: {diff:.2f} px  (0 = camera aligned with shelf)")

                        cv2.putText(label_image, f"Rows: {len(rows)}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                        cv2.putText(label_image, f"Left avg: {avg_left:.1f}px  Right avg: {avg_right:.1f}px", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                        cv2.putText(label_image, f"Diff (yaw): {diff:.2f}px", (10, 90),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if abs(diff) < 5 else (0, 0, 255), 2)

                        cv2.imwrite('/home/bob/dev_ws/yaw_detect/label_bb_image.jpg', label_image)
                        return diff
                    else:
                        print("Only one row detected - need multiple rows for yaw calculation")
                        cv2.putText(label_image, f"Rows: {len(rows)} (need 2+ for yaw)", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                else:
                    print("Not enough labels detected (need at least 2)")

                cv2.imwrite('/home/bob/dev_ws/yaw_detect/label_bb_image.jpg', label_image)
                return None


def main(args=None):

    decodeShelf()

if __name__ == '__main__':
        main()
