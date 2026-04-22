import os
import cv2
import json
import tqdm
import yaml
import logging
import argparse
import numpy as np
import pdb

from localiser.detector_onnx import DetectorOnnx
from localiser.utils.ClassColours import GetClassColours

from similarity.similarity import Similarity
from similarity.utils.stat_utils import get_knn

# ---- Distance calculation constants ----
SHELF_REAL_HEIGHT_M = 0.0254   # 1 inch in metres
FOCAL_LENGTH_MM     = 12.0     # 12mm equivalent full-frame focal length
SENSOR_WIDTH_MM     = 36.0     # full-frame sensor width (35mm equiv reference)

IMAGES_DIR = "/home/bob/dev_ws/label_detect/image/"

# Module-level detector instance — initialised once on first call to getShelfDistance()
_product_detector = None


def calc_shelf_distance(products, img_width):
    """
    Estimate distance to the shelf using the pinhole camera model.

    distance = (H_real * f_px) / avg_bbox_height_px

    Parameters
    ----------
    products   : list of detection dicts (name == 'shelf'), each with 'bndbox'
    img_width  : image width in pixels

    Returns
    -------
    distance_m     : float or None
    avg_height_px  : float or None
    """
    heights = [d['bndbox']['ymax'] - d['bndbox']['ymin']
               for d in products
               if d['bndbox']['ymax'] > d['bndbox']['ymin']]

    if not heights:
        return None, None

    print (f"Detected shelf heights (px): {heights}")

    avg_height_px =  float(heights[0]) if len(heights) == 1 else np.mean(heights)

    # Focal length in pixels: f_px = (f_mm / sensor_width_mm) * image_width_px
    f_px = (FOCAL_LENGTH_MM / SENSOR_WIDTH_MM) * img_width

    distance_m = (SHELF_REAL_HEIGHT_M * f_px) / avg_height_px
    return distance_m, avg_height_px


def init_detector(warmup_image_path=None):
    """
    Create and warm up the product detector. Call once at startup.

    Parameters
    ----------
    warmup_image_path : str or None
        Path to an image used for the warm-up pass. If None, a blank
        192x192 frame is used instead.

    Returns
    -------
    product_detector : DetectorOnnx
    """
    weights = 'data/weights/yolov5XL_MD3_grc300_192.onnx'
    input_size = 192
    iou_thres = [0.45, 0.45, 0.99]
    loc_conf = [0.45, 0.45, 0.4]

    product_detector = DetectorOnnx(onnx_file=weights,
                                    img_size=(input_size, input_size),
                                    conf_thres=loc_conf,
                                    iou_thres=iou_thres)

    # warm-up pass so the first real inference is not slow
    if warmup_image_path is not None:
        warmup_frame = cv2.imread(warmup_image_path)
    else:
        warmup_frame = np.zeros((input_size, input_size, 3), dtype=np.uint8)
    product_detector(0 * warmup_frame)

    return product_detector


def get_shelf_distance(product_detector, image):
    """
    Run detection on a single image and return the estimated shelf distance.

    Parameters
    ----------
    product_detector : DetectorOnnx
        Initialised detector returned by init_detector().
    image : np.ndarray
        BGR image (as returned by cv2.imread).

    Returns
    -------
    distance_m     : float or None
    avg_height_px  : float or None
    detections     : list  (raw detector output for the image)
    tdet           : float (detection time ms)
    tnms           : float (NMS time ms)
    """
    log = logging.getLogger(__name__)

    detections, tdet, tnms = product_detector(image)
    log.info(f'Localiser times (ms): Image {tdet:.2f}, NMS {tnms:.2f}')

    products = [x for x in detections[0] if x['name'] == 'shelf']
    n_prods = len(products)

    img_width = image.shape[1]
    distance_m, avg_height_px = calc_shelf_distance(products, img_width)

    if distance_m is not None:
        log.info(f'Shelf detections     : {n_prods}')
        log.info(f'Avg bbox height      : {avg_height_px:.1f} px')
        log.info(f'Estimated distance   : {distance_m:.3f} m  ({distance_m*100:.1f} cm)')
    else:
        log.info('Shelf distance: no shelf detections found')

    return distance_m, avg_height_px, detections, tdet, tnms


def getShelfDistance():
    global _product_detector

    verbose = 0
    logging.basicConfig(level=logging.DEBUG) if verbose else logging.basicConfig(level=logging.INFO)

    images_list = [os.path.join(IMAGES_DIR, x) for x in sorted(os.listdir(IMAGES_DIR))]

    # --- one-time initialisation ---
    if _product_detector is None:
        _product_detector = init_detector(warmup_image_path=images_list[0])

    # --- per-image loop ---
    times_detect, times_nms = [], []
    results = []
    for query_image in images_list:
        image = cv2.imread(query_image)

        distance_m, avg_height_px, detections, tdet, tnms = get_shelf_distance(
            _product_detector, image)

        times_detect.append(tdet)
        times_nms.append(tnms)
        results.append({'image': query_image, 'distance_m': distance_m, 'avg_height_px': avg_height_px})

        # DISPLAY/SAVE FINAL RESULTS
        products = [x for x in detections[0] if x['name'] == 'shelf']
        output_image = Similarity.display_results(image, products, colours=GetClassColours())

        if distance_m is not None:
            dist_label = f"Dist: {distance_m:.3f} m  avg h: {avg_height_px:.0f} px"
            cv2.putText(output_image, dist_label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        display = 1
        if display:
            cv2.imshow('output', output_image)
            k = cv2.waitKey(0)
            cv2.destroyAllWindows()
            if k in [100, 113]:  # d or q to early terminate the loop
                return

        output_path = 'outputs/'
        if output_path is not None:
            os.makedirs(output_path, exist_ok=True)
            cv2.imwrite(os.path.join(output_path, os.path.basename(query_image)), output_image)
            save_json = 0
            if save_json:
                with open(os.path.join(output_path, os.path.basename(query_image)[:-4] + '.json'), 'w') as f:
                    json.dump(detections, f)

    return results


def main():
    '''
    parser = argparse.ArgumentParser(description='Full HH images and retrieving them in catalog database')
    parser.add_argument('-i', '--images_dir', help='Directory with input HH images', required=True)
    parser.add_argument('-l', '--loc_config', help='Path to localiser config file', default='localiser/config/loc_config.yaml')
    parser.add_argument('-s', '--sim_config', help='Path to similarity config file', default='similarity/config/sim_config.yaml')
    parser.add_argument('-d', '--display', help='Choose whether to display the retrieval results or not', default=False)
    parser.add_argument('-o', '--output_path', help='Output directory to save retrieved results', default=None)
    parser.add_argument('-j', '--save_json', help='True to save a JSON file with resulting boxes and skus', default=True)
    parser.add_argument('-v', '--verbose', help='True for DEGUG logging level, otherwise INFO', default=False)

    config = parser.parse_args()
    '''
    
    getShelfDistance()


if __name__ == "__main__":
    main()
    # python3 simple_pipeline.py
    #   -i data/images/
    #   -l localiser/config/loc_config.yaml
    #   -s similarity/config/sim_config.yaml
    #   -o outputs/
