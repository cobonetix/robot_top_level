import os
import cv2
import json
import time
import tqdm
import yaml
import torch
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

    avg_height_px = float(np.mean(heights))

    # Focal length in pixels: f_px = (f_mm / sensor_width_mm) * image_width_px
    f_px = (FOCAL_LENGTH_MM / SENSOR_WIDTH_MM) * img_width

    distance_m = (SHELF_REAL_HEIGHT_M * f_px) / avg_height_px
    return distance_m, avg_height_px


def test():
    device = torch.device('cpu')  # torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    verbose = 0
    logging.basicConfig(level=logging.DEBUG) if verbose else logging.basicConfig(level=logging.INFO)
    log = logging.getLogger()
    
    images_dir = "/home/bob/dev_ws/label_detect/image/"
    
    images_list = [os.path.join(images_dir, x) for x in sorted(os.listdir(images_dir))]
    num_images = len(images_list)

    # LOCALISATION INIT
    #loc_config = argparse.Namespace(**yaml.safe_load(open(config.loc_config)))
    weights='data/weights/yolov5XL_MD3_grc300_192.onnx'
    input_size=192
    iou_thres=[0.45, 0.45, 0.99]
    loc_conf = [0.45, 0.45, 0.4]
    
    
    product_detector = DetectorOnnx(onnx_file=weights,
                                    img_size=(input_size, input_size),
                                    conf_thres=loc_conf,
                                    iou_thres=iou_thres)
                                     
    # warm-up
    query = cv2.imread(images_list[0])
    _, _, _ = product_detector(0*query)

    # SIMILARITY INIT
    #sim_config = argparse.Namespace(**yaml.safe_load(open(config.sim_config)))
    #product_sim = Similarity(sim_config, device)

    # LOCALISATION STEPS
    times_detect, times_nms, times_sim_tot, times_sim_avg = [], [], [], []
    results = []
    for a, query_image in enumerate(images_list):
        #print(str(a+1) + '/' + str(num_images) + ' Loading ' + query_image)
        image = cv2.imread(query_image)  # load query image
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # this will be used for similarity

        detections, tdet, tnms = product_detector(image)

        log.info(f'Localiser times (ms): Image {tdet:.2f}, NMS {tnms:.2f}')
        times_detect.append(tdet)
        times_nms.append(tnms)
        #pdb.set_trace()

        # SIMILARITY STEPS
        t0 = time.time()
        #products = [x for x in detections[0] if x['name'] == 'product']
        products = [x for x in detections[0] if x['name'] == 'shelf']
        #products = [x for x in detections[0] if x['name'] == 'label']
        n_prods = len(products)
        time_sim = time.time() - t0
        times_sim_tot.append(time_sim)
        times_sim_avg.append(time_sim / n_prods) if n_prods > 0 else times_sim_avg.append(time_sim)
        #log.info(f'Similarity time: {n_prods} products in {time_sim:.2f} s')

        # ---- Shelf distance calculation ----
        img_width = image.shape[1]
        distance_m, avg_height_px = calc_shelf_distance(products, img_width)
        if distance_m is not None:
            log.info(f'Shelf detections     : {n_prods}')
            log.info(f'Avg bbox height      : {avg_height_px:.1f} px')
            log.info(f'Estimated distance   : {distance_m:.3f} m  ({distance_m*100:.1f} cm)')
        else:
            log.info('Shelf distance: no shelf detections found')
        results.append({'image': query_image, 'distance_m': distance_m, 'avg_height_px': avg_height_px})

        # DISPLAY/SAVE FINAL RESULTS
        #pdb.set_trace()
        output_image = Similarity.display_results(image, products, colours=GetClassColours())

        # Annotate distance on output image
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
    
    test()


if __name__ == "__main__":
    main()
    # python3 simple_pipeline.py
    #   -i data/images/
    #   -l localiser/config/loc_config.yaml
    #   -s similarity/config/sim_config.yaml
    #   -o outputs/
