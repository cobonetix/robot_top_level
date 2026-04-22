import os
import time
import torch
import numpy as np
import onnxruntime as rt

from localiser.model.yolov3 import Yolov3
from localiser.utils.nms_utils import NMS
from localiser.utils.tools import xywh2xyxy
from localiser.utils.data_augment import Resize


class Detector(object):
    # NOTE ***SAV This could do some more for example pickup thresholds
    def __init__(self, cfg, device):
        self.input_size = cfg.image_size
        # print('***SAV input size', self.input_size)

        self.model = Yolov3(cfg).to(device)
        weight_path = cfg.weights
        print("loading weight file from : {}".format(weight_path))
        # weight = os.path.join(weight_path)
        # chkpt = torch.load(weight, map_location=device)
        # self.model.load_state_dict(chkpt)
        self.sess = rt.InferenceSession(weight_path)
        print("loading weight file is done")
        # del chkpt

        self.normalize_input = cfg.normalise_input

        self.device = next(self.model.parameters()).device

    def detect(self, img, conf_th):
        # print('in detect')
        # print('***SAV image shape', img.shape)
        bboxes = self.__predict(img, self.input_size, conf_th)
        # print('Detect bboxes', len(bboxes), bboxes)
        return bboxes  # [:, :6]

    def __predict(self, img, test_shape, conf_th):
        # print('***SAV in predict, img shape', img.shape, 'test_shape', test_shape, 'conf_th', conf_th)
        org_img = np.copy(img)
        org_h, org_w, _ = org_img.shape

        img = self.__get_img_tensor(img, test_shape).to(self.device)
        # self.model.eval()
        # with torch.no_grad():
        #    _, p_d = self.model(img)
        # pred_bbox = p_d.squeeze().cpu().numpy()
        pred_bbox = self.sess.run(None, {'input': img.numpy()})[-1]
        # print('Inside detector.__predict(): pred_bbox shape:', pred_bbox.shape)

        bboxes = self.__convert_pred(pred_bbox, test_shape, (org_h, org_w), (0, np.inf), conf_th)
        # print('Predict returns', len(bboxes), bboxes)

        return bboxes

    # @staticmethod
    def __get_img_tensor(self, img, test_shape):
        # Resize is in utils.data_augment
        img = Resize((test_shape, test_shape), correct_box=False, normalize=self.normalize_input)(img, None).transpose(2, 0, 1)
        return torch.from_numpy(img[np.newaxis, ...]).float()

    @staticmethod
    def __convert_pred(pred_bbox, test_input_size, org_img_shape, valid_scale, conf_th):
        # print('in convert_pred, pred_bbox', len(pred_bbox), pred_bbox[0])
        pred_coor = xywh2xyxy(pred_bbox[:, :4])     # converts from width, height to top, bottom coords
        pred_conf = pred_bbox[:, 4]     # confidence
        pred_prob = pred_bbox[:, 5:]    # probabilities for each of the classes
        # print('pred_conf', pred_conf, len(pred_conf))
        # print('pred_prob', pred_prob, len(pred_prob))

        # (1)
        # (xmin_org, xmax_org) = ((xmin, xmax) - dw) / resize_ratio
        # (ymin_org, ymax_org) = ((ymin, ymax) - dh) / resize_ratio
        org_h, org_w = org_img_shape
        resize_ratio = min(1.0 * test_input_size / org_w, 1.0 * test_input_size / org_h)
        dw = (test_input_size - resize_ratio * org_w) / 2
        dh = (test_input_size - resize_ratio * org_h) / 2
        pred_coor[:, 0::2] = 1.0 * (pred_coor[:, 0::2] - dw) / resize_ratio
        pred_coor[:, 1::2] = 1.0 * (pred_coor[:, 1::2] - dh) / resize_ratio

        # (2)
        pred_coor = np.concatenate([np.maximum(pred_coor[:, :2], [0, 0]),
                                    np.minimum(pred_coor[:, 2:], [org_w - 1, org_h - 1])], axis=-1)
        # (3)
        invalid_mask = np.logical_or((pred_coor[:, 0] > pred_coor[:, 2]), (pred_coor[:, 1] > pred_coor[:, 3]))
        pred_coor[invalid_mask] = 0

        # (4)
        bboxes_scale = np.sqrt(np.multiply.reduce(pred_coor[:, 2:4] - pred_coor[:, 0:2], axis=-1))
        scale_mask = np.logical_and((valid_scale[0] < bboxes_scale), (bboxes_scale < valid_scale[1]))

        # (5)
        class_id = np.argmax(pred_prob, axis=-1)    # chooses the class with highest probability
        # print(class_id, len(class_id), type(class_id),'class_id')
        scores = pred_conf * pred_prob[np.arange(len(pred_coor)), class_id]  # ***SAV not sure what this does
        # print(scores, len(scores), 'scores')

        # ***DD: Since conf_th is of type hydra.list, we convert it to generic list
        conf_th = [x for x in conf_th]

        if type(conf_th) != list:  # ***SAV
            score_mask = scores > conf_th   # this is the original vestion for single threshold
        else:
            # In this case we have a per-class list of thresholds
            score_mask = []     # NOTE: this is a slower implementation (18124.89 ms vs. 17477.02 ms @ct=0.1)
            for i in range(0, len(class_id)):
                score_mask.append(scores[i] > conf_th[class_id[i]])
            score_mask = np.asarray(score_mask)
        # print(score_mask, len(score_mask), 'score_mask')

        mask = np.logical_and(scale_mask, score_mask)

        coors = pred_coor[mask]
        scores = scores[mask]
        class_id = class_id[mask]

        bboxes = np.concatenate([coors, scores[:, np.newaxis], class_id[:, np.newaxis]], axis=-1)

        return bboxes

    def localise(self, im, filename, test_config, log=None):
        if log is None:
            log = logging.getLogger()
        log.debug(f'{im.shape[-1]}, {test_config.num_channels}')

        img_h = im.shape[0]
        img_w = im.shape[1]

        nms_thresh = test_config.nms_th

        classes = test_config.classes
        conf_thresh = test_config.conf_th
        log.debug(f'conf_thresh: {conf_thresh}, classes: {classes}')

        # Detect
        time_start = time.time()
        products = self.detect(im, conf_thresh)  # input is delta x delta so products are in delta x delta
        time_ellapsed = time.time() - time_start
        t_detect = 1000 * time_ellapsed

        # NMS
        nms = NMS()
        time_start = time.time()
        products = nms.nmsIOU(products, nms_thresh)
        time_ellapsed = time.time() - time_start
        tnms = 1000 * time_ellapsed

        # Create the annotation object
        det_dict = {'filename': filename, 'size': {'width': img_w, 'height': img_h},
                    'objects': []}
        for one_prod in products:
            obj = {}
            obj['bndbox'] = {}
            obj['bndbox']['xmin'] = np.int(one_prod[0])
            obj['bndbox']['ymin'] = np.int(one_prod[1])
            obj['bndbox']['xmax'] = np.int(one_prod[2])
            obj['bndbox']['ymax'] = np.int(one_prod[3])
            obj['confidence'] = one_prod[4]
            obj['name'] = classes[int(one_prod[5])]
            if obj['bndbox']['xmax'] - obj['bndbox']['xmin'] < test_config.wmin or \
                    obj['bndbox']['ymax'] - obj['bndbox']['ymin'] < test_config.hmin:
                continue
            det_dict['objects'].append(obj)

        log.debug(f'Processing times (ms): Image {t_detect:.2f}, NMS {tnms:.2f}')

        return det_dict, t_detect, tnms
