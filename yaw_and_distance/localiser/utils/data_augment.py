# coding=utf-8
import cv2
import random
import numpy as np
from PIL import Image
import glob
import os


class BGAugment(object):
    def __init__(self, p=0.25, bg_path='./good_bg/'):
        self.p = p
        filenames = glob.glob(bg_path+'*.png')
        self.bg_images = []
        for filename in filenames:
            img = np.asarray(Image.open(filename))
            self.bg_images.append(img)
        self.N = len(self.bg_images)

    def __call__(self, img, bboxes, debug=False):
        len_boxes = bboxes.shape[0]
        n_bg = int(self.p * len_boxes)
        if debug: 
            print(len_boxes, n_bg)
            print(bboxes, img.shape)
        if n_bg == 0:
            return img, bboxes
        np.random.shuffle(bboxes)
        if debug:
            print(bboxes)
        bg_boxes = bboxes[:n_bg]
        bboxes = bboxes[n_bg:]
        h, w, _ = img.shape
        bg_idx = np.random.randint(0, self.N)
        bg_img = cv2.resize(self.bg_images[bg_idx], (h, w))
        if debug:
            print(bg_img.shape, h, w)
            print(bboxes)

        if random.random() > 0.5:
            bg_img = np.flip(bg_img, 0)  # up-down
        if random.random() > 0.5:
            bg_img = np.flip(bg_img, 1)  # left-right
        for box in bg_boxes:
            img[int(box[1]):int(box[3]), int(box[0]):int(box[2]), :] = bg_img[int(box[1]):int(box[3]), int(box[0]):int(box[2]), :] 
        return img, bboxes
        

class RandomHorizontalFilp(object):
    def __init__(self, p=0.5):
        self.p = p

    def __call__(self, img, bboxes):
        if random.random() < self.p:
            _, w_img, _ = img.shape
            # img = np.fliplr(img)
            img = img[:, ::-1, :]
            bboxes[:, [0, 2]] = w_img - bboxes[:, [2, 0]]
        return img, bboxes


class RandomCrop(object):
    def __init__(self, p=0.5):
        self.p = p

    def __call__(self, img, bboxes):
        if random.random() < self.p:
            h_img, w_img, _ = img.shape

            max_bbox = np.concatenate([np.min(bboxes[:, 0:2], axis=0), np.max(bboxes[:, 2:4], axis=0)], axis=-1)
            max_l_trans = max_bbox[0]
            max_u_trans = max_bbox[1]
            max_r_trans = w_img - max_bbox[2]
            max_d_trans = h_img - max_bbox[3]

            crop_xmin = max(0, int(max_bbox[0] - random.uniform(0, max_l_trans)))
            crop_ymin = max(0, int(max_bbox[1] - random.uniform(0, max_u_trans)))
            crop_xmax = max(w_img, int(max_bbox[2] + random.uniform(0, max_r_trans)))
            crop_ymax = max(h_img, int(max_bbox[3] + random.uniform(0, max_d_trans)))

            img = img[crop_ymin: crop_ymax, crop_xmin: crop_xmax]

            bboxes[:, [0, 2]] = bboxes[:, [0, 2]] - crop_xmin
            bboxes[:, [1, 3]] = bboxes[:, [1, 3]] - crop_ymin
        return img, bboxes


class RandomAffine(object):
    def __init__(self, p=0.5):
        self.p = p

    def __call__(self, img, bboxes):
        if random.random() < self.p:
            h_img, w_img, _ = img.shape

            max_bbox = np.concatenate([np.min(bboxes[:, 0:2], axis=0), np.max(bboxes[:, 2:4], axis=0)], axis=-1)
            max_l_trans = max_bbox[0]
            max_u_trans = max_bbox[1]
            max_r_trans = w_img - max_bbox[2]
            max_d_trans = h_img - max_bbox[3]

            tx = random.uniform(-(max_l_trans - 1), (max_r_trans - 1))
            ty = random.uniform(-(max_u_trans - 1), (max_d_trans - 1))

            M = np.array([[1, 0, tx], [0, 1, ty]])
            img = cv2.warpAffine(img, M, (w_img, h_img))

            bboxes[:, [0, 2]] = bboxes[:, [0, 2]] + tx
            bboxes[:, [1, 3]] = bboxes[:, [1, 3]] + ty
        return img, bboxes


class Resize(object):
    """
    Resize the image to target size and transforms it into a color channel(BGR->RGB),
    as well as pixel value normalization([0,1])
    """
    def __init__(self, target_shape, normalize=True, correct_box=True):
        self.h_target, self.w_target = target_shape
        self.correct_box = correct_box
        self.normalize = normalize

    def __call__(self, img, bboxes):
        h_org, w_org, channels = img.shape

        if channels == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
        elif channels == 4:
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGBA).astype(np.float32)

        resize_ratio = min(1.0 * self.w_target / w_org, 1.0 * self.h_target / h_org)
        resize_w = int(resize_ratio * w_org)
        resize_h = int(resize_ratio * h_org)
        image_resized = cv2.resize(img, (resize_w, resize_h))
        image_paded = np.full((self.h_target, self.w_target, channels), 128.0)
        dw = int((self.w_target - resize_w) / 2)
        dh = int((self.h_target - resize_h) / 2)
        image_paded[dh:resize_h + dh, dw:resize_w + dw, :] = image_resized
        image = image_paded / 255.0 if self.normalize else image_paded  # normalize to [0, 1]
        # print('Inside resize(): ratio:', resize_ratio, 'dw, dh:', dw, dh, h_org, w_org, self.w_target, self.h_target)

        if self.correct_box:
            if len(bboxes.shape) == 1:
                bboxes = bboxes[np.newaxis, ...]
            assert len(bboxes.shape) == 2
            
            bboxes[:, [0, 2]] = bboxes[:, [0, 2]] * resize_ratio + dw
            bboxes[:, [1, 3]] = bboxes[:, [1, 3]] * resize_ratio + dh
            return image, bboxes
        return image


class Mixup(object):
    def __init__(self, p=0.5):
        self.p = p

    def __call__(self, img_org, bboxes_org, img_mix, bboxes_mix):
        if random.random() < self.p:
            lam = np.random.beta(1.5, 1.5)
            img = lam * img_org + (1 - lam) * img_mix
            bboxes_org = np.concatenate(
                [bboxes_org, np.full((len(bboxes_org), 1), lam)], axis=1)
            bboxes_mix = np.concatenate(
                [bboxes_mix, np.full((len(bboxes_mix), 1), 1 - lam)], axis=1)
            bboxes = np.concatenate([bboxes_org, bboxes_mix])

        else:
            img = img_org
            bboxes = np.concatenate([bboxes_org, np.full((len(bboxes_org), 1), 1.0)], axis=1)

        return img, bboxes


class LabelSmooth(object):
    def __init__(self, delta=0.01):
        self.delta = delta

    def __call__(self, onehot, num_classes):
        return onehot * (1 - self.delta) + self.delta * 1.0 / num_classes
