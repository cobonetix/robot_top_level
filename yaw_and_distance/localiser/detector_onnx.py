import cv2
import onnxruntime
import numpy as np
import time
import torch
import torchvision


class DetectorOnnx():
	def __init__(self, onnx_file, img_size, conf_thres=0.25, iou_thres=0.45, classes=None, merging=True, merge_class = 'shelf', merge_iou_threshold=0.2, correct_shelf_widths_flag=False):
		self.onnx_file = onnx_file
		self.img_size = img_size
		self.conf_thres = conf_thres
		self.iou_thres = iou_thres
		self.classes = classes
		self.merging = merging
		self.merge_class = merge_class
		self.merge_iou_threshold = merge_iou_threshold
		self.correct_shelf_widths_flag = correct_shelf_widths_flag

		providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
		self.session = onnxruntime.InferenceSession(onnx_file, providers=providers)
		meta = self.session.get_modelmeta().custom_metadata_map  # metadata
		if 'stride' in meta:
			self.stride, self.names = int(meta['stride']), eval(meta['names'])
		else:
			raise Exception('stride value not found in model metadata')

		self.__dict__.update(locals())

	def non_max_suppression(self,
						prediction,
						conf_thres=0.25,
						iou_thres=0.45,
						classes=None,
						agnostic=False,
						multi_label=False,
						labels=(),
						max_det=300):
		"""Non-Maximum Suppression (NMS) on inference results to reject overlapping bounding boxes

		Returns:
			 list of detections, on (n,6) tensor per image [xyxy, conf, cls]
		"""

		#print('Inside detector, single class: ', prediction.shape)
		#print('Inside NMS(): class:', classes, ' conf thres:', conf_thres, ' NMS iou thres:', iou_thres)
		
		bs = prediction.shape[0]  # batch size
		nc = prediction.shape[2] - 5  # number of classes
		xc = prediction[..., 4] > conf_thres  # candidates

		# Checks
		assert 0 <= conf_thres <= 1, f'Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0'
		assert 0 <= iou_thres <= 1, f'Invalid IoU {iou_thres}, valid values are between 0.0 and 1.0'

		# Settings
		# min_wh = 2  # (pixels) minimum box width and height
		max_wh = 7680  # (pixels) maximum box width and height
		max_nms = 30000  # maximum number of boxes into torchvision.ops.nms()
		time_limit = 0.3 + 0.03 * bs  # seconds to quit after
		redundant = True  # require redundant detections
		multi_label &= nc > 1  # multiple labels per box (adds 0.5ms/img)
		merge = False  # use merge-NMS

		t = time.time()
		output = [torch.zeros((0, 6), device=prediction.device)] * bs
		for xi, x in enumerate(prediction):  # image index, image inference
			# Apply constraints
			# x[((x[..., 2:4] < min_wh) | (x[..., 2:4] > max_wh)).any(1), 4] = 0  # width-height
			x = x[xc[xi]]  # confidence

			# Cat apriori labels if autolabelling
			if labels and len(labels[xi]):
				lb = labels[xi]
				v = torch.zeros((len(lb), nc + 5), device=x.device)
				v[:, :4] = lb[:, 1:5]  # box
				v[:, 4] = 1.0  # conf
				v[range(len(lb)), lb[:, 0].long() + 5] = 1.0  # cls
				x = torch.cat((x, v), 0)

			# If none remain process next image
			if not x.shape[0]:
				continue

			# Compute conf
			x[:, 5:] *= x[:, 4:5]  # class_conf = obj_conf * class_conf

			# Box (center x, center y, width, height) to (x1, y1, x2, y2)
			box = self.xywh2xyxy(x[:, :4])

			# Detections matrix nx6 (xyxy, conf, cls)
			if multi_label:
				i, j = (x[:, 5:] > conf_thres).nonzero(as_tuple=False).T
				x = torch.cat((box[i], x[i, j + 5, None], j[:, None].float()), 1)
			else:  # best class only
				conf, j = x[:, 5:].max(1, keepdim=True)
				x = torch.cat((box, conf, j.float()), 1)[conf.view(-1) > conf_thres]

			# Filter by class
			if classes is not None:
				x = x[(x[:, 5:6] == torch.tensor(classes, device=x.device)).any(1)]

			# Apply finite constraint
			# if not torch.isfinite(x).all():
			#	 x = x[torch.isfinite(x).all(1)]

			# Check shape
			n = x.shape[0]  # number of boxes
			if not n:  # no boxes
				continue
			elif n > max_nms:  # excess boxes
				x = x[x[:, 4].argsort(descending=True)[:max_nms]]  # sort by confidence

			# Batched NMS
			c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
			boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores
			i = torchvision.ops.nms(boxes, scores, iou_thres)  # NMS
			if i.shape[0] > max_det:  # limit detections
				i = i[:max_det]
			if merge and (1 < n < 3E3):  # Merge NMS (boxes merged using weighted mean)
				# update boxes as boxes(i,4) = weights(i,n) * boxes(n,4)
				iou = self.box_iou(boxes[i], boxes) > iou_thres  # iou matrix
				weights = iou * scores[None]  # box weights
				x[i, :4] = torch.mm(weights, x[:, :4]).float() / weights.sum(1, keepdim=True)  # merged boxes
				if redundant:
					i = i[iou.sum(1) > 1]  # require redundancy

			output[xi] = x[i]
			if (time.time() - t) > time_limit:
				LOGGER.warning(f'WARNING: NMS time limit {time_limit:.3f}s exceeded')
				break  # time limit exceeded

		return output

	def box_iou(self, box1, box2):
		# https://github.com/pytorch/vision/blob/master/torchvision/ops/boxes.py
		"""
		Return intersection-over-union (Jaccard index) of boxes.
		Both sets of boxes are expected to be in (x1, y1, x2, y2) format.
		Arguments:
			box1 (Tensor[N, 4])
			box2 (Tensor[M, 4])
		Returns:
			iou (Tensor[N, M]): the NxM matrix containing the pairwise
				IoU values for every element in boxes1 and boxes2
		"""

		# inter(N,M) = (rb(N,M,2) - lt(N,M,2)).clamp(0).prod(2)
		(a1, a2), (b1, b2) = box1[:, None].chunk(2, 2), box2.chunk(2, 1)
		inter = (torch.min(a2, b2) - torch.max(a1, b1)).clamp(0).prod(2)

		# IoU = inter / (area1 + area2 - inter)
		return inter / (box_area(box1.T)[:, None] + box_area(box2.T) - inter)

	def xywh2xyxy(self, x):
		# Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
		y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
		y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
		y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
		y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
		y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
		return y


	def scale_coords(self, img1_shape, coords, img0_shape, ratio_pad=None):
		# Rescale coords (xyxy) from img1_shape to img0_shape
		if ratio_pad is None:  # calculate from img0_shape
			gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain  = old / new
			pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
		else:
			gain = ratio_pad[0][0]
			pad = ratio_pad[1]

		coords[:, [0, 2]] -= pad[0]  # x padding
		coords[:, [1, 3]] -= pad[1]  # y padding
		coords[:, :4] /= gain
		self.clip_coords(coords, img0_shape)
		return coords


	def clip_coords(self, boxes, shape):
		# Clip bounding xyxy bounding boxes to image shape (height, width)
		if isinstance(boxes, torch.Tensor):  # faster individually
			boxes[:, 0].clamp_(0, shape[1])  # x1
			boxes[:, 1].clamp_(0, shape[0])  # y1
			boxes[:, 2].clamp_(0, shape[1])  # x2
			boxes[:, 3].clamp_(0, shape[0])  # y2
		else:  # np.array (faster grouped)
			boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, shape[1])  # x1, x2
			boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, shape[0])  # y1, y2

	def letterbox(self, im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
		# Resize and pad image while meeting stride-multiple constraints
		shape = im.shape[:2]  # current shape [height, width]
		if isinstance(new_shape, int):
			new_shape = (new_shape, new_shape)

		# Scale ratio (new / old)
		r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
		if not scaleup:  # only scale down, do not scale up (for better val mAP)
			r = min(r, 1.0)

		# Compute padding
		ratio = r, r  # width, height ratios
		new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
		dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
		if auto:  # minimum rectangle
			dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
		elif scaleFill:  # stretch
			dw, dh = 0.0, 0.0
			new_unpad = (new_shape[1], new_shape[0])
			ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

		dw /= 2  # divide padding into 2 sides
		dh /= 2

		if shape[::-1] != new_unpad:  # resize
			im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
		top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
		left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
		im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
		return im, ratio, (dw, dh)

	def preprocess_image(self, image):
		img = self.letterbox(image, self.img_size, stride=self.stride, auto=False)[0]
		# Convert
		img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
		img = np.ascontiguousarray(img)
		img = img.astype(np.float32)
		img = np.expand_dims(img, 0)

		img /= 255

		return img

	# DD functions

	def bbox_area(self, bbox_dict):
		min_x, max_x, min_y, max_y = bbox_dict['bndbox']['xmin'], bbox_dict['bndbox']['xmax'], bbox_dict['bndbox']['ymin'], bbox_dict['bndbox']['ymax']
		area = (max_y - min_y) * (max_x - min_x)
		return area

	def bbox_intersection_area(self, bbox_i, bbox_j):
		i_min_x, i_max_x, i_min_y, i_max_y = bbox_i['bndbox']['xmin'], bbox_i['bndbox']['xmax'], bbox_i['bndbox']['ymin'], bbox_i['bndbox']['ymax']
		j_min_x, j_max_x, j_min_y, j_max_y = bbox_j['bndbox']['xmin'], bbox_j['bndbox']['xmax'], bbox_j['bndbox']['ymin'], bbox_j['bndbox']['ymax']

		#find coordinates of intersecting rectangle
		min_x = max(i_min_x, j_min_x)
		min_y = max(i_min_y, j_min_y)
		max_x = min(i_max_x, j_max_x)
		max_y = min(i_max_y, j_max_y)

		intersection_area = max(0, max_x - min_x + 1) * max(0, max_y - min_y + 1)
		return intersection_area

	def iou(self, bbox_i, bbox_j):

		bbox_i_area = self.bbox_area(bbox_i)
		bbox_j_area = self.bbox_area(bbox_j)
		intersecting_area = self.bbox_intersection_area(bbox_i, bbox_j)
		union_area = bbox_i_area + bbox_j_area - intersecting_area

		if union_area == 0:
			iou = 0
		else:
			iou = intersecting_area / float(union_area)
		
		return iou

	def merge_bbox(self, bbox_i, bbox_j):
		i_min_x, i_max_x, i_min_y, i_max_y = bbox_i['bndbox']['xmin'], bbox_i['bndbox']['xmax'], bbox_i['bndbox']['ymin'], bbox_i['bndbox']['ymax']
		j_min_x, j_max_x, j_min_y, j_max_y = bbox_j['bndbox']['xmin'], bbox_j['bndbox']['xmax'], bbox_j['bndbox']['ymin'], bbox_j['bndbox']['ymax']

		min_x = min(i_min_x, j_min_x)
		min_y = min(i_min_y, j_min_y)
		max_x = max(i_max_x, j_max_x)
		max_y = max(i_max_y, j_max_y)

		bbox = {'bndbox':{'xmin':min_x, 'xmax':max_x, 'ymin':min_y, 'ymax':max_y},
				'confidence':0.5,
				'name':'shelf'}
		
		return bbox
		
	def extract_class_specific_objects(self, pred, extract_class):

		extract_class_objects, remaining_objects = [], []

		for bbox_dict in pred:
			if bbox_dict['name'] == extract_class:
				extract_class_objects.append(bbox_dict)
			else:
				remaining_objects.append(bbox_dict)
		
		return extract_class_objects, remaining_objects

	def correct_shelf_widths(self, shelf_bboxes):
		
		#find max and min dimension of shelf widths
		shelf_min_x, shelf_max_x = 99999, -1
		for shelf in shelf_bboxes:
			if shelf['bndbox']['xmin'] < shelf_min_x:
				shelf_min_x = shelf['bndbox']['xmin']
			if shelf['bndbox']['xmax'] > shelf_max_x:
				shelf_max_x = shelf['bndbox']['xmax']

		#Adjust widths 
		for idx in range(len(shelf_bboxes)):
			shelf_bboxes[idx]['bndbox']['xmin'] = shelf_min_x
			shelf_bboxes[idx]['bndbox']['xmax'] = shelf_max_x

		return shelf_bboxes

	def merge_objects(self, pred, merge_class, merge_iou_threshold):
		
		# Separate out merge_class bbox objects
		to_be_merged_objects, remaining_objects = self.extract_class_specific_objects(pred, extract_class=merge_class)

		#print('Number of shelf pred before merging:', len(to_be_merged_objects))

		#object merging
		#check iou for one-to-one object bbox and merge if iou > iou_threshold
		for idx_i, bbox_i in enumerate(to_be_merged_objects):
			#if the bbox is not already merged
			if type(to_be_merged_objects[idx_i]) != type(-1):
				for idx_j, bbox_j in enumerate(to_be_merged_objects):
					#reassign bbox_i everytime, incase its the new merged bbox 
					bbox_i = to_be_merged_objects[idx_i]
					#if the bbox is not already merged
					if type(to_be_merged_objects[idx_j]) != type(-1):
						#ignore same bbox comparison
						if idx_i == idx_j:
							continue
						if self.iou(bbox_i, bbox_j) > merge_iou_threshold:
							merged_bbox = self.merge_bbox(bbox_i, bbox_j)
							to_be_merged_objects[idx_i] = merged_bbox
							to_be_merged_objects[idx_j] = -1

		merged_objects = []
		for bbox in to_be_merged_objects:
			if type(bbox) != type(-1):
				merged_objects.append(bbox)

		#print('Number of shelf pred after merging:', len(merged_objects))

		#Correcting shelf widths
		if self.correct_shelf_widths_flag and merge_class == 'shelf':
			merged_objects = self.correct_shelf_widths(merged_objects)

		return merged_objects + remaining_objects

	def __call__(self, im0, augment=False):
		#print('Classes:', self.names)

		im = self.preprocess_image(im0)
		# print(im0.shape)
		start_time = time.time()
		y = self.session.run([self.session.get_outputs()[0].name], {self.session.get_inputs()[0].name: im})[0]
		model_pred_time = time.time() - start_time
		
		pred = torch.tensor(y, device=torch.device('cpu'))
		#print('Inside detector, raw prediction from model shape: ', pred.shape)

		# print(y.shape)

		#Do conf thres & NMS class wise
		classes = self.names if self.classes is None else self.classes
		class_wise_nms_iou_threshold = self.iou_thres
		class_wise_conf_thres = self.conf_thres
		pred_ = []	#(nc, nb, nbbox, 6)

		nms_time = []
		for class_id, class_name in enumerate(classes):
			start_time = time.time()
			pred_.append(self.non_max_suppression(pred, conf_thres=class_wise_conf_thres[class_id], iou_thres=class_wise_nms_iou_threshold[class_id], classes=[class_id]))
			nms_time.append(time.time() - start_time)
		nms_time = np.mean(nms_time)

		pred = []
		for batch_id in range(len(pred_[0])):
			batch = torch.cat([x[batch_id] for x in pred_], dim=0)
			pred.append(batch)
		
		batch_objects = []
		#print('Pred shape:', len(pred))

		for i, det in enumerate(pred): # per image 	#pred : [batch, num_bboxes, 6]
			# print(det)
			# Rescale boxes from img_size to im0 size
			det[:, :4] = self.scale_coords(im.shape[2:], det[:, :4], im0.shape).round()
			objects = []

			for j, d in enumerate(det): # per det
				# det format x y x y conf class
				final_box, final_score, class_idx = d[:4], d[4], d[5]
				obj = {}
				obj['bndbox'] = {}
				obj['bndbox']['xmin'] = int(final_box[0])
				obj['bndbox']['ymin'] = int(final_box[1])
				obj['bndbox']['xmax'] = int(final_box[2])
				obj['bndbox']['ymax'] = int(final_box[3])
				obj['confidence'] = float(final_score)
				if self.names:
					obj['name'] = self.names[int(class_idx)]
				else:
					obj['name'] = int(class_idx)

				objects.append(obj)

			#Merging objects
			if self.merging:
				if type(self.merge_class) != type([0]):	# if single class to merge, i.e. not a list
					objects = self.merge_objects(pred=objects, merge_class=self.merge_class, merge_iou_threshold=self.merge_iou_threshold)
				else:	#if merge multiple classes
					for idx, merge_class_name in enumerate(self.merge_class):
						# if merge_iou_threshold is a list, for class specific iou values
						if type(self.merge_iou_threshold) == type([0]):
							assert len(self.merge_class) == len(self.merge_iou_threshold)
							merge_iou_threshold = self.merge_iou_threshold[idx]
						else:
							merge_iou_threshold = self.merge_iou_threshold

						objects = self.merge_objects(pred=objects, merge_class=merge_class_name, merge_iou_threshold=merge_iou_threshold)



			batch_objects.append(objects)

		return batch_objects, model_pred_time, nms_time
		

if __name__ == '__main__':
	print('main does nothing')

