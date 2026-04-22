import numpy as np


class NMS(object):

    @staticmethod
    def box_area(boxA):

        return (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)

    @staticmethod
    def box_inters(objA, objB):
        # determine the (x, y)-coordinates of the intersection rectangle
        xA = max(objA[0], objB[0])
        yA = max(objA[1], objB[1])
        xB = min(objA[2], objB[2])
        yB = min(objA[3], objB[3])

        # compute the area of intersection rectangle
        inters = max(0, xB - xA + 1) * max(0, yB - yA + 1)

        return inters

    def bbox_iou(self, objA, objB):

        # compute the area of both the prediction and ground-truth rectangles
        boxAArea = self.box_area(objA)
        boxBArea = self.box_area(objB)

        # compute the area of intersection rectangle
        interArea = self.box_inters(objA, objB)

        # compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        unionArea = boxAArea + boxBArea - interArea
        if unionArea == 0:
            iou = 0
        else:
            iou = interArea / float(boxAArea + boxBArea - interArea)

        # return the intersection over union value
        return iou

    def bbox_ioa(self, objA, objB):

        # compute the area of both the prediction and ground-truth rectangles
        boxAArea = self.box_area(objA)
        boxBArea = self.box_area(objB)

        # compute the area of intersection rectangle
        interArea = self.box_inters(objA, objB)

        # compute the intersection over area by taking the intersection
        # area and dividing it by the minimum between boxA and boxB area
        minArea = float(min(boxAArea, boxBArea))
        if minArea == 0:
            ioa = 0
        else:
            ioa = interArea / minArea

        # return the intersection over union value
        return ioa

    def nmsSort(self, bbox_array, func):
        if func == 'conf':
            indices = (-bbox_array[:, 4]).argsort()
        elif func == 'area':
            areas = []
            for one_box in bbox_array:
                areas.append(self.box_area(one_box))
            indices = (-np.array(areas)).argsort()
        else:
            print('Undefined function ' + func)
            indices = []
            exit(666)

        return indices

    def nmsCreateSortedIndices(self, bbox_array, jithreshold, scorefunc, sortfunc='conf'):
        if jithreshold < 0.0 or jithreshold > 1.0:
            print("NMS threshold out of range")
            exit(1)

        indices = self.nmsSort(bbox_array, sortfunc)

        for ii, _ in enumerate(indices):
            if indices[ii] < 0:
                continue
            outer = bbox_array[indices[ii], :4]

            for jj in range(ii+1, len(indices)):
                if indices[jj] < 0:
                    continue
                if not bbox_array[indices[ii], 5] == bbox_array[indices[jj], 5]:
                    continue

                inner = bbox_array[indices[jj], :4]
                if scorefunc(inner, outer) > jithreshold:
                    indices[jj] = -1*indices[ii]

        return indices

    def nmsIOU(self, bbox_array, jithreshold):
        ind = self.nmsCreateSortedIndices(bbox_array, jithreshold, self.bbox_iou)
        return bbox_array[ind[ind >= 0], :]

    def nmsIOA(self, bbox_array, jithreshold):
        if len(bbox_array) == 0:
            return []
        ind = self.nmsCreateSortedIndices(bbox_array, jithreshold, self.bbox_ioa, 'conf')
        return bbox_array[ind[ind >= 0], :]

    def nmsIOALarger(self, bbox_array, jithreshold, fixthreshold=None):
        if len(bbox_array) == 0:
            return []
        ind = self.nmsCreateSortedIndices(bbox_array, jithreshold, self.bbox_ioa, 'area')
        return bbox_array[ind[ind >= 0], :]
