import torch
import torch.nn as nn

import numpy as np
from localiser.utils.tools import *

from localiser.model.backbones.efficientnetb0 import get_efficientnetb0

from localiser.model.necks.yolo_fpn import FPN_YOLOV3

from localiser.model.head.yolo_head import Yolo_head
from localiser.model.layers.conv_module import Convolutional


class Yolov3(nn.Module):
    """
    Note ： int the __init__(), to define the modules should be in order, because of the weight file is order
    """
    def __init__(self, cfg=None, init_weights=True):
        super(Yolov3, self).__init__()

        self.cfg = cfg
        self.__anchors = torch.tensor(cfg.anchors)
        self.__strides = torch.tensor(cfg.strides)
        self.__nC = len(cfg.classes)
        self.__out_channel = cfg.anchors_per_scale * (self.__nC + 5)

        in_channels = cfg.num_channels

        if cfg.backbone == 'efficientnetb0':
            self.__backbone = get_efficientnetb0()
            self.features = []
            self.hook_small = self.__backbone.layers[2].register_forward_hook(self.hook_t)
            self.hook_medium = self.__backbone.layers[4].register_forward_hook(self.hook_t)
            self.hook_large = self.__backbone.layers[6].register_forward_hook(self.hook_t)

        else:
            raise('Undefined backbone. Choose: darknet53 or darknet19 or cspdarknet or mobilenetv3_small or mobilenetv3_large in config file.')

        if cfg.backbone == 'efficientnetb0':
            self.__fpn = FPN_YOLOV3(fileters_in=[320,
                                                 112,
                                                 40],
                                    fileters_out=[self.__out_channel, self.__out_channel, self.__out_channel])

        # small
        self.__head_s = Yolo_head(nC=self.__nC, anchors=self.__anchors[0], stride=self.__strides[0])
        # medium
        self.__head_m = Yolo_head(nC=self.__nC, anchors=self.__anchors[1], stride=self.__strides[1])
        # large
        self.__head_l = Yolo_head(nC=self.__nC, anchors=self.__anchors[2], stride=self.__strides[2])

        if init_weights:
            self.__init_weights()

        # print(self.__backbone)

    def hook_t(self, module, input, output):
        self.features.append(output)

    def forward(self, x):
        out = []

        if self.cfg.backbone != 'efficientnetb0' and self.cfg.backbone != 'mobilenetv3_small_tv' and self.cfg.backbone != 'mobilenetv3_large_tv':
            x_s, x_m, x_l = self.__backbone(x)
        else:
            _ = self.__backbone(x)
            x_s, x_m, x_l = self.features[0], self.features[1], self.features[2]
            self.features = []

        # print('Inisde Yolov3 forward(): Backbone output shapes:', x_s.shape, x_m.shape , x_l.shape)

        x_s, x_m, x_l = self.__fpn(x_l, x_m, x_s)

        out.append(self.__head_s(x_s))
        out.append(self.__head_m(x_m))
        out.append(self.__head_l(x_l))

        if self.training:
            p, p_d = list(zip(*out))
            return p, p_d  # smalll, medium, large
        else:
            p, p_d = list(zip(*out))
            return p, torch.cat(p_d, 0)

    def __init_weights(self):

        " Note ：nn.Conv2d nn.BatchNorm2d'initing modes are uniform "
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                torch.nn.init.normal_(m.weight.data, 0.0, 0.01)
                if m.bias is not None:
                    m.bias.data.zero_()

            elif isinstance(m, nn.BatchNorm2d):
                torch.nn.init.constant_(m.weight.data, 1.0)
                torch.nn.init.constant_(m.bias.data, 0.0)

    def load_darknet_weights(self, weight_file, cutoff=52):
        """ https://github.com/ultralytics/yolov3/blob/master/models.py """

        print("load darknet weights : ", weight_file)

        with open(weight_file, 'rb') as f:
            _ = np.fromfile(f, dtype=np.int32, count=5)
            weights = np.fromfile(f, dtype=np.float32)
        count = 0
        ptr = 0
        for m in self.modules():
            if isinstance(m, Convolutional):
                # only initing backbone conv's weights
                if count == cutoff:
                    break
                count += 1

                conv_layer = m._Convolutional__conv
                if m.norm == "bn":
                    # Load BN bias, weights, running mean and running variance
                    bn_layer = m._Convolutional__norm
                    num_b = bn_layer.bias.numel()  # Number of biases
                    # Bias
                    bn_b = torch.from_numpy(weights[ptr:ptr + num_b]).view_as(bn_layer.bias.data)
                    bn_layer.bias.data.copy_(bn_b)
                    ptr += num_b
                    # Weight
                    bn_w = torch.from_numpy(weights[ptr:ptr + num_b]).view_as(bn_layer.weight.data)
                    bn_layer.weight.data.copy_(bn_w)
                    ptr += num_b
                    # Running Mean
                    bn_rm = torch.from_numpy(weights[ptr:ptr + num_b]).view_as(bn_layer.running_mean)
                    bn_layer.running_mean.data.copy_(bn_rm)
                    ptr += num_b
                    # Running Var
                    bn_rv = torch.from_numpy(weights[ptr:ptr + num_b]).view_as(bn_layer.running_var)
                    bn_layer.running_var.data.copy_(bn_rv)
                    ptr += num_b

                else:
                    # Load conv. bias
                    num_b = conv_layer.bias.numel()
                    conv_b = torch.from_numpy(weights[ptr:ptr + num_b]).view_as(conv_layer.bias.data)
                    conv_layer.bias.data.copy_(conv_b)
                    ptr += num_b
                # Load conv. weights
                num_w = conv_layer.weight.numel()
                conv_w = torch.from_numpy(weights[ptr:ptr + num_w]).view_as(conv_layer.weight.data)
                conv_layer.weight.data.copy_(conv_w)
                ptr += num_w


if __name__ == '__main__':
    net = Yolov3()
    print(net)

    in_img = torch.randn(12, 3, 416, 416)
    p, p_d = net(in_img)

    for i in range(3):
        print(p[i].shape)
        print(p_d[i].shape)
