import torch
import torch.nn as nn
import torch.nn.functional as F


# Confirm if p is set to trainable or not
class PyramidMerge(nn.Module):
    def __init__(self, **kwargs):
        super().__init__()

        self.model_name = kwargs.get('model_name')
        self.activation = kwargs.get('activation')

        # 512 + 40, 512+40*1.4
        if self.model_name == 'efficientnet-b0':
            conv_in_filters = kwargs.get('conv_in_filters', [552, 1392])
            conv_out_filters = kwargs.get('conv_out_filters', 512)
            # conv_in_filters = kwargs.get('conv_in_filters', [512+(40*width_multiplier), +(1280*width_multiplier)])
            # conv_out_filters = kwargs.get('conv_out_filters', 512)
            dense_units = kwargs.get('dense_units', 1280)
        elif self.model_name == 'efficientnet-b3':
            conv_in_filters = kwargs.get('conv_in_filters', [512+48, 1672])
            conv_out_filters = kwargs.get('conv_out_filters', 512)
            # conv_in_filters = kwargs.get('conv_in_filters', [512+(40*width_multiplier), +(1280*width_multiplier)])
            # conv_out_filters = kwargs.get('conv_out_filters', 512)
            dense_units = kwargs.get('dense_units', 1536)
        elif self.model_name == 'efficientnet-b4':
            conv_in_filters = kwargs.get('conv_in_filters', [512+56, 1952])
            conv_out_filters = kwargs.get('conv_out_filters', 512)
            # conv_in_filters = kwargs.get('conv_in_filters', [512+(40*width_multiplier), +(1280*width_multiplier)])
            # conv_out_filters = kwargs.get('conv_out_filters', 512)
            dense_units = kwargs.get('dense_units', 1792)
        else:
            raise NotImplementedError()

        self.cv0 = nn.Conv2d(in_channels=conv_in_filters[0], out_channels=conv_out_filters, kernel_size=1)
        self.cv1 = nn.Conv2d(in_channels=conv_in_filters[1], out_channels=conv_out_filters, kernel_size=1)
        self.dense = nn.Linear(dense_units+conv_out_filters*2, dense_units)

    def forward(self, first_fm, mid_fm, last_fm):
        # print(first_fm.size(), mid_fm.size(), last_fm.size())
        # Process deepest feature map
        bs, n_feat, last_h, last_w = last_fm.shape
        last_pr = self.activation(last_fm).view(bs, n_feat)

        # Process intermediate layer
        _, _, mid_h, mid_w = mid_fm.shape
        re_last_fm = nn.Upsample(size=(mid_h, mid_w))(last_fm)
        mid_fm = torch.cat([re_last_fm, mid_fm], dim=1)
        mid_fm = self.cv1(mid_fm)
        _, n_feat, mid_h, mid_w = mid_fm.shape
        mid_pr = self.activation(mid_fm).view(bs, n_feat)

        # Process first layer layer
        _, _, first_h, first_w = first_fm.shape
        re_mid_fm = nn.Upsample(size=(first_h, first_w))(mid_fm)
        first_fm = torch.cat([re_mid_fm, first_fm], dim=1)
        first_fm = self.cv0(first_fm)
        _, n_feat, first_h, first_w = first_fm.shape
        first_pr = self.activation(first_fm).view(bs, n_feat)
        fv = torch.cat([last_pr, mid_pr, first_pr], dim=-1)
        fv = self.dense(fv)
        return fv
