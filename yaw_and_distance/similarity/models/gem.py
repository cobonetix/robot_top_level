import torch
import torch.nn as nn
import torch.nn.functional as F


# Confirm if p is set to trainable or not
class GeM(nn.Module):
    def __init__(self, p=3, eps=1e-6):
        super(GeM, self).__init__()
        self.p = p
        self.eps = eps
        # self.register_buffer('p', torch.ones(1) * p)
        # self.register_buffer('eps', torch.tensor(eps))

    def forward(self, x):
        p = (torch.ones(1, requires_grad=False)*self.p).to(x.device)
        return F.avg_pool2d(x.clamp(min=self.eps).pow(p), (x.size(-2), x.size(-1))).pow(1. / p)
