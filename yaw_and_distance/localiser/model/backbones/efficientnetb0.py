import torch


def get_efficientnetb0(pretrained=False):
        model = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_efficientnet_b0', pretrained=pretrained)
        return model


if __name__ == '__main__':
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    x = torch.randn(1, 3, 416, 416).to(device)
    backbone = get_efficientnetb0(pretrained=False).to(device)
    print(backbone)
