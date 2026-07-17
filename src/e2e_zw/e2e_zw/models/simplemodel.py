# model.py
import torch
import torch.nn as nn
import torchvision.models as models

class SimpleDrivingModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.backbone = models.resnet18(pretrained=True)
        self.backbone.fc = nn.Linear(self.backbone.fc.in_features, 2)  # 输出 linear_x 和 angular_z

    def forward(self, x):
        return self.backbone(x)
