import torch
from torchvision import transforms
from PIL import Image
import os
import csv
from models.simplemodel import SimpleDrivingModel

# 载入模型
model = SimpleDrivingModel()
model.load_state_dict(torch.load("weights/driving_model.pth", map_location='cpu'))
model.eval()

# 转换器
transform = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
])

# 推理并保存
from datetime import datetime
img_dir = "/home/nvidia/e2e_ws/src/e2e_zw/record_20250924_21"        #图像地址
timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
output_csv = open(f"predictions/predictions{timestamp_str}.csv", 'w', newline='')
writer = csv.writer(output_csv)
writer.writerow(["filename", "pred_linear_x", "pred_angular_z"])

for fname in sorted(os.listdir(img_dir)):
    if not fname.endswith(".png"):
        continue
    path = os.path.join(img_dir, fname)
    img = Image.open(path).convert("RGB")
    tensor = transform(img).unsqueeze(0)
    with torch.no_grad():
        pred = model(tensor)[0]
    writer.writerow([fname, round(pred[0].item(), 4), round(pred[1].item(), 4)])

print("✅ 推理完成，结果保存在 predictions.csv")
