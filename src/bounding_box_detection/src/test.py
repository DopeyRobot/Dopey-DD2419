import matplotlib.pyplot as plt
import PIL.Image as Image
import os
import albumentations as A
import numpy as np
from detector import Detector
import torch

print(torch.cuda.is_available())
model_path = "src/bounding_box_detection/src/det_2023-02-21_19-29-24-367975.pt"
model = Detector()
model.load_state_dict(torch.load(model_path))
model.eval()

