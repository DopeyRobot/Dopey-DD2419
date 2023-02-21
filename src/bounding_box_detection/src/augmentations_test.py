import matplotlib.pyplot as plt
import PIL.Image as Image
import os
import albumentations as A
import numpy as np

TEST_IMAGE_PATH = "./test_images/"
N_TEST_IMAGES = 10
test_images = []
target_size = (480, 640)
transform = A.Compose(
    [
        A.Resize(*target_size),
        A.GaussianBlur(blur_limit=(3, 5), p=0.1),
        A.MotionBlur((3, 15), p=0.1),
        A.ISONoise(intensity=(0.10, 1.0), p=0.1),
        A.HorizontalFlip(p=0.5),
        A.PixelDropout(dropout_prob=0.002, p=0.1, per_channel=True),
        A.ColorJitter(p=0.2),
        A.Downscale(scale_min=0.85, scale_max=0.95, p=0.1),
    ]
)

if not os.path.exists(TEST_IMAGE_PATH):
    os.makedirs(TEST_IMAGE_PATH)

for file_name in sorted(os.listdir(TEST_IMAGE_PATH))[:N_TEST_IMAGES]:
    if file_name.endswith(".jpeg") or file_name.endswith(".jpg"):
        file_path = os.path.join(TEST_IMAGE_PATH, file_name)
        test_image = Image.open(file_path)
        test_images.append(test_image)

for image in test_images:
    fig, axs = plt.subplots(1, 2)
    original_image = image
    axs[0].imshow(original_image)
    image = transform(image=np.asarray(image))["image"]
    axs[1].imshow(image)
    plt.show()
