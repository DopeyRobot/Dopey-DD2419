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
        A.MotionBlur((1, 15), p=1),
        A.RandomBrightnessContrast(
            p=1, brightness_limit=(-0.2, 0.2), contrast_limit=(-0.2, 0.2)
        ),
        A.ISONoise(intensity = (0.10, 1.0),p=1),
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
    fig,axs = subplots(1,2)
    axs[0].imshow(image)
    original_image = image
    image = transform(image=np.asarray(image))["image"]
    plt.imshow(image)
    plt.show()
