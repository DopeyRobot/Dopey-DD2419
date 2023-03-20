"""Utility functions to handle object detection."""
from typing import Dict, List, TypedDict

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import torch
import cv2
import numpy as np


class BoundingBox(TypedDict):
    """Bounding box dictionary.

    Attributes:
        x: Top-left corner column
        y: Top-left corner row
        width: Width of bounding box in pixel
        height: Height of bounding box in pixel
        score: Confidence score of bounding box.
        category: Category
    """

    x: int
    y: int
    width: int
    height: int
    score: float
    category_id: int


CLASS_DICT = {
    0: "Binky",
    1: "Hugo",
    2: "Slush",
    3: "Muddles",
    4: "Kiki",
    5: "Oakie",
    6: "Cube",
    7: "Ball",
    8: "Box",
}


def add_bounding_boxes(
    ax: plt.Axes, bbs: List[BoundingBox], category_dict: Dict[int, str] = None
) -> None:
    """Add bounding boxes to specified axes.

    Args:
        ax:
            The axis to add the bounding boxes to.
        bbs:
            List of bounding boxes to display.
            Each bounding box dict has the format as specified in
            detector.Detector.decode_output.
        category_dict:
            Map from category id to string to label bounding boxes.
            No labels if None.
    """
    for bb in bbs:
        rect = patches.Rectangle(
            (bb["x"], bb["y"]),
            bb["width"],
            bb["height"],
            linewidth=1,
            edgecolor="r",
            facecolor="none",
        )
        ax.add_patch(rect)

        if category_dict is not None:
            plt.text(
                bb["x"],
                bb["y"],
                CLASS_DICT[bb["category_id"]],
            )


def draw_bb_on_image(
    image, bbs: List[BoundingBox], category_dict: Dict[int, str] = None
):
    font = cv2.FONT_HERSHEY_COMPLEX
    color = (255, 0, 0)
    thickness = 2
    fontScale = 1
    for bb in bbs:
        start_point = (int(bb["x"]), int(bb["y"]))
        end_point = (int(bb["x"] + bb["width"]), int(bb["y"] + bb["height"]))
        image = cv2.rectangle(image, start_point, end_point, color, thickness)

        if category_dict is not None:
            image = cv2.putText(
                image,
                detect_color(bb, image),
                start_point,
                font,
                fontScale,
                color,
                thickness,
                cv2.LINE_AA,
            )
    return image


def detect_color(bb: BoundingBox, image: np.ndarray) -> str:
    """
    Detects the color of the object in the bounding box.
    """
    cube_red = np.array([157, 49, 52])
    cube_green = np.array([70, 90, 85])
    cube_blue = np.array([65, 110, 130])
    cube_wood = np.array([170, 130, 120])

    ball_red = np.array([152, 48, 65])
    ball_green = np.array([94, 210, 185])
    ball_blue = np.array([50, 143, 184])

    cube_colors = {
        "red": cube_red,
        "green": cube_green,
        "blue": cube_blue,
        "wood": cube_wood,
    }

    ball_colors = {
        "red": ball_red,
        "green": ball_green,
        "blue": ball_blue,
    }

    detected_class = CLASS_DICT[bb["category_id"]]

    if detected_class == "Ball" or  detected_class == "Cube":
        cropped_image = image[
            int(bb["y"]) : int(bb["y"] + bb["height"]), int(bb["x"]) : int(bb["x"] + bb["width"]), :
        ]
        mean_pixel = np.mean(cropped_image, axis=(0, 1))
        color_dict = ball_colors if detected_class == "Ball" else cube_colors
        color = min(
            color_dict,
            key=lambda x: np.linalg.norm(
                rgb2lab(color_dict[x]) - rgb2lab(mean_pixel)
            ),
        )
        return color + " " + detected_class 

    else:
        return detected_class

def rgb2lab(inputColor):

   num = 0
   RGB = [0, 0, 0]

   for value in inputColor :
       value = float(value) / 255

       if value > 0.04045 :
           value = ( ( value + 0.055 ) / 1.055 ) ** 2.4
       else :
           value = value / 12.92

       RGB[num] = value * 100
       num = num + 1

   XYZ = [0, 0, 0,]

   X = RGB [0] * 0.4124 + RGB [1] * 0.3576 + RGB [2] * 0.1805
   Y = RGB [0] * 0.2126 + RGB [1] * 0.7152 + RGB [2] * 0.0722
   Z = RGB [0] * 0.0193 + RGB [1] * 0.1192 + RGB [2] * 0.9505
   XYZ[ 0 ] = round( X, 4 )
   XYZ[ 1 ] = round( Y, 4 )
   XYZ[ 2 ] = round( Z, 4 )

   XYZ[ 0 ] = float( XYZ[ 0 ] ) / 95.047         # ref_X =  95.047   Observer= 2°, Illuminant= D65
   XYZ[ 1 ] = float( XYZ[ 1 ] ) / 100.0          # ref_Y = 100.000
   XYZ[ 2 ] = float( XYZ[ 2 ] ) / 108.883        # ref_Z = 108.883

   num = 0
   for value in XYZ :

       if value > 0.008856 :
           value = value ** ( 0.3333333333333333 )
       else :
           value = ( 7.787 * value ) + ( 16 / 116 )

       XYZ[num] = value
       num = num + 1

   Lab = [0, 0, 0]

   L = ( 116 * XYZ[ 1 ] ) - 16
   a = 500 * ( XYZ[ 0 ] - XYZ[ 1 ] )
   b = 200 * ( XYZ[ 1 ] - XYZ[ 2 ] )

   Lab [ 0 ] = round( L, 4 )
   Lab [ 1 ] = round( a, 4 )
   Lab [ 2 ] = round( b, 4 )

   return np.array(Lab)

def non_max_suppresion(
    bbs: List[BoundingBox],
    confidence_threshold=0.5,
    IoU_threshold=0.5,
    diff_class_thresh=0.95,
) -> List[BoundingBox]:
    thresholded_bbs = []
    res = []
    sorted_bboxes = sorted(bbs, reverse=True, key=lambda x: x["score"])
    for bbox in sorted_bboxes:
        if bbox["score"] > confidence_threshold:
            thresholded_bbs.append(bbox)

    while len(thresholded_bbs) > 0:
        cur_bb = thresholded_bbs.pop(0)
        res.append(cur_bb)
        for bb in thresholded_bbs:
            if cur_bb["category_id"] == bb["category_id"]:
                iou = bb_IoU(cur_bb, bb)
                if iou > IoU_threshold:
                    thresholded_bbs.remove(bb)
            elif bb_IoU(cur_bb, bb) > diff_class_thresh:
                thresholded_bbs.remove(bb)
    return res


def bb_IoU(bb1: BoundingBox, bb2: BoundingBox):
    x1, y1, x2, y2 = (
        bb1["x"],
        bb1["y"],
        bb1["x"] + bb1["width"],
        bb1["y"] + bb1["height"],
    )
    x3, y3, x4, y4 = (
        bb2["x"],
        bb2["y"],
        bb2["x"] + bb2["width"],
        bb2["y"] + bb2["height"],
    )

    x_inter1 = max(x1, x3)
    y_inter1 = max(y1, y3)
    x_inter2 = min(x2, x4)
    y_inter2 = min(y2, y4)
    width_inter = abs(x_inter2 - x_inter1)
    height_inter = abs(y_inter2 - y_inter1)
    area_inter = width_inter * height_inter
    width_box1 = abs(x2 - x1)
    height_box1 = abs(y2 - y1)
    width_box2 = abs(x4 - x3)
    height_box2 = abs(y4 - y3)
    area_box1 = width_box1 * height_box1
    area_box2 = width_box2 * height_box2
    area_union = area_box1 + area_box2 - area_inter
    iou = area_inter / area_union

    return iou


def save_model(model: torch.nn.Module, path: str) -> None:
    """Save model to disk.

    Args:
        model: The model to save.
        path: The path to save the model to.
    """
    torch.save(model.state_dict(), path)


def load_model(model: torch.nn.Module, path: str, device: str) -> torch.nn.Module:
    """Load model weights from disk.

    Args:
        model: The model to load the weights into.
        path: The path from which to load the model weights.
        device: The device the model weights should be on.

    Returns:
        The loaded model (note that this is the same object as the passed model).
    """
    state_dict = torch.load(path, map_location=device)
    model.load_state_dict(state_dict)
    return model


def load_state_dict_and_save_as_full(model_class: torch.nn.Module, path: str) -> None:
    """Load model weights from disk and save as full model.

    Args:
        model: The model to load the weights into.
        path: The path from which to load the model weights.
    """
    model = model_class()
    state_dict = torch.load(path)
    model.load_state_dict(state_dict)
    torch.save(model, path)
