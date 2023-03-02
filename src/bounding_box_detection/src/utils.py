"""Utility functions to handle object detection."""
from typing import Dict, List

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import torch
import cv2

from detector import BoundingBox

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

def draw_bb_on_image(image, bbs: List[BoundingBox], category_dict: Dict[int, str] = None ):
    font = cv2.FONT_HERSHEY_COMPLEX
    color = (0,0,255)
    thickness = 2
    fontScale = 1
    for bb in bbs:
        start_point = (int(bb["x"]), int(bb["y"]))
        end_point = (int(bb["x"] + bb["width"]), int(bb["y"]+bb["height"]))
        image = cv2.rectangle(image, start_point, end_point, color, thickness)

        if category_dict is not None:
            image = cv2.putText(
                image,
                CLASS_DICT[bb["category_id"]],
                start_point,
                font,
                fontScale,
                color,
                thickness,
                cv2.LINE_AA
            )
    return image

def non_max_suppresion(bbs:List[BoundingBox], confidence_threshold = 0.5, IoU_threshold = 0.5, diff_class_thresh = 0.95) -> List[BoundingBox]:
    thresholded_bbs = []
    res = []
    sorted_bboxes = sorted(bbs, reverse=True, key = lambda x:x["score"])
    for bbox in sorted_bboxes:
        if bbox["score"] > confidence_threshold:
            thresholded_bbs.append(bbox)

    while len(thresholded_bbs)>0:
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

def bb_IoU(bb1:BoundingBox, bb2:BoundingBox):
    x1, y1, x2, y2= bb1["x"], bb1["y"], bb1["x"]+bb1["width"], bb1["y"] + bb1["height"]
    x3, y3, x4, y4 = bb2["x"], bb2["y"], bb2["x"]+bb2["width"], bb2["y"] + bb2["height"]


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
