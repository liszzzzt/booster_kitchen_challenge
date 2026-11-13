"""YOLO inference for T2 kitchen RGB images.

Input is an HWC uint8 RGB array. Bounding boxes are returned as
``[left, top, width, height]`` in original-image pixels.
"""

import os
from functools import lru_cache
from pathlib import Path

import numpy as np


DEFAULT_MODEL = Path(__file__).resolve().parent / "model" / "yolo26l_kitchen.pt"
LABEL_ALIASES = {"table": "simpleTable"}
BORDER_MARGIN_PX = 2


@lru_cache(maxsize=2)
def _load_model(model_path: str):
    from ultralytics import YOLO

    path = Path(model_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(
            f"YOLO weights not found: {path}. Set T2_YOLO_MODEL to a valid .pt file."
        )
    return YOLO(str(path), task="detect")


def inference(rgb, *, model_path=None, confidence=None, device=None):
    """Return detected objects with labels, confidence, and pixel-space boxes."""
    if not isinstance(rgb, np.ndarray) or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("Expected an HWC uint8 RGB image")
    if rgb.dtype != np.uint8 or rgb.shape[0] == 0 or rgb.shape[1] == 0:
        raise ValueError("Expected a nonempty HWC uint8 RGB image")

    path = model_path if model_path is not None else os.environ.get(
        "T2_YOLO_MODEL", str(DEFAULT_MODEL)
    )
    conf = float(
        os.environ.get("T2_YOLO_CONF", "0.25")
        if confidence is None else confidence
    )
    if not 0.0 <= conf <= 1.0:
        raise ValueError("confidence must be between 0 and 1")
    imgsz = int(os.environ.get("T2_YOLO_IMGSZ", "640"))
    if imgsz <= 0:
        raise ValueError("T2_YOLO_IMGSZ must be positive")

    selected_device = device if device is not None else os.environ.get("T2_YOLO_DEVICE")
    if selected_device is None:
        import torch

        selected_device = "0" if torch.cuda.is_available() else "cpu"

    # ROS Image(rgb8) uses RGB; Ultralytics interprets NumPy input as BGR.
    bgr = np.ascontiguousarray(rgb[:, :, ::-1])
    result = _load_model(str(path)).predict(
        source=bgr,
        conf=conf,
        imgsz=imgsz,
        device=selected_device,
        verbose=False,
    )[0]

    detections = []
    if result.boxes is None:
        return detections
    height, width = rgb.shape[:2]
    for box in result.boxes.data.detach().cpu().tolist():
        x1, y1, x2, y2, score, cls_id = box[:6]
        left = max(0, min(width, round(min(x1, x2))))
        right = max(0, min(width, round(max(x1, x2))))
        top = max(0, min(height, round(min(y1, y2))))
        bottom = max(0, min(height, round(max(y1, y2))))
        if right <= left or bottom <= top:
            continue
        is_fully_visible = (
            left > BORDER_MARGIN_PX
            and top > BORDER_MARGIN_PX
            and right < width - BORDER_MARGIN_PX
            and bottom < height - BORDER_MARGIN_PX
        )
        class_id = int(cls_id)
        model_label = result.names[class_id]
        detections.append({
            "bbox": [left, top, right - left, bottom - top],
            "label": LABEL_ALIASES.get(model_label, model_label),
            "model_label": model_label,
            "class_id": class_id,
            "confidence": float(score),
            "is_fully_visible": is_fully_visible,
            "center_px": (
                [(left + right) / 2, (top + bottom) / 2]
                if is_fully_visible else None
            ),
        })
    return detections
