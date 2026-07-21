"""Wraps lightly_train's segmentation model for real-time inference.

Mirrors the loading/inference contract already validated in
DINOv3-Segmentation-Training/Testing_Segmentation.py, with one addition: the
returned mask is guaranteed to match the input frame's (H, W) via a
nearest-neighbor resize, in case lightly_train's predict() internally returns
a different resolution (e.g. the 224x224 training resolution) than the input
it was given. Nearest-neighbor is mandatory here since mask values are class
indices, not continuous values -- any other interpolation would invent
fractional/blended classes at region boundaries.

NOTE: this module cannot be unit-tested in this sandbox -- lightly_train and
torch are not installed here. Smoke-test it standalone (outside ROS) on the
target machine before wiring it into vision_nav_node.py:
    python3 -c "
    from agbot_vision_nav.segmentation_model import SegmentationModel
    import cv2
    m = SegmentationModel('/path/to/exported_best.pt')
    frame = cv2.imread('/path/to/a/640x480/frame.jpg')
    mask = m.predict(frame)
    print(mask.shape, mask.dtype, set(mask.flatten().tolist()))
    "
"""

import cv2
import numpy as np
import torch
import lightly_train
from PIL import Image


class SegmentationModel:
    def __init__(self, model_path, device="auto"):
        """device: 'auto' moves the model to CUDA when available (falls back
        to wherever load_model put it on any failure); 'cpu'/'cuda' force a
        target. self.device_str reports where inference actually runs -- the
        ROS node logs it so a GPU robot silently running on CPU is visible."""
        self.model = lightly_train.load_model(model_path)
        self.model.eval()

        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else None
        if device:
            try:
                self.model = self.model.to(device)
            except Exception:
                pass  # keep load_model's placement; device_str tells the truth

        try:
            self.device_str = str(next(self.model.parameters()).device)
        except Exception:
            self.device_str = "unknown (cuda_available=%s)" % torch.cuda.is_available()

    def predict(self, frame_bgr):
        """Run inference on a BGR frame, return a (H, W) uint8 class-index mask.

        Class indices: 0=sky, 1=traversable, 2=obstacle/untraversable.
        """
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(frame_rgb)

        mask = self.model.predict(img_pil)
        if torch.is_tensor(mask):
            mask = mask.cpu().numpy()
        mask = np.squeeze(mask).astype(np.uint8)

        target_h, target_w = frame_bgr.shape[:2]
        if mask.shape != (target_h, target_w):
            mask = cv2.resize(
                mask, (target_w, target_h), interpolation=cv2.INTER_NEAREST
            )

        return mask
