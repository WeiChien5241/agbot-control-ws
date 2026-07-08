# #!/usr/bin/env python3
# """Standalone smoke test for SegmentationModel -- no ROS, no rosbag, no robot.

# Loads the trained model, runs it on one saved image, and prints the
# resulting mask's shape/dtype/class values so you can eyeball whether the
# output looks sane before wiring it into vision_nav_node.py.

# Edit the two paths below, then run:
#     source ~/agbot_venv/bin/activate
#     cd ~/agbot_control_ws/src/agbot_vision_nav   # so the src/ import below resolves
#     PYTHONPATH=src python3 smoke_test_segmentation.py
# """

# import os
# import sys

# import cv2

# from agbot_vision_nav.segmentation_model import SegmentationModel

# # ---- EDIT THESE TWO PATHS ----
# MODEL_PATH = "~/agbot_control_ws/src/agbot_vision_nav/config/exported_best.pt"
# # IMAGE_PATH = "~/ag_bot/src/DINOv3-Segmentation-Training/Annotated/Images/2026-06-11-11-47-03/frame_001170.jpg"
# IMAGE_PATH = "~/agbot_control_ws/src/frame_000180.jpg"
# # -------------------------------


# def main():
#     model_path = os.path.expanduser(MODEL_PATH)
#     image_path = os.path.expanduser(IMAGE_PATH)

#     print(f"Loading model from {model_path} ...")
#     model = SegmentationModel(model_path)
#     print("Model loaded.")

#     print(f"Reading image from {image_path} ...")
#     frame = cv2.imread(image_path)
#     if frame is None:
#         print(f"ERROR: cv2.imread returned None -- check the path: {image_path}")
#         sys.exit(1)
#     print(f"Image read OK, shape={frame.shape}")

#     print("Running predict() ...")
#     mask = model.predict(frame)

#     print()
#     print("=== Results ===")
#     print("mask.shape:", mask.shape)
#     print("mask.dtype:", mask.dtype)
#     print("unique class values:", set(mask.flatten().tolist()))
#     print()
#     print("Expected: shape matches the image's (height, width), dtype uint8, "
#           "and class values are a subset of {0, 1, 2}.")


# if __name__ == "__main__":
#     main()




#!/usr/bin/env python3
"""Standalone smoke test for SegmentationModel -- no ROS, no rosbag, no robot.

Loads the trained model, runs it on one saved image, prints the resulting
mask's shape/dtype/class values, AND saves a visual overlay image (green
wash on traversable ground + scan-row midpoints) so you can actually judge
mask quality with your eyes -- not just check that its data type is right.
The shape/dtype/value checks alone cannot tell good segmentation from bad;
this overlay is the real test of whether the model generalizes to this frame.

Edit the two paths below, then run:
    source ~/agbot_venv/bin/activate
    cd ~/agbot_control_ws/src/agbot_vision_nav   # so the src/ import below resolves
    PYTHONPATH=src python3 smoke_test_segmentation.py
"""

import os
import sys

import cv2

from agbot_vision_nav.segmentation_model import SegmentationModel
from agbot_vision_nav.centerline_estimator import estimate_centerline
from agbot_vision_nav.debug_viz import render_debug_image

# ---- EDIT THESE TWO PATHS ----
# "~" is fine to use here -- it's expanded automatically below.
MODEL_PATH = "~/agbot_control_ws/src/agbot_vision_nav/config/exported_best.pt"
# # IMAGE_PATH = "~/ag_bot/src/DINOv3-Segmentation-Training/Annotated/Images/2026-06-11-11-47-03/frame_001170.jpg"
IMAGE_PATH = "~/agbot_control_ws/src/tmp/frame_001260.jpg"
# -------------------------------

# Where to save the visual overlay -- open this file afterward and look at it.
OUTPUT_PATH = "~/agbot_control_ws/src/tmp/segmentation_overlay.jpg"


def main():
    model_path = os.path.expanduser(MODEL_PATH)
    image_path = os.path.expanduser(IMAGE_PATH)
    output_path = os.path.expanduser(OUTPUT_PATH)

    print(f"Loading model from {model_path} ...")
    model = SegmentationModel(model_path)
    print("Model loaded.")

    print(f"Reading image from {image_path} ...")
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"ERROR: cv2.imread returned None -- check the path: {image_path}")
        sys.exit(1)
    print(f"Image read OK, shape={frame.shape}")

    print("Running predict() ...")
    mask = model.predict(frame)

    print()
    print("=== Results ===")
    print("mask.shape:", mask.shape)
    print("mask.dtype:", mask.dtype)
    print("unique class values:", set(mask.flatten().tolist()))
    print()
    print("Expected: shape matches the image's (height, width), dtype uint8, "
          "and class values are a subset of {0, 1, 2}.")
    print("NOTE: this alone does not confirm the mask is ACCURATE -- see the")
    print("saved overlay image below and judge it with your eyes.")

    print()
    print("Building overlay for visual inspection ...")
    result = estimate_centerline(mask)
    overlay = render_debug_image(frame, mask, result)
    cv2.imwrite(output_path, overlay)
    print(f"Saved overlay to {output_path}")
    print(f"  valid={result.valid}  offset_norm={result.offset_norm:+.3f}  "
          f"slope_term={result.slope_term:+.3f}  "
          f"traversable_frac={result.traversable_fraction:.2f}")
    print()
    print("Open the saved image and check: does the green wash sit on the")
    print("actual drivable path, and do the red dots land in the middle of it?")


if __name__ == "__main__":
    main()