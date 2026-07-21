"""Render an annotated BGR debug image from a frame, mask, and centerline result.

Pure numpy/opencv, no rospy dependency -- the ROS node wraps the returned
array into a sensor_msgs/Image via cv_bridge for publishing.
"""

import cv2

from agbot_vision_nav.centerline_estimator import CLASS_TRAVERSABLE

TRAVERSABLE_COLOR_BGR = (0, 255, 0)  # matches Testing_Segmentation.py's convention
CENTER_LINE_COLOR_BGR = (255, 255, 255)
SCAN_ROW_COLOR_BGR = (0, 165, 255)
MIDPOINT_COLOR_BGR = (0, 0, 255)
VALID_TEXT_COLOR_BGR = (0, 255, 0)
INVALID_TEXT_COLOR_BGR = (0, 0, 255)


def render_debug_image(frame_bgr, mask, centerline_result, linear_x=None, angular_z=None, alpha=0.5, state_name=None, timing_line=None, detector_line=None):
    """Returns an annotated copy of frame_bgr for rqt_image_view inspection."""
    height, width = frame_bgr.shape[:2]
    cx = width // 2

    overlay = frame_bgr.copy()
    overlay[mask == CLASS_TRAVERSABLE] = TRAVERSABLE_COLOR_BGR
    result = cv2.addWeighted(frame_bgr, 1 - alpha, overlay, alpha, 0)

    cv2.line(result, (cx, 0), (cx, height - 1), CENTER_LINE_COLOR_BGR, 1)

    for scan_row in centerline_result.scan_rows:
        y = scan_row.row_y
        cv2.line(result, (0, y), (width - 1, y), SCAN_ROW_COLOR_BGR, 1)
        if scan_row.x_mid is not None:
            cv2.circle(result, (int(round(scan_row.x_mid)), y), 4, MIDPOINT_COLOR_BGR, -1)
            # normalized corridor width -- the row-exit detector's main
            # signal, drawn per scan row so its threshold can be tuned by eye
            width_norm = (scan_row.x_right - scan_row.x_left + 1) / float(width)
            cv2.putText(
                result,
                "w={:.2f}".format(width_norm),
                (width - 78, y - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                SCAN_ROW_COLOR_BGR,
                1,
                cv2.LINE_AA,
            )

    text_color = VALID_TEXT_COLOR_BGR if centerline_result.valid else INVALID_TEXT_COLOR_BGR
    lines = []
    if state_name is not None:
        lines.append("state={}".format(state_name))
    lines += [
        "valid={}".format(centerline_result.valid),
        "offset_norm={:+.3f}".format(centerline_result.offset_norm),
        "slope_term={:+.3f}".format(centerline_result.slope_term),
        "traversable_frac={:.2f}".format(centerline_result.traversable_fraction),
    ]
    if linear_x is not None and angular_z is not None:
        lines.append("cmd v={:+.2f} w={:+.2f}".format(linear_x, angular_z))
    if timing_line is not None:
        lines.append(timing_line)
    if detector_line is not None:
        lines.append(detector_line)

    for i, line in enumerate(lines):
        y = 20 + i * 18
        cv2.putText(
            result, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1, cv2.LINE_AA
        )

    return result
