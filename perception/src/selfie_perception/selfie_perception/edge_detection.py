"""
Edge detection module using Canny with Gaussian pre-blur (sigma=3).

Applies a Gaussian blur with the specified sigma before running OpenCV's
Canny edge detector. This produces clean, thin edges comparable to
"Canny: sigma=3" from J. Canny, "A Computational Approach to Edge
Detection," IEEE TPAMI, 1986.
"""

import cv2
import numpy as np


def detect_edges(image: np.ndarray,
                 sigma: float = 3.0,
                 low_threshold: int = 20,
                 high_threshold: int = 60) -> np.ndarray:
    """Run Canny edge detection with Gaussian pre-blur on a BGR image.

    Args:
        image: Input BGR image (numpy array).
        sigma: Gaussian blur sigma applied before Canny (default 3.0).
        low_threshold: Canny lower hysteresis threshold.
        high_threshold: Canny upper hysteresis threshold.

    Returns:
        Binary edge map (uint8, 0 or 255) same size as input.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (0, 0), sigma)
    edges = cv2.Canny(blurred, threshold1=low_threshold,
                      threshold2=high_threshold)
    return edges
