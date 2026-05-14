"""
Background removal module using rembg.

Uses the rembg library (https://github.com/danielgatis/rembg) to remove
backgrounds from selfie images, isolating the person(s) before edge detection.

Citation:
    Daniel Gatis, "Rembg — Remove Image Background",
    https://github.com/danielgatis/rembg, MIT License.
"""

import cv2
import numpy as np
from rembg import remove, new_session


# Use u2net_human_seg model — optimised for human segmentation
_SESSION = None


def _get_session():
    global _SESSION
    if _SESSION is None:
        _SESSION = new_session(model_name="u2net_human_seg")
    return _SESSION


def remove_background(image: np.ndarray) -> np.ndarray:
    """Remove the background from a BGR image, returning a BGR image on white.

    Args:
        image: Input BGR image (numpy array from cv2.imread).

    Returns:
        BGR image with background replaced by white.
    """
    session = _get_session()

    # rembg accepts numpy arrays directly and returns BGRA
    result_bgra = remove(image, session=session)

    # Composite onto white background
    if result_bgra.shape[2] == 4:
        alpha = result_bgra[:, :, 3].astype(np.float32) / 255.0
        alpha_3ch = np.stack([alpha, alpha, alpha], axis=-1)
        bgr = result_bgra[:, :, :3].astype(np.float32)
        white = np.ones_like(bgr, dtype=np.float32) * 255.0
        composited = (bgr * alpha_3ch + white * (1.0 - alpha_3ch))
        return composited.astype(np.uint8)
    else:
        return result_bgra
