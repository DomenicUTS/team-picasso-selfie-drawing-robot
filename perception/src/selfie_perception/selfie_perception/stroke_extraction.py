"""
Stroke extraction module — converts a binary edge map to ordered drawing strokes.

Extracts contours from the edge image, simplifies them with Douglas-Peucker,
scales them to the target canvas, and reorders them using a greedy
nearest-neighbour strategy to minimise pen-up travel distance.

Output format matches the RS2 motion planning and GUI integration:
    [
      [[x1, y1], [x2, y2], ...],   // stroke 1
      [[x1, y1], [x2, y2], ...],   // stroke 2
      ...
    ]
Coordinates are in pixel space (0–CANVAS_W × 0–CANVAS_H).
"""

import cv2
import json
import math
import numpy as np


# Default canvas size — must match RS2 motion planning expectations
CANVAS_PX_W = 400
CANVAS_PX_H = 300

# Contour filtering / simplification defaults
SIMPLIFICATION_EPSILON = 2.0
MIN_CONTOUR_POINTS = 5
MIN_STROKE_LENGTH = 11  # pixels; strokes shorter than this are discarded


def extract_strokes(edge_map: np.ndarray,
                    canvas_w: int = CANVAS_PX_W,
                    canvas_h: int = CANVAS_PX_H,
                    epsilon: float = SIMPLIFICATION_EPSILON,
                    min_points: int = MIN_CONTOUR_POINTS,
                    min_stroke_length: float = MIN_STROKE_LENGTH) -> list:
    """Convert a binary edge map into ordered drawing strokes.

    Args:
        edge_map:           Binary uint8 image (0 or 255).
        canvas_w:           Output canvas width in pixels.
        canvas_h:           Output canvas height in pixels.
        epsilon:            Douglas-Peucker simplification tolerance.
        min_points:         Minimum number of points to keep a contour.
        min_stroke_length:  Minimum total path length (pixels) to keep a stroke.

    Returns:
        List of strokes, each stroke is a list of [x, y] pairs.
    """
    h, w = edge_map.shape[:2]

    # Morphological closing to fill small gaps and connect broken strokes
    # Reduced kernel size (5x5) and iterations (2) for live camera input
    # Preserves more detail while still connecting nearby edges
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closed_map = cv2.morphologyEx(edge_map, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Find all contours
    contours, _ = cv2.findContours(
        closed_map, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    strokes = []
    for contour in contours:
        if len(contour) < min_points:
            continue

        # Simplify with Douglas-Peucker
        approx = cv2.approxPolyDP(contour, epsilon, closed=False)
        pts = approx.reshape(-1, 2)

        if len(pts) < 2:
            continue

        # Scale to canvas coordinates
        scaled = []
        for x, y in pts:
            sx = round(float(x) * canvas_w / w, 1)
            sy = round(float(y) * canvas_h / h, 1)
            scaled.append([sx, sy])

        # Filter by minimum stroke length
        if _stroke_length(scaled) >= min_stroke_length:
            strokes.append(scaled)

    # Reorder strokes to minimise pen-up travel
    strokes = _reorder_strokes(strokes)
    return strokes


def _reorder_strokes(strokes: list) -> list:
    """Greedy nearest-neighbour reordering to reduce pen-up travel."""
    if len(strokes) <= 1:
        return strokes

    remaining = list(range(len(strokes)))
    ordered = []

    # Start with the first stroke
    current_idx = remaining.pop(0)
    ordered.append(strokes[current_idx])
    current_end = strokes[current_idx][-1]

    while remaining:
        best_idx = None
        best_dist = float('inf')
        best_reverse = False

        for idx in remaining:
            s = strokes[idx]
            start = s[0]
            end = s[-1]

            d_start = _dist(current_end, start)
            d_end = _dist(current_end, end)

            if d_start < best_dist:
                best_dist = d_start
                best_idx = idx
                best_reverse = False
            if d_end < best_dist:
                best_dist = d_end
                best_idx = idx
                best_reverse = True

        remaining.remove(best_idx)
        stroke = strokes[best_idx]
        if best_reverse:
            stroke = stroke[::-1]
        ordered.append(stroke)
        current_end = stroke[-1]

    return ordered


def _dist(p1, p2):
    """Euclidean distance between two [x, y] points."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def _stroke_length(stroke: list) -> float:
    """Calculate total path length of a stroke (sum of segment distances)."""
    if len(stroke) < 2:
        return 0.0
    total = 0.0
    for i in range(len(stroke) - 1):
        total += _dist(stroke[i], stroke[i + 1])
    return total


def strokes_to_json(strokes: list) -> str:
    """Serialise strokes to a JSON string."""
    return json.dumps(strokes)


def save_strokes(strokes: list, path: str):
    """Write strokes to a JSON file."""
    with open(path, 'w') as f:
        json.dump(strokes, f)
