"""
Standalone perception pipeline — no ROS2 required.

Runs the full pipeline on an image file:
  1. Background removal  (rembg / u2net_human_seg)
  2. Canny edge detection (Gaussian σ=3 pre-blur)
  3. Stroke extraction   (contour → JSON)

Usage:
    python3 -m selfie_perception.pipeline [image_path]

If no image path is given, picks the most recent image from ~/perception/input/.
Outputs are saved to ~/perception/output/run_NNN/.
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np

from selfie_perception.background_removal import remove_background
from selfie_perception.edge_detection import detect_edges
from selfie_perception.stroke_extraction import (
    extract_strokes, save_strokes, CANVAS_PX_W, CANVAS_PX_H)


INPUT_DIR = os.path.join(os.path.expanduser('~'), 'perception', 'input')
OUTPUT_DIR = os.path.join(os.path.expanduser('~'), 'perception', 'output')


def _find_latest_image(directory):
    """Return path to the most recently modified image in directory."""
    extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
    files = []
    for ext in extensions:
        files.extend(glob.glob(os.path.join(directory, ext)))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def _next_run_dir(output_dir):
    """Create and return the next numbered run directory."""
    os.makedirs(output_dir, exist_ok=True)
    existing = [d for d in os.listdir(output_dir)
                if d.startswith('run_') and
                os.path.isdir(os.path.join(output_dir, d))]
    numbers = []
    for d in existing:
        try:
            numbers.append(int(d.split('_')[1]))
        except (IndexError, ValueError):
            pass
    next_num = max(numbers, default=0) + 1
    run_dir = os.path.join(output_dir, f'run_{next_num:03d}')
    os.makedirs(run_dir, exist_ok=True)
    return run_dir


def _draw_preview(strokes, canvas_w=CANVAS_PX_W, canvas_h=CANVAS_PX_H):
    """Render strokes onto a white canvas for preview."""
    canvas = np.ones((canvas_h, canvas_w, 3), dtype=np.uint8) * 255
    total_pts = 0
    for stroke in strokes:
        pts = np.array(stroke, dtype=np.int32)
        if len(pts) < 2:
            continue
        for j in range(len(pts) - 1):
            p1 = tuple(pts[j])
            p2 = tuple(pts[j + 1])
            cv2.line(canvas, p1, p2, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.circle(canvas, tuple(pts[0]), 2, (0, 160, 0), -1)
        total_pts += len(stroke)

    info = f"{len(strokes)} strokes, {total_pts} pts"
    cv2.putText(canvas, info, (10, canvas_h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
    return canvas


def run_pipeline(image_path: str, output_dir: str = OUTPUT_DIR,
                 sigma: float = 3.0, low_threshold: int = 20,
                 high_threshold: int = 60, min_stroke_length: float = 15,
                 epsilon: float = 1.5, show: bool = False):
    """Execute the full perception pipeline on a single image.

    Args:
        image_path:         Path to the input selfie image.
        output_dir:         Base output directory.
        sigma:              Gaussian blur sigma before Canny (default 3.0).
        low_threshold:      Canny lower hysteresis threshold.
        high_threshold:     Canny upper hysteresis threshold.
        min_stroke_length:  Minimum total path length in pixels to keep a stroke (default 10).
        epsilon:            Douglas-Peucker simplification tolerance (default 1.5); higher = fewer, longer strokes.
        show:               Whether to display intermediate results.

    Returns:
        Tuple of (strokes list, run directory path).
    """
    print(f"[Pipeline] Loading image: {image_path}")
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    run_dir = _next_run_dir(output_dir)
    print(f"[Pipeline] Output directory: {run_dir}")

    # --- Stage 1: Background Removal ---
    print("[Pipeline] Stage 1: Removing background (rembg / u2net_human_seg)...")
    nobg = remove_background(image)
    cv2.imwrite(os.path.join(run_dir, '0_background_removed.png'), nobg)
    print("[Pipeline]   Saved 0_background_removed.png")

    # --- Stage 2: Canny Edge Detection (σ=3) ---
    print(f"[Pipeline] Stage 2: Canny edge detection (sigma={sigma}, "
          f"thresholds={low_threshold}/{high_threshold})...")
    edges = detect_edges(nobg, sigma=sigma, low_threshold=low_threshold,
                         high_threshold=high_threshold)
    cv2.imwrite(os.path.join(run_dir, '1_edges.png'), edges)
    print("[Pipeline]   Saved 1_edges.png")

    # --- Stage 3: Stroke Extraction ---
    print("[Pipeline] Stage 3: Extracting strokes...")
    strokes = extract_strokes(edges, epsilon=epsilon, min_stroke_length=min_stroke_length)
    total_pts = sum(len(s) for s in strokes)
    print(f"[Pipeline]   {len(strokes)} strokes, {total_pts} points")

    # Save strokes to run directory
    save_strokes(strokes, os.path.join(run_dir, 'perception_strokes.json'))
    print("[Pipeline]   Saved perception_strokes.json")

    # Also save to the top-level output for RS2/GUI integration
    top_level = os.path.join(output_dir, 'perception_strokes.json')
    save_strokes(strokes, top_level)
    print(f"[Pipeline]   Updated {top_level}")

    # --- Preview ---
    preview = _draw_preview(strokes)
    cv2.imwrite(os.path.join(run_dir, '2_drawing_preview.png'), preview)
    print("[Pipeline]   Saved 2_drawing_preview.png")

    if show:
        cv2.imshow("Background Removed", nobg)
        cv2.imshow("Canny Edges", edges)
        cv2.imshow("Drawing Preview", preview)
        print("[Pipeline] Press any key to close windows...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    print("[Pipeline] Done.")
    return strokes, run_dir


def main():
    parser = argparse.ArgumentParser(
        description='Run the selfie perception pipeline (standalone)')
    parser.add_argument('image', nargs='?', default=None,
                        help='Path to input image (default: latest in input/)')
    parser.add_argument('--sigma', type=float, default=3.0,
                        help='Gaussian blur sigma before Canny (default: 3.0)')
    parser.add_argument('--low', type=int, default=20,
                        help='Canny lower threshold (default: 20)')
    parser.add_argument('--high', type=int, default=60,
                        help='Canny upper threshold (default: 60)')
    parser.add_argument('--min-len', type=float, default=15,
                        help='Minimum stroke length in pixels (default: 15)')
    parser.add_argument('--epsilon', type=float, default=1.5,
                        help='Douglas-Peucker simplification tolerance (default: 1.5); higher = fewer, longer strokes')
    parser.add_argument('--show', action='store_true',
                        help='Display intermediate results in GUI windows')
    args = parser.parse_args()

    image_path = args.image
    if image_path is None:
        image_path = _find_latest_image(INPUT_DIR)
        if image_path is None:
            print(f"No images found in {INPUT_DIR}", file=sys.stderr)
            sys.exit(1)

    run_pipeline(image_path, sigma=args.sigma,
                 low_threshold=args.low, high_threshold=args.high,
                 min_stroke_length=args.min_len, epsilon=args.epsilon,
                 show=args.show)


if __name__ == '__main__':
    main()
