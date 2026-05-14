# Selfie Perception Pipeline

**Subsystem 1: Perception and Mapping** — Team Picasso

Converts a selfie photo into vector stroke paths (JSON) that the UR3 robot can draw.
The pipeline uses deep learning for background removal and Canny edge detection
(σ=3 Gaussian pre-blur) to produce clean edge outlines of a person and their face.

## Pipeline Overview

```
Selfie Image
    │
    ▼
┌──────────────────────────┐
│  Background Removal      │  rembg / U²-Net (u2net_human_seg)
│  (isolate person)        │  https://github.com/danielgatis/rembg
└──────────┬───────────────┘
           │
           ▼
┌──────────────────────────┐
│  Canny Edge Detection    │  Gaussian blur σ=3 + Canny (50/150)
│  (σ = 3 pre-blur)        │  J. Canny, IEEE TPAMI, 1986
└──────────┬───────────────┘
           │
           ▼
┌──────────────────────────┐
│  Stroke Extraction       │  Contour extraction, Douglas-Peucker
│  + Nearest-Neighbour     │  simplification, greedy NN reordering
│    Ordering              │
└──────────┬───────────────┘
           │
           ▼
    perception_strokes.json
    (400×300 pixel canvas)
```

| Stage | Description |
|---|---|
| **Image Input** | Loads a selfie from `~/perception/input/` (jpg, png, bmp) |
| **Background Removal** | Uses [rembg](https://github.com/danielgatis/rembg) with `u2net_human_seg` model to remove the background, isolating the person on a white canvas |
| **Canny Edge Detection** | Gaussian pre-blur with σ=3 followed by OpenCV Canny (thresholds 50/150) for clean, thin edges |
| **Stroke Extraction** | Extracts ordered contours from the edge map, simplifies with Douglas-Peucker (ε=1.5), scales to 400×300 canvas, and reorders strokes via greedy nearest-neighbour to minimise pen-up travel |
| **Output** | Publishes JSON stroke data on `/drawing_strokes` topic and saves to `~/perception/output/perception_strokes.json` for RS2 motion planning and GUI integration |

## Directory Structure

```
perception/
├── README.md
├── input/                              ← PUT YOUR SELFIE HERE
├── output/
│   ├── perception_strokes.json         ← Latest stroke output (for RS2/GUI)
│   ├── drawing_preview.png             ← Latest preview image
│   └── run_NNN/                        ← Per-run output folders
│       ├── 0_background_removed.png
│       ├── 1_edges.png
│       ├── 2_drawing_preview.png
│       └── perception_strokes.json
├── src/
│   └── selfie_perception/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── launch/
│       │   └── perception_pipeline.launch.py
│       ├── selfie_perception/
│       │   ├── __init__.py
│       │   ├── background_removal.py   ← rembg wrapper
│       │   ├── edge_detection.py       ← Canny σ=3 edge detection
│       │   ├── stroke_extraction.py    ← Contour → stroke JSON
│       │   ├── pipeline.py             ← Standalone pipeline (no ROS2)
│       │   ├── perception_node.py      ← ROS2 node: full pipeline
│       │   ├── image_loader_node.py    ← ROS2 node: image publisher
│       │   └── visualization_node.py   ← ROS2 node: stroke preview
│       └── test/
└── build/, install/, log/              ← Colcon build artifacts
```

## Installation

### Prerequisites

- Python 3.8+
- OpenCV (`opencv-python`)
- NumPy
- ROS2 Humble (for ROS2 nodes; standalone mode does not require ROS)
- cv_bridge (ROS2 image conversion)

### Setup

```bash
# 1. Install Python dependencies
pip3 install "rembg[cpu]" opencv-python numpy

# 2. Build the ROS2 package (optional — only needed for ROS2 mode)
source /opt/ros/humble/setup.bash
cd ~/perception
colcon build --packages-select selfie_perception
source install/setup.bash
```

## Usage

### Standalone Mode (no ROS2, quickest for testing)

```bash
cd ~/perception/src/selfie_perception

# Place your selfie in ~/perception/input/ first, then run with PYTHONPATH:
PYTHONPATH=.:$PYTHONPATH python3 selfie_perception/pipeline.py ../../input/IMG_8820.png

# Or process the most recent image in input/:
PYTHONPATH=.:$PYTHONPATH python3 selfie_perception/pipeline.py

# Options:
PYTHONPATH=.:$PYTHONPATH python3 selfie_perception/pipeline.py ../../input/selfie.jpg
```

Output is saved to `output/run_NNN/`:
- `0_background_removed.png` — person isolated on white
- `1_edges.png` — Canny edge map (σ=3)
- `2_drawing_preview.png` — stroke preview (400×300)
- `perception_strokes.json` — stroke data for the robot

### ROS2 Pipeline

```bash
# Terminal 1 — launch all perception nodes
cd ~/perception && source install/setup.bash
ros2 launch selfie_perception perception_pipeline.launch.py

# Terminal 2 — monitor strokes (optional)
ros2 topic echo /drawing_strokes
```

### Integration with RS2 Motion Planning

```bash
# The motion node loads strokes automatically from:
#   1. /drawing_strokes topic (live ROS2 mode)
#   2. ~/perception/output/perception_strokes.json (file fallback)

# Or copy to RS2 expected location:
cp ~/perception/output/perception_strokes.json ~/RS2/outputs/strokes/face1_strokes.json
```

## ROS2 Topics

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/raw_image` | `sensor_msgs/Image` | image_loader_node, GUI | perception_node |
| `/drawing_strokes` | `std_msgs/String` (JSON) | perception_node | visualization_node, RS2, GUI |
| `/drawing_preview_image` | `sensor_msgs/Image` | perception_node, visualization_node | GUI |

## Output Format

`perception_strokes.json`:
```json
[
  [[x1, y1], [x2, y2], ...],
  [[x1, y1], [x2, y2], ...],
  ...
]
```
Coordinates are in pixel space (0–400 × 0–300), matching `CANVAS_PX_W=400` and
`CANVAS_PX_H=300` in the RS2 motion planning code (`ur3_selfie_draw.py`).

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `gaussian_sigma` | 3.0 | Gaussian blur σ applied before Canny. 3.0 = Canny σ=3 |
| `canny_low` | 50 | Canny lower hysteresis threshold |
| `canny_high` | 150 | Canny upper hysteresis threshold |
| **`morph_kernel_size`** | **7** | **Morphological closing kernel size (7×7 ellipse). Larger = closes more gaps** |
| **`morph_iterations`** | **2** | **Morphological closing iterations. More iterations = more aggressive gap closure** |
| `simplification_epsilon` | 1.5 | Douglas-Peucker simplification tolerance |
| `min_contour_points` | 8 | Minimum points per contour to keep |
| `canvas_px_w` | 400 | Output canvas width (pixels) |
| `canvas_px_h` | 300 | Output canvas height (pixels) |

## How It Works

### 1. Background Removal (rembg)

Uses the [rembg](https://github.com/danielgatis/rembg) library with the
`u2net_human_seg` model to segment the person from the background. The model
is based on U²-Net (Qin et al., 2020) and is specifically trained for human
segmentation. The result is composited onto a white background before edge
detection.

### 2. Canny Edge Detection (σ=3)

Applies a Gaussian pre-blur with σ=3 to the grayscale image, then runs
OpenCV's Canny edge detector with hysteresis thresholds (50/150). This
produces clean, thin, single-pixel-width edges. The σ=3 Gaussian blur
controls the level of detail — it smooths out fine texture/noise while
preserving the major contours of the face and body.

Reference: J. Canny, "A Computational Approach to Edge Detection,"
IEEE Transactions on Pattern Analysis and Machine Intelligence, 1986.

### 3. Morphological Closing (Gap Filling)

Applies morphological closing with a 7×7 elliptical kernel (2 iterations) to
bridge small gaps in the edge map and connect broken stroke segments. This
step is critical for merging nearby edge fragments that should form a single
continuous stroke. The larger kernel size (7×7, up from 3×3) and multiple
iterations allow more aggressive gap closure, enabling more complete strokes
to be detected from edges and reducing fragmentation around image boundaries.

Morphological closing = dilation followed by erosion, which fills internal
holes and gaps while preserving overall shape.

### 4. Stroke Extraction

1. **Contour extraction** — `cv2.findContours` on the binary edge map
2. **Filtering** — remove contours with fewer than `min_contour_points`
3. **Simplification** — Douglas-Peucker algorithm (`cv2.approxPolyDP`)
4. **Scaling** — map pixel coordinates to the 400×300 canvas
5. **Ordering** — greedy nearest-neighbour to minimise pen-up travel

## Citations

### rembg — Background Removal
```bibtex
@software{rembg,
  author = {Daniel Gatis},
  title = {Rembg},
  url = {https://github.com/danielgatis/rembg},
  license = {MIT},
}
```

### Canny Edge Detection
```bibtex
@article{canny1986computational,
  author = {Canny, John},
  title = {A Computational Approach to Edge Detection},
  journal = {IEEE Transactions on Pattern Analysis and Machine Intelligence},
  volume = {PAMI-8},
  number = {6},
  pages = {679--698},
  year = {1986},
}
```

## License

MIT
