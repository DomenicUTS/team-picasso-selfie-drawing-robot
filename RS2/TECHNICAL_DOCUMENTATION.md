# UR3 Selfie Drawing Robot — Technical Documentation

**Team Picasso · Robotics Studio 2 · UTS · Sprint 4 (May 2026)**

> **Audience.** This is the full client/coach reference. If you just landed
> on the repo and want to *start running things*, read [`README.md`](README.md)
> first — it has the 30-second quick start. Come back here for everything
> else (BOM, installation, subsystem details, calibration, troubleshooting).

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [How the System Works (30-second mental model)](#2-how-the-system-works-30-second-mental-model)
3. [Key Features & Subsystems](#3-key-features--subsystems)
4. [Dependencies](#4-dependencies)
   - 4.1 [Hardware (Bill of Materials)](#41-hardware-bill-of-materials)
   - 4.2 [Computing Specs](#42-computing-specs)
   - 4.3 [Software](#43-software)
5. [Installation](#5-installation)
   - 5.1 [Hardware Setup](#51-hardware-setup)
   - 5.2 [Software Setup](#52-software-setup)
6. [Running the System](#6-running-the-system)
   - 6.1 [Full Integration (recommended)](#61-full-integration-recommended)
   - 6.2 [File-mode (no GUI)](#62-file-mode-no-gui)
   - 6.3 [Each subsystem standalone](#63-each-subsystem-standalone)
   - 6.4 [Expected outcome (with checkpoints)](#64-expected-outcome-with-checkpoints)
7. [Subsystem Reference](#7-subsystem-reference)
   - 7.1 [GUI Subsystem](#71-gui-subsystem)
   - 7.2 [Perception Subsystem](#72-perception-subsystem)
   - 7.3 [Motion Planning Subsystem](#73-motion-planning-subsystem)
   - 7.4 [End-Effector / Marker Holder](#74-end-effector--marker-holder)
8. [Cross-Subsystem Interfaces](#8-cross-subsystem-interfaces)
   - 8.1 [Complete ROS 2 Topic Map](#81-complete-ros-2-topic-map)
   - 8.2 [Data format contract (stroke JSON)](#82-data-format-contract-stroke-json)
9. [Configuration & Calibration](#9-configuration--calibration)
10. [Troubleshooting & FAQs](#10-troubleshooting--faqs)
11. [Project Layout](#11-project-layout)

---

## 1. Project Overview

The UR3 Selfie Drawing Robot captures a selfie from a webcam and draws
a line portrait of the subject on a fixed canvas using a Universal
Robots **UR3** with a custom 3D-printed end-effector that holds **four
markers** (red, blue, green, black). The user picks one colour in the
GUI; the wrist rotates to that marker's slot and the entire artwork is
drawn in that single colour.

The system runs on **ROS 2 Humble** and consists of three
loosely-coupled subsystems (GUI, Perception, Motion Planning) that
communicate exclusively over ROS topics. MoveIt2 plans collision-aware
Cartesian paths around the table and marker holder; URScript executes
the planned joint trajectories on the real robot or a Polyscope
simulator.

---

## 2. How the System Works (30-second mental model)

```
   ┌─────────┐   /raw_image    ┌──────────────┐ /drawing_strokes ┌────────────────┐
   │  GUI    │ ──────────────► │  Perception  │ ───────────────► │ Motion Planning│
   │ PySide6 │                 │ rembg + Canny│                  │  MoveIt2       │
   └────┬────┘ /gui/command    │              │                  │  + URScript    │
        │      START:<colour>  └──────────────┘                  └────────┬───────┘
        │                            ▲                                    │
        │   /drawing_preview_image   │                                    │
        │◄───────────────────────────┘                            TCP :30002
        │                                                                 ▼
        │             /drawing_status                                ┌─────────┐
        └─────────────────────────────────────────────────────────── │   UR3   │
                                                                     └─────────┘
```

**The 5-step flow:**

1. **Capture** — GUI grabs a webcam frame, publishes to `/raw_image`.
2. **Process** — Perception removes the background (`rembg/u2net_human_seg`),
   runs Canny edge detection (σ=3), extracts contours, simplifies them,
   and publishes a JSON list of strokes on `/drawing_strokes`.
3. **Preview** — GUI shows the stroke preview; user picks a colour and
   clicks **Start Drawing**.
4. **Plan** — Motion node reads strokes, optimises the order
   (nearest-neighbour + 2-opt), and asks MoveIt2 to plan a
   collision-aware Cartesian path for each stroke.
5. **Execute** — Planned joint waypoints are serialised to URScript
   and shipped over TCP port 30002 to the UR3 (real or simulator).

The motion node **does not auto-start** when strokes arrive — it
caches them and waits for `START:<colour>` from the GUI. This makes
the colour selection authoritative.

---

## 3. Key Features & Subsystems

| # | Subsystem | Repository | Owner | Responsibility |
|---|-----------|-----------|-------|----------------|
| 1 | **GUI** | `~/gui/` | Mateusz | Webcam capture, drawing preview, START/PAUSE/STOP buttons, status feedback |
| 2 | **Perception** | `~/perception/` | Nithish | Background removal → edge detection → stroke extraction |
| 3 | **Motion Planning** | `~/RS2/` | Domenic | Stroke optimisation → MoveIt2 planning → URScript execution, single-colour marker selection |
| 4 | **End-Effector (hardware)** | 3D printed | Mateusz | Holds four markers around the wrist axis, compliant contact mechanism |

**Capability highlights:**

- PySide6 GUI with live preview, retake, and progress feedback.
- Robust background removal using **rembg / U²-Net** (`u2net_human_seg`).
- **Canny edge detection (σ=3)** with morphological gap-closing for clean strokes.
- **Nearest-Neighbour + 2-Opt** stroke ordering (~25–30% pen-up travel saved).
- **MoveIt2 Cartesian planning** with table + marker-holder collision objects.
- **User-selectable colour** (red / blue / green / black); the wrist rotates once
  at the start to align the chosen marker, then the whole artwork is drawn in
  that single colour.
- **Wrist-continuity unwrapping** prevents ±π wrist jumps mid-drawing.
- Single launch flag switches between **Polyscope simulator** and **real UR3**.

---

## 4. Dependencies

### 4.1 Hardware (Bill of Materials)

| Item | Quantity | Notes |
|------|---------:|-------|
| Universal Robots UR3 (CB-series) | 1 | The lab's UR3 |
| UR3 Teach Pendant | 1 | Required for Remote Control mode |
| 3D-printed multi-marker holder | 1 | 4 marker holes spaced 90° around the wrist axis, each tilted 20° outward; bolts to UR3 tool flange with M6 |
| Whiteboard / drawing markers | 4 | Different colours; 12 mm shaft diameter |
| Drawing canvas (whiteboard or paper) | 1 | ≥ 200 × 150 mm flat surface |
| Worktable (rigid, level) | 1 | Robot base mounts to the table edge |
| Laptop / desktop | 1 | Runs ROS 2, MoveIt2, GUI; see Computing Specs |
| USB webcam | 1 | Any V4L2-compatible camera (laptop's built-in webcam works) |
| Ethernet cable | 1 | Robot ↔ host (or robot ↔ lab network) |

CAD/STL for the 3D-printed holder lives in the team's shared drive.
The four markers are mounted at 0°, 90°, 180°, 270° around the wrist
axis, each tilted 20° outward from perpendicular.

### 4.2 Computing Specs

Tested and confirmed working on:

- **OS:** Ubuntu 22.04.5 LTS (Linux 6.8.0)
- **CPU:** x86-64, 4+ cores recommended
- **RAM:** 8 GB minimum, 16 GB recommended
- **GPU:** Optional. rembg falls back to CPU (~2–4 s per image).
- **Network:** Ethernet to robot (or routed network). Static IP required
  for the UR3 (real-robot default `192.168.0.195`; simulator default
  `192.168.56.101`).

### 4.3 Software

| Component | Version | Why |
|-----------|---------|-----|
| ROS 2 | **Humble Hawksbill** | All nodes are ROS 2 Humble |
| MoveIt2 | matching `ros-humble-moveit` | `/compute_cartesian_path` and scene services |
| Universal Robots ROS 2 driver | `ros-humble-ur` | URDF, MoveIt2 config, `start_ursim.sh` |
| Python | 3.10 | Packaged with Ubuntu 22.04 |
| `rembg` (CPU model) | latest | Background removal |
| OpenCV | ≥ 4.5 | Edge detection, contour extraction |
| NumPy | ≥ 1.20 | Coordinate math |
| PySide6 | latest | GUI |
| `cv_bridge` (ROS) | matching ROS install | `sensor_msgs/Image` ↔ OpenCV |

---

## 5. Installation

### 5.1 Hardware Setup

1. **Bolt the 3D-printed marker holder** to the UR3 tool flange. Confirm
   the holder is centred on the flange and the four marker slots are
   oriented at 0°, 90°, 180°, 270° around the wrist axis.
2. **Insert the four markers** in this order to match the default
   colour mapping in code: slot 0° = **red**, slot 90° = **blue**,
   slot 180° = **green**, slot 270° = **black**.
   If you load them differently, edit `COLOUR_TO_MARKER` in
   [`ur3_drawing_node.py`](ros2_ws/src/ur3_motion_planning/ur3_motion_planning/ur3_drawing_node.py).
3. **Place the canvas** flat on the table, ~200 × 150 mm of usable area,
   top-left corner ≈ `(x=0.185, y=0.170, z=0.010)` m in the robot base
   frame (this is the calibrated default; see §9 if your table differs).
4. **Connect the Ethernet** cable between the host laptop and the UR3.
5. **Connect the webcam** (USB or built-in) to the host.
6. **Set the teach pendant** to **Remote Control** mode (real robot only).

### 5.2 Software Setup

```bash
# 1. ROS 2 Humble + UR + MoveIt2 (Ubuntu 22.04)
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ur \
  ros-humble-moveit \
  ros-humble-cv-bridge \
  python3-colcon-common-extensions \
  python3-rosdep
sudo rosdep init || true
rosdep update

# 2. Python packages (used by perception + GUI)
pip3 install --user "rembg[cpu]" opencv-python numpy PySide6

# 3. Clone the three subsystem repos into ~ (perception, gui, RS2)
#    After cloning, the layout should be:
#       ~/perception/    ~/gui/    ~/RS2/

# 4. Build perception
export ROS_DOMAIN_ID=42                          # ← always set this, see §6
source /opt/ros/humble/setup.bash
cd ~/perception
colcon build --packages-select selfie_perception

# 5. Build motion planning
cd ~/RS2/ros2_ws
colcon build --packages-select ur3_motion_planning

# 6. (Optional but recommended) Add to ~/.bashrc so every terminal
#    is automatically on the correct domain and workspace:
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
echo 'source ~/perception/install/setup.bash' >> ~/.bashrc
echo 'source ~/RS2/ros2_ws/install/setup.bash' >> ~/.bashrc
```

If you change `setup.py` in either workspace, clean before rebuilding:

```bash
rm -rf build/<pkg> install/<pkg>
colcon build --packages-select <pkg>
```

---

## 6. Running the System

> ### ⚠️ Lab network safety: always `export ROS_DOMAIN_ID=42`
>
> The UTS lab puts every team's laptop and robot on the **same network**.
> Without a unique ROS 2 domain, our nodes pick up other teams' topics
> (and they pick up ours). **`ROS_DOMAIN_ID=42`** is Team Picasso's
> reserved domain.
>
> **Every terminal must begin with:**
> ```bash
> export ROS_DOMAIN_ID=42
> ```

### 6.1 Full Integration (recommended)

Three terminals: **backend + GUI + (optional simulator)**.

**Terminal 1 — Backend (MoveIt2 + Perception + Motion)**

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash

# Simulator (default IP):
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui \
  launch_rviz:=true \
  robot_ip:=192.168.56.101

# OR real UR3 (replace with your robot's IP):
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui \
  launch_rviz:=true \
  robot_ip:=192.168.0.195
```

Wait ~25 s for MoveIt2 to finish loading. You'll see
`[Scene] Marker holder attached to tool0` in the terminal and the
table appear in RViz when it's ready.

**Terminal 2 — GUI**

```bash
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py
```

**Terminal 3 — UR3 Polyscope Simulator** (only if not using a real robot)

```bash
export ROS_DOMAIN_ID=42
ros2 run ur_client_library start_ursim.sh -m ur3
```

Wait ~30 s for the Docker container; visit
`http://192.168.56.101:6080/vnc.html` to view the simulator.

### 6.2 File-mode (no GUI)

Useful for testing motion planning against pre-baked strokes
without launching the GUI or webcam.

```bash
# Place an image in ~/perception/input/ (perception will auto-load it),
# OR rely on the pre-baked stroke files in ~/RS2/outputs/strokes/

export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash

# With perception + image_loader_node feeding strokes:
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=file \
  launch_rviz:=true

# OR motion-only from a pre-baked stroke JSON:
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py \
  robot_ip:=192.168.56.101 launch_rviz:=true
# In another terminal:
export ROS_DOMAIN_ID=42
ros2 run ur3_motion_planning motion_planning_node --ros-args \
  -p stroke_source:=file -p face:=face1 -p robot_ip:=192.168.56.101
```

### 6.3 Each subsystem standalone

Useful for debugging or coach demos of an individual subsystem.

**GUI only** (no ROS, fake drawing progress):
```bash
cd ~/gui && python3 selfie_drawing_gui_starter.py
```

**Perception only** (process a file, no robot):
```bash
# ROS 2 launch (place image in ~/perception/input/ first):
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
ros2 launch selfie_perception perception_pipeline.launch.py

# OR no-ROS standalone CLI:
cd ~/perception/src/selfie_perception
PYTHONPATH=.:$PYTHONPATH python3 selfie_perception/pipeline.py ../../input/your_image.png
```

**Motion only** (pre-baked strokes, simulator) — see §6.2 last block.

### 6.4 Expected outcome (with checkpoints)

1. **GUI** shows live webcam preview.
2. Click **Capture** → image is published on `/raw_image`.
   - *Checkpoint:* `ros2 topic echo /raw_image --once` returns a message.
3. **Perception** runs (background removal → Canny → strokes); 2–4 s on CPU.
4. **GUI** receives the stroke preview from `/drawing_preview_image` and shows the line drawing.
   - *Checkpoint:* GUI preview pane updates with a recognisable line portrait.
5. Pick a colour from the **Colour** dropdown (red / blue / green / black).
6. Click **Start Drawing**. The motion node:
   - parses the colour out of the `START:<colour>` command,
   - rotates wrist_3 to the matching slot,
   - plans + executes every stroke in that single colour.
7. **GUI status** updates from `WAITING_FOR_PERCEPTION` → `OPTIMIZING_PATH`
   → `PLANNING_WITH_MOVEIT2` → `EXECUTING` → `COMPLETE`.

**Verifying the colour data flow.** (no-screenshot quick check) When you click Start Drawing,
the motion node should print these four log lines in order. If any
is missing or shows the wrong colour, the chain is broken at that step:

```
[GUI] >>> Received raw command: 'START:blue'
[GUI] Parsed command='START' payload='blue'
[GUI] ✓ Colour set to 'blue' (marker slot 2/4)
[Plan] >>> Pipeline reading colour: 'blue' (marker_idx=1, wrist_3_offset=-90.0°)
```

> **Note — no auto-start.** The motion node intentionally does *not*
> start drawing the moment perception strokes arrive on
> `/drawing_strokes`. Strokes are cached and the pipeline only fires
> when the user clicks Start Drawing (which sends `START:<colour>`).
> This makes the colour selection authoritative.

---

## 7. Subsystem Reference

### 7.1 GUI Subsystem

**Location:** `~/gui/`   **Owner:** Mateusz   **Subsystem README:** [`~/gui/readme.txt`](../gui/readme.txt)

**Purpose.** Operator interface — capture a selfie, preview the drawing,
start/pause/stop the robot, see live status.

**Files:**

- `selfie_drawing_gui_starter.py` — standalone PySide6 GUI (no ROS).
  Used for offline UI testing; uses *simulated* drawing progress.
- `selfie_drawing_gui_ros2.py` — subclass of the above that wires the
  buttons to ROS 2 topics. **This is the file you run for the integrated system.**

**Inputs:**

| Direction | Topic | Type | Source |
|-----------|-------|------|--------|
| Subscribe | `/drawing_strokes` | `std_msgs/String` (JSON) | perception_node |
| Subscribe | `/drawing_status` | `std_msgs/String` | ur3_drawing_node |
| Subscribe | `/drawing_preview_image` | `sensor_msgs/Image` | perception_node |

**Outputs:**

| Direction | Topic | Type | Sink |
|-----------|-------|------|------|
| Publish | `/raw_image` | `sensor_msgs/Image` | perception_node |
| Publish | `/gui/command` | `std_msgs/String` | ur3_drawing_node |
| Publish | `/gui/marker_colour` | `std_msgs/String` | ur3_drawing_node (status consumers) |

Commands published on `/gui/command`: **`START:<colour>`** (e.g.
`START:blue`), `PAUSE`, `RESUME`, `STOP`. The motion node parses the
`<colour>` suffix out of the start command — this is the
**authoritative** source of marker selection.

Values published on `/gui/marker_colour` (lowercase): `red`, `blue`,
`green`, `black`. Published in parallel with `START:<colour>` for
status/debug consumers. The motion node's drawing pipeline does
**not** rely on this topic — it uses the suffix in `START:<colour>`
to avoid a callback-ordering race in the multi-threaded executor.

**How to run independently:**

```bash
# Standalone (no ROS, fake drawing):
cd ~/gui && python3 selfie_drawing_gui_starter.py

# ROS 2 integrated (needs perception + motion running):
export ROS_DOMAIN_ID=42
python3 ~/gui/selfie_drawing_gui_ros2.py
```

**Configurable settings:**

- Camera index in `CameraHandler(camera_index=N)` (default `0`).
- Window size in `MainWindow.resize(1280, 780)`.
- Available colours in `self.colour_combo.addItems([...])` in
  `selfie_drawing_gui_starter.py`. To add a colour, also add an entry
  to `COLOUR_TO_MARKER` in the motion node.

**Known limitations:**

- One webcam at a time; GUI must be restarted to switch cameras.
- No persistent settings — combo boxes (subject/style/detail) are
  visual placeholders and do not affect perception parameters yet.

---

### 7.2 Perception Subsystem

**Location:** `~/perception/`   **Owner:** Nithish   **ROS 2 package:** `selfie_perception`   **Subsystem README:** [`~/perception/README.md`](../perception/README.md)

**Purpose.** Convert a selfie image into a list of vector strokes
(JSON) suitable for the UR3 to draw.

**Pipeline:**

```
Selfie (BGR)
   │
   ▼  rembg / U²-Net (u2net_human_seg)
Background-removed RGBA → BGR on white
   │
   ▼  Gaussian σ=3 + Canny (low=50, high=150)
Binary edge map (uint8 0/255)
   │
   ▼  Morphological closing (7×7 ellipse, 2 iter) → cv2.findContours
   ▼  Douglas-Peucker simplify (ε=1.5)
   ▼  Scale to 400×300 px canvas
   ▼  Greedy nearest-neighbour reorder
List[List[[x,y]]]  →  /drawing_strokes (JSON)
```

**Nodes (all in the `selfie_perception` package):**

| Node | Purpose | Subscribes | Publishes |
|------|---------|-----------|-----------|
| `image_loader_node` | Watches `~/perception/input/` and publishes images (file mode, no GUI) | — | `/raw_image` |
| `perception_node` | Full pipeline | `/raw_image` | `/drawing_strokes`, `/drawing_preview_image` |
| `visualization_node` | Renders the stroke preview separately (optional) | `/drawing_strokes` | `/drawing_preview_image` |

**File outputs (all under `~/perception/output/`):**

- `perception_strokes.json` — latest stroke set (motion fallback)
- `drawing_preview.png` — most recent preview render
- `run_NNN/` — per-run subfolder with intermediate images

**How to run independently:**

```bash
# ROS 2 launch (place image in ~/perception/input/ first):
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
ros2 launch selfie_perception perception_pipeline.launch.py

# Standalone CLI (no ROS):
cd ~/perception/src/selfie_perception
PYTHONPATH=.:$PYTHONPATH python3 selfie_perception/pipeline.py ../../input/your_image.png

# Monitor strokes:
ros2 topic echo /drawing_strokes
```

**Configurable parameters** (set via `--ros-args -p name:=value`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gaussian_sigma` | 3.0 | Pre-blur σ (controls level of detail) |
| `canny_low` / `canny_high` | 50 / 150 | Canny hysteresis thresholds |
| `morph_kernel_size` | 7 | Morphological closing kernel size (closes gaps) |
| `morph_iterations` | 2 | Closing iterations |
| `simplification_epsilon` | 1.5 | Douglas-Peucker tolerance |
| `min_contour_points` | 8 | Drop contours below this length |
| `canvas_px_w`, `canvas_px_h` | 400, 300 | Output canvas |

**Known limitations & assumptions:**

- `rembg` only segments **human** subjects well (model: `u2net_human_seg`).
  Drawings of pets/objects need a different model.
- Output canvas is fixed to **400 × 300 pixels** — a contract shared
  with the motion subsystem. Changing it requires updating
  `CANVAS_PX_W/H` in `~/RS2/src/motion_planning_lib.py` too.
- First run downloads the rembg model (~170 MB); subsequent runs are fast.

---

### 7.3 Motion Planning Subsystem

**Location:** `~/RS2/`   **Owner:** Domenic   **ROS 2 package:** `ur3_motion_planning`

**Purpose.** Receive strokes and a colour selection, plan
collision-safe Cartesian paths with MoveIt2, rotate the wrist to the
chosen marker, and execute on the UR3 via URScript.

**Pipeline:**

```
inputs/*.svg ──[svg_to_json_converter.py]──► outputs/strokes/*.json
                                                       │
            (perception_node, external pkg)            │ (file mode)
                  │                                    │
                  └────────── /drawing_strokes ────────┤
                                                       │
                                                       ▼
GUI ── /gui/command (START:<colour>) ─────► ur3_drawing_node.py ◄── imports ── motion_planning_lib.py
                                              │      │                          (constants + algorithms)
                       scene_publisher.py ────┘      │
                       (table + holder via           │
                        /apply_planning_scene)       │
                                                     │
                                         MoveIt2 ────┤ (collision-aware plans)
                                                     │
                                                     ▼
                                       outputs/last_drawing.script
                                                     │
                                             TCP socket :30002
                                                     ▼
                                             UR3 / Polyscope
```

**Architecture: MoveIt2 plans, URScript executes.** The motion node uses a
hybrid model — MoveIt2 does the *planning* (collision-aware Cartesian
paths), URScript does the *execution* (movej commands sent over TCP):

```
Strokes (JSON)
     ↓
Optimise (NN + 2-Opt)
     ↓
Convert to Cartesian waypoints (xyz positions)
     ↓
Call /compute_cartesian_path (MoveIt2)  ← COLLISION-AWARE
     ↓
Planned joint-space trajectory
     ↓
Apply wrist_3 colour offset + unwrap
     ↓
Convert to URScript (movej commands)
     ↓
TCP socket → UR3 (simulator or real)
```

Why this split:

- **Collision safety** — MoveIt2 knows about the table and the marker
  holder, so it never plans paths that crash.
- **Correct tool orientation** — the quaternion in each Cartesian
  waypoint guarantees the 20° marker tilt is maintained throughout.
- **Lightweight execution** — URScript over TCP works on both the
  simulator and the real robot without depending on the UR action
  server staying healthy. The last script can be inspected at
  `outputs/last_drawing.script`.

**How strokes become URScript (per-stroke, not all-at-once).** The node
never hands MoveIt2 the whole drawing in one call. For each stroke it
makes **three separate `/compute_cartesian_path` calls**:

1. **Travel** — one waypoint above the stroke start, pen-up
2. **Draw** — every stroke point at `Z_DRAW`, pen-down
3. **Lift** — one waypoint above the stroke end

Each call gets the previous call's final joint state as its start
state, so the trajectories stitch together seamlessly into one
continuous motion. Splitting it three ways isolates pen-up vs pen-down
regions (different Z heights, different thinning rules) and lets a
single failed stroke be skipped instead of killing the whole drawing.

After each call returns, `_thin_trajectory` reduces the trajectory:
for travel/lift it keeps **only the endpoint** (pen is in the air, no
need for intermediate joint configs). For the draw call it keeps
**every point** MoveIt2 produced (so the robot's `movej` chain follows
the exact Cartesian path instead of joint-space-shortcutting through
the canvas). The thinned segments are concatenated, the wrist_3 offset
is applied, wrist_3 targets are unwrapped for continuity, and the
result is serialised to a single URScript program.

**Tilted tool & TCP offset.** The marker holder is bolted to `tool0`
and extends the marker tip **11.5 cm below the flange** along the
holder axis (`EE_DRAW_HEIGHT=0.115 m`), tilted **20° from
perpendicular**. This creates a TCP offset that must be known to both
planning and execution.

Offset values (robot base frame, meters):

```
TCP_X  = 0.0393 m   (forward, due to 20° tilt)
TCP_Y  = 0.0 m      (centred)
TCP_Z  = -0.1081 m  (downward component of 11.5 cm marker reach)
Rotation = [20° around Y-axis]
```

*How MoveIt2 knows about the offset.* The marker holder is published
as an `AttachedCollisionObject` linked to `tool0`:

```python
attached = AttachedCollisionObject()
attached.link_name = "tool0"
attached.object = marker_holder_box
attached.touch_links = ["tool0", "wrist_3_link"]   # ignore self-collision
```

MoveIt2 carries this collision shape with the end effector during
planning, so paths avoid hitting objects with the marker.

*How URScript applies the offset.* Every generated program begins:

```
def draw_face():
  set_tcp(p[0.0393, 0.0, -0.1081, 0.0, 0.3491, 0.0])
  # ... rest of program ...
end
```

This is informational for the controller — the `movej` joint targets
are already baked from MoveIt2's IK, so `set_tcp()` does not change
the motion, but it keeps the pendant's TCP indicator correct.

**Single-colour drawing & wrist_3 unwrapping.** The 3D-printed holder
carries **four markers** at 0°, 90°, 180°, 270° around the wrist_3 axis,
each tilted 20° outward.

*Selection.* The GUI sends `START:<colour>` on `/gui/command`. The
motion node parses the colour, looks it up in `COLOUR_TO_MARKER` to
get a slot index, and uses that slot for the whole drawing.

*Implementation.* The trajectory is planned with marker 1's orientation
(`TOOL_QUAT`) for *every* stroke — making the plan deterministic and
independent of the chosen colour. The wrist_3 offset for the chosen
marker (`marker_idx × -90°`, canonicalised to the nearest equivalent
angle) is then applied as a **post-processing step** that adds the
offset to the 6th joint of every output waypoint. Because the holder
is rotationally symmetric, that single-axis rotation physically
swings the chosen marker into the position marker 1 was tracing —
`CANVAS_ORIGIN_ROBOT`, `px_to_robot()`, and `set_tcp()` all stay the same.

*Unwrapping.* After the offset is applied, each wrist_3 waypoint is
unwrapped to the equivalent angle nearest the previous command. This
preserves the same physical marker orientation but avoids
wrap-boundary jumps such as `+3.13 → -3.13`, which would otherwise
command an almost full wrist rotation while the marker is touching
the canvas.

A separate "rotate-only" `movej` is prepended to the URScript so the
wrist visibly swaps to the chosen marker *before* any horizontal
motion toward the canvas.

Default colour-to-slot mapping (edit `COLOUR_TO_MARKER` in
`ur3_drawing_node.py` to match your physical loading order):

| Colour | Slot index | Holder angle | wrist_3 offset |
|--------|-----------:|-------------:|---------------:|
| red    | 0          | 0°           | 0° |
| blue   | 1          | 90°          | -90° |
| green  | 2          | 180°         | -180° |
| black  | 3          | 270°         | +90° (canonical equivalent of -270°) |

*Default colour* — if a `START` arrives without a suffix and no
`/gui/marker_colour` was received, the motion node uses
`DEFAULT_COLOUR` (currently `"black"`). This keeps file-mode and
no-GUI test runs working.

**Nodes:**

| Node | Purpose |
|------|---------|
| `motion_planning_node` (alias of `ur3_drawing_node.py`) | Main orchestrator |
| `scene_publisher` (alias of `scene_publisher.py`) | Publishes `table` and `marker_holder` collision objects to MoveIt2 via `/apply_planning_scene` |

**Subscribes:**

| Topic | Type | Notes |
|-------|------|-------|
| `/drawing_strokes` | `std_msgs/String` (JSON) | When `stroke_source=topic`. **Cached, does not auto-start.** |
| `/gui/command` | `std_msgs/String` | `START:<colour>` (trigger), `PAUSE`, `RESUME`, `STOP` |
| `/gui/marker_colour` | `std_msgs/String` | Supplementary; defaults to `black` if `START:<colour>` has no suffix |

**Publishes:**

| Topic | Type | Notes |
|-------|------|-------|
| `/drawing_status` | `std_msgs/String` | Pipeline state |
| `/joint_states` | `sensor_msgs/JointState` | 10 Hz, mirrors planned trajectory |
| `/trajectory_preview` | `geometry_msgs/PoseArray` | RViz only |

**Saved file:** `~/RS2/outputs/last_drawing.script` — the last URScript
program sent to the robot. Useful for inspection and offline replay.

**How to run independently:**

```bash
# Terminal 1 — MoveIt2 + scene setup
export ROS_DOMAIN_ID=42
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py \
  robot_ip:=192.168.56.101 launch_rviz:=true

# Terminal 2 — motion node, load face1 from disk and run
export ROS_DOMAIN_ID=42
ros2 run ur3_motion_planning motion_planning_node --ros-args \
  -p stroke_source:=file -p face:=face1 -p robot_ip:=192.168.56.101
```

**Quick smoke test (no ROS, no MoveIt2)** — `motion_planning_lib.py`
also runs as a stripped-down standalone CLI:

```bash
python3 ~/RS2/src/motion_planning_lib.py 1 1   # face 1, run 1
```

This emits a `movel` URScript (no MoveIt2 collision planning, no
colour selection — always slot 0°). Treat it as a fast check for
stroke parsing + optimisation + basic UR3 execution. Use the ROS
node for the real integrated workflow.

**Configurable parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_ip` | `192.168.56.101` | UR3 (real) or simulator IP |
| `robot_port` | `30002` | URScript primary interface |
| `stroke_source` | `file` | `file` or `topic` |
| `face` | `face1` | When loading from disk |
| `enable_optimization` | `true` | NN + 2-Opt toggle |
| `max_step` | `0.005` | MoveIt2 Cartesian interpolation (m) |
| `jump_threshold` | `5.0` | MoveIt2 joint-space jump filter |
| `planning_timeout` | `30.0` | Per-stroke planning timeout (s) |

**Calibration and motion constants** (in `~/RS2/src/motion_planning_lib.py`):

| Constant | Default | Description |
|----------|---------|-------------|
| `CANVAS_ORIGIN_ROBOT` | `[0.185, 0.170, 0.010] m` | Top-left canvas corner in robot base frame |
| `CANVAS_WIDTH_M`, `CANVAS_HEIGHT_M` | `0.150 m, 0.120 m` | Canvas size |
| `EE_DRAW_HEIGHT` | `0.115 m` | EE height above canvas surface |
| `MARKER_TILT_DEG` | `20°` | Marker tilt from perpendicular |
| `JOINT_ACCEL`, `JOINT_VEL` | `2.00 rad/s²`, `2.50 rad/s` | Fast travel / pen-up `movej` profile |
| `LINEAR_ACCEL`, `LINEAR_VEL` | `1.20 m/s²`, `0.35 m/s` | Standalone legacy `movel` profile |
| `DRAW_JOINT_ACCEL`, `DRAW_JOINT_VEL` | `1.10 rad/s²`, `0.70 rad/s` | Pen-down `movej` profile (defined in `ur3_drawing_node.py`) |

**Known limitations & assumptions:**

- The colour-to-slot mapping (`COLOUR_TO_MARKER`) is hard-coded; physical
  marker order must match.
- Only one colour per drawing — the original "cycle through 4 markers
  per stroke" mode has been replaced.
- Holder collision geometry is approximated as a single 160 × 180 mm box.
- URScript only contains `movej` commands. `set_tcp()` is set as a
  courtesy but does not affect `movej` motion (joints are already
  baked from MoveIt2's IK).
- Collision avoidance is enforced **at planning time**. URScript
  execution itself is open-loop.

---

### 7.4 End-Effector / Marker Holder

**Owner:** Mateusz

**Purpose.** Allows the UR3 to draw using one of four physical markers
mounted around the wrist axis. The selected colour from the GUI is
mapped to a marker slot, and the robot rotates `wrist_3` to align the
chosen marker with the canvas.

The attachment is 3D-printed and bolts directly onto the UR3 tool
flange using M6 bolts. It holds four markers at fixed angular positions
(0°, 90°, 180°, 270°) and includes a compliant mechanism near the
marker interface. This compliance maintains consistent marker contact
with the drawing surface while limiting force on the marker tip,
the paper, and the robot during small height variations or
calibration error.

**Design requirements:**

- Securely hold four standard drawing markers.
- Allow repeatable marker positioning for colour selection.
- Maintain a consistent drawing angle and contact point (20° tilt).
- Provide slight mechanical compliance during contact with the canvas.
- Avoid collisions with the table, canvas, and robot wrist during drawing.

**Known limitations:**

- Marker colour order must match the software mapping (`COLOUR_TO_MARKER`).
- The compliant mechanism is designed for max 20 mm deflections only.

---

## 8. Cross-Subsystem Interfaces

The two contracts that all three subsystems share — the ROS 2 topic
map and the stroke data format. (Implementation deep-dives that used
to live here have moved into the relevant subsystem section: the
MoveIt2 + URScript hybrid, the TCP offset, and the wrist_3 unwrapping
are all in §7.3.)

### 8.1 Complete ROS 2 Topic Map

| Topic | Message Type | Publisher(s) | Subscriber(s) | Description |
|-------|--------------|--------------|---------------|-------------|
| `/raw_image` | `sensor_msgs/Image` | GUI node *or* image_loader_node | perception_node, visualization_node (for reset) | Raw camera frame (BGR8) |
| `/drawing_strokes` | `std_msgs/String` (JSON) | perception_node | ur3_drawing_node, GUI node, visualization_node | Stroke array `[[[x,y],…],…]` in 400×300 px |
| `/drawing_preview_image` | `sensor_msgs/Image` | perception_node, visualization_node | GUI node | Rendered preview of the planned drawing |
| `/drawing_status` | `std_msgs/String` | ur3_drawing_node | GUI node | Pipeline state (WAITING, EXECUTING, COMPLETE, ERROR, …) |
| `/trajectory_preview` | `geometry_msgs/PoseArray` | ur3_drawing_node | RViz | 3D waypoint visualisation |
| `/gui/command` | `std_msgs/String` | GUI node | ur3_drawing_node | `START:<colour>`, `PAUSE`, `RESUME`, `STOP` — authoritative trigger |
| `/gui/marker_colour` | `std_msgs/String` | GUI node | ur3_drawing_node | Same colour value, parallel; for status/debug consumers |
| `/joint_states` | `sensor_msgs/JointState` | ur3_drawing_node | RViz / MoveIt2 | 10 Hz mirror of planned trajectory |

### 8.2 Data format contract (stroke JSON)

All three subsystems agree on this stroke format:

```json
[
  [[x1, y1], [x2, y2], ...],
  [[x1, y1], [x2, y2], ...]
]
```

- **Canvas:** 400 × 300 pixels
- **Origin:** top-left, Y increases downward
- **Coordinates:** float, 1 decimal place
- **Transported via:** `/drawing_strokes` topic (`std_msgs/String` with JSON) or file

Defined once. Do not introduce translation layers — change
`CANVAS_PX_W/H` in both perception and motion if you ever need to
resize.

---

## 9. Configuration & Calibration

### Canvas calibration (most common change)

If the canvas position or size differs from the default, edit
`~/RS2/src/motion_planning_lib.py` (~lines 36–38):

```python
CANVAS_ORIGIN_ROBOT = np.array([0.185, 0.170, 0.010])  # m, robot base frame
CANVAS_WIDTH_M      = 0.150                            # 15 cm
CANVAS_HEIGHT_M     = 0.120                            # 12 cm
```

**Finding the values.** Jog the robot tip to the **top-left** corner
of the canvas (with marker 1 active), record the X/Y/Z reading from
the teach pendant, and update the constant. Width and height are
the physical canvas extents.

After editing, rebuild:

```bash
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning
```

### Motion speeds

| Motion type | Acceleration | Velocity | Defined in |
|-------------|-------------:|---------:|------------|
| Travel / pen-up / home `movej` | `2.00 rad/s²` | `2.50 rad/s` | `motion_planning_lib.py` (`JOINT_ACCEL`, `JOINT_VEL`) |
| Drawing / pen-down `movej` | `1.10 rad/s²` | `0.70 rad/s` | `ur3_drawing_node.py` (`DRAW_JOINT_ACCEL`, `DRAW_JOINT_VEL`) |
| Standalone legacy `movel` path | `1.20 m/s²` | `0.35 m/s` | `motion_planning_lib.py` (`LINEAR_ACCEL`, `LINEAR_VEL`) |

Travel moves are intentionally faster than drawing moves. Drawing is
slower to reduce marker chatter and preserve line quality while the
marker is touching the canvas.

### Marker holder geometry

Edit `EE_DRAW_HEIGHT` (length from flange to marker tip along the
tilted axis) and `MARKER_TILT_DEG` if you 3D-print a different
holder. The collision-object dimensions are also defined in
`scene_publisher.py` and `ur3_drawing_node.py::_publish_scene_objects`
— update them together.

---

## 10. Troubleshooting & FAQs

| Symptom | Likely cause / fix |
|---------|--------------------|
| Strokes from another team appear / our START fires another team's robot / topics flicker between values | **`ROS_DOMAIN_ID` mismatch.** Every terminal must `export ROS_DOMAIN_ID=42` before any `ros2`/`colcon`/`python3` command. Verify with `echo $ROS_DOMAIN_ID`. Add to `~/.bashrc` to make it sticky. |
| GUI never receives the perception preview / motion never receives strokes | Same domain mismatch — usually one terminal is missing `export ROS_DOMAIN_ID=42`. The integrated launch file sets the domain *only inside its own process tree*; external terminals must set it manually. |
| Build fails with `colcon: command not found` | Source ROS 2: `source /opt/ros/humble/setup.bash` |
| `move_group` not available; `/compute_cartesian_path` unavailable | MoveIt2 still loading. Wait ≥ 25 s after launching. The launch file already delays the scene + motion nodes with `TimerAction`, but the GUI or external triggers may need to wait too. |
| `ConnectionRefusedError 192.168.56.101:30002` | Polyscope simulator isn't running. Start it: `ros2 run ur_client_library start_ursim.sh -m ur3`. |
| Robot is connected but does nothing | Real UR3: pendant must be in **Run Program** mode (Remote Control). |
| Robot makes a protective stop on first move | Reduce the motion speed constants (§9), especially `JOINT_ACCEL` / `JOINT_VEL` for pen-up travel. Verify `CANVAS_ORIGIN_ROBOT` is reachable. |
| Strokes are drawn off the canvas | Recalibrate `CANVAS_ORIGIN_ROBOT` (§9). |
| Wrist spins while drawing / marker smears across the canvas | Rebuild and relaunch so `ur3_drawing_node.py` uses the wrist_3 continuity unwrapping. Do not replay an older `outputs/last_drawing.script` generated before the fix. |
| Robot drew with the wrong colour | The mapping `colour → slot` is in `COLOUR_TO_MARKER` in `ur3_drawing_node.py`. Either reload the markers in the slot order matching the dict, or edit the dict to match your physical loading. |
| Pipeline never starts after Process | The motion node intentionally does **not** auto-start when perception strokes arrive. Click **Start Drawing**; strokes are cached and used as soon as you press it. |
| GUI doesn't see the perception preview | Confirm `ROS_DOMAIN_ID=42` is set in the GUI terminal *and* the launch file. Both must match. |
| GUI camera shows "Disconnected" | Camera index conflict; close other apps using the webcam, or change `CameraHandler(camera_index=1)`. |
| `rembg` slow on first run | Model download (~170 MB). Subsequent runs are fast (~2–4 s on CPU). |
| Phantom robot in RViz | Only one source should publish `/joint_states`. The motion node does this — make sure no `joint_state_publisher` is launched separately. |
| RViz shows red collision flash | A planned waypoint touches the table or holder. Check `scene_publisher` ran successfully (look for `Table collision object applied`). |
| `/compute_cartesian_path fraction < 0.5` | Travel pose unreachable. Verify the canvas calibration; reduce `max_step` (smaller steps allow MoveIt2 more flexibility). |
| Entry-point script missing after rebuild | If you changed `setup.py`, clean: `rm -rf build/<pkg> install/<pkg>` then `colcon build --packages-select <pkg>`. |

### FAQs

**Q. Can I run the system without the real robot?**
Yes — start the Polyscope simulator (`ros2 run ur_client_library start_ursim.sh -m ur3`)
and use `robot_ip:=192.168.56.101`. Everything else works identically.

**Q. Can I test perception without the robot or simulator?**
Yes — use the standalone CLI (`python3 -m selfie_perception.pipeline`).
Outputs go to `~/perception/output/run_NNN/`.

**Q. Can I test motion planning without perception?**
Yes — use `stroke_source:=file -p face:=face1` to load pre-baked
strokes from `~/RS2/outputs/strokes/`.

**Q. Why does the motion node not auto-start when strokes arrive?**
Because the colour selection must come from the GUI's
`START:<colour>` command. Caching strokes and waiting for an
explicit start avoids a callback-ordering race.

**Q. What if I want to use different colours?**
Add them to `self.colour_combo.addItems([...])` in
`selfie_drawing_gui_starter.py`, and add a matching entry to
`COLOUR_TO_MARKER` in `ur3_drawing_node.py`. Reload markers in the
holder accordingly.

---

## 11. Project Layout

```
~/RS2/                                       ← Motion planning + integration docs
├── README.md                                ← Quick start (links here)
├── TECHNICAL_DOCUMENTATION.md               ← (this file — full reference)
├── inputs/face*.svg                         ← Raw vector drawings (test input)
├── outputs/
│   ├── strokes/face*.json                   ← Pre-baked strokes for offline runs
│   ├── verified/face*.svg                   ← Reference outputs
│   └── last_drawing.script                  ← Most recent URScript program
├── src/
│   ├── motion_planning_lib.py               ← Shared constants + algorithms
│   └── svg_to_json_converter.py             ← Convert SVG → stroke JSON
├── test_results/Face{1,2,3}/                ← Captured run outputs
└── ros2_ws/src/ur3_motion_planning/
    ├── launch/
    │   ├── integrated_pipeline.launch.py    ← perception + MoveIt2 + motion
    │   └── ur3_motion_planning_moveit2.launch.py  ← motion only
    └── ur3_motion_planning/
        ├── ur3_drawing_node.py              ← ROS 2 node (planning + execution)
        └── scene_publisher.py               ← Collision scene → MoveIt2

~/perception/                                ← Perception subsystem
├── README.md                                ← Subsystem-specific README
├── input/                                   ← Drop selfies here for file mode
├── output/                                  ← perception_strokes.json + run_NNN/
└── src/selfie_perception/
    ├── launch/perception_pipeline.launch.py
    └── selfie_perception/
        ├── background_removal.py
        ├── edge_detection.py
        ├── stroke_extraction.py
        ├── pipeline.py                      ← Standalone CLI (no ROS)
        ├── perception_node.py
        ├── image_loader_node.py
        └── visualization_node.py

~/gui/                                       ← GUI subsystem
├── readme.txt                               ← Subsystem-specific install notes
├── selfie_drawing_gui_starter.py            ← No-ROS prototype
└── selfie_drawing_gui_ros2.py               ← ROS 2 integrated (run this)
```

---

**Maintainers:** Team Picasso — Mateusz Kopaczynski (GUI / End-Effector),
Nithish Kannan Bhagavathi Sankaranarayanan (Perception), Domenic Kadioglu (Motion).

**Last verified:** May 2026 against ROS 2 Humble + MoveIt2 + UR ROS 2 driver.
