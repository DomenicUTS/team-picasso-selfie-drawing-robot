# Full System Integration — GUI + Perception + Motion Planning

**Project:** UR3 Selfie Drawing Robot — Team Picasso
**Last updated:** 3 May 2026
**Contributors:** Mateusz Kopaczynski (GUI/End Effector), Nithish Kannan Bhagavathi Sankaranarayanan (Perception), Domenic Kadioglu (Motion)




---

## Always set `ROS_DOMAIN_ID=42` (UTS lab)
 see [README.md](README.md).

---

## Prerequisites & Setup

**Important:** After any code changes, you must rebuild the affected packages before running:

```bash
# For motion planning changes:
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# For perception changes:
cd ~/perception && colcon build --packages-select selfie_perception

# For both (when changes span subsystems):
cd ~/perception && colcon build --packages-select selfie_perception
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning
```

**Key files for ROS 2 Python packages:**
- `setup.py` — Entry points for `ros2 run`
- `setup.cfg` — Tells colcon to install executables to `lib/<pkg>/` for `ros2 run` to find them
- `package.xml` — Package dependencies and metadata

If you add or modify entry points in `setup.py`, you may need to clean the build cache:
```bash
# Motion planning:
rm -rf ~/RS2/ros2_ws/build/ur3_motion_planning ~/RS2/ros2_ws/install/ur3_motion_planning
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Perception:
rm -rf ~/perception/build/selfie_perception ~/perception/install/selfie_perception
cd ~/perception && colcon build --packages-select selfie_perception
```

---

## System Overview

## ROS 2 Topics — Complete Map

| Topic | Message Type | Publisher(s) | Subscriber(s) | Description |
|-------|-------------|-------------|---------------|-------------|
| `/raw_image` | `sensor_msgs/Image` | **GUI** node or **image_loader_node** | perception_node, visualization_node (reset) | Raw camera frame (BGR8) |
| `/drawing_strokes` | `std_msgs/String` | perception_node | **ur3_drawing_node**, **GUI** node, visualization_node | JSON stroke array `[[[x,y],…],…]` in 400×300 px |
| `/drawing_preview_image` | `sensor_msgs/Image` | perception_node, visualization_node | **GUI** node | Rendered preview of what robot will draw |
| `/drawing_status` | `std_msgs/String` | **ur3_drawing_node** | **GUI** node | Pipeline state (WAITING, EXECUTING, COMPLETE, ERROR, …) |
| `/trajectory_preview` | `geometry_msgs/PoseArray` | ur3_drawing_node | RViz | 3D waypoint visualisation |
| `/gui/command` | `std_msgs/String` | **GUI** node | **ur3_drawing_node** | `START:<colour>` (e.g. `START:blue`), `PAUSE`, `RESUME`, `STOP`. The colour suffix is the authoritative source of the marker selection. |
| `/gui/marker_colour` | `std_msgs/String` | **GUI** node | **ur3_drawing_node** | Same colour value, published in parallel for debug/status consumers. The motion node uses the suffix in `START:<colour>` for the actual selection (avoids a callback ordering race). |

---

## Each Module's Inputs and Outputs

### GUI (`~/gui/`)

| Direction | Topic | What |
|-----------|-------|------|
| **Publishes** | `/raw_image` | Captured photo from laptop webcam |
| **Publishes** | `/gui/command` | `START:<colour>` (where `<colour>` is one of `red` / `blue` / `green` / `black`), and `PAUSE` / `RESUME` / `STOP` |
| **Publishes** | `/gui/marker_colour` | Same colour value sent in parallel — kept for any subscriber that wants the raw selection without parsing the start command |
| **Subscribes** | `/drawing_strokes` | Receives stroke JSON → renders preview |
| **Subscribes** | `/drawing_status` | Progress/state updates from motion |
| **Subscribes** | `/drawing_preview_image` | Preview image from perception visualisation |

**Files:**
- `selfie_drawing_gui_starter.py` — Original standalone GUI (no ROS, fake drawing)
- `selfie_drawing_gui_ros2.py` — ROS 2 integrated GUI (inherits from starter)

### Perception (`~/perception/`)

| Direction | Topic | What |
|-----------|-------|------|
| **Subscribes** | `/raw_image` | Input photo (from GUI or image_loader_node) |
| **Publishes** | `/drawing_strokes` | JSON strokes after bg removal + edge detection + stroke extraction |
| **Publishes** | `/drawing_preview_image` | Rendered preview of strokes |
| **File output** | `~/perception/output/perception_strokes.json` | Same strokes saved to disk |

**Nodes:**
- `image_loader_node` — Watches `~/perception/input/` for images (standalone testing, publishes up to 5 times)
- `perception_node` — Full pipeline: background removal (rembg/u2net_human_seg) → Gaussian blur + Canny edge detection (σ=3) → contour extraction, Douglas-Peucker simplification, morphological closing, nearest-neighbour stroke ordering. Publishes strokes and preview.
- `visualization_node` — Subscribes to `/drawing_strokes`, renders stroke preview, publishes to `/drawing_preview_image`, saves `drawing_preview.png`
- `pipeline` — Standalone CLI tool (no ROS) for running the full pipeline on an image file

### Motion Planning (`~/RS2/`)

| Direction | Topic | What |
|-----------|-------|------|
| **Subscribes** | `/drawing_strokes` | Receives strokes from perception |
| **Subscribes** | `/gui/command` | START / PAUSE / RESUME / STOP from GUI |
| **Publishes** | `/drawing_status` | Pipeline state updates |
| **Publishes** | `/trajectory_preview` | PoseArray for RViz |
| **File input** | `~/RS2/outputs/strokes/face*_strokes.json` | Pre-made strokes (standalone mode) |

**Nodes:**
- `ur3_drawing_node` — Full pipeline: load → optimise → plan → execute

---

## How to Run

### Mode 1: Full Integration (GUI + Perception + Motion with MoveIt2)

**Simulator (default, with MoveIt2 collision planning):**
```bash
# Build both packages (required after any code changes)
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
cd ~/perception && colcon build --packages-select selfie_perception
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Terminal 1 — Start the backend pipeline (MoveIt2 + Perception + Motion)
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui \
  launch_rviz:=true

# Terminal 2 — Start the GUI
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py

# Terminal 3 (optional) — Start the UR3 simulator
export ROS_DOMAIN_ID=42
ros2 run ur_client_library start_ursim.sh -m ur3
```

**Real Robot (replace IP with your UR3 address):**
```bash
# Build both packages (required after any code changes)
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
cd ~/perception && colcon build --packages-select selfie_perception
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Terminal 1 — Start the backend pipeline
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui \
  robot_ip:=192.168.0.195 \
  launch_rviz:=true

# Terminal 2 — Start the GUI
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py
```

**Workflow:**
1. GUI shows live webcam preview
2. Click "Capture" → sends image to perception
3. Perception processes: background removal (rembg) → Canny edge detection (σ=3) → stroke extraction → publishes strokes
4. GUI shows preview of what will be drawn
5. Click "Start Drawing" button
6. Motion node receives strokes, plans collision-safe MoveIt2 trajectories, executes via URScript
7. Robot draws the face at 20° angle using the **colour selected in the GUI** (red / blue / green / black). The wrist rotates once at the start to align the chosen marker with the canvas; wrist_3 targets are then unwrapped for continuity and the entire artwork is drawn in that single colour.

### Mode 2: Perception + Motion with MoveIt2 (no GUI)

```bash
# Build both packages (required after any code changes)
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
cd ~/perception && colcon build --packages-select selfie_perception
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Place an image in ~/perception/input/
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=file \
  launch_rviz:=true
```

The `image_loader_node` will automatically load and publish images, then perception_node processes → strokes published → motion node plans and executes.

### Mode 3: Each Subsystem Standalone

**GUI only** (fake processing, no ROS needed):
```bash
cd ~/gui
python3 selfie_drawing_gui_starter.py
```

**Perception only** (process an image, no robot):
```bash
# Via ROS 2 launch (place image in ~/perception/input/ first):
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
cd ~/perception && colcon build --packages-select selfie_perception
ros2 launch selfie_perception perception_pipeline.launch.py

# OR standalone (no ROS at all):
python3 -m selfie_perception.pipeline ~/perception/input/your_image.png
# OR auto-pick latest image:
python3 -m selfie_perception.pipeline
```

**Motion only (MoveIt2 planning from pre-made strokes):**
```bash
# Build the motion planning package
source ~/RS2/ros2_ws/install/setup.bash
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning

# Terminal 1 — Start MoveIt2 with scene setup
export ROS_DOMAIN_ID=42
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py \
  robot_ip:=192.168.56.101 \
  launch_rviz:=true

# Terminal 2 — Start the motion node in file mode:
export ROS_DOMAIN_ID=42
source ~/RS2/ros2_ws/install/setup.bash
ros2 run ur3_motion_planning motion_planning_node \
  --ros-args \
  -p stroke_source:=file \
  -p face:=face1 \
  -p robot_ip:=192.168.56.101
```

This will load `~/RS2/outputs/strokes/face1_strokes.json`, plan with MoveIt2 collision avoidance, and execute.

---

## Data Format Agreement

All three subsystems agree on this stroke format:

```json
[
  [[x1, y1], [x2, y2], ...],   // stroke 1
  [[x1, y1], [x2, y2], ...],   // stroke 2
  ...
]
```

- **Canvas:** 400 × 300 pixels
- **Origin:** top-left, Y increases downward
- **Coordinates:** float, 1 decimal place
- **Transported via:** `/drawing_strokes` topic (std_msgs/String with JSON) or file

---

## Changes Made for Integration

### GUI (`~/gui/`)

| File | Change |
|------|--------|
| `selfie_drawing_gui_ros2.py` | **NEW** — ROS 2 integrated GUI subclass. Publishes captures to `/raw_image`, subscribes to `/drawing_strokes` + `/drawing_status` + `/drawing_preview_image`, sends commands on `/gui/command`. |
| `selfie_drawing_gui_starter.py` | **Unchanged** — still works standalone without ROS. |

### Perception (`~/perception/src/selfie_perception/selfie_perception/`)

| File | Change |
|------|--------|
| `perception_node.py` | **NEW** — Single unified perception node. Subscribes to `/raw_image`, runs bg removal (rembg/u2net_human_seg) → Canny edge detection (σ=3) → stroke extraction (morphological closing + Douglas-Peucker simplification + NN reordering). Publishes `/drawing_strokes` and `/drawing_preview_image`. Replaces the old 3-node pipeline (face_detection + image_processing + mapping). |
| `background_removal.py` | **NEW** — Background removal module using `rembg` library with u2net_human_seg model. |
| `edge_detection.py` | **NEW** — Canny edge detection with configurable Gaussian pre-blur (σ=3 default). |
| `stroke_extraction.py` | **NEW** — Contour extraction, Douglas-Peucker simplification, morphological closing, greedy NN stroke reordering. |
| `pipeline.py` | **NEW** — Standalone CLI pipeline (no ROS). Runs all 3 stages and saves outputs to numbered run directories. |
| `image_loader_node.py` | Watches `~/perception/input/` for images, publishes up to `max_publishes` times (default 5). |
| `visualization_node.py` | Subscribes to `/drawing_strokes`, renders preview, publishes to `/drawing_preview_image`, saves to disk. Resets on new `/raw_image`. |
| `perception_pipeline.launch.py` | Updated to launch `image_loader_node` → `perception_node` → `visualization_node`. |

### Motion Planning (`~/RS2/`)

| File | Change |
|------|--------|
| `ur3_drawing_node.py` | **REWRITTEN** — Now uses MoveIt2 `/compute_cartesian_path` service for collision-aware trajectory planning. Calls GetCartesianPath for each stroke, then converts planned joint trajectories to URScript. |
| `scene_publisher.py` | **REWRITTEN** — Publishes table AND marker-holder (160×180mm prism) as collision objects. Marker holder is **attached to tool0** so it moves with EE and MoveIt2 knows to avoid collisions. |
| `integrated_pipeline.launch.py` | Includes `ur_moveit_config` launch. Adds delays and TimerActions to: (1) start move_group, (2) publish scene objects, (3) start drawing node. Launches `perception_node` + `visualization_node` from selfie_perception. |
| `ur3_motion_planning_moveit2.launch.py` | Updated to include scene setup (table + marker holder auto-published). |
| `setup.py` | Added `scene_publisher` console script entry point. |
| `setup.cfg` | **NEW** — Required for ROS 2 Python packages. Tells `colcon` to install entry point executables to `lib/ur3_motion_planning/` instead of `bin/` (needed for `ros2 run`). |
| `motion_planning_lib.py` | Shared motion constants + stroke optimisation. Current fast profile: `JOINT_ACCEL=2.00`, `JOINT_VEL=2.50`, `LINEAR_ACCEL=1.20`, `LINEAR_VEL=0.35`; marker tilt is 20° with `EE_DRAW_HEIGHT=0.115 m`. |

---

## Architecture Principles

1. **Loose coupling via topics** — each module only knows about topic names and message types, not about each other's internals.
2. **Standalone first** — every module can run and be tested independently. Integration is additive.
3. **Single source of truth** — stroke format is defined once (400×300 px JSON). No translation layers needed.
4. **Reset-capable** — perception and motion nodes can process multiple images in a session (the GUI can retake and re-process).

---

## MoveIt2 Trajectory Planning & Tilted Tool Drawing

### Architecture: MoveIt2 Planning → URScript Execution

The motion planning node now **plans with MoveIt2 but executes with URScript** for maximum compatibility:

```
Strokes (JSON)
     ↓
Optimize (NN + 2-Opt)
     ↓
Convert to Cartesian waypoints (xyz positions)
     ↓
Call /compute_cartesian_path (MoveIt2) ← COLLISION-AWARE
     ↓
Get planned joint-space trajectory
     ↓
Convert to URScript (movej commands)
     ↓
Send via TCP socket → Robot (simulator or real)
```

### Marker Holder Geometry & Tool Orientation

The marker is held in a **3D-printed angled holder** that:
- Is bolted to the UR3 tool0 flange
- Extends the marker tip **11.5 cm below the end effector** along the holder axis (`EE_DRAW_HEIGHT=0.115 m`)
- Is tilted **20° from perpendicular** (toward the camera)

This creates a **TCP (Tool Center Point) offset** that must be known to both:
1. **MoveIt2** — so it plans paths for the marker tip, not the bare flange
2. **URScript** — via the `set_tcp()` command at the start of each program

**Offset values** (in robot base frame, in meters):
```
TCP_X  = 0.0393 m  (forward, due to 20° tilt)
TCP_Y  = 0.0 m     (centered)
TCP_Z  = -0.1081 m (downward component of 11.5 cm marker reach)
Rotation = [20° around Y-axis]
```

**How MoveIt2 knows about the offset:**
- The drawing node specifies `link_name = "tool0"` when calling `/compute_cartesian_path`
- The marker holder is published as an **AttachedCollisionObject** linked to `tool0`
  ```python
  attached = AttachedCollisionObject()
  attached.link_name = "tool0"
  attached.object = marker_holder_box
  attached.touch_links = ["tool0", "wrist_3_link"]  # Don't self-collide
  ```
- MoveIt2 carries this collision shape with the end effector during planning, so paths avoid hitting objects with the marker

**How URScript applies the offset:**
At the start of every URScript program:
```
def draw_face():
  set_tcp(p[0.0393, 0.0, -0.1081, 0.0, 0.3491, 0.0])
  # ... rest of program ...
end
```

This tells the robot: *"Treat this offset as the tool center for all movel/movej paths."* The robot's IK solver then adjusts joint angles so the actual marker tip reaches the commanded positions.

### Single-Colour Drawing (User-Selected)

The 3D-printed end-effector holder carries **four markers** at 0°, 90°,
180°, and 270° around the wrist_3 axis, each tilted 20° outward
(radially). **The user picks one colour in the GUI before pressing Start Drawing.**
The motion node uses that single marker for the entire artwork.


**Why post-process the wrist_3 instead of asking MoveIt2 for the rotated
orientation directly?** The holder is rotationally
symmetric, so a constant offset on wrist_3 swings marker N into the
position marker 1 was tracing and is robust against IK ambiguity.
The node then unwraps each wrist_3 target to the equivalent angle
nearest the previous command. This avoids ±π wrap jumps such as
`+3.13 → -3.13`, which would otherwise command an almost full wrist
rotation while drawing and smear the canvas.

### Current Motion Speeds

The integrated ROS node emits `movej` commands:

| Motion type | Acceleration | Velocity | Source |
|-------------|-------------:|---------:|--------|
| Travel / pen-up / home | `2.00 rad/s²` | `2.50 rad/s` | `JOINT_ACCEL`, `JOINT_VEL` in `motion_planning_lib.py` |
| Drawing / pen-down | `1.10 rad/s²` | `0.70 rad/s` | `DRAW_JOINT_ACCEL`, `DRAW_JOINT_VEL` in `ur3_drawing_node.py` |

The standalone legacy `movel` path in `motion_planning_lib.py` uses
`LINEAR_ACCEL=1.20 m/s²` and `LINEAR_VEL=0.35 m/s`.

### Why This Approach?

- **Collision safety:** MoveIt2 knows about the table and marker holder shape, so it never plans paths that crash
- **Correct tool orientation:** The quaternion in the pose ensures the 20° angle is maintained throughout
- **Efficient execution:** URScript is lightweight and works on both simulator and real robot without UR driver infrastructure
- **Hybrid:** Leverages MoveIt2's planning strengths while avoiding heavy action server dependencies

---

**Pipeline trigger:** the motion node does **not** auto-start when
strokes arrive on `/drawing_strokes`. It caches them and waits for
`START:<colour>` on `/gui/command`. This guarantees the colour is
honoured.

**Default colour-to-slot mapping** (edit `COLOUR_TO_MARKER` in
[`ur3_drawing_node.py`](ros2_ws/src/ur3_motion_planning/ur3_motion_planning/ur3_drawing_node.py)
to match how you physically loaded the holder):

| Colour | Slot index | Holder angle | wrist_3 offset (rel. to baseline) |
|--------|-----------:|-------------:|----------------------------------:|
| red    | 0          | 0°           | 0°                                |
| blue   | 1          | 90°          | -90°                              |
| green  | 2          | 180°         | -180°                             |
| black  | 3          | 270°         | +90° (canonical equivalent of -270°) |

Either physically load the markers so this mapping holds, or edit the
dict in code to match the order you loaded.

**Default colour:** if a `START` arrives without a colour suffix (and
no prior `/gui/marker_colour` was received), the motion node uses
`DEFAULT_COLOUR` (currently `"black"`). This keeps file-mode and no-GUI
test runs working.

**Verifying the data flow:** when you click Start Drawing in the GUI,
the motion node should print these four log lines in order. If any line
is missing or shows the wrong colour, the chain is broken at that step.

```
[GUI] >>> Received raw command: 'START:blue'
[GUI] Parsed command='START' payload='blue'
[GUI] ✓ Colour set to 'blue' (marker slot 2/4)
[Plan] >>> Pipeline reading colour: 'blue' (marker_idx=1, wrist_3_offset=-90.0°)
```


