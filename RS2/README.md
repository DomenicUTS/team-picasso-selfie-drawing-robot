# UR3 Selfie Drawing Robot

**Team Picasso · Robotics Studio 2 · UTS**

A Universal Robots **UR3** that takes a selfie from a webcam and draws
a line portrait of the subject on a canvas, using a custom 3D-printed
end-effector that holds **four markers** (red / blue / green / black).
The user picks one colour in the GUI; the wrist rotates to align that
marker and the entire artwork is drawn in that single colour.

> 📖 **This README is the 30-second quick start.** For the complete
> reference (hardware BOM, installation, subsystem details, calibration,
> troubleshooting, architecture), read
> **[TECHNICAL_DOCUMENTATION.md](TECHNICAL_DOCUMENTATION.md)**.

---

## What's in this repository?

This repo (`~/RS2/`) is the **motion planning** subsystem (Domenic).
The integrated system also depends on two sibling repositories:

| Repo | Subsystem | Owner |
|------|-----------|-------|
| `~/RS2/` (this one) | Motion planning + integration docs | Domenic |
| `~/perception/` | Background removal + Canny edge detection + stroke extraction | Nithish |
| `~/gui/` | PySide6 webcam GUI + start/pause/stop + colour selection | Mateusz |

The three subsystems are loosely coupled — they only communicate over
ROS 2 topics. Each can be run standalone for testing; see §6 and §7
of [TECHNICAL_DOCUMENTATION.md](TECHNICAL_DOCUMENTATION.md).

---

## ⚠️ Always `export ROS_DOMAIN_ID=42` (UTS lab safety)

The UTS lab puts every team's laptop and robot on the **same network**.
Without a unique ROS 2 domain, our nodes pick up other teams' topics
(and they pick up ours). **`ROS_DOMAIN_ID=42`** is Team Picasso's
reserved domain.

```bash
export ROS_DOMAIN_ID=42        # Every terminal. Every time.
```

Add it to `~/.bashrc` for convenience.

---

## Quick Start

### One-time build

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
cd ~/perception && colcon build --packages-select selfie_perception
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning
```

### Run the integrated pipeline (3 terminals)

**Terminal 1 — Backend (MoveIt2 + Perception + Motion)**

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui launch_rviz:=true robot_ip:=192.168.0.195 #change this to sim ip if using simulator
```

Replace `192.168.56.101` with `192.168.0.195` for the real UR3.
Wait ~25 s for MoveIt2 to finish loading.

**Terminal 2 — GUI**

```bash
export ROS_DOMAIN_ID=42
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py
```

**Terminal 3 — Polyscope simulator (skip if using the real UR3)**

```bash
export ROS_DOMAIN_ID=42
ros2 run ur_client_library start_ursim.sh -m ur3
```

Then in the GUI: **Capture → (preview appears) → pick a colour → Start Drawing**.

---

## ROS 2 Interfaces (motion subsystem)

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Subscribe | `/drawing_strokes` | `std_msgs/String` | JSON strokes from perception (cached; does **not** auto-start) |
| Subscribe | `/gui/command` | `std_msgs/String` | `START:<colour>`, `PAUSE`, `RESUME`, `STOP` |
| Subscribe | `/gui/marker_colour` | `std_msgs/String` | Supplementary; for status consumers |
| Publish | `/drawing_status` | `std_msgs/String` | Pipeline state |
| Publish | `/joint_states` | `sensor_msgs/JointState` | Planned UR3 joints (10 Hz) |
| Publish | `/trajectory_preview` | `geometry_msgs/PoseArray` | RViz visualisation |

Full topic map across all three subsystems: see §8.1 of
[TECHNICAL_DOCUMENTATION.md](TECHNICAL_DOCUMENTATION.md).

---

## Common gotchas

| Problem | Fix |
|---------|-----|
| `ConnectionRefusedError 192.168.56.101:30002` | Start the simulator: `ros2 run ur_client_library start_ursim.sh -m ur3` |
| `move_group not available` | Wait ~25 s after launch — MoveIt2 takes a while to load |
| Robot not moving on real UR3 | Pendant must be in **Run Program / Remote Control** mode |
| Pipeline never starts after Process | Click **Start Drawing** — the motion node intentionally does not auto-start |
| Strokes / commands from another team | `export ROS_DOMAIN_ID=42` in **every** terminal |

For the complete troubleshooting table, see §10 of the
[Technical Documentation](TECHNICAL_DOCUMENTATION.md#10-troubleshooting--faqs).

---

## Repository layout

```
RS2/
├── README.md                        ← this file (quick start)
├── TECHNICAL_DOCUMENTATION.md       ← full reference (BOM, install, subsystems, calibration, FAQ)
├── inputs/face*.svg                 ← raw drawings (test input)
├── outputs/
│   ├── strokes/face*.json           ← pre-baked strokes for offline runs
│   └── last_drawing.script          ← most recent generated URScript
├── src/
│   ├── motion_planning_lib.py       ← shared constants + algorithms
│   └── svg_to_json_converter.py     ← convert SVG → stroke JSON
└── ros2_ws/src/ur3_motion_planning/
    ├── launch/
    │   ├── integrated_pipeline.launch.py
    │   └── ur3_motion_planning_moveit2.launch.py
    └── ur3_motion_planning/
        ├── ur3_drawing_node.py      ← ROS 2 node (planning + execution)
        └── scene_publisher.py       ← publishes collision scene to MoveIt2
```

---

**Owners:** Domenic Kadioglu (motion), Nithish Kannan Bhagavathi Sankaranarayanan (perception), Mateusz Kopaczynski (GUI / end-effector).
