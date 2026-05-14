# UR3 Selfie Drawing Robot — Motion Planning

**Team Picasso | Robotics Studio 2 | UTS**

ROS 2 motion-planning subsystem for a selfie drawing robot. A UR3 takes
strokes from the perception pipeline, plans collision-safe trajectories
with MoveIt2, and executes them on the robot via URScript. The custom
3D-printed end-effector holds **four markers** at 0°, 90°, 180°, 270°
around the wrist axis. The user picks a single colour (red / blue /
green / black) in the GUI; the motion node rotates the wrist to the
matching slot, keeps wrist_3 angle commands continuous, and draws the
entire artwork with that one marker.

For full system documentation (hardware setup, GUI/perception integration,
troubleshooting), see [TECHNICAL_DOCUMENTATION.md](TECHNICAL_DOCUMENTATION.md).
For cross-subsystem topic flow and run modes, see
[FULL_SYSTEM_INTEGRATION.md](FULL_SYSTEM_INTEGRATION.md).

---

## What This Subsystem Does

1. **Receives strokes** — JSON list of `[[(x,y), …], …]` on `/drawing_strokes`
   (perception subsystem) or from local `outputs/strokes/face*.json`.
2. **Optimises stroke order** — Nearest-Neighbour TSP + 2-Opt
   (~25–30% pen-up travel saved).
3. **Plans with MoveIt2** — `/compute_cartesian_path` with table + marker
   holder collision objects.
4. **Single colour per drawing** — The GUI sends the chosen colour as
   part of the START command (`START:<colour>` on `/gui/command`). The
   motion node rotates wrist_3 to the slot matching that colour and
   draws every stroke with that one marker (red / blue / green / black).
   Wrist targets are unwrapped between waypoints so the robot does not
   spin the wrist through a full turn while the marker is touching the canvas.
5. **Executes on the UR3** — Generates URScript and ships it over TCP
   (port 30002) to the real robot or Polyscope simulator.

---

## Always set `ROS_DOMAIN_ID=42` (UTS lab)

In the UTS lab every team's robot and laptop share the same network. By
default, ROS 2 nodes from different machines find each other via
multicast and end up talking.

To prevent that, **every terminal** in this project must run:

```bash
export ROS_DOMAIN_ID=42
```

This is already included within the command blocks given in this ReadME and the full system integration. 

---

## Quick Start

```bash
# Build (after any code change)
export ROS_DOMAIN_ID=42        # see above 
source /opt/ros/humble/setup.bash
cd ~/RS2/ros2_ws && colcon build --packages-select ur3_motion_planning
source install/setup.bash
```

**Run the integrated pipeline** (perception + motion + MoveIt2):
```bash
export ROS_DOMAIN_ID=42         # ← always first
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
ros2 launch ur3_motion_planning integrated_pipeline.launch.py \
  image_source:=gui launch_rviz:=true robot_ip:=192.168.56.101
```

**Start the GUI** in a second terminal:
```bash
export ROS_DOMAIN_ID=42         # ← always first
source ~/perception/install/setup.bash
source ~/RS2/ros2_ws/install/setup.bash
python3 ~/gui/selfie_drawing_gui_ros2.py
```

For Polyscope, start the simulator first (also on the same domain):
```bash
export ROS_DOMAIN_ID=42        # ← always first
ros2 run ur_client_library start_ursim.sh -m ur3
```

---

## Repository Layout

```
RS2/
├── README.md                       ← this file
├── FULL_SYSTEM_INTEGRATION.md      ← cross-subsystem topic map / run modes
├── TECHNICAL_DOCUMENTATION.md      ← full hardware + software documentation
├── inputs/face*.svg                ← raw drawings (test input)
├── outputs/strokes/face*.json      ← pre-baked strokes for offline runs
├── outputs/last_drawing.script     ← most recent generated URScript
├── src/
│   ├── motion_planning_lib.py          ← stroke optimisation + URScript constants (imported by the ROS node; also runs as a stripped-down standalone CLI for quick UR3 movement / stroke-reading tests, no colour selection)
│   └── svg_to_json_converter.py    ← convert raw SVGs → stroke JSON
└── ros2_ws/src/ur3_motion_planning/
    ├── launch/
    │   ├── integrated_pipeline.launch.py    ← perception + MoveIt2 + motion
    │   └── ur3_motion_planning_moveit2.launch.py  ← motion only
    └── ur3_motion_planning/
        ├── ur3_drawing_node.py     ← ROS 2 node (planning + execution)
        └── scene_publisher.py     ← publishes collision scene to MoveIt2
```

---

## ROS 2 Interfaces

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Subscribe | `/drawing_strokes` | `std_msgs/String` | JSON strokes from perception (cached; does **not** auto-start the pipeline) |
| Subscribe | `/gui/command` | `std_msgs/String` | `START:<colour>` (e.g. `START:blue`), `PAUSE`, `RESUME`, `STOP`. The colour suffix is the trigger for the entire drawing. |
| Subscribe | `/gui/marker_colour` | `std_msgs/String` | Supplementary — also published by the GUI; useful for debug / status consumers. The motion node prefers the colour parsed from `START:<colour>`. |
| Publish | `/drawing_status` | `std_msgs/String` | Pipeline state (LOADING, EXECUTING, COMPLETE, …) |
| Publish | `/joint_states` | `sensor_msgs/JointState` | Planned UR3 joints (10 Hz) |
| Publish | `/trajectory_preview` | `geometry_msgs/PoseArray` | RViz visualisation |

---

## Key Parameters

`ros2 run ur3_motion_planning motion_planning_node --ros-args -p ...`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_ip` | `192.168.56.101` | UR3 (real or simulator) IP |
| `robot_port` | `30002` | URScript primary interface |
| `face` | `face1` | Used when `stroke_source=file` |
| `stroke_source` | `file` | `file` (offline) or `topic` (perception) |
| `enable_optimization` | `true` | Toggle NN + 2-Opt |
| `max_step` | `0.005` | Cartesian interpolation step (m) |
| `jump_threshold` | `5.0` | MoveIt2 joint-jump filter |

Calibration constants live in `src/motion_planning_lib.py`:
- `CANVAS_ORIGIN_ROBOT` — canvas top-left in robot base frame
- `CANVAS_WIDTH_M`, `CANVAS_HEIGHT_M` — canvas size
- `EE_DRAW_HEIGHT`, `MARKER_TILT_DEG` — marker holder geometry
- `JOINT_ACCEL=2.00`, `JOINT_VEL=2.50` — fast pen-up/travel `movej` profile
- `LINEAR_ACCEL=1.20`, `LINEAR_VEL=0.35` — standalone `movel` script profile

Pen-down drawing speed is set in `ur3_drawing_node.py`:
- `DRAW_JOINT_ACCEL=1.10`, `DRAW_JOINT_VEL=0.70` — fast but restrained marker-contact `movej` profile

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Build fails | `source /opt/ros/humble/setup.bash` then rebuild |
| `ConnectionRefusedError 192.168.56.101:30002` | Start the simulator: `ros2 run ur_client_library start_ursim.sh -m ur3` |
| `move_group not available` | Wait ~25 s after launch — MoveIt2 takes a while to load |
| Robot not moving on real UR3 | Set teach pendant to **Run Program** mode |
| Wrong colour drawn | The mapping `colour → slot` is hard-coded in `COLOUR_TO_MARKER` in `ur3_drawing_node.py`. Either re-arrange the markers in the holder to match, or edit that dict. |
| Wrist spins or smears while drawing | `ur3_drawing_node.py` unwraps wrist_3 targets to avoid ±π jumps. Rebuild and relaunch after updating; inspect `outputs/last_drawing.script` if an old generated script is being replayed. |
| Pipeline never starts after Process | The motion node intentionally does **not** auto-start when perception strokes arrive — you must click **Start Drawing** in the GUI. Strokes are cached and used as soon as you press Start. |
| Strokes off-canvas | Recalibrate `CANVAS_ORIGIN_ROBOT` in `src/motion_planning_lib.py` |
| Strokes appear out of nowhere / robot freezes / topics from another team | `ROS_DOMAIN_ID` is wrong. **Every terminal** must `export ROS_DOMAIN_ID=42` before running anything. Verify with `echo $ROS_DOMAIN_ID`. |

---

## Standalone Test (no perception, no GUI)

```bash
export ROS_DOMAIN_ID=42                           # ← always first
ros2 launch ur3_motion_planning ur3_motion_planning_moveit2.launch.py \
  robot_ip:=192.168.56.101 launch_rviz:=true

# In another terminal — load face1 and run:
export ROS_DOMAIN_ID=42                           # ← always first
ros2 run ur3_motion_planning motion_planning_node --ros-args \
  -p stroke_source:=file -p face:=face1 -p robot_ip:=192.168.56.101
```
