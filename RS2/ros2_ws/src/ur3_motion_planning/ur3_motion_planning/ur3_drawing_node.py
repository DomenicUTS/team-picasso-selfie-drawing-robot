#!/usr/bin/env python3
"""
UR3 Motion Planning Node — central orchestrator.

Flow: GUI → perception strokes → MoveIt2 plan → URScript over TCP to UR3.

Topics:
  Sub  /drawing_strokes    ← perception (cached, no auto-start)
  Sub  /gui/command        ← GUI ("START:<colour>", PAUSE, RESUME, STOP)
  Sub  /gui/marker_colour  ← GUI (supplementary)
  Pub  /drawing_status     → GUI
  Pub  /joint_states       → RViz / MoveIt2
  Pub  /trajectory_preview → RViz

Params: robot_ip, robot_port, face, stroke_source ('file'|'topic'),
        enable_optimization, max_step, jump_threshold, planning_timeout.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
import threading
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
import time
import json
import os
import math
import sys
import socket
from typing import List, Tuple

# Shared library (~/RS2/src/motion_planning_lib.py): calibration constants
# + optimisation algorithms. Lives outside this ROS package, hence sys.path.
try:
    sys.path.insert(0, os.path.expanduser("~/RS2/src"))
    from motion_planning_lib import (
        nearest_neighbour_sort,
        two_opt_improve,
        px_to_robot,
        scale_strokes_to_workspace,
        validate_pose,
        Metrics,
        _calculate_travel,
        CANVAS_ORIGIN_ROBOT,
        CANVAS_PX_W,
        CANVAS_PX_H,
        CANVAS_WIDTH_M,
        CANVAS_HEIGHT_M,
        Z_DRAW,
        Z_TRAVEL,
        HOME_POS,
        TOOL_ORIENT,
        LINEAR_VEL,
        LINEAR_ACCEL,
        JOINT_VEL,
        JOINT_ACCEL,
        MARKER_TILT_DEG,
        MARKER_TILT_RAD,
        EE_DRAW_HEIGHT,
        TCP_OFFSET,
    )
    IMPORTS_OK = True
except ImportError as e:
    print(f"[Warning] Could not import motion_planning_lib: {e}")
    IMPORTS_OK = False

# Convert the URScript-style rotation vector (from motion_planning_lib)
# into a quaternion (what MoveIt2 wants). Called once at module load.
def _rotvec_to_quaternion(rv: List[float]) -> Tuple[float, float, float, float]:
    """Rotation-vector → quaternion (x, y, z, w).

    Rodrigues' formula → rotation matrix, then standard trace-based
    extraction. The if/elif picks the numerically stable branch.
    """
    a = np.array(rv, dtype=float)
    angle = float(np.linalg.norm(a))
    if angle < 1e-10:
        return (0.0, 0.0, 0.0, 1.0)
    axis = a / angle
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)
    tr = np.trace(R)
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    nm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / nm, y / nm, z / nm, w / nm)

# Baseline tool orientation for marker 1 (slot 0°, tilted 20° at canvas).
# Other colours derive from this via marker_tool_quat().
if IMPORTS_OK:
    TOOL_QUAT = _rotvec_to_quaternion(TOOL_ORIENT)  # (x, y, z, w)
else:
    TOOL_QUAT = (0.984808, 0.0, -0.173648, 0.0)  # fallback if lib import failed

# ── Multi-marker holder ──
# Custom 3D-printed holder carries 4 markers at 0°, 90°, 180°, 270° around the
# wrist_3 axis. Each marker is tilted 20° outward (radially). To activate
# marker N we rotate tool0 around its own Z-axis by -N * 90°. The holder is
# rotationally symmetric, so each marker tip lands at the same world position
# when its corresponding wrist_3 offset is applied — meaning px_to_robot()
# and TCP_OFFSET stay valid for every marker.
MARKER_COUNT = 4
MARKER_STEP_RAD = -math.pi / 2.0  # -90° around tool Z per marker
TAU = 2.0 * math.pi

# Map of GUI colour name → marker slot index (0–3) on the physical holder.
# IMPORTANT: physically load the markers into the holder so this mapping
# matches reality, OR edit this table to match your loading order.
COLOUR_TO_MARKER = {
    "red":   0,
    "blue":  1,
    "green": 2,
    "black": 3,
}
DEFAULT_COLOUR = "black"


def _wrap_to_pi(angle: float) -> float:
    """Return the equivalent angle in [-pi, pi)."""
    return (angle + math.pi) % TAU - math.pi


def _nearest_equivalent_angle(angle: float, reference: float) -> float:
    """Return angle + k*2pi that is closest to reference."""
    return reference + _wrap_to_pi(angle - reference)


def _quat_mul(q1, q2):
    """Hamilton product (quaternion multiplication — NOT element-wise)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def marker_tool_quat(marker_idx: int):
    """Tool orientation for marker N: TOOL_QUAT rotated -N·90° around tool Z."""
    angle = MARKER_STEP_RAD * (marker_idx % MARKER_COUNT)
    qz = (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))
    return _quat_mul(TOOL_QUAT, qz)


# MoveIt2 planning targets — must match the UR MoveIt2 config.
PLANNING_GROUP = "ur_manipulator"
EE_LINK = "tool0"

# Joint order must match the URDF.
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Safe start/end pose. wrist_3_offset is applied relative to HOME_JOINTS[5].
HOME_JOINTS = [1.047, -1.57, 1.57, -1.57, -1.57, 0.0]  # shoulder_pan 60° toward canvas

# Pen-down drawing speed. Kept separate from JOINT_VEL/ACCEL so travel moves
# can be quick while marker contact stays smooth and controlled.
DRAW_JOINT_ACCEL = 1.10
DRAW_JOINT_VEL = 0.70


class UR3DrawingNode(Node):
    """
    ROS 2 node: strokes → optimise → MoveIt2 Cartesian plan → URScript execute.
    """

    def __init__(self):
        super().__init__("ur3_drawing_node")

        # ── Parameters ──
        self.declare_parameter("robot_ip", "192.168.56.101")  # Polyscope sim default
        self.declare_parameter("robot_port", 30002)           # URScript port
        self.declare_parameter("enable_optimization", True)
        self.declare_parameter("face", "face1")               # file mode only
        self.declare_parameter("stroke_source", "file")       # 'file' or 'topic'
        self.declare_parameter("max_step", 0.005)             # MoveIt2 step (m)
        self.declare_parameter("jump_threshold", 5.0)
        self.declare_parameter("planning_timeout", 30.0)

        self.robot_ip = self.get_parameter("robot_ip").value
        self.robot_port = self.get_parameter("robot_port").value
        self.enable_optimization = self.get_parameter("enable_optimization").value
        self.face = self.get_parameter("face").value
        self.stroke_source = self.get_parameter("stroke_source").value
        self.max_step = self.get_parameter("max_step").value
        self.jump_threshold = self.get_parameter("jump_threshold").value
        self.planning_timeout = self.get_parameter("planning_timeout").value

        # ── State ──
        self._startup_done = False           # re-entry guard for the pipeline
        self._topic_strokes = None           # last strokes from perception
        self._metrics = Metrics() if IMPORTS_OK else None
        self._pipeline_thread = None
        self._current_joints = list(HOME_JOINTS)  # for /joint_states pub
        self._selected_colour = DEFAULT_COLOUR    # set by START:<colour>

        # Reentrant group so MoveIt2 service calls don't block subscribers.
        self._service_cb_group = ReentrantCallbackGroup()

        # Publishers
        self.status_pub = self.create_publisher(String, "drawing_status", 10)
        self.trajectory_display_pub = self.create_publisher(PoseArray, "trajectory_preview", 10)

        # Subscribers
        self.strokes_sub = self.create_subscription(
            String, "drawing_strokes", self._on_drawing_strokes, 10)
        self.gui_cmd_sub = self.create_subscription(
            String, "gui/command", self._on_gui_command, 10)
        self.gui_colour_sub = self.create_subscription(
            String, "gui/marker_colour", self._on_gui_colour, 10)

        # /joint_states @ 10 Hz so RViz/MoveIt2 don't show a phantom robot.
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self._js_timer = self.create_timer(0.1, self._publish_joint_states)

        # MoveIt2 service clients
        self.cartesian_path_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path",
            callback_group=self._service_cb_group)
        # /apply_planning_scene client kept for reference; scene_publisher.py
        # actually publishes the collision scene on startup.
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene',
            callback_group=self._service_cb_group)

        self.get_logger().info("[Init] UR3 Drawing Node (MoveIt2 planning mode)")
        self.get_logger().info(f"[Config] Robot: {self.robot_ip}:{self.robot_port}")
        self.get_logger().info(f"[Config] Face: {self.face} | Source: {self.stroke_source}")
        self.get_logger().info(f"[Config] MoveIt2 max_step={self.max_step} m, "
                               f"jump_threshold={self.jump_threshold}")

        # File mode auto-fires after a short delay. Topic mode waits for
        # GUI's START:<colour> (no auto-start — that broke colour selection).
        if self.stroke_source == "file":
            self.create_timer(2.0, self._file_startup_timer)
        else:
            self.get_logger().info("[Init] Waiting for strokes on /drawing_strokes ...")
            self._publish_status("WAITING_FOR_PERCEPTION")

    # ─────────────────── callbacks ───────────────────

    def _publish_joint_states(self):
        """10 Hz /joint_states pub — keeps RViz/MoveIt2 in sync with our plan."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(j) for j in self._current_joints]
        self.joint_state_pub.publish(msg)

    def _file_startup_timer(self):
        """One-shot 2 s delay so MoveIt2 is up before file-mode auto-starts."""
        self._start_pipeline_thread()

    def _on_drawing_strokes(self, msg: String):
        """Cache strokes from perception. Does NOT auto-start drawing —
        GUI's START:<colour> is the only trigger (auto-start used to fire
        before the user picked a colour, hence the bug)."""
        try:
            strokes = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[Perception] Bad JSON: {e}")
            return
        self.get_logger().info(
            f"[Perception] Received {len(strokes)} strokes "
            f"({sum(len(s) for s in strokes)} pts) — "
            f"waiting for GUI START to begin drawing")
        self._topic_strokes = strokes

    def _on_gui_colour(self, msg: String):
        """Supplementary colour topic. START:<colour> is authoritative;
        this is here for debug tools / direct `ros2 topic pub` use."""
        colour = msg.data.strip().lower()
        if colour not in COLOUR_TO_MARKER:
            self.get_logger().warn(
                f"[GUI] Unknown colour '{colour}', keeping '{self._selected_colour}'")
            return
        self._selected_colour = colour
        self.get_logger().info(
            f"[GUI] Marker colour set to '{colour}' "
            f"(slot {COLOUR_TO_MARKER[colour]+1}/{MARKER_COUNT})")

    def _on_gui_command(self, msg: String):
        """GUI commands on /gui/command. Accepts:
          START:<colour> — begin drawing (only way to trigger a run)
          START          — fallback, uses last-known colour
          PAUSE / RESUME / STOP — forwarded to /drawing_status (no live cancel yet)
        """
        raw = msg.data.strip()
        self.get_logger().info(f"[GUI] >>> Received raw command: '{raw}'")
        if ":" in raw:
            cmd, _, payload = raw.partition(":")
            cmd = cmd.upper()
            payload = payload.strip().lower()
        else:
            cmd = raw.upper()
            payload = ""
        self.get_logger().info(
            f"[GUI] Parsed command='{cmd}' payload='{payload}'")
        if cmd == "START":
            if self._topic_strokes is None:
                self.get_logger().warn(
                    "[GUI] START received but no strokes available yet "
                    "— ignoring (run perception first)")
                return
            if payload and payload in COLOUR_TO_MARKER:
                self._selected_colour = payload
                self.get_logger().info(
                    f"[GUI] ✓ Colour set to '{payload}' "
                    f"(marker slot {COLOUR_TO_MARKER[payload]+1}/{MARKER_COUNT})")
            elif payload:
                self.get_logger().warn(
                    f"[GUI] Unknown colour '{payload}' in START — "
                    f"using current '{self._selected_colour}' "
                    f"(known: {list(COLOUR_TO_MARKER.keys())})")
            else:
                self.get_logger().warn(
                    f"[GUI] START with no colour payload — "
                    f"using current '{self._selected_colour}'")
            self._startup_done = False
            self._start_pipeline_thread()
        elif cmd == "STOP":
            self._publish_status("STOPPED")
        elif cmd == "PAUSE":
            self._publish_status("PAUSED")
        elif cmd == "RESUME":
            self._publish_status("RESUMED")

    def _start_pipeline_thread(self):
        """Run the pipeline on a background thread so MoveIt2 service
        waits don't starve subscribers (joint state, GUI, etc.)."""
        if self._pipeline_thread is not None and self._pipeline_thread.is_alive():
            self.get_logger().warn("[Pipeline] Already running, ignoring")
            return
        self._pipeline_thread = threading.Thread(
            target=self._on_startup_complete, daemon=True)
        self._pipeline_thread.start()

    # ─────────────────── pipeline (background thread) ───────────────────

    def _on_startup_complete(self):
        """5-stage drawing pipeline. Status strings are published at each
        stage so the GUI can update its progress display."""
        if self._startup_done:
            return
        self._startup_done = True

        try:
            self.get_logger().info("[Pipeline] Starting ...")
            self._publish_status("LOADING_STROKES")

            # Stage 1 — load strokes (from cached topic or from disk)
            strokes = self._load_strokes()
            if not strokes:
                self._publish_status("ERROR_LOAD_FAILED")
                return
            self.get_logger().info(
                f"[Stage 1] {len(strokes)} strokes, "
                f"{sum(len(s) for s in strokes)} pts")

            # Stage 2 — scale + reorder (NN + 2-Opt)
            self._publish_status("OPTIMIZING_PATH")
            strokes = self._optimize_strokes(strokes)

            # Stage 3 — scene was set up by scene_publisher.py at launch.
            self._publish_status("SETTING_UP_SCENE")
            self.get_logger().info("[Scene] Using collision objects from scene_publisher node")

            # Stage 4 — plan with MoveIt2 + build URScript
            self._publish_status("PLANNING_WITH_MOVEIT2")
            script = self._plan_and_build_urscript(strokes)
            if script is None:
                self._publish_status("ERROR_PLANNING_FAILED")
                return

            # Stage 5 — ship URScript to the robot over TCP
            self._publish_status("EXECUTING")
            success = self._send_urscript(script)

            if success:
                self._publish_status("COMPLETE")
                self.get_logger().info("="*60)
                self.get_logger().info("DRAWING COMPLETE")
                self.get_logger().info("="*60)
            else:
                self._publish_status("ERROR_EXECUTION_FAILED")

        except Exception as e:
            self.get_logger().error(f"[Pipeline] {e}", exc_info=True)
            self._publish_status(f"ERROR_{str(e)[:40]}")

    # ─────────────────── stage 1: load ───────────────────

    def _load_strokes(self):
        """Topic mode → use cached strokes. File mode → load face*_strokes.json."""
        if self.stroke_source == "topic":
            if self._topic_strokes is not None:
                return self._topic_strokes
            self.get_logger().error("[Load] No strokes on topic yet")
            return None

        paths = [
            os.path.expanduser(f"~/RS2/outputs/strokes/{self.face}_strokes.json"),
            os.path.expanduser("~/perception/output/perception_strokes.json"),
        ]
        for p in paths:
            if os.path.exists(p):
                try:
                    with open(p) as f:
                        return json.load(f)
                except Exception as e:
                    self.get_logger().error(f"[Load] {p}: {e}")
        self.get_logger().error("[Load] No stroke file found")
        return None

    # ─────────────────── stage 2: optimise ───────────────────

    def _optimize_strokes(self, strokes):
        """Scale to canvas + reorder via NN + 2-Opt (from motion_planning_lib).
        Falls back to the raw strokes on any error."""
        if not self.enable_optimization or not IMPORTS_OK:
            return strokes
        try:
            # Normalise to float tuples (perception may send ints).
            strokes_f = [[(float(x), float(y)) for x, y in s] for s in strokes]
            strokes_f = scale_strokes_to_workspace(strokes_f)
            metrics = Metrics()
            metrics.raw_stroke_count = len(strokes_f)
            metrics.raw_waypoint_count = sum(len(s) for s in strokes_f)
            metrics.raw_travel_distance = _calculate_travel(strokes_f)

            # NN for a quick decent ordering, then 2-Opt to refine.
            nn = nearest_neighbour_sort(strokes_f, metrics=metrics)
            opt = two_opt_improve(nn, max_iterations=50, metrics=metrics)

            metrics.optimized_stroke_count = len(opt)
            metrics.optimized_waypoint_count = sum(len(s) for s in opt)
            metrics.optimized_travel_distance = _calculate_travel(opt)

            self.get_logger().info(f"[Optimise]{metrics.summary()}")
            self._metrics = metrics
            return opt
        except Exception as e:
            self.get_logger().error(f"[Optimise] {e}")
            return strokes

    # ─────────────────── stage 3: collision scene ───────────────

    def _apply_scene(self, scene_msg: PlanningScene) -> bool:
        """Apply a PlanningScene diff via the /apply_planning_scene service."""
        if not self.apply_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('[Scene] /apply_planning_scene service not available')
            return False
        req = ApplyPlanningScene.Request()
        req.scene = scene_msg
        future = self.apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None and future.result().success:
            return True
        self.get_logger().error('[Scene] ApplyPlanningScene call failed')
        return False

    def _publish_scene_objects(self):
        """Unused — scene_publisher.py handles this at launch time.
        Kept here as a fallback for in-node testing."""

        # Table
        table = CollisionObject()
        table.header.frame_id = "world"
        table.id = "table"
        table.operation = CollisionObject.ADD
        table.pose.position.z = -0.07
        table.pose.orientation.w = 1.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.7, 1.7, 0.2]
        table.primitives.append(box)

        scene1 = PlanningScene()
        scene1.is_diff = True
        scene1.world.collision_objects.append(table)
        if self._apply_scene(scene1):
            self.get_logger().info("[Scene] Table applied")

        # Marker holder attached to tool0 — top face flush with flange,
        # full height extends downward along the tilted marker axis.
        holder_co = CollisionObject()
        holder_co.header.frame_id = "tool0"
        holder_co.id = "marker_holder"
        holder_co.operation = CollisionObject.ADD
        hbox = SolidPrimitive()
        hbox.type = SolidPrimitive.BOX
        hbox.dimensions = [0.160, 0.180, EE_DRAW_HEIGHT]
        holder_co.primitives.append(hbox)
        full = EE_DRAW_HEIGHT
        hp = Pose()
        hp.position.x = full * math.sin(MARKER_TILT_RAD)
        hp.position.z = -full * math.cos(MARKER_TILT_RAD)
        hp.orientation.y = math.sin(MARKER_TILT_RAD / 2.0)
        hp.orientation.w = math.cos(MARKER_TILT_RAD / 2.0)
        holder_co.primitive_poses.append(hp)

        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object = holder_co
        attached.touch_links = ["tool0", "wrist_3_link"]

        scene2 = PlanningScene()
        scene2.is_diff = True
        scene2.robot_state.attached_collision_objects.append(attached)
        scene2.robot_state.is_diff = True
        if self._apply_scene(scene2):
            self.get_logger().info("[Scene] Marker holder attached to tool0")

    # ─────────────────── stage 4: MoveIt2 plan → URScript ───────

    def _make_pose(self, xyz, quat=None):
        """Pose for /compute_cartesian_path. Defaults to TOOL_QUAT (marker 1)."""
        if quat is None:
            quat = TOOL_QUAT
        p = Pose()
        p.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        p.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        return p

    def _call_cartesian_path(self, waypoints: List[Pose],
                             start_joints: List[float]) -> Tuple:
        """Call /compute_cartesian_path with collision avoidance ON.
        Returns (joint_trajectory, fraction) or (None, 0.0) on failure.
        `fraction` is the proportion of waypoints MoveIt2 could interpolate."""
        if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("[Plan] /compute_cartesian_path service not available")
            return None, 0.0

        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = PLANNING_GROUP
        req.link_name = EE_LINK
        req.waypoints = waypoints
        req.max_step = self.max_step
        req.jump_threshold = self.jump_threshold
        req.avoid_collisions = True

        # Start state = where the robot currently is, not some default.
        req.start_state.joint_state.name = JOINT_NAMES
        req.start_state.joint_state.position = [float(j) for j in start_joints]

        future = self.cartesian_path_client.call_async(req)

        # Wait for the future (executed from a background thread,
        # the MultiThreadedExecutor handles the response callback)
        deadline = time.time() + self.planning_timeout
        while not future.done():
            if time.time() > deadline:
                self.get_logger().error("[Plan] Cartesian path service call timed out")
                return None, 0.0
            time.sleep(0.01)

        if not future.done() or future.result() is None:
            self.get_logger().error("[Plan] Cartesian path service call failed")
            return None, 0.0

        resp = future.result()
        fraction = resp.fraction
        traj = resp.solution.joint_trajectory

        return traj, fraction

    def _thin_trajectory(self, traj_points, is_draw=False, max_joint_delta=0.05):
        """Pick which trajectory points to emit as URScript movejs.
        Travel/lift → just the endpoint (short, fast).
        Drawing → every point (anything less causes joint-space shortcuts
                  that lift the pen mid-stroke)."""
        n = len(traj_points)
        if n == 0:
            return []
        if n <= 2:
            return [list(pt.positions) for pt in traj_points]

        if not is_draw:
            # Travel/lift: single movej to endpoint (robot is high up, safe)
            return [list(traj_points[-1].positions)]

        # Drawing: keep ALL MoveIt2 points so the robot follows the exact
        # collision-free Cartesian path without joint-space shortcuts.
        return [list(pt.positions) for pt in traj_points]

    def _plan_and_build_urscript(self, strokes) -> str:
        """Heart of the node: strokes → URScript.

        For each stroke we plan 3 segments via MoveIt2 — travel, draw, lift —
        thin the trajectories, collect them into `all_segments`, then
        post-process wrist_3 (colour offset + prepended rotate-to-marker movej)
        and serialise to URScript in _build_urscript.
        """

        all_segments = []  # (joint_positions, is_travel, comment) tuples
        current_joints = list(HOME_JOINTS)  # MoveIt2 start state, updated each stroke
        total_strokes = len(strokes)
        skipped = 0

        # ── Single-marker drawing ──
        # The GUI publishes a colour (combined with START as "START:<colour>").
        # We map it to one of the 4 holder slots; the active marker is
        # selected by rotating wrist_3 by `marker_idx * -90°`.
        colour = self._selected_colour
        marker_idx = COLOUR_TO_MARKER.get(colour, COLOUR_TO_MARKER[DEFAULT_COLOUR])
        wrist_3_offset = _wrap_to_pi(marker_idx * MARKER_STEP_RAD)  # applied as a post-step
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"[Plan] >>> Pipeline reading colour: '{colour}' "
            f"(marker_idx={marker_idx}, "
            f"wrist_3_offset={math.degrees(wrist_3_offset):+.1f}°)")
        self.get_logger().info("=" * 60)

        # We always plan the trajectory as if marker 1 were active (i.e.
        # using TOOL_QUAT for orientation). After all planning is done, we
        # add `wrist_3_offset` to every joint waypoint's wrist_3. Because
        # the marker holder is rotationally symmetric, that rotation
        # physically swings the desired marker into the canvas position
        # without disturbing the marker tip's world coordinates.
        #
        # Why post-processing: MoveIt2's `/compute_cartesian_path` is free
        # to pick any IK branch that satisfies the requested orientation,
        # and on a 6-DOF UR3 a 90° tool-Z rotation can be "absorbed" by
        # wrist_1 / wrist_2 / a wrist-flip — leaving wrist_3 essentially
        # unchanged regardless of the requested orientation. We saw this:
        # the pre-rotation visibly worked, but during the draw the planner
        # rotated wrist_3 right back. Post-processing wrist_3 sidesteps
        # the IK ambiguity entirely.
        stroke_quat = marker_tool_quat(0)

        for s_idx, stroke in enumerate(strokes):
            if not stroke:
                continue

            self.get_logger().info(
                f"[Plan] Stroke {s_idx+1}/{total_strokes} "
                f"({len(stroke)} pts)")

            # ── Travel to above first point (pen-up) ──
            travel_pos = px_to_robot(*stroke[0])
            travel_pos[2] = Z_TRAVEL
            travel_wp = [self._make_pose(travel_pos, quat=stroke_quat)]

            traj_t, frac_t = self._call_cartesian_path(travel_wp, current_joints)
            if traj_t is None or frac_t < 0.5:
                self.get_logger().warn(
                    f"[Plan] Travel plan failed for stroke {s_idx+1} "
                    f"(fraction={frac_t:.2f}), skipping stroke")
                skipped += 1
                continue
            current_joints = list(traj_t.points[-1].positions)
            self._current_joints = list(current_joints)  # update for joint state publisher

            # Use MoveIt2's planned joints for travel (just final point)
            travel_joints = self._thin_trajectory(traj_t.points, is_draw=False)
            for jts in travel_joints:
                all_segments.append(
                    (jts, True, f"travel s{s_idx+1} marker{marker_idx+1}"))

            # ── Draw waypoints (pen-down + stroke points) ──
            draw_poses = []
            for px, py in stroke:
                rp = px_to_robot(float(px), float(py))
                rp[2] = Z_DRAW
                draw_poses.append(self._make_pose(rp, quat=stroke_quat))

            traj_d, frac_d = self._call_cartesian_path(draw_poses, current_joints)
            if traj_d is None or frac_d < 0.3:
                self.get_logger().warn(
                    f"[Plan] Draw plan for stroke {s_idx+1} fraction={frac_d:.2f}")
                if traj_d is None:
                    skipped += 1
                    continue

            current_joints = list(traj_d.points[-1].positions)
            self._current_joints = list(current_joints)  # update for joint state publisher
            draw_joints = self._thin_trajectory(traj_d.points, is_draw=True)
            for i, jts in enumerate(draw_joints):
                label = "pen-down" if i == 0 else f"draw s{s_idx+1}"
                all_segments.append((jts, False, label))

            # ── Lift after stroke (pen-up) ──
            lift_pos = px_to_robot(*stroke[-1])
            lift_pos[2] = Z_TRAVEL
            traj_l, frac_l = self._call_cartesian_path(
                [self._make_pose(lift_pos, quat=stroke_quat)], current_joints)
            if traj_l is not None and traj_l.points:
                current_joints = list(traj_l.points[-1].positions)
                self._current_joints = list(current_joints)  # update for joint state publisher
                lift_joints = self._thin_trajectory(traj_l.points, is_draw=False)
                for jts in lift_joints:
                    all_segments.append((jts, True, f"pen-up s{s_idx+1}"))

        if not all_segments:
            self.get_logger().error("[Plan] No waypoints planned at all")
            return None

        # ── Apply wrist_3 offset (single-marker activation) ──
        # Everything above was planned with marker 1's orientation. We now
        # shift the 6th joint of every waypoint by `wrist_3_offset` so the
        # holder rotates around tool-Z to put the chosen marker into the
        # position marker 1 was tracing. Tool0's origin lies on the
        # wrist_3 axis, so only the 6th joint needs to change — the
        # marker tip's world position is preserved by rotational symmetry.
        #
        # Important: do not normalise each waypoint independently into
        # [-pi, pi]. When MoveIt2 returns wrist_3 values that straddle the
        # wrap boundary, independent normalisation can emit +3.13 followed
        # by -3.13, which commands a full wrist spin while the pen is down.
        # Instead, unwrap each target to the equivalent angle nearest the
        # previous command so the marker stays continuous throughout a draw.
        rotated_home = list(HOME_JOINTS)
        rotated_home[5] = _nearest_equivalent_angle(
            HOME_JOINTS[5] + wrist_3_offset, HOME_JOINTS[5])

        shifted = []
        previous_wrist = rotated_home[5]
        max_wrist_step = 0.0
        for joints, is_travel, comment in all_segments:
            new_joints = list(joints)
            w = _nearest_equivalent_angle(
                float(new_joints[5]) + wrist_3_offset, previous_wrist)
            max_wrist_step = max(max_wrist_step, abs(w - previous_wrist))
            previous_wrist = w
            new_joints[5] = w
            shifted.append((new_joints, is_travel, comment))
        all_segments = shifted

        if wrist_3_offset != 0.0:
            # Prepend an explicit "rotate at home" movej so the wrist swaps
            # to the chosen marker BEFORE any horizontal motion (visible to
            # the operator). Other joints stay at HOME, only wrist_3 moves.
            self.get_logger().info(
                f"[Plan] Wrist_3 offset {math.degrees(wrist_3_offset):+.1f}° "
                f"applied; first move sets wrist_3 → {rotated_home[5]:.3f} rad "
                f"to align '{colour}' marker")
            all_segments = [
                (rotated_home, True, f"align marker {marker_idx+1} ({colour})")
            ] + all_segments

        self.get_logger().info(
            f"[Plan] Continuous wrist_3 targets generated "
            f"(max step {math.degrees(max_wrist_step):.1f}°)")

        self.get_logger().info(
            f"[Plan] {len(all_segments)} movej commands "
            f"({skipped} strokes skipped)")

        # ── Build URScript ──
        return self._build_urscript(all_segments)

    def _build_urscript(self, segments) -> str:
        """Serialise joint waypoints to a URScript program.
        Travel/lift movejs use JOINT_VEL/ACCEL.
        Draw movejs use slower DRAW_JOINT_* values with a small blend radius
        (r=0.002) for smooth lines — but the LAST draw point of a stroke
        has no blend so the robot stops cleanly before lifting."""
        lines = []
        # Must start with `def` — some CB3 firmwares reject scripts with
        # any text (including comments) before that line.
        lines.append("def draw_face():")
        # set_tcp tells the robot where the marker TIP is (not the flange).
        lines.append(f"  set_tcp(p[{TCP_OFFSET[0]:.4f},{TCP_OFFSET[1]:.4f},"
                     f"{TCP_OFFSET[2]:.4f},{TCP_OFFSET[3]:.4f},"
                     f"{TCP_OFFSET[4]:.4f},{TCP_OFFSET[5]:.4f}])")
        lines.append("")

        # Start at HOME so we begin from a known pose.
        home_str = ",".join(f"{j:.4f}" for j in HOME_JOINTS)
        lines.append(f"  movej([{home_str}], a={JOINT_ACCEL}, v={JOINT_VEL})")
        lines.append("")

        for idx, (joints, is_travel, comment) in enumerate(segments):
            j_str = ",".join(f"{j:.4f}" for j in joints)
            if is_travel:
                lines.append(f"  movej([{j_str}], a={JOINT_ACCEL}, v={JOINT_VEL})")
            else:
                # Last draw point = next segment is travel/lift (or this is the end).
                is_last_draw = (idx + 1 >= len(segments) or segments[idx + 1][1])
                if is_last_draw:
                    lines.append(f"  movej([{j_str}], a={DRAW_JOINT_ACCEL}, v={DRAW_JOINT_VEL})")
                else:
                    lines.append(f"  movej([{j_str}], a={DRAW_JOINT_ACCEL}, v={DRAW_JOINT_VEL}, r=0.002)")

        lines.append("")
        # Return to HOME — also undoes the wrist_3 colour offset.
        lines.append(f"  movej([{home_str}], a={JOINT_ACCEL}, v={JOINT_VEL})")
        lines.append("end")
        return "\n".join(lines)

    # ─────────────────── stage 5: execute ───────────────────

    def _send_urscript(self, script: str) -> bool:
        """Send the script to UR3/Polyscope on TCP port 30002.
        Also saved to outputs/last_drawing.script for inspection/replay.
        Returns True if the bytes were sent; actual motion must be
        confirmed visually (or in Polyscope's VNC viewer)."""
        save_path = os.path.expanduser("~/RS2/outputs/last_drawing.script")
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        with open(save_path, "w") as f:
            f.write(script)
        line_count = script.count('\n') + 1
        self.get_logger().info(
            f"[Execute] Script saved to {save_path} "
            f"({len(script)} bytes, {line_count} lines)")

        try:
            self.get_logger().info(
                f"[Execute] Connecting to {self.robot_ip}:{self.robot_port} ...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10.0)
            sock.connect((self.robot_ip, self.robot_port))
            self.get_logger().info("[Execute] Connected to robot")

            encoded = (script + "\n").encode("utf-8")
            sock.sendall(encoded)
            self.get_logger().info(
                f"[Execute] URScript sent ({len(encoded)} bytes). "
                f"Waiting for robot to start executing ...")

            # Keep socket open and monitor robot state for up to 10 seconds
            # to give the CB3 controller time to parse and start execution.
            # Closing too early can cause the controller to abort on real hardware.
            sock.settimeout(2.0)
            started_waiting = time.time()
            while time.time() - started_waiting < 10.0:
                try:
                    resp = sock.recv(4096)
                    if resp:
                        self.get_logger().info(
                            f"[Execute] Robot state packet ({len(resp)} bytes)")
                except socket.timeout:
                    pass
                # Check if enough time has passed for the controller to start
                if time.time() - started_waiting >= 5.0:
                    break

            sock.close()
            self.get_logger().info(
                "[Execute] Done — socket closed after monitoring period.\n"
                "  If robot is not moving, check:\n"
                f"  1. Robot is powered ON and initialized\n"
                f"  2. Robot is in 'Remote Control' mode (not 'Local')\n"
                f"  3. Robot is not in Protective Stop / Emergency Stop\n"
                f"  4. Review script: cat {save_path}")
            return True
        except ConnectionRefusedError:
            self.get_logger().error(
                f"[Execute] CONNECTION REFUSED at {self.robot_ip}:{self.robot_port}. "
                f"Is the robot running? Is the robot IP correct?")
            return False
        except socket.timeout:
            self.get_logger().error(
                f"[Execute] CONNECTION TIMED OUT to {self.robot_ip}:{self.robot_port}. "
                f"Check network connectivity to the robot/simulator.")
            return False
        except Exception as e:
            self.get_logger().error(f"[Execute] Socket error: {type(e).__name__}: {e}")
            return False

    # ─────────────────── helpers ───────────────────

    def _publish_status(self, status: str):
        """Emit pipeline state on /drawing_status (consumed by GUI).
        States: WAITING_FOR_PERCEPTION, LOADING_STROKES, OPTIMIZING_PATH,
        SETTING_UP_SCENE, PLANNING_WITH_MOVEIT2, EXECUTING, COMPLETE, ERROR_*"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    """Entry point (`motion_planning_node` in setup.py). MultiThreadedExecutor
    so the planning thread can block on MoveIt2 without starving subscribers."""
    rclpy.init(args=args)
    node = UR3DrawingNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
