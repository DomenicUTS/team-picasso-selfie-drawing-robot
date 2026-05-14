"""
UR3 Selfie Drawing Robot - Motion Planning Template
Team: Picasso | Subsystem 2: Motion Planning and Control
Lead: Domenic Kadioglu

Compatible with: Polyscope Simulator (RTDE / URScript)
Optimisation: Nearest-Neighbour TSP path ordering (inspired by
              github.com/piotte13/SIMD-Pathfinder and
              github.com/ReaganLawrence/ChanceOfPrecipitation TSP heuristics)

Pipeline:
  1. Load vector paths (from canvas_face.py or perception subsystem)
  2. Optimise stroke ordering via Nearest-Neighbour heuristic
  3. Convert canvas coords → robot world coords (affine transform)
  4. Send URScript trajectories to UR3 over TCP socket
"""

import socket
import time
import math
import numpy as np
from typing import List, Tuple, Dict, Any
import json
import logging
from datetime import datetime

# ──────────────────────────────────────────────
#  CONFIG
# ──────────────────────────────────────────────
ROBOT_IP = "192.168.56.101"  # Polyscope simulator IP
ROBOT_PORT = 30002             # Primary interface (URScript)

# Robot draw plane (world coords in metres, z = table surface)
# Pre-calibrated: top-left corner of canvas in robot base frame
# Rotated 60° CCW so the canvas is in front of the home position (not to the right)
CANVAS_ORIGIN_ROBOT = np.array([0.185, 0.170, 0.010])    # (x, y, z) metres — rotated 60° left from original
CANVAS_WIDTH_M      = 0.150    # 15 cm canvas (reduced from 20cm for safety)
CANVAS_HEIGHT_M     = 0.120    # 12 cm canvas (reduced from 15cm)

# Canvas image dimensions (pixels) — must match canvas_face.py output
CANVAS_PX_W = 400
CANVAS_PX_H = 300

# ── Marker holder geometry ──
# 3D-printed part holds the marker at 20° from the end-effector perpendicular.
# The end effector stays above the canvas; the tilted marker reaches down.
MARKER_TILT_DEG = 20.0                          # degrees from perpendicular
MARKER_TILT_RAD = math.radians(MARKER_TILT_DEG)
EE_DRAW_HEIGHT  = 0.115                         # end-effector height above canvas (m)

# Z heights (end effector, NOT marker tip)
CANVAS_SURFACE_Z = CANVAS_ORIGIN_ROBOT[2]                      # table / canvas surface
Z_DRAW    = CANVAS_SURFACE_Z + EE_DRAW_HEIGHT                  # EE at 15 cm above canvas when drawing
Z_TRAVEL  = CANVAS_SURFACE_Z + EE_DRAW_HEIGHT + 0.060          # pen-up (6 cm above draw height)

# Safe home position (close to canvas, definitely reachable by UR3)
# This is positioned above the center of the canvas, rotated 60° left
HOME_POS = np.array([0.277, 0.129, 0.230])   # rotated 60° CCW to match canvas position

# Motion params (optimized for simulator speed)
JOINT_ACCEL  = 0.97  # rad/s²  (higher mid acceleration)
JOINT_VEL    = 1.10  # rad/s   (higher mid movement)
LINEAR_ACCEL = 0.64  # m/s²    (higher mid acceleration)
LINEAR_VEL   = 0.15  # m/s     (higher mid drawing speed)

# ── Tool orientation (tilted 20° for marker holder) ──
# Rotation vector for URScript: base = Rx(π) (straight down),
# then tilted MARKER_TILT_DEG around the Y-axis of the base frame.

def _rotation_matrix_to_rotvec(R: np.ndarray) -> List[float]:
    """Convert a 3×3 rotation matrix to a rotation vector (axis × angle)."""
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if angle < 1e-10:
        return [0.0, 0.0, 0.0]
    if abs(angle - math.pi) < 1e-6:
        # θ ≈ π  → extract axis from diagonal
        diag = np.diag(R)
        idx = int(np.argmax(diag))
        col = R[:, idx]
        axis = (col + np.eye(3)[idx]) / math.sqrt(2.0 * (1.0 + diag[idx]))
        return (axis * angle).tolist()
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]]) / (2.0 * math.sin(angle))
    return (axis * angle).tolist()

def compute_tilted_tool_orient(tilt_rad: float) -> List[float]:
    """Compute URScript rotation vector for tool tilted from vertical by *tilt_rad* around Y."""
    # Base: 180° about X  → tool Z points down
    Rx = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)
    # Tilt about Y
    c, s = math.cos(tilt_rad), math.sin(tilt_rad)
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)
    R = Ry @ Rx
    return _rotation_matrix_to_rotvec(R)

TOOL_ORIENT = compute_tilted_tool_orient(MARKER_TILT_RAD)

# TCP offset for set_tcp() — marker tip position relative to flange
# The marker extends 10 cm at 20° from the EE perpendicular.
TCP_X = EE_DRAW_HEIGHT * math.sin(MARKER_TILT_RAD)   # horizontal offset  ≈ 3.4 cm
TCP_Z = -EE_DRAW_HEIGHT * math.cos(MARKER_TILT_RAD)  # downward offset    ≈ -9.4 cm
TCP_OFFSET = [TCP_X, 0.0, TCP_Z, 0.0, MARKER_TILT_RAD, 0.0]

# ──────────────────────────────────────────────
#  LOGGING & METRICS
# ──────────────────────────────────────────────

# Configure logging (will be updated dynamically in main)
logger = logging.getLogger("MotionPlanning")

def setup_logging(face_name, run_number):
    """Configure logging with dynamic filename based on face and run number."""
    log_file = f'test_run_{face_name}_{run_number}.log'
    
    # Remove any existing handlers
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
    
    # Add new handlers
    logger.addHandler(logging.StreamHandler())
    logger.addHandler(logging.FileHandler(log_file))
    logger.setLevel(logging.INFO)
    
    return log_file

class Metrics:
    """Track subsystem performance metrics for test evidence."""
    def __init__(self):
        self.timestamp = datetime.now().isoformat()
        self.raw_stroke_count = 0
        self.raw_waypoint_count = 0
        self.raw_travel_distance = 0.0
        
        self.optimized_stroke_count = 0
        self.optimized_waypoint_count = 0
        self.optimized_travel_distance = 0.0
        
        self.nn_time_ms = 0.0
        self.opt2_time_ms = 0.0
        self.script_gen_time_ms = 0.0
        self.waypoint_count = 0
        
    def summary(self) -> str:
        """Return metrics summary as string."""
        savings_pct = (
            (1 - self.optimized_travel_distance / self.raw_travel_distance) * 100
            if self.raw_travel_distance > 0 else 0
        )
        return f"""
  Strokes: {self.optimized_stroke_count} | Waypoints: {self.optimized_waypoint_count}
  Travel distance: {self.optimized_travel_distance:.1f} px (saved {savings_pct:.1f}%)
  NN time: {self.nn_time_ms:.1f}ms | 2-Opt time: {self.opt2_time_ms:.1f}ms | Script time: {self.script_gen_time_ms:.1f}ms
        """

# ──────────────────────────────────────────────
#  WORKSPACE & VALIDATION
# ──────────────────────────────────────────────

def validate_pose(pos: np.ndarray, orient: List[float], 
                  margin_m: float = 0.30) -> Tuple[bool, str]:
    """
    Validate that a pose is within robot workspace and safe bounds.
    Uses generous margins to allow for SVG coordinate variations.
    Returns: (is_valid, error_message)
    """
    x, y, z = pos
    
    # Check Z bounds (above table, below max reach)
    if z < Z_DRAW - 0.01:
        return False, f"Z too low: {z:.4f} m"
    if z > Z_TRAVEL + 0.20:
        return False, f"Z too high: {z:.4f} m"
    
    # Workspace bounds with margin for SVG coordinate variations
    canvas_min_x = CANVAS_ORIGIN_ROBOT[0] - margin_m
    canvas_max_x = CANVAS_ORIGIN_ROBOT[0] + CANVAS_WIDTH_M + margin_m
    canvas_min_y = CANVAS_ORIGIN_ROBOT[1] - CANVAS_HEIGHT_M - margin_m
    canvas_max_y = CANVAS_ORIGIN_ROBOT[1] + margin_m
    
    if not (canvas_min_x <= x <= canvas_max_x):
        return False, f"X out of bounds: {x:.4f} m"
    if not (canvas_min_y <= y <= canvas_max_y):
        return False, f"Y out of bounds: {y:.4f} m"
    
    return True, "OK"

def check_z_height_transition(last_z: float, current_z: float, 
                              expected_state: str) -> Tuple[bool, str]:
    """
    Verify pen-up/pen-down transitions maintain correct Z heights.
    expected_state: 'draw' (Z_DRAW) or 'travel' (Z_TRAVEL)
    """
    tolerance = 0.002  # 2 mm tolerance
    
    if expected_state == 'draw':
        if abs(current_z - Z_DRAW) > tolerance:
            return False, f"Draw height mismatch: {current_z:.4f} m (expected {Z_DRAW:.4f})"
    elif expected_state == 'travel':
        if abs(current_z - Z_TRAVEL) > tolerance:
            return False, f"Travel height mismatch: {current_z:.4f} m (expected {Z_TRAVEL:.4f})"
    
    return True, "OK"

# ──────────────────────────────────────────────
#  COORDINATE TRANSFORM  (pixel → robot world)
# ──────────────────────────────────────────────

def px_to_robot(px: float, py: float) -> np.ndarray:
    """
    Map a pixel coordinate (origin = top-left, y down) to robot
    world XY, keeping Z at draw height.
    """
    rx = CANVAS_ORIGIN_ROBOT[0] + (px / CANVAS_PX_W) * CANVAS_WIDTH_M
    ry = CANVAS_ORIGIN_ROBOT[1] - (py / CANVAS_PX_H) * CANVAS_HEIGHT_M  # y flipped
    rz = Z_DRAW
    return np.array([rx, ry, rz])


# ──────────────────────────────────────────────
#  PATH OPTIMISATION  — Nearest-Neighbour TSP
#  Heuristic reduces total pen-up travel by
#  greedily picking the closest stroke start
#  after each stroke completes.
#
#  Reference approach:
#    Applegate et al. "The Traveling Salesman Problem" (2006)
#    Practical NN implementation pattern from:
#    github.com/dmishin/tsp-solver (MIT licence)
# ──────────────────────────────────────────────

def nearest_neighbour_sort(strokes: List[List[Tuple[float, float]]], 
                           metrics: Metrics = None,
                           timer_start_ms: float = 0.0) -> List[List[Tuple[float, float]]]:
    """
    Reorder strokes so that total pen-up (travel) distance is minimised.
    Uses a greedy Nearest-Neighbour heuristic — O(n²) on number of strokes.

    Each stroke is a list of (x,y) pixel points.
    The 'representative point' used for distance is the stroke's first point.
    """
    if not strokes:
        return strokes

    start_time = time.time()

    unvisited = list(range(len(strokes)))
    ordered   = []

    # Start from stroke closest to top-left (natural start)
    current_end = np.array([0.0, 0.0])
    
    while unvisited:
        best_idx  = None
        best_dist = float('inf')
        best_reverse = False

        for idx in unvisited:
            stroke = strokes[idx]
            # Check both endpoints — we can reverse a stroke if cheaper
            dist_fwd = np.linalg.norm(np.array(stroke[0])  - current_end)
            dist_rev = np.linalg.norm(np.array(stroke[-1]) - current_end)

            if dist_fwd < best_dist:
                best_dist, best_idx, best_reverse = dist_fwd, idx, False
            if dist_rev < best_dist:
                best_dist, best_idx, best_reverse = dist_rev, idx, True

        stroke = strokes[best_idx]
        if best_reverse:
            stroke = stroke[::-1]
        ordered.append(stroke)
        current_end   = np.array(stroke[-1])
        unvisited.remove(best_idx)

    savings = _calculate_travel(strokes) - _calculate_travel(ordered)
    elapsed_ms = (time.time() - start_time) * 1000.0
    
    if metrics:
        metrics.nn_time_ms = elapsed_ms
    
    logger.info(f"[NN Sort] {len(ordered)} strokes | travel saved ≈ {savings:.1f} px | time: {elapsed_ms:.1f} ms")
    return ordered


def _calculate_travel(strokes: List[List[Tuple[float, float]]]) -> float:
    """Total pen-up distance for a given stroke ordering."""
    total = 0.0
    pos   = np.array([0.0, 0.0])
    for stroke in strokes:
        total += np.linalg.norm(np.array(stroke[0]) - pos)
        pos    = np.array(stroke[-1])
    return total


def two_opt_improve(strokes: List[List[Tuple[float, float]]], 
                    max_iterations: int = 50,
                    metrics: Metrics = None) -> List[List[Tuple[float, float]]]:
    """
    2-opt local search on top of NN solution.
    Tries reversing sub-sequences to further reduce total travel.
    Stops after max_iterations passes or no improvement found.

    Reference: Lin, S. (1965) "Computer solutions of the traveling salesman problem"
    """
    start_time = time.time()
    
    n = len(strokes)
    if n < 4:
        return strokes

    best = strokes[:]
    improved = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1
        for i in range(n - 1):
            for j in range(i + 2, n):
                # Cost before swap
                end_i   = np.array(best[i][-1])
                start_i1 = np.array(best[i+1][0])
                end_j   = np.array(best[j][-1])
                start_j1 = np.array(best[(j+1) % n][0]) if j+1 < n else np.array([0,0])

                before = (np.linalg.norm(end_i - start_i1) +
                          np.linalg.norm(end_j - start_j1))

                # Cost after 2-opt swap (reverse i+1..j)
                after = (np.linalg.norm(end_i - end_j) +
                         np.linalg.norm(start_i1 - start_j1))

                if after < before - 1e-6:
                    best[i+1:j+1] = best[i+1:j+1][::-1]
                    improved = True

    savings = _calculate_travel(strokes) - _calculate_travel(best)
    elapsed_ms = (time.time() - start_time) * 1000.0
    
    if metrics:
        metrics.opt2_time_ms = elapsed_ms
    
    logger.info(f"[2-Opt] {iteration} iterations | travel saved ≈ {savings:.1f} px | time: {elapsed_ms:.1f} ms")
    return best


# ──────────────────────────────────────────────
#  URSCRIPT GENERATION
# ──────────────────────────────────────────────

def pose_str(pos: np.ndarray, orient: List[float]) -> str:
    """Format a pose as URScript p[x,y,z,rx,ry,rz]"""
    return (f"p[{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
            f"{orient[0]:.4f},{orient[1]:.4f},{orient[2]:.4f}]")


def build_urscript(strokes: List[List[Tuple[float, float]]], 
                   metrics: Metrics = None) -> Tuple[str, List[str]]:
    """
    Generate a complete URScript program from optimised strokes.
    Uses movel() for drawing (linear, accurate) and
    movej() for large pen-up hops (faster joint-space).
    
    Returns: (script_text, validation_errors)
    """
    lines = []
    validation_errors = []
    waypoint_count = 0
    
    lines.append("# ── UR3 Selfie Drawing Robot | Team Picasso ──")
    lines.append("# Auto-generated URScript — load in Polyscope / simulator")
    lines.append("")
    lines.append("def draw_face():")
    lines.append(f"  # Canvas origin: {CANVAS_ORIGIN_ROBOT.tolist()}")
    lines.append(f"  # {len(strokes)} strokes after NN+2opt optimisation")
    lines.append(f"  # Marker holder: {MARKER_TILT_DEG}° tilt, EE {EE_DRAW_HEIGHT*100:.0f} cm above canvas")
    lines.append("")
    lines.append(f"  # Set TCP for angled marker holder ({MARKER_TILT_DEG}° from perpendicular)")
    lines.append(f"  set_tcp(p[{TCP_OFFSET[0]:.4f},{TCP_OFFSET[1]:.4f},{TCP_OFFSET[2]:.4f},"
                 f"{TCP_OFFSET[3]:.4f},{TCP_OFFSET[4]:.4f},{TCP_OFFSET[5]:.4f}])")
    lines.append("")

    # Safe home position (close to canvas, definitely reachable by UR3)
    home = HOME_POS.copy()
    is_valid, msg = validate_pose(home, TOOL_ORIENT)
    if not is_valid:
        logger.warning(f"Home position validation: {msg} (will attempt anyway)")
    
    # Start with joint move to safe home position (safer than linear from unknown start state)
    lines.append(f"  # Move to safe home via joint space (safe from any starting position)")
    lines.append(f"  movej([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], ")
    lines.append(f"         a={JOINT_ACCEL}, v={JOINT_VEL})")
    lines.append("")
    lines.append(f"  # Now move linearly to canvas height")
    lines.append(f"  movel({pose_str(home, TOOL_ORIENT)}, ")
    lines.append(f"         a={LINEAR_ACCEL}, v={LINEAR_VEL})")
    lines.append("")

    total_points = sum(len(s) for s in strokes)
    lines.append(f"  # Total waypoints: {total_points}")
    lines.append("")

    for s_idx, stroke in enumerate(strokes):
        if not stroke:
            continue

        lines.append(f"  # ── Stroke {s_idx + 1}/{len(strokes)} ──")

        # Pen-up: move to above first point of stroke (linear for IK stability)
        travel_pos = px_to_robot(*stroke[0])
        travel_pos[2] = Z_TRAVEL
        is_valid, msg = validate_pose(travel_pos, TOOL_ORIENT)
        if not is_valid:
            validation_errors.append(f"Stroke {s_idx+1} travel pos invalid: {msg}")
        
        lines.append(f"  movel({pose_str(travel_pos, TOOL_ORIENT)}, ")
        lines.append(f"         a={LINEAR_ACCEL}, v={LINEAR_VEL})  # travel (pen-up)")
        waypoint_count += 1

        # Pen-down: lower to first point
        draw_start = px_to_robot(*stroke[0])
        is_valid, msg = validate_pose(draw_start, TOOL_ORIENT)
        if not is_valid:
            validation_errors.append(f"Stroke {s_idx+1} pen-down pos invalid: {msg}")
        
        is_valid, msg = check_z_height_transition(travel_pos[2], draw_start[2], 'draw')
        if not is_valid:
            validation_errors.append(f"Stroke {s_idx+1} Z transition invalid: {msg}")
        
        lines.append(f"  movel({pose_str(draw_start, TOOL_ORIENT)}, "
                     f"a={LINEAR_ACCEL}, v={LINEAR_VEL})  # pen-down")
        waypoint_count += 1

        # Draw remaining waypoints
        for pt in stroke[1:]:
            wp = px_to_robot(*pt)
            is_valid, msg = validate_pose(wp, TOOL_ORIENT)
            if not is_valid:
                validation_errors.append(f"Stroke {s_idx+1} waypoint {waypoint_count} invalid: {msg}")
            
            lines.append(f"  movel({pose_str(wp, TOOL_ORIENT)}, "
                         f"a={LINEAR_ACCEL}, v={LINEAR_VEL})")
            waypoint_count += 1

        # Pen-up after stroke (linear for IK stability)
        lift = px_to_robot(*stroke[-1])
        lift[2] = Z_TRAVEL
        is_valid, msg = check_z_height_transition(draw_start[2], lift[2], 'travel')
        if not is_valid:
            validation_errors.append(f"Stroke {s_idx+1} pen-up transition invalid: {msg}")
        
        lines.append(f"  movel({pose_str(lift, TOOL_ORIENT)}, ")
        lines.append(f"         a={LINEAR_ACCEL}, v={LINEAR_VEL})  # pen-up")
        waypoint_count += 1
        lines.append("")

    # Return home (joint move for safety, away from canvas area)
    lines.append(f"  # Return to safe home via joint space")
    lines.append(f"  movej([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], ")
    lines.append(f"         a={JOINT_ACCEL}, v={JOINT_VEL})")
    lines.append("end")
    lines.append("")
    lines.append("draw_face()")

    if metrics:
        metrics.waypoint_count = waypoint_count

    return "\n".join(lines), validation_errors


# ──────────────────────────────────────────────
#  ROBOT COMMUNICATION  (TCP socket to Polyscope)
# ──────────────────────────────────────────────

class UR3Controller:
    def __init__(self, ip: str = ROBOT_IP, port: int = ROBOT_PORT):
        self.ip   = ip
        self.port = port
        self.sock = None

    def connect(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.ip, self.port))
            print(f"[Robot] Connected to {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"[Robot] Connection failed: {e}")
            print("[Robot] Running in SIMULATION mode (script will be saved only)")
            return False

    def send_script(self, script: str):
        if self.sock is None:
            print("[Robot] Not connected — script saved to file only")
            return
        try:
            payload = (script + "\n").encode("utf-8")
            self.sock.sendall(payload)
            print(f"[Robot] Script sent ({len(payload)} bytes)")
            time.sleep(0.5)
        except Exception as e:
            print(f"[Robot] Send error: {e}")


# ──────────────────────────────────────────────
#  MAIN PIPELINE
# ──────────────────────────────────────────────

def scale_strokes_to_workspace(strokes: List[List[Tuple[float, float]]]) -> List[List[Tuple[float, float]]]:
    """
    Scale strokes to fit within UR3 canvas workspace.
    Calculates bounding box and scales to 90% of available space.
    """
    if not strokes or not strokes[0]:
        return strokes
    
    # Find bounds
    all_points = [pt for stroke in strokes for pt in stroke]
    min_x = min(pt[0] for pt in all_points)
    max_x = max(pt[0] for pt in all_points)
    min_y = min(pt[1] for pt in all_points)
    max_y = max(pt[1] for pt in all_points)
    
    stroke_width = max_x - min_x
    stroke_height = max_y - min_y
    
    # Available canvas in pixel space (leave 5% margins)
    available_px_w = CANVAS_PX_W * 0.95
    available_px_h = CANVAS_PX_H * 0.95
    
    # Calculate scale to fit within canvas (use smaller to preserve aspect ratio)
    scale_x = available_px_w / stroke_width if stroke_width > 0 else 1.0
    scale_y = available_px_h / stroke_height if stroke_height > 0 else 1.0
    scale = min(scale_x, scale_y, 1.0)  # Only downscale if too large
    
    logger.info(f"[Scaling] Stroke bounds: X [{min_x:.0f}, {max_x:.0f}], Y [{min_y:.0f}, {max_y:.0f}]")
    logger.info(f"[Scaling] Stroke size: {stroke_width:.0f}×{stroke_height:.0f}px, scale_factor: {scale:.3f}")
    
    # Scale and center
    scaled = []
    for stroke in strokes:
        scaled_stroke = []
        for px, py in stroke:
            # Normalize to origin then scale
            norm_x = (px - min_x) * scale
            norm_y = (py - min_y) * scale
            scaled_stroke.append((norm_x, norm_y))
        scaled.append(scaled_stroke)
    
    if scale < 0.99:
        logger.info(f"[Scaling] Rescaled to {scale*100:.1f}% to fit within canvas")
    
    return scaled

def run_drawing_pipeline(strokes: List[List[Tuple[float, float]]]) -> Tuple[str, Metrics]:
    """
    Full pipeline:
      1. Scale strokes to fit UR3 workspace
      2. Optimise stroke order (NN heuristic + 2-opt refinement)
      3. Generate URScript
      
    Returns: (script_text, metrics_object)
    """
    pipeline_start = time.time()
    metrics = Metrics()
    
    logger.info("="*60)
    logger.info("UR3 SELFIE DRAWING ROBOT — Motion Planning Pipeline")
    logger.info("="*60)
    logger.info(f"Input:  {len(strokes)} raw strokes loaded")
    logger.info(f"Input:  {sum(len(s) for s in strokes)} total waypoints")
    
    # Step 0: Scale strokes to fit workspace
    strokes = scale_strokes_to_workspace(strokes)
    
    raw_travel = _calculate_travel(strokes)
    metrics.raw_stroke_count = len(strokes)
    metrics.raw_waypoint_count = sum(len(s) for s in strokes)
    metrics.raw_travel_distance = raw_travel
    logger.info(f"Input:  Raw travel distance: {raw_travel:.1f} px\n")

    # Step 1: Optimise with NN (after scaling)
    logger.info("[Optimise] Running Nearest-Neighbour sort...")
    optimised = nearest_neighbour_sort(strokes, metrics=metrics)

    # Step 2: Optimise with 2-opt
    logger.info("[Optimise] Running 2-opt local search refinement...")
    optimised = two_opt_improve(optimised, metrics=metrics)

    opt_travel = _calculate_travel(optimised)
    metrics.optimized_stroke_count = len(optimised)
    metrics.optimized_waypoint_count = sum(len(s) for s in optimised)
    metrics.optimized_travel_distance = opt_travel
    pct_saved  = (1 - opt_travel / raw_travel) * 100 if raw_travel > 0 else 0
    logger.info(f"[Result]  Optimised travel: {opt_travel:.1f} px ({pct_saved:.1f}% reduction)\n")

    # Step 3: Generate URScript
    script_gen_start = time.time()
    script, validation_errors = build_urscript(optimised, metrics=metrics)
    metrics.script_gen_time_ms = (time.time() - script_gen_start) * 1000.0
    
    logger.info(f"[Script] Generated ({len(script)} bytes)")
    
    logger.info(f"\n[Pipeline Complete] Total time: {time.time() - pipeline_start:.3f}s")
    logger.info("="*60 + "\n")
    
    return script, metrics


# ──────────────────────────────────────────────
#  ENTRY POINT
# ──────────────────────────────────────────────

if __name__ == "__main__":
    """
    Load stroke coordinates from JSON and run the motion planning pipeline.
    
    Expected directory structure:
    /home/domenic/RS2/
    ├── inputs/
    │   └── (SVG files here)
    ├── outputs/
    │   ├── strokes/
    │   └── verified/
    └── src/
        └── motion_planning_lib.py (this file)
    
    Workflow:
    1. Draw on https://editor.method.ac/
    2. Export as SVG → place in inputs/
    3. python3 src/svg_to_json_converter.py
       (creates JSON in outputs/strokes/)
    4. python3 src/motion_planning_lib.py [face_num] [run_num]
    
    Arguments (optional):
    - face_num: 1, 2, or 3 (default: 3)
    - run_num: test run number (default: 1)
    """
    
    import sys
    
    # Parse command-line arguments
    face_num = sys.argv[1] if len(sys.argv) > 1 else '3'
    run_num = sys.argv[2] if len(sys.argv) > 2 else '1'
    
    # Construct JSON file path
    json_file = f'outputs/strokes/face{face_num}_strokes.json'
    
    # Setup logging with face and run number
    log_file = setup_logging(f'face{face_num}', run_num)
    
    try:
        with open(json_file, 'r') as f:
            strokes = json.load(f)
        
        # Log header information
        logger.info("="*60)
        logger.info("UR3 MOTION PLANNING PIPELINE")
        logger.info("="*60)
        logger.info(f"TEST RUN: Face {face_num}, Run #{run_num}")
        logger.info(f"Log file: {log_file}")
        logger.info("")
        
        print("="*60)
        print("UR3 MOTION PLANNING PIPELINE")
        print("="*60)
        print(f"\n✓ Loaded {len(strokes)} strokes from '{json_file}'")
        print(f"  Total waypoints: {sum(len(s) for s in strokes)}")
        print(f"\n[Debug Info]")
        print(f"  Home position: {HOME_POS.tolist()}")
        print(f"  Canvas origin: {CANVAS_ORIGIN_ROBOT.tolist()}")
        print(f"  Canvas bounds X: [{CANVAS_ORIGIN_ROBOT[0]-0.05:.3f}, {CANVAS_ORIGIN_ROBOT[0]+CANVAS_WIDTH_M+0.05:.3f}] m")
        print(f"  Canvas bounds Y: [{CANVAS_ORIGIN_ROBOT[1]-CANVAS_HEIGHT_M-0.05:.3f}, {CANVAS_ORIGIN_ROBOT[1]+0.05:.3f}] m")
        print("="*60 + "\n")
        
        # Track execution time
        start_time = time.time()
        
        # Run the pipeline
        script, metrics = run_drawing_pipeline(strokes)
        
        # Report timing
        total_time = time.time() - start_time
        print("\n" + "="*60)
        print("EXECUTION SUMMARY")
        print("="*60)
        print(metrics.summary())
        print(f"Total execution time: {total_time:.3f}s")
        print("="*60)
        
        # Send script to Polyscope simulator
        print("\n[Connecting to Polyscope simulator...]")
        controller = UR3Controller(ip=ROBOT_IP, port=ROBOT_PORT)
        if controller.connect():
            print("[Sending drawing program to robot...]")
            controller.send_script(script)
            print("[Ready to run! Click 'Play' in Polyscope to execute drawing.")
        else:
            print("[WARNING] Could not connect to Polyscope simulator.")
            print("[INFO] Make sure the simulator is running:")
            print("       docker pull universalrobots/ursim_cb3")
            print("       ros2 run ur_client_library start_ursim.sh -m ur3")
        
    except FileNotFoundError:
        print("="*60)
        print("ERROR: JSON file not found")
        print("="*60)
        print(f"\nFile to load: {json_file}")
        print("\nExpected directory structure:")
        print("  outputs/strokes/face1_strokes.json")
        print("\nTo create a JSON file from SVG:")
        print("  1. Place your SVG file in inputs/")
        print("  2. python3 src/svg_to_json_converter.py")
        print("  3. python3 src/motion_planning_lib.py")
        print("="*60)
