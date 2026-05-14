#!/usr/bin/env python3
"""
Add table + marker-holder collision objects to the MoveIt2 planning scene.

Uses the /apply_planning_scene service (synchronous) so objects are
guaranteed to be ingested by move_group — no race with topic discovery.

Objects:
  1. table        – free-standing box below the robot base
  2. marker_holder – 160 × 180 mm rectangular prism ATTACHED to tool0
                     (moves with the end effector so MoveIt plans around it)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    CollisionObject, AttachedCollisionObject, PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
import math

# ── Marker holder geometry (must match motion_planning_lib.py) ──
MARKER_TILT_DEG = 20.0
MARKER_TILT_RAD = math.radians(MARKER_TILT_DEG)
EE_DRAW_HEIGHT  = 0.115  # m – end-effector height above canvas


class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')
        self.cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info('Waiting for /apply_planning_scene service …')
        self.cli.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('/apply_planning_scene service available')

    # ── helpers ────────────────────────────────────────────────
    def _apply(self, scene_msg: PlanningScene, retries=3):
        for attempt in range(retries):
            req = ApplyPlanningScene.Request()
            req.scene = scene_msg
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            if future.result() is not None and future.result().success:
                return True
            self.get_logger().warn(
                f'ApplyPlanningScene attempt {attempt+1}/{retries} failed, retrying in 5s …')
            import time
            time.sleep(5.0)
        self.get_logger().error('ApplyPlanningScene call failed after all retries')
        return False

    # ── Table ──────────────────────────────────────────────────
    def add_table(self):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = "table"
        obj.operation = CollisionObject.ADD

        obj.pose.position.x = 0.0
        obj.pose.position.y = 0.0
        obj.pose.position.z = -0.07
        obj.pose.orientation.w = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.7, 1.7, 0.2]
        obj.primitives.append(box)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        if self._apply(scene):
            self.get_logger().info('Table collision object applied')

    # ── Marker holder (attached to tool0) ─────────────────────
    def add_marker_holder(self):
        obj = CollisionObject()
        obj.header.frame_id = "tool0"
        obj.id = "marker_holder"
        obj.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.160, 0.180, EE_DRAW_HEIGHT]
        obj.primitives.append(box)

        # Offset so the TOP face of the box is flush with tool0 flange;
        # the full height extends downward along the tilted marker axis.
        full = EE_DRAW_HEIGHT
        pose = Pose()
        pose.position.x = full * math.sin(MARKER_TILT_RAD)
        pose.position.y = 0.0
        pose.position.z = -full * math.cos(MARKER_TILT_RAD)
        pose.orientation.x = 0.0
        pose.orientation.y = math.sin(MARKER_TILT_RAD / 2.0)
        pose.orientation.z = 0.0
        pose.orientation.w = math.cos(MARKER_TILT_RAD / 2.0)
        obj.primitive_poses.append(pose)

        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object = obj
        attached.touch_links = ["tool0", "wrist_3_link"]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        scene.robot_state.is_diff = True

        if self._apply(scene):
            self.get_logger().info(
                f'Marker holder attached to tool0 '
                f'(160x180x{EE_DRAW_HEIGHT*1000:.0f} mm, tilt={MARKER_TILT_DEG}°)')


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
    node.add_table()
    node.add_marker_holder()
    node.get_logger().info('Scene setup complete – shutting down')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
