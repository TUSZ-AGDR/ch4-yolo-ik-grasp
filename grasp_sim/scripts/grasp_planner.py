#!/usr/bin/python3
"""Grasp planner: YOLO → world coords → MoveIt2 IK → 7-joint trajectory."""
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped, Pose
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Time, Duration
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformListener, Buffer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection2DArray

import rclpy
import rclpy.duration

JointAngles = List[float]

GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 0.8
GRIPPER_EMPTY_THRESHOLD = 0.75
LIFT_HEIGHT = 0.15
TOOL0_TO_FINGERTIP = 0.13
APPROACH_CLEARANCE = 0.05

CAMERA_X, CAMERA_Y, CAMERA_Z = 0.5, 0.0, 1.145
CAMERA_PITCH = math.pi / 2
BASE_X, BASE_Y, BASE_Z = -0.1, 0.0, 0.80

# Drop-off location: robot's left side, off-table, away from YOLO view
PLACE_X, PLACE_Y, PLACE_Z = 0.40, 0.50, 0.55

JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    "robotiq_85_left_knuckle_joint",
]
GRIPPER_IDX = 6

# Compact folded pose — arm returns here between grasps
HOME_JOINTS = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

# Left-side drop pose: same folded config as HOME, but shoulder rotated 90° left
LEFT_DROP_JOINTS = [1.57, -1.57, 1.57, -1.57, -1.57, 0.0]

# Forward-reaching IK seed — used only to initialise _current_joints for good convergence
IK_SEED = [-0.551, -0.429, 2.045, 3.094, -1.571, -2.122]


def _normalize_angle(a):
    """Normalize angle to [-π, π]"""
    return math.atan2(math.sin(a), math.cos(a))


def rpy_to_quat(rpy):
    cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
    cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
    cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


class GraspPlanner(Node):
    def __init__(self):
        super().__init__("grasp_planner")

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_camera_tf()

        self._current_joints: Dict[str, float] = dict(zip(JOINTS, IK_SEED + [GRIPPER_OPEN]))
        self._joint_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_cb, 10,
        )

        self._det_sub = self.create_subscription(
            Detection2DArray, "/detections", self._detection_cb, 10,
        )

        self._traj_client = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        from rclpy.callback_groups import ReentrantCallbackGroup
        ik_cb_group = ReentrantCallbackGroup()
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=ik_cb_group,
        )

        self._busy = False
        self._busy_lock = threading.Lock()
        self._pending_msg: Optional[Detection2DArray] = None  # latest detection, always cached
        self._grasp_target: Optional[Tuple[float, float, float]] = None
        self._current_label: str = ""
        self._approach_arm: JointAngles = [0.0] * 6
        self._grasped_positions: set = set()  # rounded world coords of grasped objects
        # TF listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Go HOME first so the very first IK doesn't time out
        self._first_home_sent = False
        self._home_timer = self.create_timer(2.0, self._send_initial_home)

        self.get_logger().info("GraspPlanner ready")

    def _publish_camera_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "camera_color_optical_frame"
        t.transform.translation.x = CAMERA_X
        t.transform.translation.y = CAMERA_Y
        t.transform.translation.z = CAMERA_Z
        # Approximate rotation for visualization (Gazebo camera has axis flip
        # that can't be represented as pure rotation, see _optical_to_world)
        sp = math.sin(CAMERA_PITCH / 2)
        cp = math.cos(CAMERA_PITCH / 2)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = sp
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = cp
        self._tf_broadcaster.sendTransform(t)
        self.get_logger().info("Published camera TF")

    def _send_initial_home(self):
        """One-shot timer: move arm to HOME before processing any detections."""
        self._home_timer.cancel()
        self.get_logger().info("Moving to HOME before first grasp...")
        target = list(HOME_JOINTS) + [GRIPPER_OPEN]
        self._send_trajectory(target, duration=2.0, done_cb=self._on_initial_home_done)

    def _on_initial_home_done(self, result):
        if result.error_code != 0:
            self.get_logger().error(f"Initial HOME failed: {result.error_code}")
        self._first_home_sent = True
        with self._busy_lock:
            self._busy = False
        self.get_logger().info("Initial HOME reached — ready for detections")

    def _joint_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self._current_joints:
                self._current_joints[name] = pos

    def _optical_to_world(self, opt_x, opt_y, opt_z) -> Tuple[float, float, float]:
        """Transform point from camera optical frame to world.

        Camera SDF: pose (0.5, 0, 1.145) rpy (0, π/2, 0) — overhead, optical
        axis pointing down. ROS optical convention (x=image right, y=image
        down, z=optical forward) under this rotation maps to:
            opt_x+ → world  -y   (image right = robot right)
            opt_y+ → world  -x   (image down  = table back)
            opt_z+ → world  -z   (optical forward = down)
        Hence the inverse used here.
        """
        wx = CAMERA_X - opt_y
        wy = CAMERA_Y - opt_x
        wz = CAMERA_Z - opt_z
        return (wx, wy, wz)

    # ── IK ─────────────────────────────────────────────────────
    def _call_ik(self, x, y, z,
                 rpy=(math.pi, 0.0, 0.0),
                 seed_state: Optional[Dict[str, float]] = None) -> Optional[JointAngles]:
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_ik not available")
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur5_manipulator"
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = x - BASE_X
        req.ik_request.pose_stamped.pose.position.y = y - BASE_Y
        req.ik_request.pose_stamped.pose.position.z = z - BASE_Z
        qw, qx, qy, qz = rpy_to_quat(rpy)
        req.ik_request.pose_stamped.pose.orientation.w = qw
        req.ik_request.pose_stamped.pose.orientation.x = qx
        req.ik_request.pose_stamped.pose.orientation.y = qy
        req.ik_request.pose_stamped.pose.orientation.z = qz
        req.ik_request.timeout.sec = 1

        if seed_state is not None:
            req.ik_request.robot_state.joint_state.name = list(seed_state.keys())
            req.ik_request.robot_state.joint_state.position = list(seed_state.values())

        event = threading.Event()
        result = [None]

        def done(future):
            try:
                result[0] = future.result()
            except Exception as e:
                self.get_logger().error(f"IK future exception: {e}")
            finally:
                event.set()

        try:
            self._ik_client.call_async(req).add_done_callback(done)
        except Exception as e:
            self.get_logger().error(f"IK call failed: {e}")
            return None

        if not event.wait(timeout=3.0):
            self.get_logger().error("IK timed out")
            return None

        resp = result[0]
        if resp is None or resp.error_code.val != 1:
            self.get_logger().warn(f"IK failed: code={resp.error_code.val if resp else 'None'}")
            return None

        name_to_pos = dict(zip(resp.solution.joint_state.name,
                               resp.solution.joint_state.position))
        try:
            raw = [name_to_pos[j] for j in JOINTS[:6]]
            angles = [_normalize_angle(j) for j in raw]
        except KeyError as e:
            self.get_logger().error(f"IK missing joint: {e}")
            return None

        # If shoulder_pan is extreme (> 80°), the arm is reaching backward.
        # Retry once without seed so MoveIt uses its internal state.
        if abs(angles[0]) > 1.40 and seed_state is not None:  # ~80°
            self.get_logger().warn(f"shoulder_pan={math.degrees(angles[0]):.1f}° extreme, "
                                   "retrying IK without seed...")
            return self._call_ik(x, y, z, rpy=rpy, seed_state=None)

        return angles

    # ── Trajectory helper (7 joints) — two-stage callback ────────
    def _send_trajectory(self, target: JointAngles, duration=2.0, done_cb=None):
        if not self._traj_client.wait_for_server(timeout_sec=3):
            self.get_logger().error("Trajectory server not available")
            with self._busy_lock:
                self._busy = False
            return

        with self._busy_lock:
            self._busy = True
        goal = FollowJointTrajectory.Goal()
        traj = goal.trajectory
        traj.joint_names = JOINTS
        # Use stamp=0 so JTC executes immediately without waiting for a future time
        traj.header.stamp = Time(sec=0, nanosec=0)

        p1 = JointTrajectoryPoint()
        # Unwrap arm joint targets to the representation closest to current
        # joint positions, so JTC always takes the short path (no 360° flips).
        unwrapped = list(target)
        for i, jn in enumerate(JOINTS[:6]):
            if jn in self._current_joints:
                curr = self._current_joints[jn]
                diff = unwrapped[i] - curr
                unwrapped[i] = curr + math.atan2(math.sin(diff), math.cos(diff))
        p1.positions = unwrapped
        p1.velocities = [0.0] * 7
        sec = int(duration)
        nanosec = int((duration - sec) * 1e9)
        p1.time_from_start = Duration(sec=sec, nanosec=nanosec)

        traj.points = [p1]
        goal.goal_time_tolerance = Duration(sec=0, nanosec=0)  # 0 = disable goal time check

        goal_handle_future = self._traj_client.send_goal_async(goal)

        def _on_goal_response(future):
            goal_handle = future.result()
            self.get_logger().info(f"Goal accepted={goal_handle.accepted}")
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected")
                threading.Timer(2.0, self._reset_busy).start()
                return

            def _on_result(future):
                response = future.result()
                error_code = response.result.error_code
                self.get_logger().info(f"Trajectory done: error_code={error_code}")
                if done_cb:
                    done_cb(response.result)

            goal_handle.get_result_async().add_done_callback(_on_result)

        goal_handle_future.add_done_callback(_on_goal_response)

    # ── Stage 1: approach ──────────────────────────────────────
    def _detection_cb(self, msg: Detection2DArray):
        # Always cache the latest detection so real-time coords survive busy periods
        if msg.detections:
            self._pending_msg = msg

        # Don't process detections until initial HOME trajectory completes
        if not self._first_home_sent:
            return

        with self._busy_lock:
            if self._busy:
                return
            self._busy = True

        # Consume the freshest pending message
        msg = self._pending_msg
        self._pending_msg = None
        if msg is None or not msg.detections:
            self._reset_busy()
            return

        self._process_detections(msg)

    def _process_detections(self, msg: Detection2DArray):
        """Process a detection message: pick best ungrasped object → IK → approach."""
        candidates = []
        for d in msg.detections:
            if not d.results:
                continue
            label = d.results[0].hypothesis.class_id
            score = d.results[0].hypothesis.score
            pose = d.results[0].pose.pose.position
            if abs(pose.z) < 1e-6:
                self.get_logger().warn(f"  Skip {label}: no depth (pose.z=0)")
                continue
            wx, wy, wz = self._optical_to_world(pose.x, pose.y, pose.z)
            # Only grasp objects on the table (table top z=0.41).
            # Anything below 0.3 is on the ground (already-placed objects
            # that fell off after release at the drop-off location).
            if wz < 0.3:
                continue
            candidates.append((score, label, d, (wx, wy, wz)))
        # Sort by Y (left-to-right) to avoid arm sweeping across other objects
        candidates.sort(key=lambda x: (x[3][1], -x[0]))

        all_labels = [c[1] for c in candidates]
        self.get_logger().info(f"Detected: {all_labels} | already grasped: {len(self._grasped_positions)}")

        best_det, best_score, best_label, best_world = None, 0.0, "", None
        for score, label, det, world in candidates:
            wkey = (round(world[0], 2), round(world[1], 2), round(world[2], 2))
            if wkey in self._grasped_positions:
                continue
            best_det, best_score, best_label, best_world = det, score, label, world
            break

        if best_det is None or best_score < 0.3:
            ungrasped = [c[1] for c in candidates
                         if (round(c[3][0], 2), round(c[3][1], 2), round(c[3][2], 2))
                         not in self._grasped_positions]
            if not ungrasped and candidates:
                self.get_logger().info(
                    f"All {len(self._grasped_positions)} objects grasped. Done!"
                )
                self._reset_busy()
            else:
                threading.Timer(0.5, self._reset_busy).start()
            return

        self._current_label = best_label
        wx, wy, wz = best_world
        self.get_logger().info(f"→ {best_label} ({best_score:.2f}) world=({wx:.3f}, {wy:.3f}, {wz:.3f})")

        self._grasp_target = (wx, wy, wz)
        approach_z = wz + TOOL0_TO_FINGERTIP + APPROACH_CLEARANCE

        # Always use forward-reaching seed for approach IK — using _current_joints
        # (which becomes HOME_JOINTS after first cycle) leads to extreme shoulder_pan.
        seed = {jn: IK_SEED[i] for i, jn in enumerate(JOINTS[:6])}
        arm_joints = self._call_ik(wx, wy, approach_z, seed_state=seed)
        if arm_joints is None:
            self._open_and_reset()
            return

        degs = [f"{math.degrees(j):.1f}°" for j in arm_joints]
        self.get_logger().info(f"  Approach: {degs}")
        # Save approach config so lift IK uses the same solution branch,
        # preventing wrist_1 from flipping +175° → -175° (350° trajectory).
        self._approach_arm = list(arm_joints)

        target = list(arm_joints) + [GRIPPER_OPEN]
        self._send_trajectory(target, duration=2.0, done_cb=self._on_approach_done)

    # ── Stage 2: descend to grasp height ────────────────────────
    def _on_approach_done(self, result):
        if result.error_code != 0:
            self.get_logger().error(f"Approach trajectory failed: error_code={result.error_code}")
            self._open_and_reset()
            return
        self.get_logger().info("Approach done — descending to grasp height")

        wx, wy, wz = self._grasp_target
        grasp_z = wz + TOOL0_TO_FINGERTIP
        # Use approach config (same IK branch) for descend, just like lift
        seed = {jn: self._approach_arm[i] for i, jn in enumerate(JOINTS[:6])}
        arm_joints = self._call_ik(wx, wy, grasp_z, seed_state=seed)
        if arm_joints is None:
            self.get_logger().error("Descend IK failed")
            self._open_and_reset()
            return

        target = list(arm_joints) + [GRIPPER_OPEN]
        self._send_trajectory(target, duration=1.5, done_cb=self._on_descend_done)

    # ── Stage 3: close gripper ─────────────────────────────────
    def _on_descend_done(self, result):
        if result.error_code != 0:
            self.get_logger().error(f"Descend trajectory failed: error_code={result.error_code}")
            self._open_and_reset()
            return

        try:
            tf = self._tf_buffer.lookup_transform(
                "world", "tool0", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            self.get_logger().info(
                f"Descend done — tool0 at ({t.x:.3f},{t.y:.3f},{t.z:.3f}) "
                f"quat=({r.x:.3f},{r.y:.3f},{r.z:.3f},{r.w:.3f})"
            )
        except Exception:
            self.get_logger().info("Descend done — closing gripper")

        seed = dict(self._current_joints)
        target = [self._current_joints[jn] for jn in JOINTS[:6]] + [GRIPPER_CLOSE]
        self._send_trajectory(target, duration=3.0,
                              done_cb=lambda r, seed=seed: self._on_close_done(r, seed))

    def _on_close_done(self, result, lift_seed: Dict[str, float]):
        if result.error_code != 0:
            self.get_logger().error(f"Close trajectory failed: error_code={result.error_code}")
            self._open_and_reset()
            return
        # Wait for physics to settle: control-level mimic + 4-bar linkage
        # needs time after JTC trajectory ends to converge on contact force.
        time.sleep(0.5)
        gripper_pos = self._current_joints["robotiq_85_left_knuckle_joint"]
        self.get_logger().info(f"Close done: gripper_pos={gripper_pos:.4f}")

        if gripper_pos > GRIPPER_EMPTY_THRESHOLD:
            self.get_logger().warn(
                f"Gripper closed to {gripper_pos:.3f} (> {GRIPPER_EMPTY_THRESHOLD}) — no object"
            )
            self._open_and_reset()
            return

        if gripper_pos < 0.02:
            self.get_logger().warn("Gripper barely moved — treating as no object")
            self._open_and_reset()
            return

        self.get_logger().info("Object grasped — lifting...")
        self._send_lift(lift_seed)

    # ── Stage 3: lift ──────────────────────────────────────────
    def _send_lift(self, seed_state: Dict[str, float]):
        if self._grasp_target is None:
            self._open_and_reset()
            return
        wx, wy, wz = self._grasp_target
        lift_z = wz + TOOL0_TO_FINGERTIP + LIFT_HEIGHT

        # Use approach config (not descend config) as IK seed so the
        # solver stays in the same solution branch and wrist_1 doesn't flip.
        lift_seed = {jn: self._approach_arm[i] for i, jn in enumerate(JOINTS[:6])}
        arm_joints = self._call_ik(wx, wy, lift_z, seed_state=lift_seed)
        if arm_joints is None:
            self._open_and_reset()
            return

        degs = [f"{math.degrees(j):.1f}°" for j in arm_joints]
        self.get_logger().info(f"  Lift: {degs}")

        gripper_pos = self._current_joints["robotiq_85_left_knuckle_joint"]
        target = list(arm_joints) + [GRIPPER_CLOSE]
        self._send_trajectory(target, duration=2.0, done_cb=self._on_lift_done)

    def _on_lift_done(self, result):
        if result.error_code == 0:
            self.get_logger().info("=== LIFT OK — returning home ===")
            if self._grasp_target:
                wx, wy, wz = self._grasp_target
                wkey = (round(wx, 2), round(wy, 2), round(wz, 2))
                self._grasped_positions.add(wkey)
                self.get_logger().info(f"  Recorded: world=({wx:.3f},{wy:.3f},{wz:.3f}) "
                                       f"({len(self._grasped_positions)} objects grasped)")
        else:
            self.get_logger().error(f"Lift trajectory failed: error_code={result.error_code}")
            target = [self._current_joints[jn] for jn in JOINTS[:6]] + [GRIPPER_OPEN]
            self._send_trajectory(target, duration=1.0, done_cb=self._on_open_done)
            return
        # Go home with gripper closed, then release — no lateral PLACE move
        target = list(HOME_JOINTS) + [GRIPPER_CLOSE]
        self._send_trajectory(target, duration=2.0, done_cb=self._on_home_with_object_done)

    def _on_home_with_object_done(self, result):
        if result.error_code != 0:
            self.get_logger().error(f"Home-with-object failed: error_code={result.error_code}")
        self.get_logger().info("=== AT HOME — swinging left to drop ===")
        target = list(LEFT_DROP_JOINTS) + [GRIPPER_CLOSE]
        self._send_trajectory(target, duration=2.0, done_cb=self._on_left_drop_done)

    def _on_left_drop_done(self, result):
        if result.error_code != 0:
            self.get_logger().error(f"Left-drop move failed: error_code={result.error_code}")
        self.get_logger().info("=== LEFT DROP — releasing object ===")
        target = list(LEFT_DROP_JOINTS) + [GRIPPER_OPEN]
        self._send_trajectory(target, duration=1.0, done_cb=self._on_open_done)

    def _send_to_place(self):
        """IK to PLACE_X/Y/Z + tool offset, keep gripper closed."""
        place_z = PLACE_Z + TOOL0_TO_FINGERTIP
        # Use forward-reaching IK_SEED instead of current (possibly curled) joints,
        # so the arm extends outward rather than folding inward toward the base.
        seed = {jn: IK_SEED[i] for i, jn in enumerate(JOINTS[:6])}
        arm_joints = self._call_ik(PLACE_X, PLACE_Y, place_z, seed_state=seed)
        if arm_joints is None:
            self.get_logger().error("IK to place location failed — opening here")
            target = [self._current_joints[jn] for jn in JOINTS[:6]] + [GRIPPER_OPEN]
            self._send_trajectory(target, duration=1.0, done_cb=self._on_open_done)
            return
        degs = [f"{math.degrees(j):.1f}°" for j in arm_joints]
        self.get_logger().info(f"  Place move: {degs}")
        gripper_pos = self._current_joints["robotiq_85_left_knuckle_joint"]
        target = list(arm_joints) + [gripper_pos]  # keep gripper closed during transit
        self._send_trajectory(target, duration=2.5, done_cb=self._on_place_done)

    def _on_place_done(self, result):
        if result.error_code != 0:
            self.get_logger().error(f"Place move failed: error_code={result.error_code}")
        else:
            self.get_logger().info("=== AT PLACE — releasing object ===")
        # Open gripper at the place location (object falls)
        target = [self._current_joints[jn] for jn in JOINTS[:6]] + [GRIPPER_OPEN]
        self._send_trajectory(target, duration=1.0, done_cb=self._on_open_done)

    def _on_open_done(self, result):
        self.get_logger().info("Returning to home position...")
        target = list(HOME_JOINTS) + [GRIPPER_OPEN]
        self._send_trajectory(target, duration=2.0, done_cb=self._on_home_done)

    def _on_home_done(self, result):
        threading.Timer(2.0, self._reset_busy).start()

    def _reset_busy(self):
        self._pending_msg = None  # discard stale cache, wait for fresh detection
        with self._busy_lock:
            self._busy = False
        self.get_logger().info("Ready for next grasp")

    def _open_and_reset(self):
        self.get_logger().info("Opening gripper and returning home...")
        # Build target: current arm position (don't move) + open gripper
        target = [self._current_joints[jn] for jn in JOINTS[:6]] + [GRIPPER_OPEN]
        self._send_trajectory(target, duration=1.0, done_cb=self._on_reset_open_done)

    def _on_reset_open_done(self, result):
        # Now go to home position
        target = list(HOME_JOINTS) + [GRIPPER_OPEN]
        self._send_trajectory(target, duration=2.0, done_cb=self._on_home_done)


def main():
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    node = GraspPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, Exception) as e:
        if not isinstance(e, KeyboardInterrupt):
            node.get_logger().error(f"Fatal: {e}")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()