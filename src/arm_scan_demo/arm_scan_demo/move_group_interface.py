#!/usr/bin/env python3
"""
Sphere scan using pymoveit2 (ROS 2 Humble).
"""

import inspect
import sys
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, PoseArray, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from pymoveit2 import MoveIt2


# ============================================================
# CONFIG
# ============================================================

BASE_FRAME = "base_link"
PLANNING_GROUP = "ur_manipulator"
END_EFFECTOR_LINK = "tool_tip"

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

USE_CARTESIAN_FALLBACK = True
CARTESIAN_FRACTION_THRESHOLD = 0.95
CARTESIAN_MAX_STEP = 0.01

MAX_VELOCITY_SCALING = 0.15
MAX_ACCELERATION_SCALING = 0.15


# ============================================================
# ROBOT PEDESTAL
# ============================================================

PEDESTAL_HEIGHT = 0.40
PEDESTAL_SIZE_XY = (0.15, 0.15)
PEDESTAL_COLLISION_ID = "pedestal"
PEDESTAL_MARKER_NS = "scan_pedestal"
PEDESTAL_MARKER_ID = 1


# ============================================================
# FLOOR
# ============================================================

FLOOR_BOX_SIZE = (3.0, 3.0, 0.02)

# Floor top is at -0.40 m.
FLOOR_Z = -PEDESTAL_HEIGHT - (FLOOR_BOX_SIZE[2] / 2.0)

FLOOR_TOP_Z = FLOOR_Z + (FLOOR_BOX_SIZE[2] / 2.0)

FLOOR_COLLISION_ID = "floor"
FLOOR_MARKER_NS = "scan_floor"
FLOOR_MARKER_ID = 0


# ============================================================
# SCAN OBJECT
# ============================================================

SPHERE_COLLISION_ID = "scan_object"

SPHERE_CENTER = np.array([
    0.55,
    0.00,
    0.45,
])

SPHERE_RADIUS = 0.15
SCAN_RADIUS = 0.35


# ============================================================
# SCAN OBJECT SUPPORT
# ============================================================

SPHERE_SUPPORT_COLLISION_ID = "scan_object_support"
SPHERE_SUPPORT_SIZE_XY = (0.10, 0.10)

SPHERE_SUPPORT_MARKER_NS = "scan_sphere_support"
SPHERE_SUPPORT_MARKER_ID = 2

# IMPORTANT:
# The support now starts at the top of the floor and reaches the
# bottom of the scan sphere.
SPHERE_SUPPORT_BOTTOM_Z = FLOOR_TOP_Z

SPHERE_SUPPORT_TOP_Z = float(
    SPHERE_CENTER[2] - SPHERE_RADIUS
)

SPHERE_SUPPORT_HEIGHT = float(
    SPHERE_SUPPORT_TOP_Z - SPHERE_SUPPORT_BOTTOM_Z
)

SPHERE_SUPPORT_CENTER_Z = float(
    (SPHERE_SUPPORT_BOTTOM_Z + SPHERE_SUPPORT_TOP_Z) / 2.0
)


# ============================================================
# ATTACHED TOOL COLLISION SPHERE
# ============================================================

TOOL_COLLISION_LINK = "tool_tip"
TOOL_COLLISION_SPHERE_ID = "tool_collision_sphere"
TOOL_COLLISION_SPHERE_RADIUS = 0.15
TOOL_COLLISION_SPHERE_OFFSET = (0.0, 0.0, 0.0)

TOOL_COLLISION_TOUCH_LINKS = [
    "tool_tip",
    "tool_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
]


# ============================================================
# SCAN PATTERN
# ============================================================

NUM_POINTS = 24
SCAN_PATTERN = "full_sphere"
ELEVATIONS_DEG = [-30.0, 0.0, 30.0]
# Only bites once NUM_POINTS is large enough to place a point above
# 90 - POLE_EXCLUSION_DEG; at NUM_POINTS = 24 the highest point sits at
# 73.4 deg, so nothing is dropped.
POLE_EXCLUSION_DEG = 15.0


# ============================================================
# WAYPOINT ORDERING
# ============================================================

USE_NEAREST_NEIGHBOR_ORDERING = True


# ============================================================
# WAYPOINT MARKERS (scan points on the sphere)
# ============================================================

# Toggle the two visualization styles independently.
SHOW_WAYPOINT_POINTS = True
SHOW_WAYPOINT_AXES = True

WAYPOINT_POINTS_NS = "scan_waypoint_points"
WAYPOINT_POINTS_ID = 100
WAYPOINT_POINT_SIZE = 0.015
WAYPOINT_POINT_COLOR = (0.0, 1.0, 1.0)  # cyan

WAYPOINT_AXES_NS = "scan_waypoint_axes"
WAYPOINT_AXIS_LENGTH = 0.05
WAYPOINT_AXIS_WIDTH = 0.004
# RGB convention: X=red, Y=green, Z=blue (Z is the approach/look axis).
WAYPOINT_AXIS_COLORS = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.4, 1.0),
)

WAYPOINT_LABELS_NS = "scan_waypoint_labels"
SHOW_WAYPOINT_LABELS = False
WAYPOINT_LABEL_SIZE = 0.02


# ============================================================
# TERMINAL PROGRESS
# ============================================================

PROGRESS_UPDATE_PERIOD_SEC = 0.5

PROGRESS_NEWLINE_MODE = False

PROGRESS_STALL_WARNING_SEC = 3.0

MIN_JOINT_MOTION_RAD = 0.01


# ============================================================
# QUATERNION / GEOMETRY
# ============================================================

def quaternion_multiply(q1, q0):
    x1, y1, z1, w1 = q1
    x0, y0, z0, w0 = q0

    return np.array([
        w1 * x0 + x1 * w0 + y1 * z0 - z1 * y0,
        w1 * y0 - x1 * z0 + y1 * w0 + z1 * x0,
        w1 * z0 + x1 * y0 - y1 * x0 + z1 * w0,
        w1 * w0 - x1 * x0 - y1 * y0 - z1 * z0,
    ])


def rotate_vector_by_quaternion(v, q):
    """
    Rotate a 3D vector `v` by a quaternion `q` given as (x, y, z, w).
    """
    x, y, z, w = q
    qv = np.array([x, y, z])

    t = 2.0 * np.cross(qv, v)

    return v + w * t + np.cross(qv, t)


def rotation_matrix_to_quaternion(R):
    trace = np.trace(R)

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s

    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(
            1.0 + R[0, 0] - R[1, 1] - R[2, 2]
        )
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s

    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(
            1.0 + R[1, 1] - R[0, 0] - R[2, 2]
        )
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s

    else:
        s = 2.0 * np.sqrt(
            1.0 + R[2, 2] - R[0, 0] - R[1, 1]
        )
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return np.array([x, y, z, w])


def look_at(camera_position, camera_target, up_vector):
    """
    Return a camera-to-world rotation with local +Z aimed at target.
    """
    forward = camera_target - camera_position
    forward_norm = np.linalg.norm(forward)

    if forward_norm < 1e-9:
        raise ValueError(
            "camera_position and camera_target are identical."
        )

    forward /= forward_norm

    right = np.cross(up_vector, forward)
    right_norm = np.linalg.norm(right)

    if right_norm < 1e-6:
        fallback_up = np.array([1.0, 0.0, 0.0])
        right = np.cross(fallback_up, forward)
        right_norm = np.linalg.norm(right)

    right /= right_norm

    up = np.cross(forward, right)
    up /= np.linalg.norm(up)

    return np.column_stack((right, up, forward))


# Keep this unchanged so the scanner points toward the object.
TOOL_MOUNT_OFFSET_QUAT_XYZW = np.array([
    0.0,
    0.0,
    0.0,
    1.0,
])


def orientation_from_point(point, target):
    """
    Orient the tool local +Z axis directly toward the scan object.
    """
    rotation = look_at(
        point,
        target,
        np.array([0.0, 0.0, 1.0]),
    )

    quaternion = rotation_matrix_to_quaternion(rotation)

    if not np.allclose(
        TOOL_MOUNT_OFFSET_QUAT_XYZW,
        [0.0, 0.0, 0.0, 1.0],
    ):
        quaternion = quaternion_multiply(
            quaternion,
            TOOL_MOUNT_OFFSET_QUAT_XYZW,
        )

    return quaternion


# ============================================================
# RVIZ MARKERS
# ============================================================

def create_sphere_marker(x, y, z, radius):
    marker = Marker()

    marker.header.frame_id = BASE_FRAME
    marker.ns = "scan_sphere"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = float(z)

    marker.pose.orientation.w = 1.0

    marker.scale.x = float(radius * 2.0)
    marker.scale.y = float(radius * 2.0)
    marker.scale.z = float(radius * 2.0)

    marker.color.a = 0.25
    marker.color.r = 1.0
    marker.color.g = 0.8
    marker.color.b = 0.0

    return marker


def create_floor_marker():
    marker = Marker()

    marker.header.frame_id = BASE_FRAME
    marker.ns = FLOOR_MARKER_NS
    marker.id = FLOOR_MARKER_ID
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = FLOOR_Z

    marker.pose.orientation.w = 1.0

    marker.scale.x = float(FLOOR_BOX_SIZE[0])
    marker.scale.y = float(FLOOR_BOX_SIZE[1])
    marker.scale.z = float(FLOOR_BOX_SIZE[2])

    marker.color.a = 0.25
    marker.color.r = 0.55
    marker.color.g = 0.55
    marker.color.b = 0.55

    return marker


def create_pedestal_marker():
    marker = Marker()

    marker.header.frame_id = BASE_FRAME
    marker.ns = PEDESTAL_MARKER_NS
    marker.id = PEDESTAL_MARKER_ID
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = -PEDESTAL_HEIGHT / 2.0

    marker.pose.orientation.w = 1.0

    marker.scale.x = float(PEDESTAL_SIZE_XY[0])
    marker.scale.y = float(PEDESTAL_SIZE_XY[1])
    marker.scale.z = float(PEDESTAL_HEIGHT)

    marker.color.a = 0.9
    marker.color.r = 0.35
    marker.color.g = 0.35
    marker.color.b = 0.38

    return marker


def create_sphere_support_marker():
    """
    Square pedestal extending from the top of the floor to the bottom
    of the scan sphere.
    """
    marker = Marker()

    marker.header.frame_id = BASE_FRAME
    marker.ns = SPHERE_SUPPORT_MARKER_NS
    marker.id = SPHERE_SUPPORT_MARKER_ID
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    marker.pose.position.x = float(SPHERE_CENTER[0])
    marker.pose.position.y = float(SPHERE_CENTER[1])
    marker.pose.position.z = float(SPHERE_SUPPORT_CENTER_Z)

    marker.pose.orientation.w = 1.0

    marker.scale.x = float(SPHERE_SUPPORT_SIZE_XY[0])
    marker.scale.y = float(SPHERE_SUPPORT_SIZE_XY[1])
    marker.scale.z = float(SPHERE_SUPPORT_HEIGHT)

    marker.color.a = 0.9
    marker.color.r = 0.35
    marker.color.g = 0.35
    marker.color.b = 0.38

    return marker


def create_waypoint_points_marker(xyz):
    """
    A single POINTS marker showing every scan waypoint on the sphere
    as a small dot. Cheap to render even for many waypoints.
    """
    marker = Marker()

    marker.header.frame_id = BASE_FRAME
    marker.ns = WAYPOINT_POINTS_NS
    marker.id = WAYPOINT_POINTS_ID
    marker.type = Marker.POINTS
    marker.action = Marker.ADD

    marker.pose.orientation.w = 1.0

    marker.scale.x = float(WAYPOINT_POINT_SIZE)
    marker.scale.y = float(WAYPOINT_POINT_SIZE)

    marker.color.a = 1.0
    marker.color.r = float(WAYPOINT_POINT_COLOR[0])
    marker.color.g = float(WAYPOINT_POINT_COLOR[1])
    marker.color.b = float(WAYPOINT_POINT_COLOR[2])

    for position in xyz:
        point = Point()
        point.x = float(position[0])
        point.y = float(position[1])
        point.z = float(position[2])
        marker.points.append(point)

    return marker


def create_waypoint_axes_marker_array(xyz, quaternions):
    """
    One small RGB coordinate-frame (X/Y/Z axis lines) per waypoint,
    showing the scan orientation at each point on the sphere.
    Packed into a single LINE_LIST marker for efficient rendering.
    """
    marker = Marker()

    marker.header.frame_id = BASE_FRAME
    marker.ns = WAYPOINT_AXES_NS
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    marker.pose.orientation.w = 1.0
    marker.scale.x = float(WAYPOINT_AXIS_WIDTH)

    axis_vectors = (
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
    )

    for position, quaternion in zip(xyz, quaternions):

        origin = Point()
        origin.x = float(position[0])
        origin.y = float(position[1])
        origin.z = float(position[2])

        for axis_vector, color in zip(
            axis_vectors,
            WAYPOINT_AXIS_COLORS,
        ):

            rotated = rotate_vector_by_quaternion(
                axis_vector,
                quaternion,
            )

            tip = np.asarray(position) + (
                rotated * WAYPOINT_AXIS_LENGTH
            )

            tip_point = Point()
            tip_point.x = float(tip[0])
            tip_point.y = float(tip[1])
            tip_point.z = float(tip[2])

            marker.points.append(origin)
            marker.points.append(tip_point)

            line_color = ColorRGBA(
                r=float(color[0]),
                g=float(color[1]),
                b=float(color[2]),
                a=1.0,
            )

            marker.colors.append(line_color)
            marker.colors.append(line_color)

    return marker


def create_waypoint_delete_all_markers():
    """
    Clears previously published waypoint point/axis markers so that
    re-running the script (e.g. with a different NUM_POINTS) doesn't
    leave stale markers behind in RViz.
    """
    markers = []

    for ns in (WAYPOINT_POINTS_NS, WAYPOINT_AXES_NS):
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.ns = ns
        marker.action = Marker.DELETEALL
        markers.append(marker)

    return markers


# ============================================================
# POSE GENERATION
# ============================================================

def generate_ring(center, scan_radius, num_points, elevation_deg=0.0):
    theta = np.linspace(
        0.0,
        2.0 * np.pi,
        num_points,
        endpoint=False,
    )

    elevation = np.deg2rad(elevation_deg)

    xyz = np.zeros((num_points, 3))

    xyz[:, 0] = (
        center[0]
        + scan_radius * np.cos(theta) * np.cos(elevation)
    )

    xyz[:, 1] = (
        center[1]
        + scan_radius * np.sin(theta) * np.cos(elevation)
    )

    xyz[:, 2] = (
        center[2]
        + scan_radius * np.sin(elevation)
    )

    quaternions = np.array([
        orientation_from_point(point, center)
        for point in xyz
    ])

    return xyz, quaternions


def generate_full_sphere(
    center,
    scan_radius,
    num_points,
    pole_exclusion_deg=15.0,
):
    golden_angle = np.pi * (3.0 - np.sqrt(5.0))

    indices = np.arange(num_points)

    z_normalized = (
        1.0
        - 2.0 * (indices + 0.5) / num_points
    )

    elevation = np.arcsin(
        np.clip(z_normalized, -1.0, 1.0)
    )

    azimuth = indices * golden_angle

    pole_limit = np.deg2rad(
        90.0 - pole_exclusion_deg
    )

    keep = np.abs(elevation) <= pole_limit

    elevation = elevation[keep]
    azimuth = azimuth[keep]

    radial_xy = np.cos(elevation) * scan_radius

    x = (
        center[0]
        + radial_xy * np.cos(azimuth)
    )

    y = (
        center[1]
        + radial_xy * np.sin(azimuth)
    )

    z = (
        center[2]
        + np.sin(elevation) * scan_radius
    )

    xyz = np.stack(
        [x, y, z],
        axis=1,
    )

    quaternions = np.array([
        orientation_from_point(point, center)
        for point in xyz
    ])

    return xyz, quaternions


def generate_scan(
    center,
    sphere_radius,
    scan_radius,
    num_points,
    pattern="full_sphere",
    elevations_deg=None,
    pole_exclusion_deg=15.0,
):
    if pattern == "ring":

        xyz, quaternions = generate_ring(
            center=center,
            scan_radius=scan_radius,
            num_points=num_points,
            elevation_deg=0.0,
        )

    elif pattern == "bands":

        if not elevations_deg:

            raise ValueError(
                "SCAN_PATTERN 'bands' requires a "
                "non-empty ELEVATIONS_DEG."
            )

        xyz_list = []
        quaternion_list = []

        for elevation_deg in elevations_deg:

            ring_xyz, ring_quaternions = generate_ring(
                center=center,
                scan_radius=scan_radius,
                num_points=num_points,
                elevation_deg=elevation_deg,
            )

            xyz_list.append(ring_xyz)
            quaternion_list.append(ring_quaternions)

        xyz = np.vstack(xyz_list)
        quaternions = np.vstack(quaternion_list)

    elif pattern == "full_sphere":

        xyz, quaternions = generate_full_sphere(
            center=center,
            scan_radius=scan_radius,
            num_points=num_points,
            pole_exclusion_deg=pole_exclusion_deg,
        )

    else:
        raise ValueError(
            f"Unknown SCAN_PATTERN: {pattern!r}"
        )

    return (
        center,
        sphere_radius,
        xyz,
        quaternions,
    )


# ============================================================
# WAYPOINT ORDERING
# ============================================================

def nearest_neighbor_order(
    xyz,
    quaternions,
    start_position,
):
    count = len(xyz)

    if count <= 1:
        return (
            xyz,
            quaternions,
            np.arange(count),
        )

    remaining = set(range(count))
    ordered_indices = []

    current = np.asarray(
        start_position,
        dtype=float,
    )

    while remaining:

        candidates = np.fromiter(
            remaining,
            dtype=int,
        )

        distances = np.linalg.norm(
            xyz[candidates] - current,
            axis=1,
        )

        chosen = int(
            candidates[np.argmin(distances)]
        )

        ordered_indices.append(chosen)

        remaining.remove(chosen)

        current = xyz[chosen]

    order = np.asarray(
        ordered_indices,
        dtype=int,
    )

    return (
        xyz[order],
        quaternions[order],
        order,
    )


# ============================================================
# VISUALIZATION NODE
# ============================================================

class SphereScanVizNode(Node):

    def __init__(self):
        super().__init__(
            "sphere_scan_viz_node"
        )

        self.pose_pub = self.create_publisher(
            PoseArray,
            "/poses",
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/visualization_marker",
            10,
        )

        self.marker_array_pub = self.create_publisher(
            MarkerArray,
            "/visualization_marker_array",
            10,
        )

        self._sphere_marker = None
        self._floor_marker = None
        self._pedestal_marker = None
        self._sphere_support_marker = None
        self._waypoint_points_marker = None
        self._waypoint_axes_marker = None
        self._pose_array = None

        self._viz_timer = self.create_timer(
            1.0,
            self._republish_visualization,
        )

        self.get_logger().info(
            "Sphere Scan (pymoveit2) node initialized."
        )

    def publish_visualization(
        self,
        sphere_center,
        sphere_radius,
        xyz,
        quaternions,
    ):
        sphere_marker = create_sphere_marker(
            *sphere_center,
            sphere_radius,
        )

        floor_marker = create_floor_marker()
        pedestal_marker = create_pedestal_marker()
        sphere_support_marker = (
            create_sphere_support_marker()
        )

        now = self.get_clock().now().to_msg()

        sphere_marker.header.stamp = now
        floor_marker.header.stamp = now
        pedestal_marker.header.stamp = now
        sphere_support_marker.header.stamp = now

        pose_array = PoseArray()

        pose_array.header.frame_id = BASE_FRAME
        pose_array.header.stamp = now

        for position, quaternion in zip(
            xyz,
            quaternions,
        ):
            pose = Pose()

            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])

            pose.orientation.x = float(quaternion[0])
            pose.orientation.y = float(quaternion[1])
            pose.orientation.z = float(quaternion[2])
            pose.orientation.w = float(quaternion[3])

            pose_array.poses.append(pose)

        self._sphere_marker = sphere_marker
        self._floor_marker = floor_marker
        self._pedestal_marker = pedestal_marker
        self._sphere_support_marker = (
            sphere_support_marker
        )
        self._pose_array = pose_array

        # ------------------------------------------------
        # Waypoint markers (points / axes on the sphere)
        # ------------------------------------------------

        delete_markers = create_waypoint_delete_all_markers()

        for marker in delete_markers:
            marker.header.stamp = now

        self.marker_array_pub.publish(
            MarkerArray(markers=delete_markers)
        )

        waypoint_points_marker = None
        waypoint_axes_marker = None

        if SHOW_WAYPOINT_POINTS:

            waypoint_points_marker = (
                create_waypoint_points_marker(xyz)
            )
            waypoint_points_marker.header.stamp = now

        if SHOW_WAYPOINT_AXES:

            waypoint_axes_marker = (
                create_waypoint_axes_marker_array(
                    xyz,
                    quaternions,
                )
            )
            waypoint_axes_marker.header.stamp = now

        self._waypoint_points_marker = waypoint_points_marker
        self._waypoint_axes_marker = waypoint_axes_marker

        self.marker_pub.publish(
            sphere_marker
        )

        self.marker_pub.publish(
            floor_marker
        )

        self.marker_pub.publish(
            pedestal_marker
        )

        self.marker_pub.publish(
            sphere_support_marker
        )

        waypoint_markers = [
            marker
            for marker in (
                waypoint_points_marker,
                waypoint_axes_marker,
            )
            if marker is not None
        ]

        if waypoint_markers:

            self.marker_array_pub.publish(
                MarkerArray(markers=waypoint_markers)
            )

        self.pose_pub.publish(
            pose_array
        )

    def _republish_visualization(self):

        if self._sphere_marker is None:
            return

        stamp = self.get_clock().now().to_msg()

        self._sphere_marker.header.stamp = stamp

        self.marker_pub.publish(
            self._sphere_marker
        )

        if self._floor_marker is not None:

            self._floor_marker.header.stamp = stamp

            self.marker_pub.publish(
                self._floor_marker
            )

        if self._pedestal_marker is not None:

            self._pedestal_marker.header.stamp = stamp

            self.marker_pub.publish(
                self._pedestal_marker
            )

        if self._sphere_support_marker is not None:

            self._sphere_support_marker.header.stamp = stamp

            self.marker_pub.publish(
                self._sphere_support_marker
            )

        waypoint_markers = [
            marker
            for marker in (
                self._waypoint_points_marker,
                self._waypoint_axes_marker,
            )
            if marker is not None
        ]

        if waypoint_markers:

            for marker in waypoint_markers:
                marker.header.stamp = stamp

            self.marker_array_pub.publish(
                MarkerArray(markers=waypoint_markers)
            )

        if self._pose_array is not None:

            self._pose_array.header.stamp = stamp

            self.pose_pub.publish(
                self._pose_array
            )


# ============================================================
# PLANNING SCENE
# ============================================================

def add_planning_scene_objects(
    moveit2,
    logger,
):
    logger.info(
        "Adding floor collision object..."
    )

    moveit2.add_collision_box(
        id=FLOOR_COLLISION_ID,
        size=FLOOR_BOX_SIZE,
        position=[
            0.0,
            0.0,
            FLOOR_Z,
        ],
        quat_xyzw=[
            0.0,
            0.0,
            0.0,
            1.0,
        ],
        frame_id=BASE_FRAME,
    )

    time.sleep(1.0)

    logger.info(
        f"Adding pedestal collision object "
        f"(height={PEDESTAL_HEIGHT:.2f} m)..."
    )

    moveit2.add_collision_box(
        id=PEDESTAL_COLLISION_ID,
        size=(
            PEDESTAL_SIZE_XY[0],
            PEDESTAL_SIZE_XY[1],
            PEDESTAL_HEIGHT,
        ),
        position=[
            0.0,
            0.0,
            -PEDESTAL_HEIGHT / 2.0,
        ],
        quat_xyzw=[
            0.0,
            0.0,
            0.0,
            1.0,
        ],
        frame_id=BASE_FRAME,
    )

    time.sleep(1.0)

    logger.info(
        "Adding scan-object collision sphere..."
    )

    moveit2.add_collision_sphere(
        id=SPHERE_COLLISION_ID,
        radius=float(SPHERE_RADIUS),
        position=SPHERE_CENTER.tolist(),
        quat_xyzw=[
            0.0,
            0.0,
            0.0,
            1.0,
        ],
        frame_id=BASE_FRAME,
    )

    time.sleep(1.0)

    # A sphere resting on (or below) the floor leaves no room for a support,
    # and a zero/negative box dimension is rejected by MoveIt.
    if SPHERE_SUPPORT_HEIGHT <= 0.0:

        logger.warn(
            f"Skipping scan-object support: computed height "
            f"{SPHERE_SUPPORT_HEIGHT:.3f} m is not positive. "
            f"Raise SPHERE_CENTER[2] above "
            f"{FLOOR_TOP_Z + SPHERE_RADIUS:.3f} m."
        )

    else:

        logger.info(
            f"Adding scan-object support pedestal "
            f"from floor to sphere "
            f"(height={SPHERE_SUPPORT_HEIGHT:.2f} m)..."
        )

        moveit2.add_collision_box(
            id=SPHERE_SUPPORT_COLLISION_ID,
            size=(
                SPHERE_SUPPORT_SIZE_XY[0],
                SPHERE_SUPPORT_SIZE_XY[1],
                SPHERE_SUPPORT_HEIGHT,
            ),
            position=[
                float(SPHERE_CENTER[0]),
                float(SPHERE_CENTER[1]),
                float(SPHERE_SUPPORT_CENTER_Z),
            ],
            quat_xyzw=[
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            frame_id=BASE_FRAME,
        )

        time.sleep(1.0)

    logger.info(
        "Adding tool collision sphere..."
    )

    moveit2.add_collision_sphere(
        id=TOOL_COLLISION_SPHERE_ID,
        radius=float(
            TOOL_COLLISION_SPHERE_RADIUS
        ),
        position=list(
            TOOL_COLLISION_SPHERE_OFFSET
        ),
        quat_xyzw=[
            0.0,
            0.0,
            0.0,
            1.0,
        ],
        frame_id=TOOL_COLLISION_LINK,
    )

    time.sleep(0.5)

    logger.info(
        f"Attaching tool collision sphere to "
        f"{TOOL_COLLISION_LINK}..."
    )

    moveit2.attach_collision_object(
        id=TOOL_COLLISION_SPHERE_ID,
        link_name=TOOL_COLLISION_LINK,
        touch_links=TOOL_COLLISION_TOUCH_LINKS,
    )

    time.sleep(1.0)

    logger.info(
        "Planning-scene setup complete."
    )


def publish_scene_status(logger):

    logger.info(
        f"Floor: ID={FLOOR_COLLISION_ID}, "
        f"center_z={FLOOR_Z:.3f}, "
        f"top_z={FLOOR_TOP_Z:.3f}, "
        f"size={FLOOR_BOX_SIZE}"
    )

    logger.info(
        f"Robot pedestal: ID={PEDESTAL_COLLISION_ID}, "
        f"height={PEDESTAL_HEIGHT:.3f}, "
        f"footprint={PEDESTAL_SIZE_XY}"
    )

    logger.info(
        f"Scan object: ID={SPHERE_COLLISION_ID}, "
        f"center={SPHERE_CENTER.tolist()}, "
        f"radius={SPHERE_RADIUS:.3f}"
    )

    logger.info(
        f"Scan object support: "
        f"bottom={SPHERE_SUPPORT_BOTTOM_Z:.3f}, "
        f"top={SPHERE_SUPPORT_TOP_Z:.3f}, "
        f"height={SPHERE_SUPPORT_HEIGHT:.3f}, "
        f"footprint={SPHERE_SUPPORT_SIZE_XY}"
    )

    logger.info(
        f"Tool collision: "
        f"ID={TOOL_COLLISION_SPHERE_ID}, "
        f"link={TOOL_COLLISION_LINK}, "
        f"radius={TOOL_COLLISION_SPHERE_RADIUS:.3f}"
    )


# ============================================================
# PROGRESS REPORTING
# ============================================================

def get_joint_positions(moveit2):
    """
    Current joint positions, reordered into JOINT_NAMES order.

    /joint_states is published in whatever order the driver chooses and may
    carry joints outside the planning group, so it must be indexed by name
    before it can be compared against a trajectory.
    """
    joint_state = moveit2.joint_state

    if joint_state is None:
        return None

    index_by_name = {
        name: index
        for index, name in enumerate(joint_state.name)
    }

    try:

        return np.array([
            float(
                joint_state.position[
                    index_by_name[name]
                ]
            )
            for name in JOINT_NAMES
        ])

    except (KeyError, IndexError):

        return None


def get_current_tool_pose(moveit2):
    """
    Current end-effector pose.

    pymoveit2's MoveIt2 has no get_current_pose(); forward kinematics is the
    supported way to ask for it.
    """
    result = moveit2.compute_fk()

    if result is None:
        return None

    if isinstance(result, (list, tuple)):

        if not result:
            return None

        result = result[0]

    return getattr(result, "pose", result)


def format_duration(seconds):

    seconds = max(
        0.0,
        float(seconds),
    )

    if seconds < 60.0:
        return f"{seconds:5.1f}s"

    minutes = int(seconds // 60)
    remainder = int(seconds % 60)

    return f"{minutes:d}m {remainder:02d}s"


class ExecutionProgressReporter:

    def __init__(
        self,
        moveit2,
        logger,
        waypoint_index,
        total_waypoints,
        start_joints,
        target_joints,
        stage_name,
    ):

        self.moveit2 = moveit2
        self.logger = logger

        self.waypoint_index = waypoint_index
        self.total_waypoints = total_waypoints

        self.start_joints = (
            None
            if start_joints is None
            else np.asarray(
                start_joints,
                dtype=float,
            ).copy()
        )

        self.target_joints = (
            None
            if target_joints is None
            else np.asarray(
                target_joints,
                dtype=float,
            ).copy()
        )

        self.stage_name = stage_name

        self._stop_event = threading.Event()
        self._thread = None
        self._started_at = None

        self._last_feedback = None
        self._last_motion_time = None
        self._printed_live_line = False

    def start(self):

        self._started_at = time.time()
        self._last_motion_time = self._started_at

        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
        )

        self._thread.start()

    def stop(self):

        self._stop_event.set()

        if self._thread is not None:

            self._thread.join(
                timeout=PROGRESS_UPDATE_PERIOD_SEC + 1.0
            )

        if (
            self._printed_live_line
            and not PROGRESS_NEWLINE_MODE
        ):
            print(
                file=sys.stdout,
                flush=True,
            )

    def _run(self):

        while not self._stop_event.is_set():

            self._print_progress()

            self._stop_event.wait(
                PROGRESS_UPDATE_PERIOD_SEC
            )

    def _print_progress(self):

        now = time.time()

        elapsed = (
            now - self._started_at
        )

        current_joints = get_joint_positions(
            self.moveit2
        )

        max_joint_delta = None
        estimated_percent = None

        if (
            current_joints is not None
            and self.start_joints is not None
            and current_joints.shape
            == self.start_joints.shape
        ):

            if (
                self._last_feedback is None
                or self._last_feedback.shape
                != current_joints.shape
                or np.max(
                    np.abs(
                        current_joints
                        - self._last_feedback
                    )
                ) > 0.001
            ):

                self._last_motion_time = now

            self._last_feedback = (
                current_joints.copy()
            )

            max_joint_delta = float(
                np.max(
                    np.abs(
                        current_joints
                        - self.start_joints
                    )
                )
            )

            if (
                self.target_joints is not None
                and self.target_joints.shape
                == current_joints.shape
            ):

                total_joint_distance = np.linalg.norm(
                    self.target_joints
                    - self.start_joints
                )

                current_joint_distance = np.linalg.norm(
                    current_joints
                    - self.start_joints
                )

                if total_joint_distance > 1e-6:

                    estimated_percent = float(
                        np.clip(
                            100.0
                            * current_joint_distance
                            / total_joint_distance,
                            0.0,
                            100.0,
                        )
                    )

        no_motion_seconds = (
            now - self._last_motion_time
        )

        if current_joints is None:

            movement_text = (
                "waiting for joint-state feedback"
            )

        elif estimated_percent is not None:

            movement_text = (
                f"estimated {estimated_percent:5.1f}% | "
                f"joint delta {max_joint_delta:.3f} rad"
            )

        elif (
            no_motion_seconds
            >= PROGRESS_STALL_WARNING_SEC
        ):

            movement_text = (
                f"no detected joint change for "
                f"{no_motion_seconds:.1f}s"
            )

        elif max_joint_delta is not None:

            movement_text = (
                f"moving | joint delta from start "
                f"{max_joint_delta:.3f} rad"
            )

        else:

            movement_text = (
                "waiting for valid joint feedback"
            )

        line = (
            f"[SCAN] Waypoint "
            f"{self.waypoint_index}/"
            f"{self.total_waypoints} | "
            f"{self.stage_name} | "
            f"elapsed {format_duration(elapsed)} | "
            f"{movement_text}"
        )

        if PROGRESS_NEWLINE_MODE:

            self.logger.info(line)

        else:

            print(
                "\r"
                + line.ljust(150),
                end="",
                file=sys.stdout,
                flush=True,
            )

            self._printed_live_line = True


# ============================================================
# MOTION
# ============================================================

def joint_motion_occurred(
    moveit2,
    positions_before,
    minimum_change=MIN_JOINT_MOTION_RAD,
):

    if positions_before is None:
        return True

    positions_after = get_joint_positions(
        moveit2
    )

    if positions_after is None:
        return True

    positions_before = np.asarray(
        positions_before,
        dtype=float,
    )

    if (
        positions_before.shape
        != positions_after.shape
    ):
        return True

    return bool(
        np.max(
            np.abs(
                positions_after
                - positions_before
            )
        )
        >= minimum_change
    )


def execute_joint_space_pose(
    moveit2,
    logger,
    position,
    orientation,
    waypoint_index,
    total_waypoints,
):

    positions_before = get_joint_positions(
        moveit2
    )

    moveit2.move_to_pose(
        position=[
            float(value)
            for value in position
        ],
        quat_xyzw=[
            float(value)
            for value in orientation
        ],
        frame_id=BASE_FRAME,
        cartesian=False,
    )

    reporter = ExecutionProgressReporter(
        moveit2=moveit2,
        logger=logger,
        waypoint_index=waypoint_index,
        total_waypoints=total_waypoints,
        start_joints=positions_before,
        target_joints=None,
        stage_name="joint-space execution",
    )

    reporter.start()

    try:

        success = (
            moveit2.wait_until_executed()
        )

    finally:

        reporter.stop()

    if not success:

        logger.warn(
            "Joint-space pose planning/execution failed."
        )

        return False

    if not joint_motion_occurred(
        moveit2,
        positions_before,
    ):

        logger.warn(
            "MoveIt reported success but "
            "the joints barely moved."
        )

        return False

    return True


def extract_final_joint_positions(plan):

    if plan is None:
        return None

    trajectory = plan

    if hasattr(
        plan,
        "joint_trajectory",
    ):

        trajectory = plan.joint_trajectory

    if not hasattr(
        trajectory,
        "points",
    ):

        return None

    if len(trajectory.points) == 0:
        return None

    final_positions = (
        trajectory.points[-1].positions
    )

    if len(final_positions) == 0:
        return None

    # Reorder to JOINT_NAMES so this can be compared against
    # get_joint_positions().
    joint_names = list(
        getattr(trajectory, "joint_names", [])
    )

    if joint_names:

        index_by_name = {
            name: index
            for index, name in enumerate(joint_names)
        }

        try:

            return np.array([
                float(
                    final_positions[
                        index_by_name[name]
                    ]
                )
                for name in JOINT_NAMES
            ])

        except (KeyError, IndexError):

            return None

    return np.asarray(
        final_positions,
        dtype=float,
    )


def execute_cartesian_fallback(
    moveit2,
    logger,
    position,
    orientation,
    waypoint_index,
    total_waypoints,
):

    try:

        positions_before = (
            get_joint_positions(moveit2)
        )

        # pymoveit2 has no plan_cartesian_path(). A straight-line plan is
        # requested through plan(cartesian=True), which returns a
        # JointTrajectory (or None). The fraction threshold is applied inside
        # pymoveit2 rather than being returned to the caller.
        if hasattr(
            moveit2,
            "cartesian_fraction_threshold",
        ):

            moveit2.cartesian_fraction_threshold = (
                CARTESIAN_FRACTION_THRESHOLD
            )

        plan_kwargs = {
            "position": [
                float(value)
                for value in position
            ],
            "quat_xyzw": [
                float(value)
                for value in orientation
            ],
            "frame_id": BASE_FRAME,
            "cartesian": True,
        }

        # The step-size keyword was renamed between pymoveit2 releases.
        accepted = inspect.signature(
            moveit2.plan
        ).parameters

        for name in (
            "cartesian_max_step",
            "max_step",
        ):

            if name in accepted:

                plan_kwargs[name] = CARTESIAN_MAX_STEP

                break

        plan = moveit2.plan(**plan_kwargs)

        if plan is None:

            logger.warn(
                "Cartesian planner returned no "
                "trajectory (path fraction below "
                f"{CARTESIAN_FRACTION_THRESHOLD:.3f} "
                "or IK failure)."
            )

            return False

        target_joints = (
            extract_final_joint_positions(
                plan
            )
        )

        moveit2.execute(plan)

        reporter = ExecutionProgressReporter(
            moveit2=moveit2,
            logger=logger,
            waypoint_index=waypoint_index,
            total_waypoints=total_waypoints,
            start_joints=positions_before,
            target_joints=target_joints,
            stage_name="Cartesian fallback execution",
        )

        reporter.start()

        try:

            success = (
                moveit2.wait_until_executed()
            )

        finally:

            reporter.stop()

        if not success:

            logger.warn(
                "Cartesian fallback execution failed."
            )

            return False

        if not joint_motion_occurred(
            moveit2,
            positions_before,
        ):

            logger.warn(
                "Cartesian fallback reported success "
                "but joints barely moved."
            )

            return False

        return True

    except Exception as error:

        logger.warn(
            f"Cartesian fallback unavailable/failed: "
            f"{error}"
        )

        return False


def move_to_pose_and_wait(
    moveit2,
    logger,
    position,
    orientation,
    waypoint_index,
    total_waypoints,
):

    started_at = time.time()

    logger.info(
        f"[SCAN] Waypoint "
        f"{waypoint_index}/{total_waypoints}: "
        f"trying joint-space pose planning."
    )

    if execute_joint_space_pose(
        moveit2=moveit2,
        logger=logger,
        position=position,
        orientation=orientation,
        waypoint_index=waypoint_index,
        total_waypoints=total_waypoints,
    ):

        elapsed = (
            time.time() - started_at
        )

        logger.info(
            f"[SCAN] Waypoint "
            f"{waypoint_index}/{total_waypoints}: "
            f"joint-space execution finished "
            f"in {elapsed:.1f}s."
        )

        return True

    if not USE_CARTESIAN_FALLBACK:
        return False

    logger.info(
        f"[SCAN] Waypoint "
        f"{waypoint_index}/{total_waypoints}: "
        f"joint-space plan failed; "
        f"trying Cartesian fallback."
    )

    success = execute_cartesian_fallback(
        moveit2=moveit2,
        logger=logger,
        position=position,
        orientation=orientation,
        waypoint_index=waypoint_index,
        total_waypoints=total_waypoints,
    )

    elapsed = (
        time.time() - started_at
    )

    if success:

        logger.info(
            f"[SCAN] Waypoint "
            f"{waypoint_index}/{total_waypoints}: "
            f"Cartesian fallback finished "
            f"in {elapsed:.1f}s."
        )

    else:

        logger.warn(
            f"[SCAN] Waypoint "
            f"{waypoint_index}/{total_waypoints}: "
            f"failed after {elapsed:.1f}s."
        )

    return success


# ============================================================
# SCAN EXECUTION
# ============================================================

def run_scan(
    moveit2,
    viz_node,
    xyz,
    quaternions,
):

    logger = viz_node.get_logger()

    total_waypoints = len(xyz)

    if total_waypoints == 0:

        logger.error(
            "[SCAN] No scan waypoints were generated."
        )

        return

    succeeded = 0
    failed = 0

    scan_started_at = time.time()

    logger.info(
        f"[SCAN] Starting scan with "
        f"{total_waypoints} waypoints."
    )

    for index, (
        position,
        orientation,
    ) in enumerate(
        zip(xyz, quaternions),
        start=1,
    ):

        completion_before = (
            100.0
            * (index - 1)
            / total_waypoints
        )

        logger.info(
            "[SCAN] ----------------------------------------"
        )

        logger.info(
            f"[SCAN] Waypoint "
            f"{index}/{total_waypoints} "
            f"({completion_before:.1f}% complete)"
        )

        logger.info(
            f"[SCAN] Target position: "
            f"x={position[0]:.3f}, "
            f"y={position[1]:.3f}, "
            f"z={position[2]:.3f} m"
        )

        waypoint_started_at = time.time()

        success = move_to_pose_and_wait(
            moveit2=moveit2,
            logger=logger,
            position=position,
            orientation=orientation,
            waypoint_index=index,
            total_waypoints=total_waypoints,
        )

        waypoint_elapsed = (
            time.time()
            - waypoint_started_at
        )

        if success:

            succeeded += 1
            status = "SUCCESS"

        else:

            failed += 1
            status = "FAILED"

        completion_after = (
            100.0
            * index
            / total_waypoints
        )

        scan_elapsed = (
            time.time()
            - scan_started_at
        )

        logger.info(
            f"[SCAN] Waypoint "
            f"{index}/{total_waypoints}: "
            f"{status} in "
            f"{waypoint_elapsed:.1f}s."
        )

        logger.info(
            f"[SCAN] Overall progress: "
            f"{index}/{total_waypoints} "
            f"({completion_after:.1f}%) | "
            f"success={succeeded}, "
            f"failed={failed} | "
            f"elapsed="
            f"{format_duration(scan_elapsed)}."
        )

    total_elapsed = (
        time.time()
        - scan_started_at
    )

    logger.info(
        "[SCAN] ========================================"
    )

    logger.info(
        f"[SCAN] Sphere scan complete: "
        f"{succeeded}/{total_waypoints} "
        f"successful, "
        f"{failed}/{total_waypoints} failed."
    )

    logger.info(
        f"[SCAN] Total scan duration: "
        f"{format_duration(total_elapsed)}."
    )


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    viz_node = SphereScanVizNode()

    callback_group = (
        ReentrantCallbackGroup()
    )

    moveit2 = MoveIt2(
        node=viz_node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_FRAME,
        end_effector_name=END_EFFECTOR_LINK,
        group_name=PLANNING_GROUP,
        callback_group=callback_group,
    )

    moveit2.max_velocity = (
        MAX_VELOCITY_SCALING
    )

    moveit2.max_acceleration = (
        MAX_ACCELERATION_SCALING
    )

    executor = MultiThreadedExecutor(
        num_threads=2
    )

    executor.add_node(viz_node)

    def spin_executor():
        # Without this the thread can die on an rclpy error while the scan loop
        # keeps running against a node that no longer processes callbacks.
        try:

            executor.spin()

        except Exception as error:

            if rclpy.ok():

                viz_node.get_logger().error(
                    f"[SCAN] Executor thread stopped: "
                    f"{error}"
                )

    executor_thread = threading.Thread(
        target=spin_executor,
        daemon=True,
    )

    executor_thread.start()

    try:

        time.sleep(1.5)

        # ----------------------------------------------------
        # Planning scene
        # ----------------------------------------------------

        add_planning_scene_objects(
            moveit2,
            viz_node.get_logger(),
        )

        publish_scene_status(
            viz_node.get_logger()
        )

        # ----------------------------------------------------
        # Generate scan
        # ----------------------------------------------------

        (
            sphere_center,
            sphere_radius,
            xyz,
            quaternions,
        ) = generate_scan(
            center=SPHERE_CENTER,
            sphere_radius=SPHERE_RADIUS,
            scan_radius=SCAN_RADIUS,
            num_points=NUM_POINTS,
            pattern=SCAN_PATTERN,
            elevations_deg=ELEVATIONS_DEG,
            pole_exclusion_deg=POLE_EXCLUSION_DEG,
        )

        # ----------------------------------------------------
        # Nearest-neighbor ordering
        # ----------------------------------------------------

        if (
            USE_NEAREST_NEIGHBOR_ORDERING
            and len(xyz) > 1
        ):

            try:

                current_pose = (
                    get_current_tool_pose(moveit2)
                )

                if current_pose is None:

                    raise RuntimeError(
                        "forward kinematics returned "
                        "no pose"
                    )

                start_position = np.array([
                    current_pose.position.x,
                    current_pose.position.y,
                    current_pose.position.z,
                ])

                (
                    xyz,
                    quaternions,
                    order,
                ) = nearest_neighbor_order(
                    xyz=xyz,
                    quaternions=quaternions,
                    start_position=start_position,
                )

                viz_node.get_logger().info(
                    "Nearest-neighbor waypoint "
                    "ordering enabled."
                )

                viz_node.get_logger().info(
                    f"Waypoint order: "
                    f"{order.tolist()}"
                )

            except Exception as error:

                viz_node.get_logger().warn(
                    "Could not obtain current tool pose "
                    "for nearest-neighbor ordering. "
                    f"Using original order. Error: {error}"
                )

        # ----------------------------------------------------
        # RViz visualization
        # ----------------------------------------------------

        viz_node.publish_visualization(
            sphere_center=sphere_center,
            sphere_radius=sphere_radius,
            xyz=xyz,
            quaternions=quaternions,
        )

        time.sleep(0.5)

        # ----------------------------------------------------
        # Terminal information
        # ----------------------------------------------------

        print()

        print("==============================================")
        print("       MOVEIT2 SPHERE SCAN (pymoveit2)")
        print("==============================================")

        print(
            f"Object center:      "
            f"{tuple(np.round(sphere_center, 2))}"
        )

        print(
            f"Object radius:      "
            f"{sphere_radius:.2f} m"
        )

        print(
            f"Scan radius:        "
            f"{SCAN_RADIUS:.2f} m"
        )

        print(
            f"Waypoints:          "
            f"{len(xyz)}"
        )

        print(
            f"Pattern:            "
            f"{SCAN_PATTERN}"
        )

        print(
            f"Floor top:          "
            f"{FLOOR_TOP_Z:.2f} m"
        )

        print(
            f"Robot pedestal:     "
            f"{PEDESTAL_HEIGHT:.2f} m"
        )

        print(
            f"Sphere support:     "
            f"{SPHERE_SUPPORT_HEIGHT:.2f} m"
        )

        print(
            f"Support bottom:     "
            f"{SPHERE_SUPPORT_BOTTOM_Z:.2f} m"
        )

        print(
            f"Support top:        "
            f"{SPHERE_SUPPORT_TOP_Z:.2f} m"
        )

        print(
            f"Nearest-neighbor:   "
            f"{USE_NEAREST_NEIGHBOR_ORDERING}"
        )

        print(
            f"Cartesian fallback: "
            f"{USE_CARTESIAN_FALLBACK}"
        )

        print(
            f"Velocity scale:     "
            f"{MAX_VELOCITY_SCALING:.2f}"
        )

        print(
            f"Accel scale:        "
            f"{MAX_ACCELERATION_SCALING:.2f}"
        )

        print(
            f"Waypoint points:    "
            f"{SHOW_WAYPOINT_POINTS}"
        )

        print(
            f"Waypoint axes:      "
            f"{SHOW_WAYPOINT_AXES}"
        )

        print(
            "Collision scene:    "
            "floor + robot pedestal + scan sphere + "
            "floor-to-sphere support + attached tool sphere"
        )

        print("==============================================")
        print()

        # ----------------------------------------------------
        # Execute
        # ----------------------------------------------------

        run_scan(
            moveit2=moveit2,
            viz_node=viz_node,
            xyz=xyz,
            quaternions=quaternions,
        )

    except KeyboardInterrupt:

        viz_node.get_logger().info(
            "[SCAN] Scan interrupted by user."
        )

    except Exception as error:

        viz_node.get_logger().error(
            f"[SCAN] Scan failed: {error}"
        )

    finally:

        executor.shutdown()

        # Let the spin thread finish before the node it is spinning goes away.
        if executor_thread.is_alive():

            executor_thread.join(
                timeout=2.0
            )

        viz_node.destroy_node()

        # On Ctrl-C rclpy's own signal handler has already shut the context
        # down; calling shutdown() again raises RCLError and exits non-zero.
        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":
    main()