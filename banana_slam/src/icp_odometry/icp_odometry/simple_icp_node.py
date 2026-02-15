#!/usr/bin/env python3
"""
SIMPLIFIED ICP Node - Pure Scan-to-Scan matching like pZ
NO features, NO global map, NO pose graph - JUST scan matching!
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from collections import deque
from math import sin, cos, atan2, pi, sqrt
from scipy.spatial import KDTree 

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

def normalize_angle(angle):
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle

def quaternion_from_euler(roll, pitch, yaw):
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    q = Quaternion()
    q.w = cy
    q.x = 0.0
    q.y = 0.0
    q.z = sy
    return q

def euler_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return 0.0, 0.0, yaw

def scan_to_points(scan: LaserScan, min_range: float, max_range: float) -> np.ndarray:
    """Convert LaserScan to 2D points"""
    angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)
    ranges = np.array(scan.ranges)

    min_len = min(len(angles), len(ranges))
    angles = angles[:min_len]
    ranges = ranges[:min_len]

    valid_mask = (ranges >= min_range) & (ranges <= max_range) & np.isfinite(ranges)
    valid_ranges = ranges[valid_mask]
    valid_angles = angles[valid_mask]

    x = valid_ranges * np.cos(valid_angles)
    y = valid_ranges * np.sin(valid_angles)

    return np.column_stack((x, y))

def transform_points(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """Transform points by (x, y, theta)"""
    if len(points) == 0:
        return points

    c, s = cos(theta), sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = np.array([x, y])

    return points @ R.T + t

def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """
    Voxel downsampling (like pZ)
    Group points into voxels and take mean
    """
    if len(points) == 0:
        return points


    voxel_indices = np.floor(points / voxel_size).astype(np.int32)

    # Use dict to group points by voxel
    voxel_dict = {}
    for i, voxel_idx in enumerate(voxel_indices):
        key = tuple(voxel_idx)
        if key not in voxel_dict:
            voxel_dict[key] = []
        voxel_dict[key].append(points[i])


    downsampled = np.array([np.mean(pts, axis=0) for pts in voxel_dict.values()])
    return downsampled

def simple_icp_2d(source: np.ndarray, target: np.ndarray,
                  init_x=0.0, init_y=0.0, init_theta=0.0,
                  max_iterations=50, tolerance=1e-6, max_dist=0.5):
    """
    Point-to-Point ICP with KDTree and outlier rejection (EXACTLY like pZ!)
    Returns: (dx, dy, dtheta, converged, quality_dict)
    """
    if len(source) < 30 or len(target) < 30:  # Reference uses 30 min correspondences
        return 0.0, 0.0, 0.0, False, {'mean_error': 999.0, 'correspondence_ratio': 0.0}

    # Apply initial guess
    transformed = transform_points(source, init_x, init_y, init_theta)

    # Build KDTree for target
    tree = KDTree(target)

    prev_error = float('inf')

    for iteration in range(max_iterations):

        nearest_distances, nearest_indices = tree.query(transformed, k=1)


        valid_mask = nearest_distances < max_dist

        if np.sum(valid_mask) < 30:  # Reference uses 30 min correspondences
            return init_x, init_y, init_theta, False, {'mean_error': 999.0, 'correspondence_ratio': 0.0}

        # 80th percentile outlier rejection
        valid_distances = nearest_distances[valid_mask]
        if len(valid_distances) > 10:
            percentile_80 = np.percentile(valid_distances, 80)
            outlier_mask = valid_mask.copy()
            outlier_mask[valid_mask] &= (nearest_distances[valid_mask] <= percentile_80)
            if np.sum(outlier_mask) >= 10:
                valid_mask = outlier_mask


        src_matched = transformed[valid_mask]
        tgt_matched = target[nearest_indices[valid_mask]]


        src_centroid = np.mean(src_matched, axis=0)
        tgt_centroid = np.mean(tgt_matched, axis=0)

        # Center the points
        src_centered = src_matched - src_centroid
        tgt_centered = tgt_matched - tgt_centroid


        H = src_centered.T @ tgt_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # Ensure proper rotation (det = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T


        dtheta = atan2(R[1, 0], R[0, 0])


        t = tgt_centroid - R @ src_centroid
        delta_x, delta_y = t[0], t[1]

        # ACCUMULATE transformation
        init_x += delta_x
        init_y += delta_y
        init_theta += dtheta


        transformed = transform_points(source, init_x, init_y, init_theta)


        mean_error = np.mean(nearest_distances[valid_mask])

        if abs(prev_error - mean_error) < tolerance:
            quality = {
                'mean_error': mean_error,
                'correspondence_ratio': np.sum(valid_mask) / len(source)
            }
            return init_x, init_y, init_theta, True, quality

        prev_error = mean_error

    # Did not converge
    quality = {
        'mean_error': prev_error,
        'correspondence_ratio': np.sum(valid_mask) / len(source)
    }
    return init_x, init_y, init_theta, False, quality


class SimpleICPNode(Node):
    """
    Simple ICP Odometry - Pure scan-to-scan matching (like pZ)
    """

    def __init__(self):
        super().__init__('simple_icp_node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_range', 0.12),
                ('max_range', 3.5),
                ('max_correspondence_distance', 0.3),
                ('max_iterations', 50),
                ('tolerance', 1e-6),
                ('keyframe_distance', 0.05),  # More frequent: 5cm (was 0.1m)
                ('keyframe_angle', 0.01745),  # More frequent: 1° (was 5°)
                ('max_local_scans', 15),  # Match reference: 15 keyframes for faster KD-tree builds
                ('voxel_size', 0.03),
            ]
        )

        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.max_corr_dist = self.get_parameter('max_correspondence_distance').value
        self.max_iter = self.get_parameter('max_iterations').value
        self.tolerance = self.get_parameter('tolerance').value
        self.keyframe_dist = self.get_parameter('keyframe_distance').value
        self.keyframe_angle = self.get_parameter('keyframe_angle').value
        self.max_local_scans = self.get_parameter('max_local_scans').value
        self.voxel_size = self.get_parameter('voxel_size').value

        # State
        self.icp_pose = np.zeros(3)  # [x, y, theta]
        self.prev_scan = None
        self.prev_ekf_pose = None
        self.latest_ekf_pose = None

        # Keyframe management
        self.keyframe_scans = deque(maxlen=self.max_local_scans)
        self.keyframe_poses = deque(maxlen=self.max_local_scans)  # Store poses too!
        self.last_keyframe_pose = np.zeros(3)
        self.is_first_scan = True

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odometry/icp', 10)
        self.quality_pub = self.create_publisher(Float32, '/icp/quality', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.scan_callback,
            SENSOR_QOS
        )

        self.ekf_sub = self.create_subscription(
            Odometry,
            '/odometry/wheel',
            self.ekf_callback,
            10
        )

        self.get_logger().info('Simple ICP Node initialized')
        self.get_logger().info(f'  Range: [{self.min_range}, {self.max_range}]m')
        self.get_logger().info(f'  Max correspondence: {self.max_corr_dist}m')
        self.get_logger().info(f'  Keyframe threshold: {self.keyframe_dist}m')
        self.get_logger().info(f'  Local map size: {self.max_local_scans} scans')

    def ekf_callback(self, msg: Odometry):
        """Store latest EKF pose for initial guess"""
        _, _, yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.latest_ekf_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ])

    def scan_callback(self, msg: LaserScan):
        """Process scan with simple ICP matching"""
        current_time = self.get_clock().now()

        # Convert scan to points
        current_scan = scan_to_points(msg, self.min_range, self.max_range)


        current_scan = voxel_downsample(current_scan, self.voxel_size)

        if len(current_scan) < 10:
            self.get_logger().warn('Too few points', throttle_duration_sec=5.0)
            return

        # First scan - initialize
        if self.is_first_scan:
            self.prev_scan = current_scan
            self.prev_ekf_pose = self.latest_ekf_pose.copy() if self.latest_ekf_pose is not None else np.zeros(3)
            self.keyframe_scans.append(current_scan.copy())
            self.keyframe_poses.append(self.icp_pose.copy())  # Store pose too!
            self.is_first_scan = False
            self.publish_odometry(current_time)
            self.get_logger().info(f'Initialized with {len(current_scan)} points')
            return

        # Integrate EKF motion prediction FIRST!
        if self.latest_ekf_pose is not None and self.prev_ekf_pose is not None:
            self.icp_pose[0] += self.latest_ekf_pose[0] - self.prev_ekf_pose[0]
            self.icp_pose[1] += self.latest_ekf_pose[1] - self.prev_ekf_pose[1]
            self.icp_pose[2] = normalize_angle(self.icp_pose[2] + (self.latest_ekf_pose[2] - self.prev_ekf_pose[2]))


        dist_from_last_kf = sqrt((self.icp_pose[0] - self.last_keyframe_pose[0])**2 +
                                  (self.icp_pose[1] - self.last_keyframe_pose[1])**2)
        angle_from_last_kf = abs(normalize_angle(self.icp_pose[2] - self.last_keyframe_pose[2]))
        should_run_icp = (dist_from_last_kf >= self.keyframe_dist or
                         angle_from_last_kf >= self.keyframe_angle)

        # Only run ICP at keyframes to refine the EKF prediction
        if should_run_icp and len(self.keyframe_scans) > 0:

            local_map_points = []
            for scan, pose in zip(self.keyframe_scans, self.keyframe_poses):

                transformed_scan = transform_points(scan, pose[0], pose[1], pose[2])
                local_map_points.append(transformed_scan)

            local_map = np.vstack(local_map_points)

            local_map = voxel_downsample(local_map, self.voxel_size)


            refined_x, refined_y, refined_theta, converged, quality = simple_icp_2d(
                current_scan, local_map,
                self.icp_pose[0], self.icp_pose[1], self.icp_pose[2],
                self.max_iter, self.tolerance, self.max_corr_dist
            )


            dx_refine = refined_x - self.icp_pose[0]
            dy_refine = refined_y - self.icp_pose[1]
            dtheta_refine = normalize_angle(refined_theta - self.icp_pose[2])
            refinement_dist = sqrt(dx_refine**2 + dy_refine**2)


            # Quality based on: convergence, correspondence ratio, and error
            corr_ratio = quality['correspondence_ratio']
            mean_err = quality['mean_error']


            if converged and mean_err < 0.2:  # Reasonable ICP
                # Quality increases with higher correspondence and lower error
                quality_score = corr_ratio * 100.0 * (1.0 - min(mean_err / 0.2, 1.0))
            else:
                quality_score = 0.0

            # Adaptive weighting: trust ICP more when quality is high
            # quality_score: 0-100%
            # icp_weight: 0.0 (trust EKF only) to 1.0 (trust ICP fully)
            if quality_score > 80:
                icp_weight = 0.9  # High confidence in ICP
            elif quality_score > 60:
                icp_weight = 0.7  # Good confidence
            elif quality_score > 40:
                icp_weight = 0.5  # Medium confidence
            elif quality_score > 20:
                icp_weight = 0.3  # Low confidence
            else:
                icp_weight = 0.1  # Very low confidence, mostly trust EKF

            ekf_weight = 1.0 - icp_weight

            if converged and refinement_dist < 0.3 and abs(dtheta_refine) < 0.0873:  # 5° like reference (was 0.15 rad = 8.6°)

                blended_x = ekf_weight * self.icp_pose[0] + icp_weight * refined_x
                blended_y = ekf_weight * self.icp_pose[1] + icp_weight * refined_y
                blended_theta = normalize_angle(ekf_weight * self.icp_pose[2] + icp_weight * refined_theta)

                self.icp_pose[0] = blended_x
                self.icp_pose[1] = blended_y
                self.icp_pose[2] = blended_theta

                # Add keyframe with refined pose
                self.keyframe_scans.append(current_scan.copy())
                self.keyframe_poses.append(self.icp_pose.copy())
                self.last_keyframe_pose = self.icp_pose.copy()

                self.get_logger().info(
                    f'[KEYFRAME] ICP refined (quality={quality_score:.1f}%, icp_weight={icp_weight:.2f}, '
                    f'corr={corr_ratio:.2f}, err={mean_err:.3f}m) → {len(self.keyframe_scans)} scans',
                    throttle_duration_sec=1.0
                )

                # Publish quality
                quality_msg = Float32()
                quality_msg.data = quality_score
                self.quality_pub.publish(quality_msg)
            else:

                self.get_logger().warn(
                    f'[REJECTED] ICP rejected (converged={converged}, refinement={refinement_dist:.3f}m, '
                    f'dtheta={abs(dtheta_refine):.3f}rad) - using EKF only',
                    throttle_duration_sec=2.0
                )
                quality_msg = Float32()
                quality_msg.data = 0.0
                self.quality_pub.publish(quality_msg)
        else:
            # No ICP this frame (not at keyframe yet)
            quality_msg = Float32()
            quality_msg.data = 0.0
            self.quality_pub.publish(quality_msg)


        self.prev_scan = current_scan
        self.prev_ekf_pose = self.latest_ekf_pose.copy() if self.latest_ekf_pose is not None else self.prev_ekf_pose

        # Publish
        self.publish_odometry(current_time)

    def publish_odometry(self, timestamp):
        """Publish ICP odometry"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.icp_pose[0]
        odom_msg.pose.pose.position.y = self.icp_pose[1]
        odom_msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, self.icp_pose[2])

        self.odom_pub.publish(odom_msg)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link_icp'
        t.transform.translation.x = self.icp_pose[0]
        t.transform.translation.y = self.icp_pose[1]
        t.transform.rotation = quaternion_from_euler(0.0, 0.0, self.icp_pose[2])
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleICPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
