#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under MIT

"""
ICP Odometry Node for FRA532 Lab 1.

This node refines EKF odometry using LiDAR scan matching (ICP).

Data Flow (from diagram):
- Lidar_RAW -> Pointcloud Jitter Compensation -> Motion Compensation -> DeDistort
  -> Range Filter (Cutoff < 2m) -> Lidar_Filter -> Feature Selection (Edge/Planar)
  -> Lidar Odometry

For 2D LaserScan, we simplify:
- /scan -> Range Filter -> ICP Scan Matching -> Lidar Odometry

The EKF odometry is used as the initial guess for ICP alignment.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from math import sin, cos, atan2, pi, sqrt
from dataclasses import dataclass
from typing import Dict, List, Tuple

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32
import struct
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

# QoS profile for sensor data (matches rosbag default)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


def euler_from_quaternion(q):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return 0.0, 0.0, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """Convert euler angles to quaternion."""
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    q = Quaternion()
    q.w = cy
    q.x = 0.0
    q.y = 0.0
    q.z = sy
    return q


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


def scan_to_points(scan: LaserScan, min_range: float, max_range: float) -> np.ndarray:
    """
    Convert LaserScan to 2D points with range filtering.
    
    Returns Nx2 numpy array of (x, y) points in laser frame.
    """
    angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)
    ranges = np.array(scan.ranges)
    
    # Ensure same length
    min_len = min(len(angles), len(ranges))
    angles = angles[:min_len]
    ranges = ranges[:min_len]
    
    # Filter by range
    valid_mask = (ranges >= min_range) & (ranges <= max_range) & np.isfinite(ranges)
    valid_ranges = ranges[valid_mask]
    valid_angles = angles[valid_mask]
    
    # Convert to Cartesian
    x = valid_ranges * np.cos(valid_angles)
    y = valid_ranges * np.sin(valid_angles)
    
    return np.column_stack((x, y))


def transform_points(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """Transform points by translation (x, y) and rotation theta."""
    c, s = cos(theta), sin(theta)
    R = np.array([[c, -s], [s, c]])
    return (R @ points.T).T + np.array([x, y])


def nearest_neighbor(src: np.ndarray, dst: np.ndarray):
    """
    Find nearest neighbor for each source point in destination using brute force.
    Returns indices and distances.
    """
    # src: (N, 2), dst: (M, 2)
    # Distances matrix: (N, M)
    # broadcasting: (N, 1, 2) - (1, M, 2) -> (N, M, 2)
    # norm -> (N, M)
    
    diff = src[:, np.newaxis, :] - dst[np.newaxis, :, :]
    dist_sq = np.sum(diff**2, axis=2)
    
    indices = np.argmin(dist_sq, axis=1)
    distances = np.sqrt(np.min(dist_sq, axis=1))
    
    return indices, distances

def icp_2d(source: np.ndarray, target: np.ndarray,
           init_x: float = 0.0, init_y: float = 0.0, init_theta: float = 0.0,
           max_iterations: int = 20, tolerance: float = 1e-4,
           max_correspondence_dist: float = 0.5):
    """
    2D Iterative Closest Point algorithm with outlier rejection.
    Returns: (x, y, theta, converged, quality_metrics)
    quality_metrics = {'mean_error': float, 'num_correspondences': int, 'correspondence_ratio': float}
    """
    if source is None or target is None:
        return init_x, init_y, init_theta, False, {'mean_error': 999.0, 'num_correspondences': 0, 'correspondence_ratio': 0.0}

    if len(source) < 5 or len(target) < 5:
        return init_x, init_y, init_theta, False, {'mean_error': 999.0, 'num_correspondences': 0, 'correspondence_ratio': 0.0}

    # Apply initial transformation
    x, y, theta = init_x, init_y, init_theta
    src_transformed = transform_points(source, x, y, theta)

    prev_error = float('inf')
    final_mean_error = 999.0
    final_num_correspondences = 0

    for iteration in range(max_iterations):
        # Find correspondences (Brute Force)
        indices, distances = nearest_neighbor(src_transformed, target)

        # Filter by max correspondence distance
        valid_mask = distances < max_correspondence_dist
        num_valid = np.sum(valid_mask)
        if num_valid < 5:
            # Lost tracking
            quality = {'mean_error': 999.0, 'num_correspondences': num_valid, 'correspondence_ratio': num_valid / len(source)}
            return x, y, theta, False, quality

        # OUTLIER REJECTION: Statistical filtering (like slam_toolbox)
        valid_distances = distances[valid_mask]
        if len(valid_distances) > 10:
            # Remove points beyond 2 standard deviations (robust outlier rejection)
            mean_dist = np.mean(valid_distances)
            std_dist = np.std(valid_distances)
            outlier_threshold = mean_dist + 2.0 * std_dist

            # Apply outlier filter on top of distance filter
            outlier_mask = valid_mask.copy()
            outlier_mask[valid_mask] &= (distances[valid_mask] < outlier_threshold)

            if np.sum(outlier_mask) >= 5:
                valid_mask = outlier_mask

        src_valid = src_transformed[valid_mask]
        tgt_valid = target[indices[valid_mask]]

        # Compute mean error
        mean_error = np.mean(distances[valid_mask])
        num_correspondences = np.sum(valid_mask)

        # Store final quality metrics
        final_mean_error = mean_error
        final_num_correspondences = num_correspondences

        # Check convergence
        if abs(prev_error - mean_error) < tolerance:
            quality = {
                'mean_error': mean_error,
                'num_correspondences': num_correspondences,
                'correspondence_ratio': num_correspondences / len(source)
            }
            return x, y, theta, True, quality
        prev_error = mean_error
        
        # Compute centroids
        src_centroid = np.mean(src_valid, axis=0)
        tgt_centroid = np.mean(tgt_valid, axis=0)
        
        # Center the points
        src_centered = src_valid - src_centroid
        tgt_centered = tgt_valid - tgt_centroid
        
        # Compute optimal rotation using SVD
        H = src_centered.T @ tgt_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Ensure proper rotation (det = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Extract rotation angle
        delta_theta = atan2(R[1, 0], R[0, 0])
        
        # Compute translation
        t_vec = tgt_centroid - R @ src_centroid
        
        # Update transformation state (x, y, theta)
        # Global Update
        theta = normalize_angle(theta + delta_theta)
        
        c_d, s_d = cos(delta_theta), sin(delta_theta)
        R_delta_mat = np.array([[c_d, -s_d], [s_d, c_d]])
        
        pos_vec = np.array([x, y])
        new_pos = R_delta_mat @ pos_vec + t_vec
        x, y = new_pos[0], new_pos[1]
        
        # Re-transform source points for next iteration
        src_transformed = transform_points(source, x, y, theta)

    # Max iterations reached without convergence
    quality = {
        'mean_error': final_mean_error,
        'num_correspondences': final_num_correspondences,
        'correspondence_ratio': final_num_correspondences / len(source) if len(source) > 0 else 0.0
    }
    return x, y, theta, True, quality


# ============================================================================
# LOAM-Style Feature Extraction
# ============================================================================

@dataclass
class FeatureCloud:
    """
    Feature cloud for LOAM-style SLAM.

    Stores classified features with curvature information.
    """
    edge_points: np.ndarray      # Nx2 array of high-curvature points (corners)
    planar_points: np.ndarray    # Nx2 array of low-curvature points (walls)
    ground_points: np.ndarray    # Nx2 array of ground plane points
    all_points: np.ndarray       # Nx2 array of all points (for fallback)
    curvatures: np.ndarray       # N array of curvature values
    feature_types: np.ndarray    # N array of labels (0=other, 1=edge, 2=planar, 3=ground)


def compute_point_curvature(points: np.ndarray, index: int, search_radius: float = 0.3) -> float:
    """
    Compute 2D curvature at point using angle-based measure.

    Curvature = sum of absolute angle changes with neighbors within search_radius.
    High curvature = sharp corners/edges (large angle changes)
    Low curvature = flat surfaces (small angle changes)

    Args:
        points: Nx2 array of (x, y) points
        index: Index of point to compute curvature for
        search_radius: Radius for neighbor search (meters)

    Returns:
        Curvature value (radians)
    """
    if index < 0 or index >= len(points):
        return 0.0

    current_point = points[index]

    # Find neighbors within search_radius
    distances = np.linalg.norm(points - current_point, axis=1)
    neighbors_mask = (distances < search_radius) & (distances > 0.01)  # Exclude self and too-close
    neighbor_indices = np.where(neighbors_mask)[0]

    if len(neighbor_indices) < 2:
        return 0.0  # Not enough neighbors

    # Sort neighbors by distance
    neighbor_distances = distances[neighbor_indices]
    sorted_idx = np.argsort(neighbor_distances)
    neighbor_indices = neighbor_indices[sorted_idx]

    # Compute angle changes between consecutive neighbors
    angles = []
    for i in range(min(len(neighbor_indices) - 1, 10)):  # Use up to 10 neighbors
        p1 = points[neighbor_indices[i]]
        p2 = current_point
        p3 = points[neighbor_indices[i+1]]

        # Vectors
        v1 = p2 - p1
        v2 = p3 - p2

        # Norm
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)

        if norm_v1 < 1e-8 or norm_v2 < 1e-8:
            continue

        # Angle between vectors
        cos_angle = np.dot(v1, v2) / (norm_v1 * norm_v2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        angles.append(angle)

    # Curvature = mean absolute angle change
    return np.mean(np.abs(angles)) if angles else 0.0


def extract_features(points: np.ndarray,
                     edge_curvature_threshold: float = 0.1,
                     planar_curvature_threshold: float = 0.02,
                     ground_height_threshold: float = -0.15,
                     search_radius: float = 0.3) -> FeatureCloud:
    """
    Extract LOAM-style features from 2D point cloud.

    Features:
    - EDGE: High curvature (corners, discontinuities)
    - PLANAR: Low curvature (walls, flat surfaces)
    - GROUND: Points below height threshold (floor)
    - OTHER: Remaining points

    Args:
        points: Nx2 array of (x, y) points
        edge_curvature_threshold: Curvature above = EDGE (radians)
        planar_curvature_threshold: Curvature below = PLANAR (radians)
        ground_height_threshold: Y-coordinate below = GROUND (meters)
        search_radius: Radius for neighbor search (meters)

    Returns:
        FeatureCloud with classified features
    """
    if len(points) < 10:
        # Not enough points, return empty features
        return FeatureCloud(
            edge_points=np.array([]).reshape(0, 2),
            planar_points=np.array([]).reshape(0, 2),
            ground_points=np.array([]).reshape(0, 2),
            all_points=points,
            curvatures=np.zeros(len(points)),
            feature_types=np.zeros(len(points), dtype=int)
        )

    # Compute curvature for each point
    curvatures = np.array([compute_point_curvature(points, i, search_radius)
                           for i in range(len(points))])

    # Initialize feature types (0 = other, 1 = edge, 2 = planar, 3 = ground)
    feature_types = np.zeros(len(points), dtype=int)

    # Ground detection (height-based)
    # For 2D laser scan, y-coordinate can represent height if laser is tilted
    # Or we can use simplified ground detection for horizontal lasers
    ground_mask = points[:, 1] < ground_height_threshold
    feature_types[ground_mask] = 3

    # Edge detection (high curvature)
    # Only check non-ground points
    non_ground_mask = ~ground_mask
    edge_mask = non_ground_mask & (curvatures > edge_curvature_threshold)
    feature_types[edge_mask] = 1

    # Planar detection (low curvature)
    planar_mask = non_ground_mask & (curvatures < planar_curvature_threshold) & (~edge_mask)
    feature_types[planar_mask] = 2

    # Extract feature subsets
    edge_points = points[feature_types == 1]
    planar_points = points[feature_types == 2]
    ground_points = points[feature_types == 3]

    return FeatureCloud(
        edge_points=edge_points,
        planar_points=planar_points,
        ground_points=ground_points,
        all_points=points,
        curvatures=curvatures,
        feature_types=feature_types
    )


class GlobalFeatureMap:
    """
    Global feature map for LOAM-style SLAM.

    Stores features in world coordinates with voxel-based downsampling.
    Supports query by spatial region for efficient scan-to-map matching.
    """

    def __init__(self, voxel_size: float = 0.1,
                 max_features_per_voxel: int = 10,
                 feature_decay_distance: float = 5.0):
        """
        Args:
            voxel_size: Size of voxel grid for downsampling (meters)
            max_features_per_voxel: Maximum features to store per voxel
            feature_decay_distance: Remove features beyond this distance from robot
        """
        self.voxel_size = voxel_size
        self.max_features_per_voxel = max_features_per_voxel
        self.feature_decay_distance = feature_decay_distance

        # Feature storage: dict[voxel_key] -> list of {point, type, timestamp}
        # voxel_key = (voxel_x, voxel_y) tuple
        self.edge_voxels: Dict[Tuple[int, int], List[Dict]] = {}
        self.planar_voxels: Dict[Tuple[int, int], List[Dict]] = {}
        self.ground_voxels: Dict[Tuple[int, int], List[Dict]] = {}

        # Statistics
        self.total_edge_count = 0
        self.total_planar_count = 0
        self.total_ground_count = 0

    def world_to_voxel(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to voxel key."""
        voxel_x = int(np.floor(x / self.voxel_size))
        voxel_y = int(np.floor(y / self.voxel_size))
        return (voxel_x, voxel_y)

    def add_features(self, features: FeatureCloud, robot_pose: np.ndarray, timestamp: float):
        """
        Add features to global map in world coordinates.

        Args:
            features: FeatureCloud in robot frame
            robot_pose: [x, y, theta] robot pose in world frame
            timestamp: Time of observation
        """
        # Transform features to world frame
        edge_world = transform_points(features.edge_points,
                                       robot_pose[0], robot_pose[1], robot_pose[2])
        planar_world = transform_points(features.planar_points,
                                         robot_pose[0], robot_pose[1], robot_pose[2])
        ground_world = transform_points(features.ground_points,
                                         robot_pose[0], robot_pose[1], robot_pose[2])

        # Add to voxel grids
        self._add_to_voxels(self.edge_voxels, edge_world, 1, timestamp)
        self._add_to_voxels(self.planar_voxels, planar_world, 2, timestamp)
        self._add_to_voxels(self.ground_voxels, ground_world, 3, timestamp)

        # Update counts
        self.total_edge_count = sum(len(v) for v in self.edge_voxels.values())
        self.total_planar_count = sum(len(v) for v in self.planar_voxels.values())
        self.total_ground_count = sum(len(v) for v in self.ground_voxels.values())

    def _add_to_voxels(self, voxel_dict: Dict, points: np.ndarray,
                       feature_type: int, timestamp: float):
        """Add points to voxel dictionary with downsampling."""
        for point in points:
            voxel_key = self.world_to_voxel(point[0], point[1])

            if voxel_key not in voxel_dict:
                voxel_dict[voxel_key] = []

            # Add feature with metadata
            voxel_dict[voxel_key].append({
                'point': point.copy(),
                'type': feature_type,
                'timestamp': timestamp
            })

            # Downsample if too many features in this voxel
            if len(voxel_dict[voxel_key]) > self.max_features_per_voxel:
                # Keep most recent features
                voxel_dict[voxel_key] = sorted(
                    voxel_dict[voxel_key],
                    key=lambda x: x['timestamp'],
                    reverse=True
                )[:self.max_features_per_voxel]

    def query_region(self, robot_pose: np.ndarray, query_radius: float = 3.0) -> FeatureCloud:
        """
        Query features within radius of robot pose.

        Args:
            robot_pose: [x, y, theta] in world frame
            query_radius: Radius to query (meters)

        Returns:
            FeatureCloud in world coordinates
        """
        robot_x, robot_y = robot_pose[0], robot_pose[1]

        # Determine voxel range to search
        voxel_radius = int(np.ceil(query_radius / self.voxel_size))
        center_voxel = self.world_to_voxel(robot_x, robot_y)

        edges = []
        planars = []
        grounds = []

        # Search voxels in range
        for dx in range(-voxel_radius, voxel_radius + 1):
            for dy in range(-voxel_radius, voxel_radius + 1):
                voxel_key = (center_voxel[0] + dx, center_voxel[1] + dy)

                # Collect edge features
                if voxel_key in self.edge_voxels:
                    for feature in self.edge_voxels[voxel_key]:
                        point = feature['point']
                        dist = np.linalg.norm(point - np.array([robot_x, robot_y]))
                        if dist < query_radius:
                            edges.append(point)

                # Collect planar features
                if voxel_key in self.planar_voxels:
                    for feature in self.planar_voxels[voxel_key]:
                        point = feature['point']
                        dist = np.linalg.norm(point - np.array([robot_x, robot_y]))
                        if dist < query_radius:
                            planars.append(point)

                # Collect ground features
                if voxel_key in self.ground_voxels:
                    for feature in self.ground_voxels[voxel_key]:
                        point = feature['point']
                        dist = np.linalg.norm(point - np.array([robot_x, robot_y]))
                        if dist < query_radius:
                            grounds.append(point)

        edge_points = np.array(edges) if edges else np.array([]).reshape(0, 2)
        planar_points = np.array(planars) if planars else np.array([]).reshape(0, 2)
        ground_points = np.array(grounds) if grounds else np.array([]).reshape(0, 2)

        all_points = np.vstack([edge_points, planar_points, ground_points]) if len(edges) + len(planars) + len(grounds) > 0 else np.array([]).reshape(0, 2)

        return FeatureCloud(
            edge_points=edge_points,
            planar_points=planar_points,
            ground_points=ground_points,
            all_points=all_points,
            curvatures=np.zeros(len(all_points)),
            feature_types=np.concatenate([
                np.ones(len(edge_points), dtype=int),
                np.full(len(planar_points), 2, dtype=int),
                np.full(len(ground_points), 3, dtype=int)
            ]) if len(all_points) > 0 else np.array([], dtype=int)
        )

    def get_statistics(self) -> Dict:
        """Get map statistics for logging."""
        return {
            'edge_features': self.total_edge_count,
            'planar_features': self.total_planar_count,
            'ground_features': self.total_ground_count,
            'total_features': self.total_edge_count + self.total_planar_count + self.total_ground_count,
            'edge_voxels': len(self.edge_voxels),
            'planar_voxels': len(self.planar_voxels),
            'ground_voxels': len(self.ground_voxels),
        }


class PoseGraph:
    """Sliding-window pose graph for local trajectory optimization"""

    def __init__(self, window_size=30):
        self.poses = []  # List of [x, y, theta] poses
        self.edges = []  # List of (from_idx, to_idx, dx, dy, dtheta, info_matrix)
        self.window_size = window_size

    def add_pose(self, pose: np.ndarray, edge_from_prev: Tuple = None):
        """Add new pose and edge from previous pose"""
        # Slide window if full
        if len(self.poses) >= self.window_size:
            self.poses.pop(0)
            # Adjust edge indices after removing first pose
            self.edges = [(max(f-1, 0), max(t-1, 0), dx, dy, dtheta, info)
                         for f, t, dx, dy, dtheta, info in self.edges if t > 0]

        self.poses.append(pose.copy())

        # Add edge from previous pose
        if edge_from_prev is not None and len(self.poses) > 1:
            dx, dy, dtheta, covariance = edge_from_prev
            from_idx = len(self.poses) - 2
            to_idx = len(self.poses) - 1

            # Information matrix (inverse of covariance)
            try:
                # Ensure covariance is valid
                if not isinstance(covariance, np.ndarray) or covariance.shape != (3, 3):
                    covariance = np.diag([0.01, 0.01, 0.005])

                info_matrix = np.linalg.inv(covariance + np.eye(3) * 1e-6)
            except (np.linalg.LinAlgError, ValueError):
                # Fallback to default information matrix
                info_matrix = np.diag([100.0, 100.0, 200.0])  # Default: high confidence

            self.edges.append((from_idx, to_idx, dx, dy, dtheta, info_matrix))

    def optimize(self, iterations=10) -> np.ndarray:
        """Run Gauss-Newton optimization on pose graph"""
        if len(self.poses) < 3:
            return self.poses[-1] if self.poses else np.zeros(3)

        # Convert poses to numpy array for easier manipulation
        poses = np.array([p.copy() for p in self.poses])  # Shape: (N, 3)
        N = len(poses)

        # Anchor first pose (prior constraint)
        first_pose_fixed = poses[0].copy()

        # Gauss-Newton iteration
        for iteration in range(iterations):
            # Build Hessian (H) and gradient (b) for normal equations: H * delta = -b
            H = np.zeros((N * 3, N * 3))
            b = np.zeros(N * 3)

            # Add prior constraint for first pose (strong anchor)
            prior_info = np.eye(3) * 1000.0  # Very high confidence
            H[0:3, 0:3] += prior_info
            residual = poses[0] - first_pose_fixed
            residual[2] = normalize_angle(residual[2])
            b[0:3] += prior_info @ residual

            # Add odometry edge constraints
            for from_idx, to_idx, dx, dy, dtheta, info in self.edges:
                if from_idx < 0 or to_idx >= N:
                    continue  # Skip invalid edges

                # Get poses
                p_from = poses[from_idx]  # [x1, y1, theta1]
                p_to = poses[to_idx]      # [x2, y2, theta2]

                # Predicted relative transformation from p_from to p_to
                cos_theta = cos(p_from[2])
                sin_theta = sin(p_from[2])

                # Transform global delta to local frame of p_from
                global_dx = p_to[0] - p_from[0]
                global_dy = p_to[1] - p_from[1]

                pred_dx = cos_theta * global_dx + sin_theta * global_dy
                pred_dy = -sin_theta * global_dx + cos_theta * global_dy
                pred_dtheta = normalize_angle(p_to[2] - p_from[2])

                # Residual (error): measured - predicted
                residual = np.array([
                    dx - pred_dx,
                    dy - pred_dy,
                    normalize_angle(dtheta - pred_dtheta)
                ])

                # Jacobians w.r.t. p_from and p_to
                # J_from: derivative of residual w.r.t. [x1, y1, theta1]
                # J_to: derivative of residual w.r.t. [x2, y2, theta2]

                J_from = np.array([
                    [-cos_theta, -sin_theta, -sin_theta * global_dx + cos_theta * global_dy],
                    [sin_theta, -cos_theta, -cos_theta * global_dx - sin_theta * global_dy],
                    [0, 0, -1]
                ])

                J_to = np.array([
                    [cos_theta, sin_theta, 0],
                    [-sin_theta, cos_theta, 0],
                    [0, 0, 1]
                ])

                # Add to Hessian and gradient
                i = from_idx * 3
                j = to_idx * 3

                H[i:i+3, i:i+3] += J_from.T @ info @ J_from
                H[i:i+3, j:j+3] += J_from.T @ info @ J_to
                H[j:j+3, i:i+3] += J_to.T @ info @ J_from
                H[j:j+3, j:j+3] += J_to.T @ info @ J_to

                b[i:i+3] += J_from.T @ info @ residual
                b[j:j+3] += J_to.T @ info @ residual

            # Solve normal equations: H * delta = -b
            try:
                delta = np.linalg.solve(H + np.eye(N * 3) * 1e-6, -b)
            except np.linalg.LinAlgError:
                # Singular matrix, stop optimization
                break

            # Update poses
            poses += delta.reshape(N, 3)

            # Normalize angles
            poses[:, 2] = np.array([normalize_angle(theta) for theta in poses[:, 2]])

            # Check convergence
            if np.linalg.norm(delta) < 1e-4:
                break

        # Update internal poses
        self.poses = [poses[i] for i in range(N)]

        return self.poses[-1]


def icp_feature_based(source_features: FeatureCloud,
                      target_features: FeatureCloud,
                      init_x: float = 0.0, init_y: float = 0.0, init_theta: float = 0.0,
                      max_iterations: int = 20, tolerance: float = 1e-4,
                      max_correspondence_dist: float = 0.5,
                      edge_weight: float = 2.0,
                      planar_weight: float = 1.0):
    """
    Feature-weighted ICP for LOAM-style scan-to-map matching.

    Matches features with type-specific weighting:
    - EDGE features: Higher weight (sharper constraints)
    - PLANAR features: Lower weight (softer constraints)
    - GROUND features: Medium weight

    Args:
        source_features: Current scan features (robot frame)
        target_features: Map features (world frame)
        init_x, init_y, init_theta: Initial transformation guess
        max_iterations: Maximum ICP iterations
        tolerance: Convergence tolerance
        max_correspondence_dist: Maximum matching distance
        edge_weight: Weight for edge feature matches
        planar_weight: Weight for planar feature matches

    Returns:
        (x, y, theta, converged, quality_metrics) transformation with quality
    """
    # Fallback to traditional ICP if not enough features
    if len(source_features.all_points) < 5 or len(target_features.all_points) < 5:
        return init_x, init_y, init_theta, False, {'mean_error': 999.0, 'num_correspondences': 0, 'correspondence_ratio': 0.0}

    # Build weighted point clouds
    # Strategy: Duplicate edge points to give them higher weight
    source_points_weighted = []
    target_points_weighted = []

    # Add edge features with higher weight (duplicate them)
    for _ in range(int(edge_weight)):
        if len(source_features.edge_points) > 0:
            source_points_weighted.append(source_features.edge_points)
        if len(target_features.edge_points) > 0:
            target_points_weighted.append(target_features.edge_points)

    # Add planar features with normal weight
    for _ in range(int(planar_weight)):
        if len(source_features.planar_points) > 0:
            source_points_weighted.append(source_features.planar_points)
        if len(target_features.planar_points) > 0:
            target_points_weighted.append(target_features.planar_points)

    # Add ground features with medium weight
    if len(source_features.ground_points) > 0:
        source_points_weighted.append(source_features.ground_points)
    if len(target_features.ground_points) > 0:
        target_points_weighted.append(target_features.ground_points)

    # Fallback if no weighted features
    if not source_points_weighted or not target_points_weighted:
        # Use all points as fallback
        source_combined = source_features.all_points
        target_combined = target_features.all_points
    else:
        source_combined = np.vstack(source_points_weighted)
        target_combined = np.vstack(target_points_weighted)

    # Run standard ICP on weighted feature cloud
    return icp_2d(
        source=source_combined,
        target=target_combined,
        init_x=init_x,
        init_y=init_y,
        init_theta=init_theta,
        max_iterations=max_iterations,
        tolerance=tolerance,
        max_correspondence_dist=max_correspondence_dist
    )


class ICPOdometryNode(Node):
    """
    ICP Odometry Node.
    
    Uses scan-to-scan ICP to refine odometry.
    EKF odometry provides the initial guess for scan alignment.
    """
    
    def __init__(self):
        super().__init__('icp_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_range', 0.05),
                ('max_range', 3.5),
                ('max_iterations', 25),
                ('tolerance', 1e-4),
                ('max_correspondence_distance', 0.3),
                ('keyframe_distance', 0.2),
                ('keyframe_angle', 0.15),
                ('odom_frame', 'odom'),
                ('base_frame', 'base_link'),
                ('laser_frame', 'base_scan'),
                ('scan_topic', '/scan'),
                ('ekf_odom_topic', '/odometry/ekf'),
                ('icp_odom_output_topic', '/odometry/icp'),
                ('covariance', 0.000032),  # Measured Lidar variance
                # Feature Extraction Parameters
                ('use_feature_extraction', True),
                ('edge_curvature_threshold', 0.1),
                ('planar_curvature_threshold', 0.02),
                ('ground_height_threshold', -0.15),
                ('feature_search_radius', 0.3),
                # Global Map Parameters
                ('voxel_size', 0.1),
                ('max_features_per_voxel', 10),
                ('map_update_distance', 0.5),
                ('map_update_angle', 0.3),
                ('map_query_radius', 3.0),
                # Feature Weights
                ('edge_weight', 2.0),
                ('planar_weight', 1.0),
                # Pose Graph Optimization
                ('use_pose_graph', False),
                ('pose_graph_window_size', 30),
                ('optimization_interval', 5),
            ]
        )
        
        # Get parameters
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.tolerance = self.get_parameter('tolerance').value
        self.max_correspondence_distance = self.get_parameter('max_correspondence_distance').value
        self.keyframe_distance = self.get_parameter('keyframe_distance').value
        self.keyframe_angle = self.get_parameter('keyframe_angle').value

        self.covariance = self.get_parameter('covariance').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Feature extraction parameters
        self.use_feature_extraction = self.get_parameter('use_feature_extraction').value
        self.edge_curvature_threshold = self.get_parameter('edge_curvature_threshold').value
        self.planar_curvature_threshold = self.get_parameter('planar_curvature_threshold').value
        self.ground_height_threshold = self.get_parameter('ground_height_threshold').value
        self.feature_search_radius = self.get_parameter('feature_search_radius').value

        # Global map parameters
        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_features_per_voxel = self.get_parameter('max_features_per_voxel').value
        self.map_update_distance = self.get_parameter('map_update_distance').value
        self.map_update_angle = self.get_parameter('map_update_angle').value
        self.map_query_radius = self.get_parameter('map_query_radius').value

        # Feature weights
        self.edge_weight = self.get_parameter('edge_weight').value
        self.planar_weight = self.get_parameter('planar_weight').value

        # Pose graph optimization parameters
        self.use_pose_graph = self.get_parameter('use_pose_graph').value
        self.pose_graph_window_size = self.get_parameter('pose_graph_window_size').value
        self.optimization_interval = self.get_parameter('optimization_interval').value

        # State
        self.prev_scan_points = None
        self.prev_ekf_odom = None

        # --- Feature Map or Legacy Occupancy Grid ---
        if self.use_feature_extraction:
            # Global feature map (LOAM-style)
            self.global_feature_map = GlobalFeatureMap(
                voxel_size=self.voxel_size,
                max_features_per_voxel=self.max_features_per_voxel,
                feature_decay_distance=5.0
            )
            # Track last map update pose
            self.last_map_update_pose = np.zeros(3)
            self.get_logger().info('Using LOAM-style feature extraction and global map')

        # Pose graph optimization
        if self.use_pose_graph:
            self.pose_graph = PoseGraph(window_size=self.pose_graph_window_size)
            self.scan_count = 0  # Counter for optimization interval
            self.get_logger().info(f'Pose graph optimization enabled (window={self.pose_graph_window_size}, interval={self.optimization_interval})')
        else:
            self.pose_graph = None

        if not self.use_feature_extraction:
            # Dense occupancy grid mode (slam_toolbox style)
            self.grid_resolution = 0.05  # 5cm per cell (match slam_toolbox)
            self.grid_size = 1000  # 1000x1000 = 50m x 50m map (much larger!)
            self.grid_origin = np.array([self.grid_size // 2, self.grid_size // 2])

            # Log-odds occupancy grid (Bayesian probabilistic update)
            self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)

            # Log-odds parameters for Bayesian updates
            self.log_odds_occ = np.log(0.7 / 0.3)   # +0.85 when hit
            self.log_odds_free = np.log(0.3 / 0.7)  # -0.85 when free
            self.log_odds_min = -5.0  # Clamp to prevent overflow
            self.log_odds_max = 5.0

            # For ICP: extract occupied cells above threshold
            self.occupied_threshold = 0.6  # P(occupied) > 0.6
            self.get_logger().info('Using legacy scan-to-scan with occupancy grid')

        # Sliding window of recent scans for more stable ICP
        self.recent_scans = []
        self.max_recent_scans = 2  # Keep last 2 scans (smaller = better motion tracking)

        # Keyframe tracking
        self.keyframe_points = None
        self.keyframe_pose = np.zeros(3)  # [x, y, theta] of keyframe in odom frame

        # Keyframe-based ICP (match pboon09 architecture)
        from collections import deque
        self.keyframe_scans = deque(maxlen=15)  # Local map of 15 recent keyframe scans
        self.last_keyframe_pose = np.zeros(3)  # Last keyframe pose for distance check
        self.accumulated_ekf_delta = np.zeros(3)  # EKF delta since last keyframe

        # ICP correction limits (match pboon09)
        self.max_translation_correction = 0.3  # meters
        self.max_rotation_correction = 0.087  # radians (~5 degrees)

        # Integrated ICP pose
        self.icp_pose = np.zeros(3)  # [x, y, theta] in odom frame

        # Bootstrap phase variables (for feature-based mode)
        self.bootstrap_scans = 0
        self.bootstrap_threshold = 5  # Build map for first 5 scans
        self.bootstrapping = True

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Latest EKF odometry (for initial guess)
        self.latest_ekf_pose = None
        self.prev_ekf_pose = None
        
        # Current robot velocity for deskewing
        self.current_v = 0.0
        self.current_w = 0.0
        
        # Publishers
        self.icp_odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('icp_odom_output_topic').value,
            10
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )

        # ICP quality publisher (percentage 0-100)
        self.icp_quality_pub = self.create_publisher(
            Float32,
            '/icp/quality',
            10
        )

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            SENSOR_QOS  # Use BEST_EFFORT to match rosbag
        )
        
        self.ekf_odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('ekf_odom_topic').value,
            self.ekf_odom_callback,
            10
        )
        
        self.get_logger().info('ICP Odometry Node initialized')
        self.get_logger().info(f'  Range filter: [{self.min_range}, {self.max_range}] m')
        self.get_logger().info(f'  Max ICP iterations: {self.max_iterations}')
    
    def ekf_odom_callback(self, msg: Odometry):
        """Store latest EKF odometry for use as ICP initial guess."""
        _, _, yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.latest_ekf_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ])
        
        # Extract velocity for deskewing
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z

    def world_to_grid(self, x, y):
        """Convert world coordinates (m) to grid coordinates (cells)."""
        grid_x = int(x / self.grid_resolution) + self.grid_origin[0]
        grid_y = int(y / self.grid_resolution) + self.grid_origin[1]
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates (cells) to world coordinates (m)."""
        x = (grid_x - self.grid_origin[0]) * self.grid_resolution
        y = (grid_y - self.grid_origin[1]) * self.grid_resolution
        return x, y

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm to get cells along a ray."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return cells

    def update_occupancy_grid(self, scan_points_world, robot_x, robot_y):
        """
        Update occupancy grid with new scan (Bayesian log-odds update).
        - Mark free cells along rays from robot to scan endpoints
        - Mark occupied cells at scan endpoints
        """
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)

        for point in scan_points_world:
            endpoint_gx, endpoint_gy = self.world_to_grid(point[0], point[1])

            # Check bounds
            if not (0 <= endpoint_gx < self.grid_size and 0 <= endpoint_gy < self.grid_size):
                continue

            # Ray trace from robot to endpoint
            ray_cells = self.bresenham_line(robot_gx, robot_gy, endpoint_gx, endpoint_gy)

            # Mark all cells along ray as FREE (except endpoint)
            for i, (gx, gy) in enumerate(ray_cells[:-1]):  # Skip last cell (endpoint)
                if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                    self.occupancy_grid[gy, gx] += self.log_odds_free
                    self.occupancy_grid[gy, gx] = np.clip(self.occupancy_grid[gy, gx],
                                                           self.log_odds_min,
                                                           self.log_odds_max)

            # Mark endpoint as OCCUPIED
            if 0 <= endpoint_gx < self.grid_size and 0 <= endpoint_gy < self.grid_size:
                self.occupancy_grid[endpoint_gy, endpoint_gx] += self.log_odds_occ
                self.occupancy_grid[endpoint_gy, endpoint_gx] = np.clip(self.occupancy_grid[endpoint_gy, endpoint_gx],
                                                                         self.log_odds_min,
                                                                         self.log_odds_max)

    def extract_occupied_points(self):
        """Extract points from occupancy grid where P(occupied) > threshold."""
        # Convert log-odds to probability: P = 1 / (1 + exp(-log_odds))
        prob_grid = 1.0 / (1.0 + np.exp(-self.occupancy_grid))

        # Find cells above threshold
        occupied_cells = np.argwhere(prob_grid > self.occupied_threshold)

        if len(occupied_cells) == 0:
            return np.array([]).reshape(0, 2)

        # Convert grid coords to world coords
        points = []
        for cell in occupied_cells:
            gy, gx = cell  # Note: numpy uses (row, col) = (y, x)
            wx, wy = self.grid_to_world(gx, gy)
            points.append([wx, wy])

        return np.array(points)

    def publish_occupancy_grid(self, stamp: Time):
        """Publish occupancy grid as OccupancyGrid message for RViz visualization."""
        # Convert log-odds to probability: P = 1 / (1 + exp(-log_odds))
        prob_grid = 1.0 / (1.0 + np.exp(-self.occupancy_grid))

        # Convert to OccupancyGrid format [0, 100] with -1 for unknown
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = stamp.to_msg()
        grid_msg.header.frame_id = self.odom_frame

        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size

        # Origin is the bottom-left corner in world coordinates
        origin_x = -self.grid_origin[0] * self.grid_resolution
        origin_y = -self.grid_origin[1] * self.grid_resolution
        grid_msg.info.origin.position.x = origin_x
        grid_msg.info.origin.position.y = origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        # Convert probabilities to occupancy values [0, 100]
        # ROS convention: 0 = free, 100 = occupied, -1 = unknown
        occupancy_data = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                if abs(self.occupancy_grid[i, j]) < 0.01:  # Nearly zero log-odds = unknown
                    occupancy_data[i, j] = -1
                else:
                    # Convert probability to [0, 100]
                    occupancy_data[i, j] = int(prob_grid[i, j] * 100)

        # Flatten to row-major order (ROS convention)
        grid_msg.data = occupancy_data.flatten().tolist()

        self.map_pub.publish(grid_msg)

    def scan_callback(self, msg: LaserScan):
        """Process incoming laser scan with ICP."""
        current_time = self.get_clock().now()

        # Convert scan to points with range filtering and motion compensation
        current_points = self.scan_to_points_deskewed(msg, self.min_range, self.max_range, self.current_v, self.current_w)

        self.get_logger().info(f'Received scan with {len(current_points)} valid points', throttle_duration_sec=2.0)

        if len(current_points) < 10:
            self.get_logger().warn('Too few valid scan points', throttle_duration_sec=5.0)
            return

        # Extract features (LOAM-style) if enabled
        if self.use_feature_extraction:
            current_features = extract_features(
                current_points,
                edge_curvature_threshold=self.edge_curvature_threshold,
                planar_curvature_threshold=self.planar_curvature_threshold,
                ground_height_threshold=self.ground_height_threshold,
                search_radius=self.feature_search_radius
            )

            self.get_logger().info(
                f'Features: {len(current_features.edge_points)} edges, '
                f'{len(current_features.planar_points)} planars, '
                f'{len(current_features.ground_points)} ground',
                throttle_duration_sec=2.0
            )
        else:
            current_features = None
        
        # Initialize with first scan
        if self.prev_scan_points is None:
            if self.use_feature_extraction:
                # Initialize global feature map with first scan
                self.global_feature_map.add_features(
                    current_features,
                    robot_pose=np.zeros(3),
                    timestamp=current_time.nanoseconds / 1e9
                )
                self.last_map_update_pose = np.zeros(3)

                stats = self.global_feature_map.get_statistics()
                self.get_logger().info(
                    f'Initialized feature map: {stats["total_features"]} features '
                    f'({stats["edge_features"]} edges, {stats["planar_features"]} planars)'
                )
            else:
                # Legacy: Initialize occupancy grid with first scan
                scan_world = transform_points(current_points, 0.0, 0.0, 0.0)
                self.update_occupancy_grid(scan_world, 0.0, 0.0)

            self.keyframe_points = current_points
            self.keyframe_pose = np.zeros(3)
            self.prev_scan_points = current_points
            self.prev_ekf_pose = self.latest_ekf_pose.copy() if self.latest_ekf_pose is not None else np.zeros(3)

            # Initialize sliding window
            if not self.use_feature_extraction:
                self.recent_scans = [current_points]

            self.get_logger().info(f'Initialized with {len(current_points)} points')
            self.publish_icp_odometry(current_time)
            return

        # Compute initial guess from EKF motion (if available)
        init_x, init_y, init_theta = 0.0, 0.0, 0.0
        if self.latest_ekf_pose is not None and self.prev_ekf_pose is not None:
            # Motion in EKF frame
            ekf_dx = self.latest_ekf_pose[0] - self.prev_ekf_pose[0]
            ekf_dy = self.latest_ekf_pose[1] - self.prev_ekf_pose[1]
            ekf_dtheta = normalize_angle(self.latest_ekf_pose[2] - self.prev_ekf_pose[2])

            # Transform to robot frame at previous time
            c_prev, s_prev = cos(-self.prev_ekf_pose[2]), sin(-self.prev_ekf_pose[2])
            init_x = c_prev * ekf_dx - s_prev * ekf_dy
            init_y = s_prev * ekf_dx + c_prev * ekf_dy
            init_theta = ekf_dtheta

        # KEYFRAME-BASED ICP (match pboon09 architecture)
        # Accumulate EKF delta since last keyframe
        self.accumulated_ekf_delta[0] += init_x
        self.accumulated_ekf_delta[1] += init_y
        self.accumulated_ekf_delta[2] += init_theta

        # Check if we're at a keyframe
        accumulated_distance = np.sqrt(self.accumulated_ekf_delta[0]**2 + self.accumulated_ekf_delta[1]**2)
        accumulated_rotation = abs(self.accumulated_ekf_delta[2])
        is_keyframe = (accumulated_distance >= self.keyframe_distance or
                      accumulated_rotation >= self.keyframe_angle)

        # If NOT at keyframe and we have keyframe scans, skip ICP and use EKF only
        if not is_keyframe and len(self.keyframe_scans) >= 3:
            # Just use accumulated EKF delta (no ICP refinement)
            c, s = cos(self.icp_pose[2]), sin(self.icp_pose[2])
            global_dx = c * init_x - s * init_y
            global_dy = s * init_x + c * init_y

            self.icp_pose[0] += global_dx
            self.icp_pose[1] += global_dy
            self.icp_pose[2] = normalize_angle(self.icp_pose[2] + init_theta)

            self.prev_scan_points = current_points
            self.prev_ekf_pose = self.latest_ekf_pose.copy() if self.latest_ekf_pose is not None else self.prev_ekf_pose

            self.get_logger().info(
                f'[NON-KEYFRAME] Using EKF only, accumulated: {accumulated_distance:.3f}m, {accumulated_rotation:.3f}rad',
                throttle_duration_sec=1.0
            )

            self.publish_icp_odometry(current_time)
            return

        # At keyframe or first few scans - run ICP
        self.get_logger().info(
            f'[KEYFRAME] Distance: {accumulated_distance:.3f}m, Rotation: {accumulated_rotation:.3f}rad',
            throttle_duration_sec=1.0
        )

        # Bootstrap phase: Build map with scan-to-scan ICP for first N scans
        # Set flag to skip scan-to-map matching during bootstrap
        using_bootstrap = False
        delta_x, delta_y, delta_theta, converged = 0.0, 0.0, 0.0, False  # Initialize

        if self.bootstrapping and self.use_feature_extraction and self.prev_scan_points is not None:
            using_bootstrap = True

            # DEBUG: Log pose BEFORE ICP
            self.get_logger().info(
                f'[BOOTSTRAP] Pose BEFORE ICP: ({self.icp_pose[0]:.3f}, {self.icp_pose[1]:.3f}, {self.icp_pose[2]:.3f})'
            )

            # STEP 1: Run scan-to-scan ICP FIRST
            delta_x, delta_y, delta_theta, converged, icp_quality = icp_2d(
                source=current_points,
                target=self.prev_scan_points,
                init_x=init_x,
                init_y=init_y,
                init_theta=init_theta,
                max_iterations=self.max_iterations,
                tolerance=self.tolerance,
                max_correspondence_dist=self.max_correspondence_distance
            )

            # DEBUG: Log ICP delta
            self.get_logger().info(
                f'[BOOTSTRAP] ICP delta: ({delta_x:.3f}, {delta_y:.3f}, {delta_theta:.3f}), converged={converged}'
            )

            if not converged:
                self.get_logger().debug('ICP did not converge during bootstrap, using best estimate')

            # STEP 2: Integrate delta into icp_pose SECOND (with ADAPTIVE EKF blending)
            # ADAPTIVE BLENDING: Adjust trust based on ICP quality
            mean_error = icp_quality['mean_error']
            corr_ratio = icp_quality['correspondence_ratio']

            if not converged or mean_error > 0.3 or corr_ratio < 0.3:
                # BAD ICP quality → Trust wheel odom heavily (90%)
                ekf_weight = 0.9
                quality_status = "POOR - trusting wheel 90%"
            elif mean_error > 0.15 or corr_ratio < 0.5:
                # MEDIUM ICP quality → Balanced trust (60% wheel)
                ekf_weight = 0.6
                quality_status = "MEDIUM - trusting wheel 60%"
            else:
                # GOOD ICP quality → Trust ICP more (20% wheel)
                ekf_weight = 0.2
                quality_status = "GOOD - trusting ICP 80%"

            blended_dx = (1 - ekf_weight) * delta_x + ekf_weight * init_x
            blended_dy = (1 - ekf_weight) * delta_y + ekf_weight * init_y
            blended_dtheta = (1 - ekf_weight) * delta_theta + ekf_weight * init_theta

            # Compute quality percentage (0-100%) - REALISTIC & STRINGENT
            quality_pct = 0.0

            # 1. Convergence (only 15%, not that meaningful)
            if converged:
                quality_pct += 15.0

            # 2. Mean Error (50% weight, MUCH stricter thresholds)
            # Good ICP should have error < 0.05m, acceptable < 0.1m
            if mean_error < 0.02:
                error_score = 50.0  # Excellent
            elif mean_error < 0.05:
                error_score = 40.0  # Good
            elif mean_error < 0.1:
                error_score = 25.0  # Acceptable
            elif mean_error < 0.2:
                error_score = 10.0  # Poor
            else:
                error_score = 0.0   # Very poor
            quality_pct += error_score

            # 3. Correspondence Ratio (20% weight, need high ratio)
            if corr_ratio > 0.9:
                ratio_score = 20.0  # Excellent
            elif corr_ratio > 0.7:
                ratio_score = 15.0  # Good
            elif corr_ratio > 0.5:
                ratio_score = 10.0  # Acceptable
            elif corr_ratio > 0.3:
                ratio_score = 5.0   # Poor
            else:
                ratio_score = 0.0   # Very poor
            quality_pct += ratio_score

            # 4. EKF Consistency (15% weight - CRITICAL!)
            # If ICP differs wildly from EKF, it's probably wrong
            icp_motion = np.sqrt(delta_x**2 + delta_y**2)
            ekf_motion = np.sqrt(init_x**2 + init_y**2)
            motion_diff = abs(icp_motion - ekf_motion)

            if motion_diff < 0.05:  # Within 5cm
                consistency_score = 15.0
            elif motion_diff < 0.1:  # Within 10cm
                consistency_score = 10.0
            elif motion_diff < 0.2:  # Within 20cm
                consistency_score = 5.0
            else:  # More than 20cm difference - ICP likely wrong!
                consistency_score = 0.0
            quality_pct += consistency_score

            quality_pct = min(100.0, max(0.0, quality_pct))

            # Publish quality percentage
            quality_msg = Float32()
            quality_msg.data = quality_pct
            self.icp_quality_pub.publish(quality_msg)

            self.get_logger().info(
                f'[ADAPTIVE] Quality: {quality_status} ({quality_pct:.1f}%) | '
                f'err={mean_error:.3f}, ratio={corr_ratio:.2f}',
                throttle_duration_sec=1.0
            )

            c, s = cos(self.icp_pose[2]), sin(self.icp_pose[2])
            global_dx = c * blended_dx - s * blended_dy
            global_dy = s * blended_dx + c * blended_dy

            self.icp_pose[0] += global_dx
            self.icp_pose[1] += global_dy
            self.icp_pose[2] = normalize_angle(self.icp_pose[2] + blended_dtheta)

            # DEBUG: Log pose AFTER integration
            self.get_logger().info(
                f'[BOOTSTRAP] Pose AFTER integration: ({self.icp_pose[0]:.3f}, {self.icp_pose[1]:.3f}, {self.icp_pose[2]:.3f})'
            )

            # STEP 3: Add features to map at UPDATED pose THIRD
            self.global_feature_map.add_features(
                current_features,
                robot_pose=self.icp_pose,  # NOW at correct integrated pose!
                timestamp=current_time.nanoseconds / 1e9
            )
            self.bootstrap_scans += 1

            # Check if bootstrap complete
            stats = self.global_feature_map.get_statistics()
            if self.bootstrap_scans >= self.bootstrap_threshold and stats['total_features'] >= 100:
                self.bootstrapping = False
                self.get_logger().info(
                    f'Bootstrap complete: {stats["total_features"]} features in map '
                    f'({stats["edge_features"]} edges, {stats["planar_features"]} planars)'
                )
                self.last_map_update_pose = self.icp_pose.copy()
            else:
                self.get_logger().info(
                    f'Bootstrap {self.bootstrap_scans}/{self.bootstrap_threshold}: '
                    f'{stats["total_features"]} features in map',
                    throttle_duration_sec=1.0
                )

        # ICP Matching: SCAN-TO-MAP (features) or SCAN-TO-SCAN (legacy)
        # Skip ALL ICP matching if we already did it in bootstrap
        if not using_bootstrap and self.use_feature_extraction:
            # DEBUG: Log pose BEFORE scan-to-map ICP
            self.get_logger().info(
                f'[SCAN-TO-MAP] Pose BEFORE ICP: ({self.icp_pose[0]:.3f}, {self.icp_pose[1]:.3f}, {self.icp_pose[2]:.3f})'
            )

            # SCAN-TO-MAP FEATURE ICP
            # Query global map for nearby features (in world coordinates)
            map_features_world = self.global_feature_map.query_region(
                robot_pose=self.icp_pose,
                query_radius=self.map_query_radius
            )

            # Transform map features to robot frame for matching
            # (ICP expects both clouds in same frame)
            cos_theta = cos(-self.icp_pose[2])
            sin_theta = sin(-self.icp_pose[2])

            map_features_robot = FeatureCloud(
                edge_points=transform_points(map_features_world.edge_points,
                                              -self.icp_pose[0], -self.icp_pose[1], -self.icp_pose[2])
                    if len(map_features_world.edge_points) > 0 else np.array([]).reshape(0, 2),
                planar_points=transform_points(map_features_world.planar_points,
                                                -self.icp_pose[0], -self.icp_pose[1], -self.icp_pose[2])
                    if len(map_features_world.planar_points) > 0 else np.array([]).reshape(0, 2),
                ground_points=transform_points(map_features_world.ground_points,
                                                -self.icp_pose[0], -self.icp_pose[1], -self.icp_pose[2])
                    if len(map_features_world.ground_points) > 0 else np.array([]).reshape(0, 2),
                all_points=transform_points(map_features_world.all_points,
                                             -self.icp_pose[0], -self.icp_pose[1], -self.icp_pose[2])
                    if len(map_features_world.all_points) > 0 else np.array([]).reshape(0, 2),
                curvatures=map_features_world.curvatures,
                feature_types=map_features_world.feature_types
            )

            # Log map query results and total map size
            map_stats = self.global_feature_map.get_statistics()
            self.get_logger().info(
                f'Map query: {len(map_features_world.all_points)} features found | '
                f'Total map: {map_stats["total_features"]} features '
                f'({map_stats["edge_features"]} edges, {map_stats["planar_features"]} planars)',
                throttle_duration_sec=2.0
            )

            # Fallback to scan-to-scan if map has too few features OR current scan has too few features
            if (len(map_features_world.all_points) < 50 or
                len(current_features.all_points) < 15):
                self.get_logger().warn(
                    f'Too few features (map:{len(map_features_world.all_points)}, '
                    f'scan:{len(current_features.all_points)}), '
                    f'falling back to scan-to-scan',
                    throttle_duration_sec=2.0
                )
                delta_x, delta_y, delta_theta, converged, icp_quality = icp_2d(
                    source=current_points,
                    target=self.prev_scan_points,
                    init_x=init_x,
                    init_y=init_y,
                    init_theta=init_theta,
                    max_iterations=self.max_iterations,
                    tolerance=self.tolerance,
                    max_correspondence_dist=self.max_correspondence_distance
                )
            else:
                # Feature-based scan-to-map ICP
                delta_x, delta_y, delta_theta, converged, icp_quality = icp_feature_based(
                    source_features=current_features,
                    target_features=map_features_robot,
                    init_x=init_x,
                    init_y=init_y,
                    init_theta=init_theta,
                    max_iterations=self.max_iterations,
                    tolerance=self.tolerance,
                    max_correspondence_dist=self.max_correspondence_distance,
                    edge_weight=self.edge_weight,
                    planar_weight=self.planar_weight
                )

            # DEBUG: Log scan-to-map ICP delta with quality
            self.get_logger().info(
                f'[SCAN-TO-MAP] ICP delta: ({delta_x:.3f}, {delta_y:.3f}, {delta_theta:.3f}), '
                f'converged={converged}, quality: err={icp_quality["mean_error"]:.3f}'
            )
        elif not using_bootstrap:
            # SCAN-TO-MAP ICP using LOCAL KEYFRAME MAP (match pboon09)
            # Concatenate all keyframe scans to create local map
            if len(self.keyframe_scans) >= 3:
                map_points = np.vstack(self.keyframe_scans)
                self.get_logger().info(
                    f'[LOCAL MAP] Using {len(self.keyframe_scans)} keyframe scans → {len(map_points)} points',
                    throttle_duration_sec=2.0
                )
            else:
                # Fallback to global occupancy grid if not enough keyframes
                map_points = self.extract_occupied_points()

            if len(map_points) < 50:
                # Map too small, fallback to scan-to-scan
                self.get_logger().warn(
                    f'Map has only {len(map_points)} points, using scan-to-scan fallback',
                    throttle_duration_sec=2.0
                )
                delta_x, delta_y, delta_theta, converged, icp_quality = icp_2d(
                    source=current_points,
                    target=self.prev_scan_points,
                    init_x=init_x,
                    init_y=init_y,
                    init_theta=init_theta,
                    max_iterations=self.max_iterations,
                    tolerance=self.tolerance,
                    max_correspondence_dist=self.max_correspondence_distance
                )
            else:
                # SCAN-TO-MAP: Transform current scan to world frame with current estimate
                current_world_estimate = transform_points(current_points,
                                                         self.icp_pose[0] + init_x,
                                                         self.icp_pose[1] + init_y,
                                                         self.icp_pose[2] + init_theta)

                # Run ICP in world frame: match current scan against map
                delta_x, delta_y, delta_theta, converged, icp_quality = icp_2d(
                    source=current_world_estimate,
                    target=map_points,
                    init_x=0.0,  # Already at estimated pose
                    init_y=0.0,
                    init_theta=0.0,
                    max_iterations=self.max_iterations,
                    tolerance=self.tolerance,
                    max_correspondence_dist=self.max_correspondence_distance
                )

                self.get_logger().info(
                    f'[SCAN-TO-MAP] Map: {len(map_points)} pts | ICP: ({delta_x:.3f}, {delta_y:.3f}) | '
                    f'Quality: err={icp_quality["mean_error"]:.3f}, matches={icp_quality["num_correspondences"]}, '
                    f'ratio={icp_quality["correspondence_ratio"]:.2f}',
                    throttle_duration_sec=2.0
                )

        if not converged:
            self.get_logger().debug('ICP did not converge, using best estimate')

        # Integrate ICP delta into global pose (ONLY if not already done in bootstrap)
        if not using_bootstrap:
            # ICP CORRECTION LIMITS (match pboon09)
            # Reject ICP if correction too large (likely bad match)
            translation_correction = np.sqrt(delta_x**2 + delta_y**2)
            rotation_correction = abs(delta_theta)

            if translation_correction > self.max_translation_correction or rotation_correction > self.max_rotation_correction:
                # ICP correction too large - reject and use EKF only
                self.get_logger().warn(
                    f'[ICP REJECTED] Correction too large: {translation_correction:.3f}m, {rotation_correction:.3f}rad '
                    f'(limits: {self.max_translation_correction}m, {self.max_rotation_correction:.3f}rad)',
                    throttle_duration_sec=1.0
                )

                # Use EKF delta only
                c, s = cos(self.icp_pose[2]), sin(self.icp_pose[2])
                global_dx = c * init_x - s * init_y
                global_dy = s * init_x + c * init_y

                self.icp_pose[0] += global_dx
                self.icp_pose[1] += global_dy
                self.icp_pose[2] = normalize_angle(self.icp_pose[2] + init_theta)

                # Publish with low quality
                quality_msg = Float32()
                quality_msg.data = 0.0
                self.icp_quality_pub.publish(quality_msg)

            else:
                # ICP correction within limits - apply adaptive blending
                # ADAPTIVE BLENDING based on ICP quality
                mean_error = icp_quality['mean_error']
                corr_ratio = icp_quality['correspondence_ratio']

                if not converged or mean_error > 0.3 or corr_ratio < 0.3:
                    # BAD ICP quality → Trust wheel odom heavily (90%)
                    ekf_weight = 0.9
                    quality_status = "POOR - trusting wheel 90%"
                elif mean_error > 0.15 or corr_ratio < 0.5:
                    # MEDIUM ICP quality → Balanced trust (60% wheel)
                    ekf_weight = 0.6
                    quality_status = "MEDIUM - trusting wheel 60%"
                else:
                    # GOOD ICP quality → Trust ICP more (20% wheel)
                    ekf_weight = 0.2
                    quality_status = "GOOD - trusting ICP 80%"

                # Compute ICP delta
                icp_dx, icp_dy, icp_dtheta = delta_x, delta_y, delta_theta

                # Get EKF delta (motion prediction from wheel odometry)
                ekf_dx, ekf_dy, ekf_dtheta = init_x, init_y, init_theta

                # BLEND: weighted combination of ICP and EKF based on quality
                blended_dx = (1 - ekf_weight) * icp_dx + ekf_weight * ekf_dx
                blended_dy = (1 - ekf_weight) * icp_dy + ekf_weight * ekf_dy
                blended_dtheta = (1 - ekf_weight) * icp_dtheta + ekf_weight * ekf_dtheta

                # Transform to global frame
                c, s = cos(self.icp_pose[2]), sin(self.icp_pose[2])
                global_dx = c * blended_dx - s * blended_dy
                global_dy = s * blended_dx + c * blended_dy

                self.icp_pose[0] += global_dx
                self.icp_pose[1] += global_dy
                self.icp_pose[2] = normalize_angle(self.icp_pose[2] + blended_dtheta)

                # Compute quality percentage (0-100%) - REALISTIC & STRINGENT
                quality_pct = 0.0

                # 1. Convergence (15%)
                if converged:
                    quality_pct += 15.0

                # 2. Mean Error (50%, strict thresholds)
                if mean_error < 0.02:
                    error_score = 50.0
                elif mean_error < 0.05:
                    error_score = 40.0
                elif mean_error < 0.1:
                    error_score = 25.0
                elif mean_error < 0.2:
                    error_score = 10.0
                else:
                    error_score = 0.0
                quality_pct += error_score

                # 3. Correspondence Ratio (20%)
                if corr_ratio > 0.9:
                    ratio_score = 20.0
                elif corr_ratio > 0.7:
                    ratio_score = 15.0
                elif corr_ratio > 0.5:
                    ratio_score = 10.0
                elif corr_ratio > 0.3:
                    ratio_score = 5.0
                else:
                    ratio_score = 0.0
                quality_pct += ratio_score

                # 4. EKF Consistency (15% - catch when ICP is completely wrong!)
                icp_motion = np.sqrt(icp_dx**2 + icp_dy**2)
                ekf_motion = np.sqrt(ekf_dx**2 + ekf_dy**2)
                motion_diff = abs(icp_motion - ekf_motion)

                if motion_diff < 0.05:
                    consistency_score = 15.0
                elif motion_diff < 0.1:
                    consistency_score = 10.0
                elif motion_diff < 0.2:
                    consistency_score = 5.0
                else:
                    consistency_score = 0.0
                quality_pct += consistency_score

                quality_pct = min(100.0, max(0.0, quality_pct))

                # Publish quality percentage
                quality_msg = Float32()
                quality_msg.data = quality_pct
                self.icp_quality_pub.publish(quality_msg)

                # DEBUG: Log adaptive blending
                self.get_logger().info(
                    f'[ADAPTIVE] {quality_status} ({quality_pct:.1f}%) | '
                    f'err={mean_error:.3f}, ratio={corr_ratio:.2f} | '
                    f'ICP: ({icp_dx:.3f}, {icp_dy:.3f}) → Final: ({self.icp_pose[0]:.3f}, {self.icp_pose[1]:.3f})',
                    throttle_duration_sec=1.0
                )

        # POSE GRAPH OPTIMIZATION
        if self.pose_graph is not None:
            try:
                # Simple covariance based on convergence
                if converged:
                    # Low uncertainty for converged ICP
                    covariance = np.diag([0.01, 0.01, 0.005])  # [x, y, theta] variances
                else:
                    # Higher uncertainty for non-converged
                    covariance = np.diag([0.05, 0.05, 0.02])

                # Add pose and edge to graph
                edge = (delta_x, delta_y, delta_theta, covariance)
                self.pose_graph.add_pose(self.icp_pose, edge)

                # Optimize every N scans
                self.scan_count += 1
                if self.scan_count % self.optimization_interval == 0 and len(self.pose_graph.poses) >= 3:
                    optimized_pose = self.pose_graph.optimize(iterations=10)
                    if optimized_pose is not None and len(optimized_pose) == 3:
                        self.icp_pose = optimized_pose
                        self.get_logger().info(
                            f'Pose graph optimized: ({self.icp_pose[0]:.3f}, {self.icp_pose[1]:.3f}, {self.icp_pose[2]:.3f})',
                            throttle_duration_sec=2.0
                        )
            except Exception as e:
                self.get_logger().error(f'Pose graph optimization failed: {str(e)}', throttle_duration_sec=5.0)

        # UPDATE MAP: Global Feature Map or Local Occupancy Grid
        if self.use_feature_extraction:
            # Update global feature map (check if robot moved enough)
            dist_since_update = sqrt(
                (self.icp_pose[0] - self.last_map_update_pose[0])**2 +
                (self.icp_pose[1] - self.last_map_update_pose[1])**2
            )
            angle_since_update = abs(normalize_angle(self.icp_pose[2] - self.last_map_update_pose[2]))

            if dist_since_update > self.map_update_distance or angle_since_update > self.map_update_angle:
                self.global_feature_map.add_features(
                    current_features,
                    robot_pose=self.icp_pose,
                    timestamp=current_time.nanoseconds / 1e9
                )
                self.last_map_update_pose = self.icp_pose.copy()

                stats = self.global_feature_map.get_statistics()
                self.get_logger().info(
                    f'ICP delta: ({delta_x:.3f}, {delta_y:.3f}, {delta_theta:.3f}) | '
                    f'Map updated: {stats["total_features"]} features '
                    f'({stats["edge_features"]} edges, {stats["planar_features"]} planars)',
                    throttle_duration_sec=2.0
                )
            else:
                self.get_logger().info(
                    f'ICP delta: ({delta_x:.3f}, {delta_y:.3f}, {delta_theta:.3f})',
                    throttle_duration_sec=5.0
                )
        else:
            # BUILD GLOBAL OCCUPANCY GRID (slam_toolbox style - world-centric map!)
            # Don't decay - maintain stable map like slam_toolbox
            # self.occupancy_grid *= 0.95  # REMOVED - keep map stable

            # Add current scan in WORLD frame at current robot pose
            # Transform points to world frame for occupancy grid
            world_points = transform_points(current_points, self.icp_pose[0], self.icp_pose[1], self.icp_pose[2])
            # Update grid with robot's current world pose for ray tracing
            self.update_occupancy_grid(world_points, self.icp_pose[0], self.icp_pose[1])

            # PUBLISH LOCAL OCCUPANCY GRID for RViz
            self.publish_occupancy_grid(current_time)

            # Log stats
            map_points = self.extract_occupied_points()
            self.get_logger().info(f'ICP delta: ({delta_x:.3f}, {delta_y:.3f}, {delta_theta:.3f}) | Grid: {len(map_points)} occupied', throttle_duration_sec=5.0)

        # Update previous scan
        self.prev_scan_points = current_points
        self.prev_ekf_pose = self.latest_ekf_pose.copy() if self.latest_ekf_pose is not None else self.prev_ekf_pose

        # Check if we need a new keyframe
        dist_from_keyframe = sqrt(
            (self.icp_pose[0] - self.keyframe_pose[0])**2 +
            (self.icp_pose[1] - self.keyframe_pose[1])**2
        )
        angle_from_keyframe = abs(normalize_angle(self.icp_pose[2] - self.keyframe_pose[2]))

        if dist_from_keyframe > self.keyframe_distance or angle_from_keyframe > self.keyframe_angle:
            self.keyframe_points = current_points
            self.keyframe_pose = self.icp_pose.copy()

            # KEYFRAME MANAGEMENT (match pboon09)
            # Add current scan to local map
            self.keyframe_scans.append(current_points.copy())
            # Reset accumulated EKF delta (we just ran ICP at this keyframe)
            self.accumulated_ekf_delta = np.zeros(3)
            self.last_keyframe_pose = self.icp_pose.copy()

            if self.use_feature_extraction:
                stats = self.global_feature_map.get_statistics()
                self.get_logger().info(
                    f'Keyframe #{len(self.keyframe_scans)}: ({self.icp_pose[0]:.2f}, {self.icp_pose[1]:.2f}), '
                    f'map={stats["total_features"]} features',
                    throttle_duration_sec=1.0
                )
            else:
                map_points = self.extract_occupied_points()
                self.get_logger().info(
                    f'Keyframe #{len(self.keyframe_scans)}: ({self.icp_pose[0]:.2f}, {self.icp_pose[1]:.2f}), '
                    f'local_map={len(self.keyframe_scans)} scans, global_map={len(map_points)} pts',
                    throttle_duration_sec=1.0
                )
        
        # Publish odometry
        self.publish_icp_odometry(current_time)
    
    def scan_to_points_deskewed(self, scan: LaserScan, min_range: float, max_range: float, v: float, w: float) -> np.ndarray:
        """
        Convert LaserScan to 2D points with deskewing (motion compensation).
        Projects all points to the START of the scan (or end, depending on convention).
        Here we project to the scan timestamp (Header Stamp).
        """
        angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)
        ranges = np.array(scan.ranges)
        
        # Ensure same length
        min_len = min(len(angles), len(ranges))
        angles = angles[:min_len]
        ranges = ranges[:min_len]
        
        # Valid mask
        valid_mask = (ranges >= min_range) & (ranges <= max_range) & np.isfinite(ranges)
        
        # Filter
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # Calculate time offsets for valid points
        # time_offset = index * time_increment
        # We need indices of valid points
        indices = np.where(valid_mask)[0]
        dt = indices * scan.time_increment
        
        # Deskewing:
        # Robot moves during scan. 
        # Transformation from time t_i to t_0 (scan start):
        # We want P_0 (point relative to robot at t_0).
        # Measurement P_i is relative to robot at t_i.
        # Robot motion from t_0 to t_i is Delta_Pose(t_i relative to t_0).
        # P_0 = Delta_Pose * P_i
        #
        # Delta_Pose approx:
        # dx = v * dt
        # dy = 0 (assuming non-holonomic or x-only motion in body frame)
        # dtheta = w * dt
        
        dx = v * dt
        dy = 0.0 # Assuming differential drive / non-holonomic
        dtheta = w * dt
        
        # Convert P_i (polar) to Cartesian
        x_i = valid_ranges * np.cos(valid_angles)
        y_i = valid_ranges * np.sin(valid_angles)
        
        # Apply transformation: Rotate then Translate
        # P_0x = x_i * cos(dtheta) - y_i * sin(dtheta) + dx
        # P_0y = x_i * sin(dtheta) + y_i * cos(dtheta) + dy
        
        c = np.cos(dtheta)
        s = np.sin(dtheta)
        
        x_0 = x_i * c - y_i * s + dx
        y_0 = x_i * s + y_i * c + dy
        
        return np.column_stack((x_0, y_0))

    def publish_icp_odometry(self, stamp: Time):
        """Publish ICP odometry message."""
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame + '_icp'
        
        # Position
        odom.pose.pose.position.x = self.icp_pose[0]
        odom.pose.pose.position.y = self.icp_pose[1]
        odom.pose.pose.position.z = 0.0
        
        # Orientation
        odom.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, self.icp_pose[2])
        
        # Covariance (6x6 matrix expanded to 36 array)
        # x, y, z, roll, pitch, yaw
        # We only really care about x, y, yaw
        cov = np.zeros(36)
        cov[0] = self.covariance   # x variance
        cov[7] = self.covariance   # y variance
        cov[35] = self.covariance  # yaw variance
        odom.pose.covariance = cov.tolist()
        
        self.icp_odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ICPOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
