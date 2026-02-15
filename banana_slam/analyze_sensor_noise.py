#!/usr/bin/env python3
"""
Sensor Noise Analysis Script

Analyzes recorded IMU and wheel encoder data to determine empirical noise parameters
for EKF tuning.

Usage:
    python3 analyze_sensor_noise.py [--imu FILE] [--wheel FILE]

Outputs recommended EKF parameters based on measured sensor variance.
"""

import numpy as np
import pandas as pd
import argparse
import os
import matplotlib.pyplot as plt


def analyze_imu_noise(imu_file):
    """
    Analyze IMU gyro noise characteristics.

    Returns:
        dict: {
            'gyro_mean': mean gyro_z value,
            'gyro_std': standard deviation (measurement noise),
            'gyro_bias': estimated bias
        }
    """
    print("\n=== IMU Gyro Analysis ===")

    # Load data
    df = pd.read_csv(imu_file)
    gyro_z = df['gyro_z'].values

    # Statistics
    gyro_mean = np.mean(gyro_z)
    gyro_std = np.std(gyro_z)
    gyro_median = np.median(gyro_z)

    # Detect if robot is stationary (low variance)
    if gyro_std < 0.01:
        print(f"  Robot appears stationary (low gyro variance)")
        print(f"  Gyro bias: {gyro_mean:.6f} rad/s")
        gyro_bias = gyro_mean
    else:
        print(f"  Robot is moving (high gyro variance)")
        gyro_bias = gyro_median  # Use median as bias estimate

    print(f"  Gyro mean: {gyro_mean:.6f} rad/s")
    print(f"  Gyro std: {gyro_std:.6f} rad/s")
    print(f"  Gyro median: {gyro_median:.6f} rad/s")
    print(f"  Samples: {len(gyro_z)}")

    return {
        'gyro_mean': gyro_mean,
        'gyro_std': gyro_std,
        'gyro_bias': gyro_bias,
        'samples': len(gyro_z)
    }


def analyze_wheel_noise(wheel_file):
    """
    Analyze wheel encoder noise characteristics.

    Returns:
        dict: {
            'process_noise_x': recommended translation noise,
            'process_noise_y': recommended translation noise,
            'process_noise_theta': recommended rotation noise (from wheels)
        }
    """
    print("\n=== Wheel Odometry Analysis ===")

    # Load data
    df = pd.read_csv(wheel_file)

    # Skip first few samples (might have zero velocities)
    df = df[df['v_linear'].abs() > 0.001]

    if len(df) < 10:
        print("  ERROR: Not enough wheel data with motion!")
        return None

    timestamps = df['timestamp'].values
    v_linear = df['v_linear'].values
    v_angular = df['v_angular'].values

    # Compute time deltas
    dt = np.diff(timestamps)
    dt = dt[dt > 0]  # Filter out zero dt

    if len(dt) == 0:
        print("  ERROR: No valid time deltas!")
        return None

    # Compute incremental motion
    delta_x = v_linear[:-1] * dt
    delta_y = np.zeros_like(delta_x)  # Assume straight motion for now
    delta_theta = v_angular[:-1] * dt

    # Calculate standard deviations
    std_x = np.std(delta_x)
    std_y = np.std(delta_y) if np.std(delta_y) > 0 else std_x  # Use std_x if no lateral motion
    std_theta = np.std(delta_theta)

    # Also analyze velocity variance
    std_v_linear = np.std(v_linear)
    std_v_angular = np.std(v_angular)

    print(f"  Linear velocity std: {std_v_linear:.6f} m/s")
    print(f"  Angular velocity std: {std_v_angular:.6f} rad/s")
    print(f"  Incremental x std: {std_x:.6f} m")
    print(f"  Incremental theta std: {std_theta:.6f} rad")
    print(f"  Samples: {len(dt)}")

    # Process noise should be scaled by typical dt
    mean_dt = np.mean(dt)
    print(f"  Mean dt: {mean_dt:.4f} s")

    # Recommended values (conservative estimates)
    # Process noise is variance per unit time
    process_noise_x = std_v_linear * np.sqrt(mean_dt)
    process_noise_y = process_noise_x  # Assume symmetric
    process_noise_theta = std_v_angular * np.sqrt(mean_dt)

    return {
        'process_noise_x': process_noise_x,
        'process_noise_y': process_noise_y,
        'process_noise_theta': process_noise_theta,
        'std_v_linear': std_v_linear,
        'std_v_angular': std_v_angular,
        'samples': len(dt)
    }


def compare_rotation_sources(imu_results, wheel_results):
    """
    Compare rotation estimates from IMU vs wheels.

    Returns recommended IMU vs wheel weighting.
    """
    print("\n=== Rotation Source Comparison ===")

    imu_noise = imu_results['gyro_std']
    wheel_noise = wheel_results['process_noise_theta']

    # Ratio tells us how much noisier wheels are than IMU
    noise_ratio = wheel_noise / imu_noise if imu_noise > 0 else 1.0

    print(f"  IMU gyro noise: {imu_noise:.6f} rad/s")
    print(f"  Wheel rotation noise: {wheel_noise:.6f} rad")
    print(f"  Noise ratio (wheel/IMU): {noise_ratio:.2f}x")

    if noise_ratio > 2.0:
        print(f"  → Wheels are MUCH noisier for rotation")
        print(f"  → EKF should trust IMU more for rotation")
        recommendation = "HIGH"
    elif noise_ratio > 1.2:
        print(f"  → Wheels are somewhat noisier for rotation")
        recommendation = "MODERATE"
    else:
        print(f"  → Wheel and IMU rotation noise comparable")
        recommendation = "LOW"

    return {
        'noise_ratio': noise_ratio,
        'recommendation': recommendation
    }


def generate_recommendations(imu_results, wheel_results, comparison):
    """
    Generate final EKF parameter recommendations.
    """
    print("\n" + "="*60)
    print("RECOMMENDED EKF PARAMETERS")
    print("="*60)

    # IMU measurement noise (use measured std)
    imu_noise_theta = max(imu_results['gyro_std'], 0.001)  # Minimum 0.001

    # Process noise from wheels
    process_noise_x = max(wheel_results['process_noise_x'], 0.001)
    process_noise_y = max(wheel_results['process_noise_y'], 0.001)

    # Rotation process noise - adjust based on comparison
    process_noise_theta_base = max(wheel_results['process_noise_theta'], 0.001)

    # Scale rotation noise based on comparison with IMU
    if comparison['recommendation'] == "HIGH":
        # Wheels much noisier - increase process noise for rotation
        process_noise_theta = process_noise_theta_base * 2.0
        print("\n  Strategy: HIGH rotation process noise (trust IMU over wheels)")
    elif comparison['recommendation'] == "MODERATE":
        process_noise_theta = process_noise_theta_base * 1.5
        print("\n  Strategy: MODERATE rotation process noise")
    else:
        process_noise_theta = process_noise_theta_base
        print("\n  Strategy: BALANCED rotation process noise")

    # Ensure rotation noise is at least as high as translation
    process_noise_theta = max(process_noise_theta, process_noise_x * 1.5)

    print(f"\n  process_noise_x: {process_noise_x:.6f}")
    print(f"  process_noise_y: {process_noise_y:.6f}")
    print(f"  process_noise_theta: {process_noise_theta:.6f}")
    print(f"  imu_noise_theta: {imu_noise_theta:.6f}")

    print("\n  Ratio (theta/translation): {:.2f}x".format(
        process_noise_theta / process_noise_x
    ))

    print("\n" + "="*60)
    print("COPY THESE VALUES TO ekf_params.yaml:")
    print("="*60)
    print(f"process_noise_x: {process_noise_x:.6f}      # Empirically measured")
    print(f"process_noise_y: {process_noise_y:.6f}      # Empirically measured")
    print(f"process_noise_theta: {process_noise_theta:.6f}  # Empirically measured ({comparison['noise_ratio']:.1f}x translation)")
    print(f"imu_noise_theta: {imu_noise_theta:.6f}      # Empirically measured")
    print("="*60)

    return {
        'process_noise_x': process_noise_x,
        'process_noise_y': process_noise_y,
        'process_noise_theta': process_noise_theta,
        'imu_noise_theta': imu_noise_theta
    }


def main():
    parser = argparse.ArgumentParser(description='Analyze sensor noise for EKF tuning')
    parser.add_argument('--imu', default='/tmp/imu_noise.csv', help='IMU data CSV file')
    parser.add_argument('--wheel', default='/tmp/wheel_noise.csv', help='Wheel data CSV file')
    args = parser.parse_args()

    # Check files exist
    if not os.path.exists(args.imu):
        print(f"ERROR: IMU file not found: {args.imu}")
        print("Run: ros2 run banana_odom sensor_noise_analyzer")
        return 1

    if not os.path.exists(args.wheel):
        print(f"ERROR: Wheel file not found: {args.wheel}")
        print("Run: ros2 run banana_odom sensor_noise_analyzer")
        return 1

    print("Sensor Noise Analysis")
    print(f"  IMU data: {args.imu}")
    print(f"  Wheel data: {args.wheel}")

    # Analyze IMU
    imu_results = analyze_imu_noise(args.imu)

    # Analyze wheels
    wheel_results = analyze_wheel_noise(args.wheel)
    if wheel_results is None:
        return 1

    # Compare rotation sources
    comparison = compare_rotation_sources(imu_results, wheel_results)

    # Generate recommendations
    recommendations = generate_recommendations(imu_results, wheel_results, comparison)

    return 0


if __name__ == '__main__':
    exit(main())
