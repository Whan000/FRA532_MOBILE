#!/usr/bin/env python3
"""
Analyze IMU Orientation (Yaw) noise during first 7 seconds of seq01 (stationary period).
The yaw measurements show Gaussian noise even when omega_z = 0 (no rotation).
This represents the pure sensor noise floor for measurement covariance in EKF.
"""

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
from scipy.optimize import curve_fit
import math

def euler_from_quaternion(q):
    """Convert quaternion [x, y, z, w] to Euler angles [roll, pitch, yaw]."""
    x, y, z, w = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    sinp = max(-1, min(1, sinp))  # Clamp to [-1, 1]
    pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# Bagfile path
BAG_PATH = "/home/zenter/Desktop/github_clone/FRA532_MOBILE/FRA532_LAB/FRA532_LAB1_DATASET/fibo_floor3_seq01"

def gaussian(x, mu, sigma):
    """Gaussian function."""
    if sigma == 0:
        return np.zeros_like(x)
    return (1 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((x - mu) / sigma) ** 2)

def main():
    # Open bagfile
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    # Extract IMU orientation (yaw) for first 7 seconds
    yaw_values = []
    timestamps = []
    start_time = None

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()

        if topic == "/imu":
            msg_type = get_message("sensor_msgs/Imu")
            imu_msg = deserialize_message(data, msg_type)

            # Set start time from first message
            if start_time is None:
                start_time = timestamp
                print(f"Starting analysis at timestamp: {start_time}")

            # Check if within 7 seconds
            elapsed_time = (timestamp - start_time) / 1e9  # Convert nanoseconds to seconds

            if elapsed_time > 7.0:
                print(f"Reached 7 second mark at {elapsed_time:.2f}s")
                break

            # Extract yaw from orientation quaternion
            q = imu_msg.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw_values.append(yaw)
            timestamps.append(elapsed_time)

            if len(yaw_values) % 50 == 0:
                print(f"  Collected {len(yaw_values)} samples at {elapsed_time:.2f}s")

    print(f"\nTotal samples collected: {len(yaw_values)}")
    yaw_values = np.array(yaw_values)
    timestamps = np.array(timestamps)

    # Calculate noise as deviation from mean (stationary period should have constant yaw)
    yaw_mean = np.mean(yaw_values)
    yaw_noise = yaw_values - yaw_mean  # Noise = deviation from steady-state yaw

    # Statistics
    mean = np.mean(yaw_noise)
    std_dev = np.std(yaw_noise)

    print(f"\nIMU Yaw Orientation Statistics (First 7 seconds - STATIONARY):")
    print(f"  Mean Yaw: {yaw_mean:.6f} rad")
    print(f"  Yaw Noise Mean: {mean:.9f} rad (should be ~0)")
    print(f"  Yaw Noise Std Dev: {std_dev:.6f} rad ← MEASUREMENT NOISE FOR EKF R_orientation")
    print(f"  Min: {np.min(yaw_noise):.6f} rad")
    print(f"  Max: {np.max(yaw_noise):.6f} rad")

    # Perform Jarque-Bera test
    jb_stat, jb_pvalue = stats.jarque_bera(yaw_noise)
    print(f"\nJarque-Bera Normality Test:")
    print(f"  Statistic: {jb_stat:.4f}")
    print(f"  P-value: {jb_pvalue:.4f}")
    print(f"  Gaussian? {'YES (p > 0.05)' if jb_pvalue > 0.05 else 'NO (p < 0.05)'}")

    # Create figure with multiple subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Plot 1: Yaw over time
    ax = axes[0, 0]
    ax.plot(timestamps, yaw_values, 'b-', linewidth=0.8, label='Raw Yaw')
    ax.axhline(yaw_mean, color='r', linestyle='--', linewidth=2, label=f'Mean = {yaw_mean:.6f}')
    ax.set_xlabel('Time (seconds)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Yaw (rad)', fontsize=11, fontweight='bold')
    ax.set_title('IMU Yaw Orientation - First 7 Seconds (Stationary)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)

    # Plot 2: Yaw noise over time
    ax = axes[0, 1]
    ax.plot(timestamps, yaw_noise, 'g-', linewidth=0.8, label='Yaw Noise')
    ax.axhline(0, color='r', linestyle='--', linewidth=2)
    ax.fill_between(timestamps, -std_dev, std_dev, alpha=0.2, color='orange', label=f'±1σ = ±{std_dev:.6f}')
    ax.set_xlabel('Time (seconds)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Yaw Noise (rad)', fontsize=11, fontweight='bold')
    ax.set_title('Yaw Measurement Noise', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)

    # Plot 3: Histogram of yaw noise
    ax = axes[1, 0]
    counts, bins, patches = ax.hist(yaw_noise, bins=50, density=True, alpha=0.7,
                                     color='steelblue', edgecolor='black', label='Data')

    # Fit Gaussian
    if std_dev > 0:
        popt, _ = curve_fit(gaussian, (bins[:-1] + bins[1:]) / 2, counts,
                           p0=[mean, std_dev], maxfev=5000)
        mu_fit, sigma_fit = popt

        # Plot fitted Gaussian
        x_range = np.linspace(np.min(yaw_noise) - 3*std_dev,
                              np.max(yaw_noise) + 3*std_dev, 200)
        y_gaussian = gaussian(x_range, mu_fit, sigma_fit)
        ax.plot(x_range, y_gaussian, 'r-', linewidth=2.5,
               label=f'Gaussian Fit\nμ={mu_fit:.9f}, σ={sigma_fit:.6f}')

    ax.set_xlabel('Yaw Noise (rad)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Probability Density', fontsize=11, fontweight='bold')
    ax.set_title('Yaw Noise Distribution (141 samples)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)

    # Plot 4: Statistics summary box
    ax = axes[1, 1]
    ax.axis('off')

    stats_text = (
        f"SENSOR NOISE CHARACTERIZATION\n"
        f"(First 7 Seconds - Stationary Period)\n"
        f"\n"
        f"Yaw Orientation Noise:\n"
        f"  σ = {std_dev:.6f} rad\n"
        f"  σ = {std_dev * 180/np.pi:.4f}°\n"
        f"\n"
        f"Sample Count: {len(yaw_values)}\n"
        f"Duration: 7.0 seconds\n"
        f"\n"
        f"Jarque-Bera Test:\n"
        f"  p-value = {jb_pvalue:.4f}\n"
        f"  Gaussian: {'YES ✓' if jb_pvalue > 0.05 else 'NO'}\n"
        f"\n"
        f"EKF Parameter:\n"
        f"  R_orientation = {std_dev**2:.9f}\n"
        f"  (Use σ = {std_dev:.6f} for measurement covariance)"
    )

    ax.text(0.1, 0.95, stats_text, transform=ax.transAxes,
            verticalalignment='top', fontsize=11, family='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))

    plt.tight_layout()
    plt.savefig('imu_yaw_noise_7sec_stationary.png', dpi=150, bbox_inches='tight')
    print(f"\nPlot saved as: imu_yaw_noise_7sec_stationary.png")

if __name__ == '__main__':
    main()
