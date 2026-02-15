#!/usr/bin/env python3
"""
Analyze LIDAR noise distribution from ROS 2 bag file.
Extracts range measurements at 90 degrees and plots Gaussian distribution.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    print(f"Error: {e}")
    sys.exit(1)

def read_lidar_data(bag_path, angle_target=90.0, angle_tolerance=5.0, start_time=0.0, end_time=7.0):
    """
    Extract LIDAR range measurements at a specific angle.

    Args:
        bag_path: Path to bag file
        angle_target: Target angle in degrees (0-360)
        angle_tolerance: Tolerance in degrees
        start_time: Start time in seconds
        end_time: End time in seconds

    Returns:
        numpy array of range measurements at target angle
    """
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    lidar_samples = []
    start_stamp = None
    message_count = 0

    print(f"🔄 Reading LIDAR data (target angle: {angle_target}° ± {angle_tolerance}°)...")

    while reader.has_next():
        try:
            topic, msg, timestamp = reader.read_next()
            message_count += 1

            if start_stamp is None:
                start_stamp = timestamp

            elapsed = (timestamp - start_stamp) / 1e9

            if elapsed > end_time:
                print(f"Reached end time ({end_time}s)")
                break

            if elapsed < start_time:
                continue

            # Process LaserScan
            if 'scan' in topic.lower() or 'lidar' in topic.lower():
                try:
                    msg_type = get_message(type_map[topic])
                    msg_obj = deserialize_message(msg, msg_type)

                    if hasattr(msg_obj, 'ranges') and len(msg_obj.ranges) > 0:
                        ranges = np.array(msg_obj.ranges)
                        angle_min = msg_obj.angle_min  # radians
                        angle_increment = msg_obj.angle_increment  # radians

                        # Calculate angles for each range measurement
                        angles = angle_min + np.arange(len(ranges)) * angle_increment
                        angles_deg = np.degrees(angles)

                        # Find indices near target angle
                        # Handle wraparound (e.g., 350° is close to 10°)
                        angle_diff = np.abs(angles_deg - angle_target)
                        angle_diff = np.minimum(angle_diff, 360 - angle_diff)

                        near_target = angle_diff <= angle_tolerance
                        target_ranges = ranges[near_target]

                        # Filter valid measurements (non-zero, within range)
                        valid_ranges = target_ranges[
                            (target_ranges > msg_obj.range_min) &
                            (target_ranges < msg_obj.range_max)
                        ]

                        if len(valid_ranges) > 0:
                            # Take average of measurements near target angle
                            avg_range = np.mean(valid_ranges)
                            lidar_samples.append(avg_range)

                            if len(lidar_samples) <= 5 or len(lidar_samples) % 50 == 0:
                                print(f"  [{elapsed:.3f}s] LIDAR @{angle_target}°: {avg_range:.4f}m (n={len(valid_ranges)} samples)")

                except Exception as e:
                    pass

        except Exception as e:
            print(f"Error reading message: {e}")
            break

    print(f"\n✅ Total messages processed: {message_count}")
    print(f"   LIDAR samples collected: {len(lidar_samples)}")

    return np.array(lidar_samples)

def plot_lidar_distribution(lidar_data, angle_target=90.0):
    """Plot LIDAR range measurement Gaussian distribution."""
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f'LIDAR Range Gaussian Distribution at {angle_target}° (0-7s window)',
                 fontsize=14, fontweight='bold')

    gs = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.3)

    # Plot 1: Histogram
    ax1 = fig.add_subplot(gs[0, 0])
    if len(lidar_data) > 5:
        mu = np.mean(lidar_data)
        sigma = np.std(lidar_data)

        ax1.hist(lidar_data, bins=40, density=True, alpha=0.7, color='purple',
                edgecolor='black', label='Data')

        x = np.linspace(lidar_data.min(), lidar_data.max(), 200)
        y = norm.pdf(x, mu, sigma)
        ax1.plot(x, y, 'purple', linewidth=2.5, label='Fitted Gaussian')

        ax1.axvline(mu, color='red', linestyle='--', linewidth=2)
        ax1.axvline(mu + sigma, color='orange', linestyle=':', linewidth=1.5)
        ax1.axvline(mu - sigma, color='orange', linestyle=':', linewidth=1.5)

        ax1.set_xlabel('Range (m)', fontsize=11)
        ax1.set_ylabel('Frequency', fontsize=11)
        ax1.set_title(f'LIDAR Range Distribution (n={len(lidar_data)})', fontsize=12, fontweight='bold')
        ax1.text(0.98, 0.95, f'μ={mu:.4f}m\nσ={sigma:.4f}m',
                transform=ax1.transAxes, fontsize=10, verticalalignment='top',
                horizontalalignment='right', bbox=dict(boxstyle='round',
                facecolor='wheat', alpha=0.8))
        ax1.legend()
        ax1.grid(alpha=0.3)

    # Plot 2: Gaussian Fit
    ax2 = fig.add_subplot(gs[0, 1])
    if len(lidar_data) > 5:
        mu = np.mean(lidar_data)
        sigma = np.std(lidar_data)

        ax2.hist(lidar_data, bins=40, density=True, alpha=0.7, color='purple',
                edgecolor='black', label='Data')

        x = np.linspace(mu - 4*sigma, mu + 4*sigma, 300)
        y = norm.pdf(x, mu, sigma)
        ax2.plot(x, y, 'purple', linewidth=3, label='Fitted Gaussian')

        ax2.set_xlabel('Range (m)', fontsize=11)
        ax2.set_ylabel('Probability Density', fontsize=11)
        ax2.set_title('LIDAR Range - Gaussian Fit', fontsize=12, fontweight='bold')
        ax2.text(0.98, 0.95,
                f'μ = {mu:.009f} m\nσ = {sigma:.009f} m',
                transform=ax2.transAxes, fontsize=10, verticalalignment='top',
                horizontalalignment='right', bbox=dict(boxstyle='round',
                facecolor='wheat', alpha=0.8))
        ax2.legend()
        ax2.grid(alpha=0.3)

    # Plot 3: Time series
    ax3 = fig.add_subplot(gs[1, :])
    if len(lidar_data) > 1:
        time = np.arange(len(lidar_data)) * 0.05  # Assume 20 Hz
        ax3.plot(time, lidar_data, 'o-', color='purple', markersize=4, linewidth=1, alpha=0.7)
        ax3.axhline(np.mean(lidar_data), color='red', linestyle='--', linewidth=2, label='Mean')
        ax3.axhline(np.mean(lidar_data) + np.std(lidar_data), color='orange', linestyle=':', linewidth=1.5, label='±1σ')
        ax3.axhline(np.mean(lidar_data) - np.std(lidar_data), color='orange', linestyle=':', linewidth=1.5)

        ax3.set_xlabel('Time (s)', fontsize=11)
        ax3.set_ylabel('Range (m)', fontsize=11)
        ax3.set_title('LIDAR Range Measurements Over Time', fontsize=12, fontweight='bold')
        ax3.legend()
        ax3.grid(alpha=0.3)

    output_file = "/home/zenter/Desktop/github_clone/FRA532_MOBILE/lidar_noise_distribution.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✅ Plot saved: {output_file}")
    plt.show()

if __name__ == '__main__':
    bag_path = "/home/zenter/Desktop/github_clone/FRA532_MOBILE/FRA532_LAB/FRA532_LAB1_DATASET/fibo_floor3_seq01/fibo_floor3_seq01_0.db3"

    print("=" * 60)
    print("🔍 LIDAR Noise Analysis Tool")
    print("=" * 60)

    # Analyze LIDAR at 90 degrees
    lidar_data = read_lidar_data(bag_path, angle_target=90.0, angle_tolerance=5.0,
                                 start_time=0.0, end_time=7.0)

    if len(lidar_data) > 5:
        mu = np.mean(lidar_data)
        sigma = np.std(lidar_data)

        print(f"\n📊 LIDAR Range Statistics (90°):")
        print(f"   μ = {mu:.009f} m")
        print(f"   σ = {sigma:.009f} m")
        print(f"   Min: {np.min(lidar_data):.009f} m")
        print(f"   Max: {np.max(lidar_data):.009f} m")
        print(f"   Variance: {np.var(lidar_data):.009f}")
        print(f"   Samples: {len(lidar_data)}")

        # Normality test (Jarque-Bera)
        from scipy.stats import jarque_bera
        stat, p_value = jarque_bera(lidar_data)
        print(f"\n🔬 Gaussian Distribution Test:")
        print(f"   Jarque-Bera statistic: {stat:.6f}")
        print(f"   p-value: {p_value:.6f}")
        if p_value > 0.05:
            print(f"   ✅ Data appears normally distributed (p > 0.05)")
        else:
            print(f"   ⚠️  Data may NOT be normally distributed (p < 0.05)")

        print(f"\n📈 Plotting...")
        plot_lidar_distribution(lidar_data, angle_target=90.0)
    else:
        print("\n⚠️  Not enough LIDAR samples collected")

    print("\n" + "=" * 60)
