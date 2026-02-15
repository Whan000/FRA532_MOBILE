#!/usr/bin/env python3
"""
Analyze IMU Z (angular velocity) noise distribution during first 7 seconds of MOTION.
Skips the initial stationary period and captures a 7-second window with actual rotation.
Plots histogram with Gaussian fit to identify noise distribution.
"""

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
from scipy.optimize import curve_fit

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

    # First pass: find when motion starts
    print("First pass: Finding motion start...")
    imu_z_values_all = []
    timestamps_all = []
    start_time = None
    motion_start_idx = None

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()

        if topic == "/imu":
            msg_type = get_message("sensor_msgs/Imu")
            imu_msg = deserialize_message(data, msg_type)

            if start_time is None:
                start_time = timestamp

            omega_z = imu_msg.angular_velocity.z
            elapsed_time = (timestamp - start_time) / 1e9

            imu_z_values_all.append(omega_z)
            timestamps_all.append(elapsed_time)

            # Find when motion starts (first non-zero value)
            if motion_start_idx is None and omega_z != 0:
                motion_start_idx = len(imu_z_values_all) - 1
                motion_start_time = elapsed_time
                print(f"Motion starts at {motion_start_time:.2f}s (sample {motion_start_idx})")

    # Second pass: extract 7 seconds of motion data
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    imu_z_values = []
    start_time = None
    motion_window_start = None
    sample_count = 0

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()

        if topic == "/imu":
            msg_type = get_message("sensor_msgs/Imu")
            imu_msg = deserialize_message(data, msg_type)

            if start_time is None:
                start_time = timestamp

            omega_z = imu_msg.angular_velocity.z
            elapsed_time = (timestamp - start_time) / 1e9

            # Start collecting when we get non-zero values
            if motion_window_start is None and omega_z != 0:
                motion_window_start = elapsed_time
                print(f"\nCollecting 7-second motion window starting at {motion_window_start:.2f}s...")

            # Collect for 7 seconds after motion starts
            if motion_window_start is not None:
                motion_elapsed = elapsed_time - motion_window_start
                if motion_elapsed > 7.0:
                    print(f"Reached 7 second mark at {motion_elapsed:.2f}s")
                    break

                imu_z_values.append(omega_z)
                sample_count += 1

                if sample_count % 50 == 0:
                    print(f"  Collected {sample_count} samples at {motion_elapsed:.2f}s")

    print(f"\nTotal motion samples collected: {len(imu_z_values)}")
    imu_z_values = np.array(imu_z_values)

    # Calculate statistics
    mean = np.mean(imu_z_values)
    std_dev = np.std(imu_z_values)

    print(f"\nIMU Z Statistics (First 7 seconds of MOTION):")
    print(f"  Mean: {mean:.6f} rad/s")
    print(f"  Std Dev: {std_dev:.6f} rad/s")
    print(f"  Min: {np.min(imu_z_values):.6f} rad/s")
    print(f"  Max: {np.max(imu_z_values):.6f} rad/s")

    # Perform Jarque-Bera test
    jb_stat, jb_pvalue = stats.jarque_bera(imu_z_values)
    print(f"\nJarque-Bera Normality Test:")
    print(f"  Statistic: {jb_stat:.4f}")
    print(f"  P-value: {jb_pvalue:.4f}")
    print(f"  Gaussian? {'YES (p > 0.05)' if jb_pvalue > 0.05 else 'NO (p < 0.05)'}")

    # Create histogram with Gaussian fit
    fig, ax = plt.subplots(figsize=(12, 7))

    # Plot histogram
    counts, bins, patches = ax.hist(imu_z_values, bins=50, density=True, alpha=0.7,
                                     color='steelblue', edgecolor='black', label='Data')

    # Fit Gaussian
    if std_dev > 0:
        popt, _ = curve_fit(gaussian, (bins[:-1] + bins[1:]) / 2, counts,
                           p0=[mean, std_dev], maxfev=5000)
        mu_fit, sigma_fit = popt

        # Plot fitted Gaussian
        x_range = np.linspace(np.min(imu_z_values) - 3*std_dev,
                              np.max(imu_z_values) + 3*std_dev, 200)
        y_gaussian = gaussian(x_range, mu_fit, sigma_fit)
        ax.plot(x_range, y_gaussian, 'r-', linewidth=2.5,
               label=f'Gaussian Fit\nμ={mu_fit:.6f}, σ={sigma_fit:.6f}')

    # Formatting
    ax.set_xlabel('Angular Velocity Z (rad/s)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Probability Density', fontsize=12, fontweight='bold')
    ax.set_title('IMU Z Noise Distribution - First 7 Seconds of MOTION (seq01)',
                 fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(fontsize=11, loc='upper right')

    # Add statistics box
    stats_text = (f"Statistics:\\n"
                 f"N = {len(imu_z_values)}\\n"
                 f"μ = {mean:.6f}\\n"
                 f"σ = {std_dev:.6f}\\n"
                 f"JB p-value = {jb_pvalue:.4f}")
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
            verticalalignment='top', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
            family='monospace')

    plt.tight_layout()
    plt.savefig('imu_z_noise_7sec_motion.png', dpi=150, bbox_inches='tight')
    print(f"\nPlot saved as: imu_z_noise_7sec_motion.png")
    plt.show()

if __name__ == '__main__':
    main()
