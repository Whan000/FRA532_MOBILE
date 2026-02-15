
import sqlite3
import numpy as np
import matplotlib
matplotlib.use('Agg') # Save to file
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import scipy.stats as stats
import os

# Configuration
BAG_FILE = '/home/zenter/Desktop/mobile/FRA532_LAB/FRA532_LAB1_DATASET/fibo_floor3_seq00/fibo_floor3_seq00_0.db3'
TARGET_ANGLE_DEG = 30.0
DURATION_SEC = 5.0
OUTPUT_FILE = '/home/zenter/Desktop/lidar_noise_distribution.png'

print(f"Opening bag: {BAG_FILE}")
conn = sqlite3.connect(BAG_FILE)
c = conn.cursor()

# Get Scan Topic ID
row = c.execute("SELECT id FROM topics WHERE name='/scan'").fetchone()
if not row:
    print("Error: No /scan topic found!")
    exit(1)
topic_id = row[0]

# Get first scan to determine index
row = c.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={topic_id} ORDER BY timestamp ASC LIMIT 1").fetchone()
first_ts = row[0]
first_msg = deserialize_message(row[1], LaserScan)

# Calculate Index for Target Angle
# angle = min + index * inc => index = (angle - min) / inc
target_angle_rad = np.radians(TARGET_ANGLE_DEG)
angle_idx = int((target_angle_rad - first_msg.angle_min) / first_msg.angle_increment)

# Validate Index
if angle_idx < 0 or angle_idx >= len(first_msg.ranges):
    print(f"Error: 45 degrees is out of bounds (Index {angle_idx}, Len {len(first_msg.ranges)})")
    # Fallback to center ray
    angle_idx = len(first_msg.ranges) // 2
    print(f"Falling back to center ray: Index {angle_idx}")

print(f"Analyzing Beam at {TARGET_ANGLE_DEG} degrees (Index {angle_idx})")

# Fetch 3 Seconds of Data
end_ts = first_ts + int(DURATION_SEC * 1e9)
rows = c.execute(f"SELECT data FROM messages WHERE topic_id={topic_id} AND timestamp >= {first_ts} AND timestamp <= {end_ts}").fetchall()

ranges = []
for r in rows:
    msg = deserialize_message(r[0], LaserScan)
    val = msg.ranges[angle_idx]
    
    # Filter invalid readings
    if msg.range_min < val < msg.range_max:
        ranges.append(val)

ranges = np.array(ranges)
count = len(ranges)

if count < 10:
    print("Error: Not enough valid data points found.")
    exit(1)

# Statistics
mean_val = np.mean(ranges)
std_dev = np.std(ranges)
variance_val = np.var(ranges)

print(f"\nResults over {DURATION_SEC} seconds ({count} samples):")
print(f"  Mean Range: {mean_val:.6f} m")
print(f"  Std Dev:    {std_dev:.6f} m")
print(f"  Variance:   {variance_val:.8f} m^2")

# Identify outliers (beyond 3 sigma) for cleaner plot
clean_ranges = ranges[abs(ranges - mean_val) < 3 * std_dev]
clean_mean = np.mean(clean_ranges)
clean_std = np.std(clean_ranges)

# Plotting
plt.figure(figsize=(10, 6))

# Histogram
count, bins, ignored = plt.hist(clean_ranges, bins=30, density=True, alpha=0.6, color='#00AAFF', edgecolor='black', label='Measured Data')

# Gaussian Fit
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = stats.norm.pdf(x, clean_mean, clean_std)
plt.plot(x, p, 'r', linewidth=2, label=f'Gaussian Fit\n$\sigma={clean_std:.4f}m$')

plt.title(f'Lidar Noise Distribution @ {TARGET_ANGLE_DEG}° (Stationary)', fontsize=14)
plt.xlabel('Measured Range (m)', fontsize=12)
plt.ylabel('Density', fontsize=12)
plt.legend()
plt.grid(True, alpha=0.3)

plt.savefig(OUTPUT_FILE)
print(f"\nPlot saved to: {OUTPUT_FILE}")

conn.close()
