
import sqlite3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, Imu
import scipy.stats as stats
import math

# Configuration
BAG_FILE = '/home/zenter/Desktop/mobile/FRA532_LAB/FRA532_LAB1_DATASET/fibo_floor3_seq00/fibo_floor3_seq00_0.db3'
IMU_PLOT_FILE = '/home/zenter/Desktop/imu_noise_distribution.png'
WHEEL_PLOT_FILE = '/home/zenter/Desktop/wheel_noise_distribution.png'

print(f"Opening bag: {BAG_FILE}")
conn = sqlite3.connect(BAG_FILE)
c = conn.cursor()

def get_yaw(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

# ==========================================
# 1. IMU INITIAL NOISE (Stationary 3s)
# ==========================================
print("\n--- Analzying IMU Gyro Noise (Stationary) ---")
imu_id = c.execute("SELECT id FROM topics WHERE name='/imu'").fetchone()[0]
rows = c.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={imu_id} ORDER BY timestamp ASC LIMIT 300").fetchall() # ~3s at 100hz maybe?

# Check actual time
start_t = rows[0][0]
end_t = start_t + 3 * 1e9
gyro_z_data = []

for ts, data in rows:
    if ts > end_t: break
    msg = deserialize_message(data, Imu)
    gyro_z_data.append(msg.angular_velocity.z)

gyro_z = np.array(gyro_z_data)
imu_mean = np.mean(gyro_z)
imu_std = np.std(gyro_z)
imu_var = np.var(gyro_z)

print(f"IMU Samples: {len(gyro_z)}")
print(f"Mean Bias:   {imu_mean:.6f} rad/s")
print(f"Std Dev:     {imu_std:.6f} rad/s")
print(f"Variance:    {imu_var:.8f} (rad/s)^2")

# Plot IMU
plt.figure(figsize=(10, 6))
count, bins, ignored = plt.hist(gyro_z, bins=30, density=True, alpha=0.6, color='#00FF00', edgecolor='black', label='Gyro Z Data')
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = stats.norm.pdf(x, imu_mean, imu_std)
plt.plot(x, p, 'r', linewidth=2, label=f'Gaussian Fit\n$\sigma={imu_std:.5f}$')
plt.title('IMU Gyro-Z Noise Distribution (Stationary)', fontsize=14)
plt.xlabel('Angular Velocity Z (rad/s)', fontsize=12)
plt.ylabel('Density', fontsize=12)
plt.legend()
plt.grid(True, alpha=0.3)
plt.savefig(IMU_PLOT_FILE)
print(f"Saved IMU plot to {IMU_PLOT_FILE}")


# ==========================================
# 2. WHEEL PROCESS NOISE
# ==========================================
print("\n--- Analzying Wheel Odometry Process Noise ---")
# Strategy: Compare Wheel Delta Theta vs IMU Delta Theta (Ground Truth attempt)
# Error = (Wheel_Delta - IMU_Delta)
joint_id = c.execute("SELECT id FROM topics WHERE name='/joint_states'").fetchone()[0]

# Get all data sorted
all_msgs = []
# IMU
r_imu = c.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={imu_id} ORDER BY timestamp ASC").fetchall()
# Unwrapped IMU Yaw for interpolation
unwrap_yaw = 0.0
last_raw = 0.0
imu_ts = []
imu_vals = []
for ts, data in r_imu:
    msg = deserialize_message(data, Imu)
    y = get_yaw(msg.orientation)
    diff = normalize_angle(y - last_raw)
    unwrap_yaw += diff
    last_raw = y
    imu_ts.append(ts)
    imu_vals.append(unwrap_yaw)

imu_ts = np.array(imu_ts)
imu_vals = np.array(imu_vals)

# Joints
r_joints = c.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={joint_id} ORDER BY timestamp ASC").fetchall()
wheel_errors = []

wheel_radius = 0.033
wheel_sep = 0.160

for i in range(1, len(r_joints)):
    t1, d1 = r_joints[i-1]
    t2, d2 = r_joints[i]
    
    # Delta time
    dt = (t2 - t1) / 1e9
    if dt < 0.001: continue
    
    m1 = deserialize_message(d1, JointState)
    m2 = deserialize_message(d2, JointState)
    
    dl = m2.position[0] - m1.position[0]
    dr = m2.position[1] - m1.position[1]
    
    # Wheel Delta Theta
    dist_l = dl * wheel_radius
    dist_r = dr * wheel_radius
    dtheta_wheel = (dist_r - dist_l) / wheel_sep
    
    if abs(dtheta_wheel) < 0.0001: continue # Skip stationary
    
    # Interpolate IMU
    y1 = np.interp(t1, imu_ts, imu_vals)
    y2 = np.interp(t2, imu_ts, imu_vals)
    dtheta_imu = y2 - y1
    
    # Error
    err = dtheta_wheel - dtheta_imu
    wheel_errors.append(err)

wheel_errors = np.array(wheel_errors)
# Filter outliers (3 sigma)
mean_err = np.mean(wheel_errors)
std_err = np.std(wheel_errors)
clean_errors = wheel_errors[abs(wheel_errors - mean_err) < 3 * std_err]

clean_mean = np.mean(clean_errors)
clean_std = np.std(clean_errors)
clean_var = np.var(clean_errors)

print(f"Motion Samples: {len(wheel_errors)} (Filtered: {len(clean_errors)})")
print(f"Mean Error:     {clean_mean:.6f} rad (Systematic Slip/Bias)")
print(f"Std Dev:        {clean_std:.6f} rad")
print(f"Variance:       {clean_var:.9f} (rad)^2")

# Plot Wheel
plt.figure(figsize=(10, 6))
count, bins, ignored = plt.hist(clean_errors, bins=50, density=True, alpha=0.6, color='#FF8800', edgecolor='black', label='Odometry Error')
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = stats.norm.pdf(x, clean_mean, clean_std)
plt.plot(x, p, 'r', linewidth=2, label=f'Gaussian Fit\n$\sigma={clean_std:.5f}$')
plt.title('Wheel Odometry Process Noise Distribution', fontsize=14)
plt.xlabel('Error: Wheel Delta - IMU Delta (rad)', fontsize=12)
plt.ylabel('Density', fontsize=12)
plt.legend()
plt.grid(True, alpha=0.3)
plt.savefig(WHEEL_PLOT_FILE)
print(f"Saved Wheel plot to {WHEEL_PLOT_FILE}")

conn.close()
