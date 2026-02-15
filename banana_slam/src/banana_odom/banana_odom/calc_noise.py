
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, Imu
import math

def get_yaw(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

db_path = '/home/zenter/Desktop/mobile/FRA532_LAB/FRA532_LAB1_DATASET/fibo_floor3_seq00/fibo_floor3_seq00_0.db3'
conn = sqlite3.connect(db_path)
c = conn.cursor()

# 1. Get IMU Noise (from stationary period at start)
print("=== 1. Calculating IMU Measurement Noise (R) ===")
# We assume the robot is stationary for the first 5 seconds
imu_topic_id = c.execute("SELECT id FROM topics WHERE name='/imu'").fetchone()[0]
rows = c.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={imu_topic_id} AND timestamp < (SELECT MIN(timestamp) FROM messages) + 5000000000").fetchall()

gyro_z_values = []
for ts, data in rows:
    msg = deserialize_message(data, Imu)
    gyro_z_values.append(msg.angular_velocity.z)

gyro_var = np.var(gyro_z_values)
print(f"Num samples: {len(gyro_z_values)}")
print(f"Gyro Z Mean: {np.mean(gyro_z_values):.6f} rad/s (Bias)")
print(f"Gyro Z Variance: {gyro_var:.8f} (rad/s)^2")
print(f"-> Recommended 'imu_noise_theta': {gyro_var:.6f}")


# 2. Get Wheel Odometry Process Noise (Q)
print("\n=== 2. Calculating Wheel Odometry Process Noise (Q) ===")
# We compare Wheel Delta Theta vs IMU Delta Theta over the whole trajectory
# This assumes IMU is 'truth' for rotation

joint_topic_id = c.execute("SELECT id FROM topics WHERE name='/joint_states'").fetchone()[0]
wheel_radius = 0.033
wheel_separation = 0.160

# Get all messages sorted by time
msgs = []
# Get IMU
rows_imu = c.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={imu_topic_id} ORDER BY timestamp ASC").fetchall()
for ts, data in rows_imu:
    msgs.append({'t': ts, 'type': 'imu', 'msg': deserialize_message(data, Imu)})

# Get Joints
rows_joints = c.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={joint_topic_id} ORDER BY timestamp ASC").fetchall()
for ts, data in rows_joints:
    msgs.append({'t': ts, 'type': 'joint', 'msg': deserialize_message(data, JointState)})

# Sort by timestamp
msgs.sort(key=lambda x: x['t'])

# Synchronization variables
prev_imu_yaw = None
prev_wheel_pos = None
errors = []

for m in msgs:
    if m['type'] == 'imu':
        yaw = get_yaw(m['msg'].orientation)
        if prev_imu_yaw is None:
            prev_imu_yaw = yaw
            continue
            
        # We need corresponding wheel data... this simple loop implies strict sequencing which isn't guaranteed
        # Instead, let's accumulate IMU rotation between wheel updates
        pass

# Improved approach: Iterate through Wheel messages, integrate IMU in between
print("Scanning trajectory...")
wheel_errors = []
prev_joint_msg = None
last_imu_yaw = None
imu_yaw_at_last_wheel = None
current_imu_yaw = 0.0 # unwrapped

# First, unwrap all IMU data to get continuous yaw
imu_data = []
raw_imu_rows = c.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={imu_topic_id} ORDER BY timestamp ASC").fetchall()
unwrap_yaw = 0.0
last_raw_yaw = 0.0
for ts, data in raw_imu_rows:
    msg = deserialize_message(data, Imu)
    y = get_yaw(msg.orientation)
    # unwrap
    diff = y - last_raw_yaw
    if diff > math.pi: diff -= 2*math.pi
    if diff < -math.pi: diff += 2*math.pi
    unwrap_yaw += diff
    last_raw_yaw = y
    imu_data.append((ts, unwrap_yaw))

# Convert to numpy for fast interpolation
imu_ts = np.array([x[0] for x in imu_data])
imu_yaws = np.array([x[1] for x in imu_data])

# Process joints
joint_rows = c.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={joint_topic_id} ORDER BY timestamp ASC").fetchall()

for i in range(1, len(joint_rows)):
    t1, d1 = joint_rows[i-1]
    t2, d2 = joint_rows[i]
    
    m1 = deserialize_message(d1, JointState)
    m2 = deserialize_message(d2, JointState)
    
    # Calculate Wheel Delta
    dl = m2.position[0] - m1.position[0]
    dr = m2.position[1] - m1.position[1]
    
    dist_l = dl * wheel_radius
    dist_r = dr * wheel_radius
    
    wheel_dtheta = (dist_r - dist_l) / wheel_separation
    
    # Get IMU Delta (interpolate)
    # Find IMU yaw at t1 and t2
    yaw1 = np.interp(t1, imu_ts, imu_yaws)
    yaw2 = np.interp(t2, imu_ts, imu_yaws)
    imu_dtheta = yaw2 - yaw1
    
    if abs(wheel_dtheta) > 0.001: # Filter out stationary moments where error is dominated by sensor noise
        error = normalize_angle(wheel_dtheta - imu_dtheta)
        wheel_errors.append(error)

if len(wheel_errors) > 0:
    process_var = np.var(wheel_errors)
    print(f"Num movement samples: {len(wheel_errors)}")
    print(f"Wheel Theta Variance: {process_var:.8f} (rad)^2")
    print(f"-> Recommended 'process_noise_theta': {process_var:.6f}")
else:
    print("Not enough movement data found.")

conn.close()
