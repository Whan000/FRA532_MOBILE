#!/usr/bin/env python3
import sys
import pandas as pd
import numpy as np
import json
import argparse

def calculate_smoothness(trajectory):
    """Calculate smoothness (jerk cost)."""
    # Use acceleration changes (jerk)
    # v = diff(x)/dt
    # a = diff(v)/dt
    # j = diff(a)/dt
    # Sum of j^2
    
    # We have v, w in log.
    v = trajectory['v'].values
    w = trajectory['w'].values
    dt = np.diff(trajectory['timestamp'].values)
    
    # Avoid div by zero
    dt[dt < 1e-6] = 1e-6
    
    a_v = np.diff(v) / dt
    j_v = np.diff(a_v) / dt[:-1]
    
    cost_v = np.mean(j_v**2)
    
    a_w = np.diff(w) / dt
    j_w = np.diff(a_w) / dt[:-1]
    
    cost_w = np.mean(j_w**2)
    
    return cost_v + cost_w

def calculate_consistency(df):
    """Calculate mean Euclidean distance between trajectories."""
    # Resample all to same timestamps?
    # Or just use nearest neighbor interpolation.
    
    sources = df['source'].unique()
    wheel = df[df['source'] == 'wheel']
    icp = df[df['source'] == 'icp']
    ekf = df[df['source'] == 'ekf']
    
    if len(wheel) == 0 or len(icp) == 0 or len(ekf) == 0:
        return {}

    # Pivot to timestamp index
    # But timestamps differ slightly.
    # Interpolate to EKF timestamps.
    
    ekf_t = ekf['timestamp'].values
    wheel_t = wheel['timestamp'].values
    icp_t = icp['timestamp'].values
    
    # Interpolate Wheel to EKF time
    wheel_x = np.interp(ekf_t, wheel_t, wheel['x'])
    wheel_y = np.interp(ekf_t, wheel_t, wheel['y'])
    
    # Interpolate ICP to EKF time
    icp_x = np.interp(ekf_t, icp_t, icp['x'])
    icp_y = np.interp(ekf_t, icp_t, icp['y'])
    
    dist_ekf_wheel = np.mean(np.sqrt((ekf['x'] - wheel_x)**2 + (ekf['y'] - wheel_y)**2))
    dist_ekf_icp = np.mean(np.sqrt((ekf['x'] - icp_x)**2 + (ekf['y'] - icp_y)**2))
    
    return {
        'dist_ekf_wheel': dist_ekf_wheel,
        'dist_ekf_icp': dist_ekf_icp
    }

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', help='Log CSV file')
    args = parser.parse_args()
    
    try:
        df = pd.read_csv(args.file)
    except Exception as e:
        print(json.dumps({'error': str(e)}))
        return

    ekf = df[df['source'] == 'ekf']
    if len(ekf) < 10:
        print(json.dumps({'error': 'Not enough data'}))
        return
        
    smoothness = calculate_smoothness(ekf)
    consistency = calculate_consistency(df)
    
    result = {
        'smoothness': smoothness,
        'consistency': consistency
    }
    
    print(json.dumps(result))

if __name__ == '__main__':
    main()
