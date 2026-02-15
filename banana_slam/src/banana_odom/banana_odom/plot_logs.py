#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt
import argparse

def main():
    parser = argparse.ArgumentParser(description='Plot Odometry Logs')
    parser.add_argument('file', help='Path to CSV log file')
    args = parser.parse_args()
    
    try:
        df = pd.read_csv(args.file)
    except Exception as e:
        print(f"Error reading {args.file}: {e}")
        return

    # Sources
    sources = df['source'].unique()
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # 1. Trajectory (X vs Y)
    ax = axes[0, 0]
    for source in sources:
        data = df[df['source'] == source]
        ax.plot(data['x'], data['y'], label=source, marker='o', markersize=2, alpha=0.6)
    ax.set_title('Trajectory (X vs Y)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')

    # 2. Linear Velocity
    ax = axes[0, 1]
    for source in sources:
        data = df[df['source'] == source]
        t = data['timestamp'] - data['timestamp'].iloc[0]
        ax.plot(t, data['v'], label=source, alpha=0.8)
    ax.set_title('Linear Velocity (v)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('v (m/s)')
    ax.legend()
    ax.grid(True)

    # 3. Angular Velocity
    ax = axes[1, 0]
    for source in sources:
        data = df[df['source'] == source]
        t = data['timestamp'] - data['timestamp'].iloc[0]
        ax.plot(t, data['w'], label=source, alpha=0.8)
    ax.set_title('Angular Velocity (w)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('w (rad/s)')
    ax.legend()
    ax.grid(True)

    # 4. Heading (Theta)
    ax = axes[1, 1]
    for source in sources:
        data = df[df['source'] == source]
        t = data['timestamp'] - data['timestamp'].iloc[0]
        ax.plot(t, data['theta'], label=source, alpha=0.8)
    ax.set_title('Heading (Theta)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Theta (rad)')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
