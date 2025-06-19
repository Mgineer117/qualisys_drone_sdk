#!/usr/bin/env python3
"""
Angular Velocity Analysis Tool for Koopman CP Data

This script analyzes JSON trajectory data to extract and calculate angular velocity
from circular trajectory data in the koopman_cp dataset.
"""

import json
import math
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple, Optional


def load_json_data(file_path: str) -> Dict:
    """Load JSON data from file."""
    with open(file_path, 'r') as f:
        return json.load(f)


def extract_trajectory_data(data: Dict) -> Tuple[List[float], List[float], List[float]]:
    """Extract x, y, z coordinates from pose data."""
    poses = data.get('pose', [])
    x_coords = []
    y_coords = []
    z_coords = []
    
    targets = data.get('target', [])
    # identify the timestep where liftoff ends
    liftoff_end_index = 0
    for i, target in enumerate(targets):
        if isinstance(target, list) and len(target) >= 3:
            # Assuming the first target with z > 0.4 is the liftoff point
            if target != [0.0, 1.0, 0.0]:
                liftoff_end_index = i + 1
                break

    for pose in poses:
        if isinstance(pose, list) and len(pose) >= 3:
            x_coords.append(pose[0])
            y_coords.append(pose[1])
            z_coords.append(pose[2])
    
    return x_coords[liftoff_end_index:], y_coords[liftoff_end_index:], z_coords[liftoff_end_index:]


def calculate_angular_velocity_from_trajectory(x_coords: List[float], y_coords: List[float], 
                                             times: List[float]) -> Tuple[float, float]:
    """
    Calculate angular velocity from trajectory data.
    
    Args:
        x_coords: X coordinates of trajectory
        y_coords: Y coordinates of trajectory  
        dt: Time step between measurements (default: 0.1s)
    
    Returns:
        Tuple of (angular_velocity_rad_per_sec, angular_velocity_deg_per_sec)
    """
    if len(x_coords) < 2 or len(y_coords) < 2:
        return 0.0, 0.0
    
    # Calculate angles for each point
    angles = []
    for x, y in zip(x_coords, y_coords):
        angle = math.atan2(y, x)
        angles.append(angle)
    
    # Calculate angular differences
    angular_velocities = []
    for i in range(1, len(angles)):
        # Handle angle wraparound
        angle_diff = angles[i] - angles[i-1]
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        angular_velocity = angle_diff / (times[i] - times[i-1]) 
        angular_velocities.append(angular_velocity)
    
    # Calculate average angular velocity
    avg_angular_velocity_rad = np.mean(angular_velocities) if angular_velocities else 0.0
    avg_angular_velocity_deg = math.degrees(avg_angular_velocity_rad)
    
    return avg_angular_velocity_rad, avg_angular_velocity_deg


def analyze_circle_around_xy_file(file_path: str) -> Dict:
    """Analyze a single JSON file from circle_around_xy_0.1 directory."""
    data = load_json_data(file_path)
    
    # Extract metadata if available
    radius = data.get('radius', 'N/A')
    angular_speed = data.get('angular_speed', 'N/A')
    times = data.get('time', [])

    # Extract trajectory data
    x_coords, y_coords, z_coords = extract_trajectory_data(data)
    
    # Calculate angular velocity from trajectory
    calc_angular_vel_rad, calc_angular_vel_deg = calculate_angular_velocity_from_trajectory(
        x_coords, y_coords, times
    )
    
    return {
        'file': Path(file_path).name,
        'metadata': {
            'radius': radius,
            'angular_speed_metadata': angular_speed,
            'save_freq': data.get('save_freq', 'N/A')
        },
        'trajectory_points': len(x_coords),
        'calculated_angular_velocity': {
            'rad_per_sec': calc_angular_vel_rad,
            'deg_per_sec': calc_angular_vel_deg
        }
    }


def analyze_koopman_cp_file(file_path: str) -> Dict:
    """Analyze a single JSON file from Koopman_CP directory."""
    data = load_json_data(file_path)
    
    # Extract trajectory data
    x_coords, y_coords, z_coords = extract_trajectory_data(data)
    times = data.get('time', [])
    
    # Calculate angular velocity from trajectory (assuming 0.1s intervals)
    calc_angular_vel_rad, calc_angular_vel_deg = calculate_angular_velocity_from_trajectory(
        x_coords, y_coords, times
    )
    
    return {
        'file': Path(file_path).name,
        'trajectory_points': len(x_coords),
        'calculated_angular_velocity': {
            'rad_per_sec': calc_angular_vel_rad,
            'deg_per_sec': calc_angular_vel_deg
        }
    }


def main():
    """Main analysis function."""
    base_path = Path(__file__).parent.parent / 'data'
    
    # Analyze circle_around_xy_0.1 files
    circle_xy_path = base_path / 'circle_around_z_0.1'
    if circle_xy_path.exists():
        print("=== Analysis of circle_around_xy_0.1 files ===")
        json_files = list(circle_xy_path.glob('*.json'))
        
        if json_files:
            # Analyze first file as example
            first_file = json_files[0]
            result = analyze_circle_around_xy_file(str(first_file))
            
            print(f"File: {result['file']}")
            print(f"Trajectory points: {result['trajectory_points']}")
            print(f"Metadata:")
            print(f"  - Radius: {result['metadata']['radius']}")
            print(f"  - Angular speed (from metadata): {result['metadata']['angular_speed_metadata']}")
            print(f"  - Save frequency: {result['metadata']['save_freq']}")
            print(f"Calculated angular velocity:")
            print(f"  - {result['calculated_angular_velocity']['rad_per_sec']:.4f} rad/s")
            print(f"  - {result['calculated_angular_velocity']['deg_per_sec']:.4f} deg/s")
            print()
        else:
            print("No JSON files found in circle_around_xz_0.1 directory")
    
    # Analyze Koopman_CP files
    koopman_cp_path = base_path / 'Koopman_CP'
    if koopman_cp_path.exists():
        print("=== Analysis of Koopman_CP files ===")
        json_files = list(koopman_cp_path.glob('*.json'))
        
        if json_files:
            # Analyze first file as example
            file_name = koopman_cp_path / "xy_001.json"
            result = analyze_koopman_cp_file(str(file_name))
            
            print(f"File: {result['file']}")
            print(f"Trajectory points: {result['trajectory_points']}")
            print(f"Calculated angular velocity:")
            print(f"  - {result['calculated_angular_velocity']['rad_per_sec']:.4f} rad/s")
            print(f"  - {result['calculated_angular_velocity']['deg_per_sec']:.4f} deg/s")
            print()
        else:
            print("No JSON files found in Koopman_CP directory")


if __name__ == "__main__":
    main()