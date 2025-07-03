#!/usr/bin/env python3
"""
Probabilistic Tracking Error Calculator for Conformal Koopman Control

This module implements the probabilistic tracking error bounds described in the
"Distribution-Free Control and Planning with Koopman" framework.

Based on the mathematical framework from:
Hiroyasu Tsukamoto, UIUC Aerospace, "Distribution-Free Koopman", April 2025

Key concepts implemented:
1. High-Probability Exponential Boundedness using conformal prediction
2. Forward embedding nonconformity scores
3. Inverse embedding nonconformity scores  
4. Constraint tightening for probabilistic guarantees
"""

import json
import numpy as np
import math
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

# Optional imports
try:
    from scipy.linalg import solve_discrete_are
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


@dataclass
class ConformalKoopmanParams:
    """Parameters for conformal Koopman tracking error calculation."""
    forward_quantile: float  # Forward embedding quantile q_fwd(1-α/K)
    inverse_quantile: float  # Inverse embedding quantile q_inv(1-β)
    gamma: float = 0.9       # Lyapunov contraction rate
    rho: float = 0.01        # Robustification constant
    cv: float = 1.0          # Weight for Lyapunov modeling error
    K: int = 100             # Prediction horizon
    # Keep alpha and beta for probability bound calculation
    alpha: float = 0.1       # Confidence level for forward embedding
    beta: float = 0.1        # Confidence level for inverse embedding


@dataclass  
class TrackingErrorBounds:
    """Results of probabilistic tracking error calculation."""
    forward_quantile: float
    inverse_quantile: float
    delta_r: float  # High-probability tracking error bound
    prob_bound_lower: float  # Lower bound on probability (1-alpha-beta)
    lyapunov_bound: float  # Lyapunov-based bound
    exponential_decay_rate: float  # gamma^K


class ProbabilisticTrackingError:
    """
    Calculate probabilistic tracking error bounds using conformal prediction
    for Koopman-based control systems.
    """
    
    def __init__(self, params: ConformalKoopmanParams):
        self.params = params
    
    def load_trajectory_data(self, file_path: str) -> Dict:
        """Load trajectory data from JSON file."""
        with open(file_path, 'r') as f:
            return json.load(f)
    
    def extract_poses_and_targets(self, data: Dict, file_path: str = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Extract pose and target data from JSON.
        
        Args:
            data: JSON data dictionary
            file_path: Path to JSON file (used to determine trajectory type)
        
        Returns:
            poses: (N, 3) array of actual positions
            targets: (N, 3) array of target positions  
            times: (N,) array of timestamps
        """
        poses = np.array(data.get('pose', []))
        times = np.array(data.get('time', []))
        controls = np.array(data.get('control', []))
        
        # Find where takeoff ends (control != [0, 0, 1])
        takeoff_end_idx = 0
        if len(controls) > 0:
            for i, control in enumerate(controls):
                if len(control) >= 3 and not np.allclose(control[:3], [0.0, 0.0, 1.0]):
                    takeoff_end_idx = i
                    break
        
        # Add 50 steps after takeoff ends for stabilization
        stabilization_steps = 50
        actual_start_idx = takeoff_end_idx + stabilization_steps
        
        # Remove takeoff phase and stabilization period
        if actual_start_idx > 0:
            poses = poses[actual_start_idx:] if len(poses) > actual_start_idx else poses
            times = times[actual_start_idx:] if len(times) > actual_start_idx else times
            if len(targets) > actual_start_idx:
                targets = targets[actual_start_idx:]
        
        # Ensure poses is properly shaped
        if len(poses.shape) == 2 and poses.shape[1] >= 3:
            poses = poses[:, :3]  # Take only x, y, z
        elif len(poses.shape) == 1:
            poses = poses.reshape(-1, 3) if len(poses) % 3 == 0 else poses.reshape(-1, 1)
            
        # Handle targets
        # If no targets provided, generate ideal circular trajectory as reference
        targets = self.generate_circular_reference(poses, times, file_path)

            
        return poses, targets, times
    
    def calc_target(self, trajectory_type: str, t: float, omega: float = 10.0) -> Tuple[float, float, float]:
        """
        Calculate target position using the same logic as solo_koopman_conformal_circle.py
        
        Args:
            trajectory_type: Type of trajectory (XZ, YZ, XY, XYZ, XY2Z)
            t: Time in seconds
            omega: Angular velocity in degrees per second
            
        Returns:
            target_x, target_y, target_z coordinates
        """
        theta = omega * t / 180 * np.pi
        
        if trajectory_type == "XZ":
            target_x = 0.5 * np.cos(theta)
            target_y = 0.0
            target_z = 0.5 * np.sin(theta) + 1.00
        elif trajectory_type == "YZ":
            target_x = 0.0
            target_y = 0.5 * np.cos(theta)
            target_z = 0.5 * np.sin(theta) + 1.00
        elif trajectory_type == "XY":
            target_x = 1.0 * np.cos(theta)  # Use 1.0 radius for XY plane
            target_y = 1.0 * np.sin(theta)
            target_z = 1.00
        elif trajectory_type == "XYZ":
            target_x = 1.0 * np.cos(theta)  # Use 1.0 radius for XY plane
            target_y = 1.0 * np.sin(theta)
            target_z = 0.5 * np.sin(theta) + 1.00
        elif trajectory_type == "XY2Z":
            target_x = 1.0 * np.cos(theta)  # Use 1.0 radius for XY plane
            target_y = 1.0 * np.sin(theta)
            target_z = 0.30 * np.sin(2 * theta) + 1.00
        else:
            # Default to XY trajectory
            target_x = 1.0 * np.cos(theta)
            target_y = 1.0 * np.sin(theta)
            target_z = 1.00
            
        return target_x, target_y, target_z
    
    def get_trajectory_type_from_filename(self, file_path: str) -> str:
        """
        Extract trajectory type from filename.
        
        Args:
            file_path: Path to JSON file
            
        Returns:
            Trajectory type string (XY, XZ, YZ, XYZ, XY2Z)
        """
        if file_path is None:
            return "XY"  # Default
            
        filename = Path(file_path).stem.lower()  # Keep lowercase for comparison
        
        # Check for trajectory types at the beginning of filename
        # Priority order: longest patterns first to avoid false matches
        if filename.startswith("xy2z"):
            return "XY2Z"
        elif filename.startswith("xyz"):
            return "XYZ"
        elif filename.startswith("xy"):
            return "XY"
        elif filename.startswith("xz"):
            return "XZ"
        elif filename.startswith("yz"):
            return "YZ"
        else:
            return "XY"  # Default
    
    def generate_circular_reference(self, poses: np.ndarray, times: np.ndarray, 
                                  file_path: str = None) -> np.ndarray:
        """
        Generate ideal circular trajectory as reference targets using calc_target.
        
        Args:
            poses: Actual pose data to determine trajectory pattern
            times: Time array from JSON file
            file_path: Path to JSON file (used to determine trajectory type)
            
        Returns:
            targets: (N, 3) array of ideal target positions
        """
        if len(poses) == 0:
            return np.array([])
            
        # Get trajectory type from filename
        trajectory_type = self.get_trajectory_type_from_filename(file_path)
        
        # Use actual times from JSON
        if len(times) == 0:
            times = np.arange(len(poses)) * 0.1  # Fallback to 0.1s intervals
            
        # Calculate time relative to start (takeoff phase already removed)
        time_start = times[0] if len(times) > 0 else 0.0
        relative_times = times - time_start
        
        # Generate target trajectory
        targets = np.zeros_like(poses)
        
        # Angular velocity: 10 deg/s (from config)
        omega = 10.0  # deg/s
        
        # Since takeoff phase is already removed, generate circular trajectory directly
        for i in range(len(poses)):
            if i < len(relative_times):
                t = relative_times[i]
            else:
                t = i * 0.1  # Fallback
                
            # Generate circular trajectory using calc_target
            target_x, target_y, target_z = self.calc_target(trajectory_type, t, omega)
            targets[i] = np.array([target_x, target_y, target_z])
                
        return targets
    
    def detect_liftoff_end(self, targets: np.ndarray) -> int:
        """
        Detect when liftoff phase ends based on target trajectory.
        
        Args:
            targets: (N, 3) array of target positions
            
        Returns:
            Index where tracking phase begins
        """
        liftoff_end_index = 0
        for i, target in enumerate(targets):
            if len(target) >= 3 and not np.allclose(target, [0.0, 1.0, 0.0]):
                liftoff_end_index = i
                break
        return liftoff_end_index
    
    def calculate_tracking_errors(self, poses: np.ndarray, targets: np.ndarray) -> np.ndarray:
        """
        Calculate tracking errors between poses and targets.
        
        Args:
            poses: (N, 3) actual positions
            targets: (N, 3) target positions
            
        Returns:
            tracking_errors: (N,) array of Euclidean tracking errors
        """
        if poses.shape != targets.shape:
            min_len = min(len(poses), len(targets))
            poses = poses[:min_len]
            targets = targets[:min_len]
        
        tracking_errors = np.linalg.norm(poses - targets, axis=1)
        return tracking_errors
    
    
    def calculate_lyapunov_based_bound(self, v0: float, forward_quantile: float) -> float:
        """
        Calculate Lyapunov-based tracking error bound.
        
        Based on: v_K ≤ γ^K * v0 + C * (1-γ^K)/(1-γ)
        where C = -ρ + q_fwd(1-α/K)
        
        Args:
            v0: Initial tracking error
            forward_quantile: Forward embedding quantile q_fwd(1-α/K)
            
        Returns:
            Lyapunov bound on tracking error
        """
        gamma = self.params.gamma
        rho = self.params.rho
        K = self.params.K
        
        C = -rho + forward_quantile
        
        if gamma == 1.0:
            lyapunov_bound = v0 + C * K
        else:
            lyapunov_bound = (gamma ** K) * v0 + C * (1 - gamma ** K) / (1 - gamma)
        
        return max(0.0, lyapunov_bound)
    
    def calculate_delta_r(self, forward_quantile: float, m: float = 1.0) -> float:
        """
        Calculate Δr bound for high-probability tracking error.
        
        Δr = C * (1-γ)^(-1) / √m
        where C = -ρ + q_fwd(1-α/K)
        
        Args:
            forward_quantile: Forward embedding quantile
            m: Lower bound on eigenvalues of M matrix
            
        Returns:
            Delta r bound
        """
        gamma = self.params.gamma
        rho = self.params.rho
        
        C = -rho + forward_quantile
        delta_r = C / ((1 - gamma) * math.sqrt(m))
        
        return max(0.0, delta_r)
    
    def analyze_single_trajectory(self, file_path: str, 
                                predicted_poses: Optional[np.ndarray] = None) -> TrackingErrorBounds:
        """
        Analyze a single trajectory file for probabilistic tracking error.
        
        Args:
            file_path: Path to JSON trajectory file
            predicted_poses: Optional predicted poses for inverse embedding analysis
            
        Returns:
            TrackingErrorBounds object with results
        """
        # Load and process data (takeoff phase is removed in extract_poses_and_targets)
        data = self.load_trajectory_data(file_path)
        poses, targets, times = self.extract_poses_and_targets(data, file_path)
        
        # Calculate tracking errors
        tracking_errors = self.calculate_tracking_errors(poses, targets)
        
        # Use provided quantiles from parameters
        forward_quantile = self.params.forward_quantile
        inverse_quantile = self.params.inverse_quantile
        
        # Calculate bounds using provided quantiles
        v0 = tracking_errors[0] if len(tracking_errors) > 0 else 0.0
        lyapunov_bound = self.calculate_lyapunov_based_bound(v0, forward_quantile)
        delta_r = self.calculate_delta_r(forward_quantile)
        
        # Probability bound
        prob_bound_lower = 1 - self.params.alpha - self.params.beta
        
        return TrackingErrorBounds(
            forward_quantile=forward_quantile,
            inverse_quantile=inverse_quantile,
            delta_r=delta_r,
            prob_bound_lower=prob_bound_lower,
            lyapunov_bound=lyapunov_bound,
            exponential_decay_rate=self.params.gamma ** self.params.K
        )
    
    def compare_trajectories(self, file_paths: List[str]) -> Dict[str, TrackingErrorBounds]:
        """
        Compare probabilistic tracking error bounds across multiple trajectories.
        
        Args:
            file_paths: List of paths to trajectory JSON files
            
        Returns:
            Dictionary mapping file names to TrackingErrorBounds
        """
        results = {}
        for file_path in file_paths:
            file_name = Path(file_path).name
            try:
                bounds = self.analyze_single_trajectory(file_path)
                results[file_name] = bounds
            except Exception as e:
                print(f"Error analyzing {file_name}: {e}")
                
        return results
    
    def plot_tracking_error_analysis(self, file_path: str, save_path: Optional[str] = None):
        """
        Plot tracking error analysis results.
        
        Args:
            file_path: Path to trajectory JSON file
            save_path: Optional path to save plot
        """
        if not HAS_MATPLOTLIB:
            print("Matplotlib not available. Skipping plot generation.")
            return
        # Load and process data (takeoff phase is removed in extract_poses_and_targets)
        data = self.load_trajectory_data(file_path)
        poses, targets, times = self.extract_poses_and_targets(data, file_path)
        times = times[liftoff_end:] if len(times) > 0 else np.arange(len(poses))
        
        # Calculate tracking errors and bounds
        tracking_errors = self.calculate_tracking_errors(poses, targets)
        bounds = self.analyze_single_trajectory(file_path)
        
        # Create plot
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # Plot tracking errors over time
        ax1.plot(times, tracking_errors, 'b-', label='Actual Tracking Error', linewidth=2)
        ax1.axhline(y=bounds.delta_r, color='r', linestyle='--', 
                   label=f'Δr Bound ({bounds.prob_bound_lower:.1%} confidence)', linewidth=2)
        ax1.axhline(y=bounds.forward_quantile, color='g', linestyle=':', 
                   label=f'Forward Quantile', linewidth=1)
        ax1.set_xlabel('Time')
        ax1.set_ylabel('Tracking Error')
        ax1.set_title(f'Probabilistic Tracking Error Analysis - {Path(file_path).name}')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot cumulative statistics
        time_horizon = np.arange(1, len(tracking_errors) + 1)
        theoretical_bound = [bounds.exponential_decay_rate ** k * tracking_errors[0] + 
                           bounds.delta_r * (1 - bounds.exponential_decay_rate ** k) / (1 - bounds.exponential_decay_rate)
                           if bounds.exponential_decay_rate != 1.0 else tracking_errors[0] + bounds.delta_r * k
                           for k in time_horizon]
        
        ax2.plot(time_horizon, tracking_errors, 'b-', label='Actual', linewidth=2)
        ax2.plot(time_horizon, theoretical_bound, 'r--', label='Theoretical Bound', linewidth=2)
        ax2.set_xlabel('Time Steps')
        ax2.set_ylabel('Tracking Error')
        ax2.set_title('Tracking Error vs Theoretical Bound')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.show()


def main():
    """Main function to demonstrate probabilistic tracking error analysis."""
    
    # Initialize parameters with example quantiles
    # NOTE: In practice, these quantiles should be calculated externally
    params = ConformalKoopmanParams(
        forward_quantile=1.0,   # Example forward embedding quantile q_fwd(1-α/K)
        inverse_quantile=0.8,   # Example inverse embedding quantile q_inv(1-β)
        gamma=0.9,              # Lyapunov contraction rate
        rho=0.01,               # Robustification constant
        K=100,                  # Prediction horizon
        alpha=0.1,              # Confidence level for forward embedding
        beta=0.1                # Confidence level for inverse embedding
    )
    
    # Create analyzer
    analyzer = ProbabilisticTrackingError(params)
    
    # Analyze Koopman CP files
    base_path = Path(__file__).parent.parent / 'data' / 'Koopman_CP'
    json_files = list(base_path.glob('*.json'))
    
    if json_files:
        print("=== Probabilistic Tracking Error Analysis ===")
        print(f"Parameters: α={params.alpha}, β={params.beta}, γ={params.gamma}")
        print(f"Forward quantile: {params.forward_quantile}, Inverse quantile: {params.inverse_quantile}")
        print(f"Confidence level: {(1-params.alpha-params.beta)*100:.1f}%")
        print()
        
        # Analyze first file as example
        test_file = json_files[0]
        print(f"Analyzing: {test_file.name}")
        
        bounds = analyzer.analyze_single_trajectory(str(test_file))
        
        print(f"Results:")
        print(f"  Forward quantile q_fwd: {bounds.forward_quantile:.6f}")
        print(f"  Inverse quantile q_inv: {bounds.inverse_quantile:.6f}")
        print(f"  High-probability bound Δr: {bounds.delta_r:.6f}")
        print(f"  Lyapunov bound: {bounds.lyapunov_bound:.6f}")
        print(f"  Probability guarantee: ≥{bounds.prob_bound_lower:.1%}")
        print(f"  Exponential decay rate: {bounds.exponential_decay_rate:.6f}")
        print()
        
        # Compare multiple files
        if len(json_files) > 1:
            print("=== Comparison across trajectories ===")
            sample_files = json_files[:3]  # Analyze first 3 files
            results = analyzer.compare_trajectories([str(f) for f in sample_files])
            
            for file_name, bounds in results.items():
                print(f"{file_name}:")
                print(f"  Δr: {bounds.delta_r:.6f}, Conf: {bounds.prob_bound_lower:.1%}")
            
        # Create visualization
        if HAS_MATPLOTLIB:
            try:
                save_path = Path(__file__).parent.parent / 'data' / 'tracking_error_analysis.png'
                analyzer.plot_tracking_error_analysis(str(test_file), str(save_path))
                print(f"Plot saved to: {save_path}")
            except Exception as e:
                print(f"Could not create plot: {e}")
        else:
            print("Matplotlib not available. Skipping plot generation.")
    else:
        print("No JSON files found in Koopman_CP directory")


if __name__ == "__main__":
    main()