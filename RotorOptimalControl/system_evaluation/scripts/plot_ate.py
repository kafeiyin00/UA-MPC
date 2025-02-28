import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def align_coordinate_system(gt, est):
    """
    Aligns the estimated trajectory to the ground truth using least squares.
    Args:
        gt (numpy.ndarray): Ground truth trajectory (Nx3).
        est (numpy.ndarray): Estimated trajectory (Nx3).
    Returns:
        aligned_est (numpy.ndarray): Aligned estimated trajectory.
        R (numpy.ndarray): Rotation matrix.
        t (numpy.ndarray): Translation vector.
    """
    # Compute centroids
    gt_centroid = np.mean(gt, axis=0)
    est_centroid = np.mean(est, axis=0)
    
    # Subtract centroids
    gt_centered = gt - gt_centroid
    est_centered = est - est_centroid
    
    # Compute covariance matrix
    W = np.dot(est_centered.T, gt_centered)
    
    # Singular Value Decomposition
    U, _, Vt = np.linalg.svd(W)
    R = np.dot(U, Vt)
    
    # Ensure a proper rotation (det(R) should be 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(U, Vt)
    
    # Compute translation
    t = gt_centroid - np.dot(R, est_centroid)
    
    # Apply alignment
    aligned_est = np.dot(est, R.T) + t
    return aligned_est, R, t

def calculate_ate(gt, est):
    """
    Calculates the Absolute Trajectory Error (ATE) between ground truth and estimated trajectory.
    Args:
        gt (numpy.ndarray): Ground truth trajectory (Nx3).
        est (numpy.ndarray): Aligned estimated trajectory (Nx3).
    Returns:
        ate (float): Absolute Trajectory Error.
    """
    errors = np.linalg.norm(gt - est, axis=1)
    ate = np.sqrt(np.mean(errors ** 2))
    return ate

if __name__ == "__main__":
    # Load data from a CSV file
    file_path = "/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/EvaluateATE_KTH_optimal.txt"  # Replace with your file path
    data = pd.read_csv(file_path)
    # print("Column names in the file:", data.columns.tolist())
    data.columns = data.columns.str.strip()


    # Extract ground truth and estimated trajectories
    gt = data[['Odom1_X', 'Odom1_Y', 'Odom1_Z']].to_numpy()
    est = data[['Odom2_X', 'Odom2_Y', 'Odom2_Z']].to_numpy()
    timestamps = data['Timestamp'].to_numpy()
    
    # Align the estimated trajectory to the ground truth
    aligned_est, R, t = align_coordinate_system(gt, est)
    
    # Calculate ATE
    errors = np.linalg.norm(gt - aligned_est, axis=1)  # Per-point errors
    ate = np.sqrt(np.mean(errors ** 2))  # Overall ATE
    
    print("Rotation Matrix (R):\n", R)
    print("Translation Vector (t):\n", t)
    print("Absolute Trajectory Error (ATE):", ate)
    
    # Plot errors over time
    plt.figure(figsize=(10, 3))
    plt.plot(timestamps, errors, label='Trajectory Error', color='black', linestyle='-', alpha=0.7)
    # plt.scatter(timestamps, errors, color='black', label='Error Points', s=20)  # 黑色点
    plt.title('Trajectory Error Over Time')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Error (m)')
    plt.grid()
    plt.legend()
    plt.show()