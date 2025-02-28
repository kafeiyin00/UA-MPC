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

def get_error(file_path):
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
    return errors

if __name__ == "__main__":
    # Load data from a CSV file
    dataname = "KTH"
    file_path0 = "/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/EvaluateATE_%s_optimal_100.txt" % dataname
    file_path1 = "/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/EvaluateATE_%s_optimal_100.txt" % dataname
    file_path2 = "/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/EvaluateATE_%s_optimal_1000.txt" % dataname
    file_path3 = "/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/EvaluateATE_%s_optimal_10000.txt" % dataname
    file_path4 = "/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/EvaluateATE_%s_constant_7.2.txt"    % dataname
    file_path5 = "/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/EvaluateATE_%s_constant_3.6.txt"     % dataname
    file_path6 = "/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/EvaluateATE_%s_constant_1.8.txt"   % dataname
    file_path7 = "/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/EvaluateATE_%s_zero.txt"      % dataname
    
    print("Error0:")
    error0 = get_error(file_path0)
    print("Error1:")
    error1 = get_error(file_path1)
    print("Error2:")
    error2 = get_error(file_path2)
    print("Error3:")
    error3 = get_error(file_path3)
    print("Error4:")
    error4 = get_error(file_path4)
    print("Error5:")
    error5 = get_error(file_path5)
    print("Error6:")
    error6 = get_error(file_path6)
    print("Error7:")
    error7 = get_error(file_path7)
    
    errors_list = [error0, error1, error2, error3, error4, error5, error6, error7]
    min_error = min(errors_list, key=len)  # Find the shortest sequence
    frame_ids = range(len(min_error))  # Generate frame IDs based on its length

    # Plot errors using frame IDs as the x-axis
    plt.figure(figsize=(12, 4))

    # Plot each error series with conventional colors and distinct styles
    plt.plot(frame_ids, error0[frame_ids], label='UA-MPC Trial 1', color='red', linestyle='-', linewidth=2)
    plt.plot(frame_ids, error1[frame_ids], label='UA-MPC Trial 2', color='blue', linestyle='-', linewidth=2)
    plt.plot(frame_ids, error2[frame_ids], label='UA-MPC Trial 3', color='green', linestyle='-', linewidth=2)
    plt.plot(frame_ids, error3[frame_ids], label='UA-MPC Trial 4', color='purple', linestyle='-', linewidth=2)
    plt.plot(frame_ids, error4[frame_ids], label='Constant-Speed 7.2', color='orange', linestyle='--', linewidth=2)
    plt.plot(frame_ids, error5[frame_ids], label='Constant-Speed 3.6', color='brown', linestyle=':', linewidth=2)
    plt.plot(frame_ids, error6[frame_ids], label='Constant-Speed 1.8', color='magenta', linestyle='-.', linewidth=2)
    plt.plot(frame_ids, error7[frame_ids], label='Zero-Speed', color='gray', linestyle='--', linewidth=2)

    # Improve plot styling
    plt.title('Trajectory Error Comparison', fontsize=12, pad=15)
    plt.xlabel('Frame ID', fontsize=10)
    plt.ylabel('Error (m)', fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=9)
    plt.tight_layout()
    plt.show()