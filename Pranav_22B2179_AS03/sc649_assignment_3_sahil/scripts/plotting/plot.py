import os
import matplotlib.pyplot as plt
import numpy as np
import csv
import re

# Load trajectory data from CSV
def load_data_from_csv(filename):
    data = []
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header
        for row in reader:
            data.append([float(value) for value in row])
    return np.array(data)

# Extract parameters A, k1, and k2 from the filename
def extract_parameters(filename):
    match = re.search(r'A_(\d+)_k1_(\d+)_k2_(\d+)', filename)
    if match:
        A = match.group(1)
        k1 = match.group(2)
        k2 = match.group(3)
        return A, k1, k2
    return None, None, None

# Plot the robot's trajectory versus the reference trajectory
def plot_trajectory(data, A, k1, k2, output_path):
    time = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    x_r = data[:, 4]
    y_r = data[:, 5]

    plt.figure()
    plt.plot(x_r, y_r, label='Reference Trajectory', linestyle='--')
    plt.plot(x, y, label='Robot Trajectory')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title(f'Trajectory Tracking\nA={A}, k1={k1}, k2={k2}')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.savefig(output_path)
    plt.close()  # Close the plot to avoid memory issues

# Plot norm of positional error and theta error over time
def plot_errors(data, A, k1, k2, output_path):
    time = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    r = np.sqrt(np.power(x, 2) + np.power(y, 2))
    theta = data[:, 3]
    x_r = data[:, 4]
    y_r = data[:, 5]
    r_r = np.sqrt(np.power(x_r, 2) + np.power(y_r, 2))
    error_norm = r_r - r
    theta_r = np.arctan2(y_r[1:] - y_r[:-1], x_r[1:] - x_r[:-1])  # Calculate reference orientation
    theta_r = np.concatenate(([theta[0]], theta_r))  # Align array lengths
    error_theta = theta - theta_r

    # Normalize theta error to the range [-pi, pi]
    error_theta = (error_theta + np.pi) % (2 * np.pi) - np.pi

    # Plot norm of positional error
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(time, error_norm, label='Norm of Positional Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error Norm')
    plt.title(f'Norm of Positional Error Over Time\nA={A}, k1={k1}, k2={k2}')
    plt.legend()
    plt.grid(True)

    # Plot error in theta
    plt.subplot(2, 1, 2)
    plt.plot(time, error_theta, label='Error in Theta', color='purple')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta Error (rad)')
    plt.title(f'Tracking Error in Theta Over Time\nA={A}, k1={k1}, k2={k2}')
    plt.legend()
    plt.grid(True)
    
    # Save the plot
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()  # Close the plot to avoid memory issues

# Main function to generate plots for all CSV files in a folder
def generate_plots_for_folder(folder_path):
    # Iterate through each CSV file in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith('.csv'):
            file_path = os.path.join(folder_path, filename)
            data = load_data_from_csv(file_path)

            # Extract parameters from filename
            A, k1, k2 = extract_parameters(filename)
            if A and k1 and k2:
                # Generate output file paths
                base_name = os.path.splitext(filename)[0]
                trajectory_output = os.path.join(folder_path, f'{base_name}_trajectory.png')
                errors_output = os.path.join(folder_path, f'{base_name}_errors.png')

                # Generate trajectory plot
                plot_trajectory(data, A, k1, k2, trajectory_output)

                # Generate error plots
                plot_errors(data, A, k1, k2, errors_output)
                print(f"Generated plots for {filename}")

if __name__ == '__main__':
    # Path to the folder containing the CSV files
    experiment_results_folder = '/home/luffy/ros_ws/data/experiment_results'
    generate_plots_for_folder(experiment_results_folder)
