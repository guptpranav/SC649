import os
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import csv
import re
from tqdm import tqdm  # Import tqdm for progress bar

# Load trajectory data from CSV
def load_data_from_csv(filename, downsample_factor=10):
    data = []
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header
        for i, row in enumerate(reader):
            if i % downsample_factor == 0:  # Downsample data
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

# Generate animated trajectory plot
def animate_trajectory(data, A, k1, k2, output_path):
    x = data[:, 1]
    y = data[:, 2]
    x_r = data[:, 4]
    y_r = data[:, 5]

    fig, ax = plt.subplots()
    ax.plot(x_r, y_r, label='Reference Trajectory', linestyle='--')
    robot_line, = ax.plot([], [], label='Robot Trajectory', color='orange')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title(f'Trajectory Tracking Animation\nA={A}, k1={k1}, k2={k2}')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')  # Ensures equal scaling

    # Initialization function: set up the background of the plot
    def init():
        robot_line.set_data([], [])
        return robot_line,

    # Animation function: this is called sequentially
    def update(frame):
        robot_line.set_data(x[:frame], y[:frame])
        return robot_line,

    # Adjust the frames to match the downsampled data length
    ani = animation.FuncAnimation(
        fig, update, frames=len(x), init_func=init, blit=True, interval=50
    )

    # Save animation as GIF with a higher fps for faster playback
    ani.save(output_path, writer='pillow', fps=50)
    plt.close(fig)

# Main function to generate animated GIFs for all CSV files in a folder
def generate_gifs_for_folder(folder_path):
    # Get a list of all CSV files in the folder
    csv_files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]

    # Use tqdm to add a progress bar
    for filename in tqdm(csv_files, desc="Generating GIFs"):
        file_path = os.path.join(folder_path, filename)

        # Load data from CSV
        data = load_data_from_csv(file_path, downsample_factor=10)

        # Extract parameters from filename
        A, k1, k2 = extract_parameters(filename)
        if A and k1 and k2:
            # Set output GIF file path
            output_gif_path = os.path.join(folder_path, f'{filename[:-4]}.gif')

            # Generate animated trajectory plot
            animate_trajectory(data, A, k1, k2, output_gif_path)

if __name__ == '__main__':
    # Path to the folder containing the CSV files
    experiment_results_folder = '/home/luffy/ros_ws/data/experiment_results'
    generate_gifs_for_folder(experiment_results_folder)
