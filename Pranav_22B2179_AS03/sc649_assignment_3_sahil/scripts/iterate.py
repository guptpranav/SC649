#!/usr/bin/env python3
import subprocess
import os
import time

A_values = [5, 8, 12]
k1_values = [1, 2, 4, 8, 16]
k2_values = [1, 2, 4, 8, 16]


output_directory = "/home/luffy/ros_ws/data/experiment_results"

os.makedirs(output_directory, exist_ok=True)

for A in A_values:
    for k1 in k1_values:
        for k2 in k2_values:

            csv_filename = f"A_{A}_k1_{k1}_k2_{k2}.csv"
            csv_path = os.path.join(output_directory, csv_filename)

            command = [
                "roslaunch", "sc649_assignment_3", "racecar_iterate.launch",
                f"A:={A}", f"k1:={k1}", f"k2:={k2}", f"csv_path:={csv_path}"
            ]

            print(f"Running simulation with A={A}, k1={k1}, k2={k2}")
            process = subprocess.Popen(command)


            time.sleep(70)

            process.terminate()
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.kill()

            print(f"Simulation with A={A}, k1={k1}, k2={k2} completed. Results saved to {csv_path}.")
            time.sleep(10)
