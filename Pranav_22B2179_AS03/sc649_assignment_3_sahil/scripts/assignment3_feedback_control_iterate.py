#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion
import csv
import math
import numpy as np
import time
import threading
import argparse
import sys  

global pose, trajectory_data
pose = [0, 0, 0]
trajectory_data = []
lock = threading.Lock() 

def RobotPose(data):
    global pose
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    orientation_q = data.pose.pose.orientation
    (_, _, yaw) = euler_from_quaternion(
        [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    )
    pose = [x, y, yaw]


def save_data_to_csv(filename):
    global trajectory_data
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time', 'x', 'y', 'theta', 'x_r', 'y_r'])
        while not rospy.is_shutdown():
            with lock:
                if trajectory_data:
                    writer.writerows(trajectory_data)
                    trajectory_data.clear() 
            time.sleep(0.1)  

def controller(k1, k2, A, csv_path):
    global pose, trajectory_data
    print(f"Running simulation with A={A}, k1={k1}, k2={k2}")
    rospy.init_node('simulation_controller', anonymous=True)
    rospy.Subscriber('/pf/pose/odom', Odometry, RobotPose)
    pub = rospy.Publisher('/racecar/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=10)
    velocity_msg = AckermannDriveStamped()
    rate = rospy.Rate(50)  

    csv_thread = threading.Thread(target=save_data_to_csv, args=(csv_path,))
    csv_thread.daemon = True 
    csv_thread.start()

    omega = 0.2            
    L = 0.335               
    l1 = 0.2                
    l2 = 0              
    start_time = rospy.get_time()

    while len(pose) == 0:
        continue  

    while not rospy.is_shutdown():
        
        t = rospy.get_time() - start_time

        x_r = A * math.cos(omega * t)
        y_r = A * math.sin(omega * t)
        dx_r = -A * omega * math.sin(omega * t)
        dy_r = A * omega * math.cos(omega * t)

        x = pose[0]
        y = pose[1]
        theta = pose[2]

        x_P = x + l1 * math.cos(theta) - l2 * math.sin(theta)
        y_P = y + l1 * math.sin(theta) + l2 * math.cos(theta)

        e1 = x_P - x_r
        e2 = y_P - y_r

        v1 = dx_r - k1 * e1
        v2 = dy_r - k2 * e2

        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        M = np.array([
            [cos_theta, -l1 * sin_theta - l2 * cos_theta],
            [sin_theta,  l1 * cos_theta - l2 * sin_theta]
        ])

        try:
            u = np.linalg.solve(M, np.array([v1, v2]))
            u1 = u[0]
            u2 = u[1]
        except np.linalg.LinAlgError:
            u1 = 0.0
            u2 = 0.0

        if u1 != 0:
            steering_angle = math.atan(u2 * L / u1)
        else:
            steering_angle = 0.0

        speed = u1

        velocity_msg.drive.speed = speed
        velocity_msg.drive.steering_angle = steering_angle
        pub.publish(velocity_msg)

        with lock:
            trajectory_data.append([t, x_P, y_P, theta, x_r, y_r])

        rate.sleep()

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)

    parser = argparse.ArgumentParser(description='Feedback Control Parameters')
    parser.add_argument('--k1', type=float, default=10, help='Gain for x error')
    parser.add_argument('--k2', type=float, default=10, help='Gain for y error')
    parser.add_argument('--A', type=float, default=8, help='Amplitude of the circular trajectory')
    parser.add_argument('--csv_path', type=str, default='/home/luffy/ros_ws/data/trajectory_data.csv', help='Path to save the trajectory data CSV')

    parsed_args = parser.parse_args(args[1:]) 

    time.sleep(1)  
    controller(parsed_args.k1, parsed_args.k2, parsed_args.A, parsed_args.csv_path)

