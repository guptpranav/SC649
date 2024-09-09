#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
import subprocess
import time

class TurtleBotController:
    def __init__(self):
        rospy.init_node('turtlebot_steering_control', anonymous=True)
        while rospy.Time.now().to_sec() == 0:
            pass
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(1000)  
        self.R = 1000
        self.theta = -1
        self.alpha = 0.0
        self.x = 0.0
        self.y = 0.0
        self.prev_x = 0
        self.prev_y = 0
        self.Ks = 0.1
        self.v = 0.2 
        self.data = []
        self.start_time = rospy.Time.now()
        self.last_update_time = self.start_time
        self.set_initial_position(2, -2, 0)  #Initial Pos

    def set_initial_position(self, x, y, theta):
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)      
        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_burger'
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 0
        quat = quaternion_from_euler(0, 0, theta)
        state_msg.pose.orientation.x = quat[0]
        state_msg.pose.orientation.y = quat[1]
        state_msg.pose.orientation.z = quat[2]
        state_msg.pose.orientation.w = quat[3]
        resp = set_state(state_msg)
        rospy.sleep(1)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.theta = math.atan2(self.y, self.x)
        self.R = math.sqrt(self.x**2 + self.y**2)
        self.alpha = yaw
        if self.alpha < 0:
            self.alpha = 2 * math.pi + self.alpha
        if self.theta < 0:
            self.theta = 2 * math.pi + self.theta
        self.prev_x = self.x
        self.prev_y = self.y
        current_time = rospy.Time.now()
        if (current_time - self.last_update_time).to_sec() >= 0.1:
            sim_time = (current_time - self.start_time).to_sec()
            self.data.append([sim_time, self.x, self.y, self.theta, self.alpha])
            self.last_update_time = current_time

    def steering_control(self):
        omega = -self.Ks * np.sign(self.alpha - self.theta - math.pi)
        return omega

    def run(self):
        while not rospy.is_shutdown():
            omega = self.steering_control()
            twist = Twist()
            twist.linear.x = self.v
            twist.angular.z = omega
            self.pub.publish(twist)
            
            if self.R < 0.1:
                break
            
            self.rate.sleep()

        twist = Twist()
        self.pub.publish(twist)

        np.savetxt('turtlebot_data.csv', self.data, delimiter=',', 
                   header='time,x,y,theta,alpha', comments='')
        
        self.plot_path()

    def plot_path(self):
        data = np.array(self.data)
        plt.figure(figsize=(10, 10))
        plt.plot(data[:, 1], data[:, 2])
        plt.title('TurtleBot Path')
        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.axis('equal')
        plt.grid(True)
        plt.savefig('turtlebot_path.png')
        plt.close()  

def launch_simulation():
    subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_empty_world.launch', 
                      'use_sim_time:=true'])
    time.sleep(5)


if __name__ == '__main__':
    try:
        launch_simulation()
        controller = TurtleBotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass