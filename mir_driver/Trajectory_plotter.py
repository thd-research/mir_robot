#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import random


class TrajectoryPlotter:
    def __init__(self):
        rospy.init_node('trajectory_plotter')
        self.path_sub = rospy.Subscriber('/move_base_node/DWBLocalPlanner/local_plan', Path, self.path_callback)
        self.path_sub2 = rospy.Subscriber('/move_base_node/SBPLLatticePlanner/plan', Path, self.Globpath_callback)
        self.fig, self.ax = plt.subplots()
        self.x_data = []
        self.y_data = []
        self.globx_data = []
        self.globy_data = []

    def path_callback(self, path_msg):
        poses = path_msg.poses
        self.x_data = []
        self.y_data = []
        self.timestamps = []
        for pose in poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            timestamp = pose.header.stamp.to_sec()
            self.x_data.append(x)
            self.y_data.append(y)
            self.timestamps.append(timestamp)
        print('Received new trajectory:')
        print('X:', self.x_data)
        print('Y:', self.y_data)
        print('Timestamps:', self.timestamps)
        
    def Globpath_callback(self, globpath_msg):
        globposes = globpath_msg.poses
        self.globx_data = []
        self.globy_data = []
        dummy_x=[]
        dummy_y=[]
        for pose in globposes:
            globx = pose.pose.position.x
            globy = pose.pose.position.y
            
            dummy_x.append(globx)
            dummy_y.append(globy)
        #self.globx_data=dummy_x-dummy_x[0] 
        #self.globy_data=dummy_y-dummy_y[0]  
        self.globx_data = [xx - dummy_x[0] for xx in dummy_x]
        self.globy_data = [yy - dummy_y[0] for yy in dummy_y] 
        print('Received new Global trajectory:')
        print('Global_X:', self.globx_data)
        print('Global_Y:', self.globy_data)
        

    def update_plot(self):
        
        colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k'] # predefined color order
        color = random.choice(colors) # randomly choose a color from the order
        self.ax.plot(self.x_data, self.y_data, color+'-')
        self.ax.plot(self.globx_data, self.globy_data, 'b--', linewidth=.5)
        self.ax.set_xlim([-10, 10])
        self.ax.set_ylim([-10, 10])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Robot Trajectory')
        plt.draw()
        plt.pause(0.001)

    def run(self):
        rate = rospy.Rate(30)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()


if __name__ == '__main__':
    try:
        tp = TrajectoryPlotter()
        tp.run()
    except rospy.ROSInterruptException:
        pass

