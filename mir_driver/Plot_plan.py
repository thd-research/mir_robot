#!/usr/bin/env python3


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from datetime import datetime
import tf.transformations as tftr
import tf

import rospy
import threading

import os
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

#------------------------------------define ROS-preset class
class ROS_preset:

    def __init__(self):

        # connection to ROS topics
        self.sub_SBPL_plan = rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, self.plan_callback, queue_size=10)
        self.sub_DWB_plan = rospy.Subscriber("/move_base_node/DWBLocalPlanner/local_plan", Path, self.DWB_callback, queue_size=10)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)

        #-----------------

        self.fig, self.ax = plt.subplots(1, 2)

        # list of pose for global plan
        self.x_glob = [] 
        self.y_glob = []
        self.t_glob = []
        self.z_glob = []

        # list of pose for local plan
        self.x_DWBL = []
        self.y_DWBL = []
        self.t_DWBL = []
        self.z_DWBL = []

        # initialize odom frame
        self.odom_x = 10 
        self.odom_y = 10


        self.listener = tf.TransformListener()



    # callback function for global plan
    def plan_callback(self, p):   

        self.x_glob = [p.poses[i].pose.position.x for i in range(len(p.poses))]
        self.y_glob = [p.poses[i].pose.position.y for i in range(len(p.poses))]
        self.t_glob = [p.poses[i].header.stamp.to_sec() for i in range(len(p.poses))]
        self.orien_glob = [p.poses[i].pose.orientation for i in range(len(p.poses))]
        self.z_glob = [tftr.euler_from_quaternion((self.orien_glob[i].x, self.orien_glob[i].y, self.orien_glob[i].z, self.orien_glob[i].w))[2] for i in range(len(p.poses))]

    # callback function for local plan
    def DWB_callback(self, p):

        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
            self.odom_x = trans[0]
            self.odom_y = trans[1]
        except:
            pass

        self.x_DWBL = [p.poses[i].pose.position.x + self.odom_x for i in range(len(p.poses))]
        self.y_DWBL = [p.poses[i].pose.position.y + self.odom_y for i in range(len(p.poses))]
        self.t_DWBL = [p.poses[i].header.stamp.to_sec() for i in range(len(p.poses))]
        self.orien_DWBL = [p.poses[i].pose.orientation for i in range(len(p.poses))]
        self.z_DWBL = [tftr.euler_from_quaternion((self.orien_DWBL[i].x, self.orien_DWBL[i].y, self.orien_DWBL[i].z, self.orien_DWBL[i].w))[2] for i in range(len(p.poses))]

        for i in range(len(self.t_DWBL)):
            print("time: " + str(round(self.t_DWBL[i], 4)) + "    x: " + str(self.x_DWBL[i]) + "    y: " + str(self.y_DWBL[i]) )

        print("end of one trajectory")


    # callback function for odometry
    def odometry_callback(self, msg):


        self.cur_position = msg.pose.pose.position
        #cur_q = msg.pose.pose.orientation
        #cur_rpy = tf.transformations.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
        #cur_rot_z = cur_rpy[2]


    # animation and plotting function
    def anim_DWBL(self, frame):

        self.ax[0].plot(self.x_glob, self.y_glob, 'g-')
        self.ax[0].plot(self.x_DWBL , self.y_DWBL, '.-')
        self.ax[0].plot(self.odom_x + self.cur_position.x , self.odom_y + self.cur_position.y, 'r*')
        self.ax[0].legend(('global path', 'local path', 'current position'),
           loc='upper center', shadow=True)
        self.ax[0].set_title("SBPL global planner")
        self.ax[0].set_xlabel('x')
        self.ax[0].set_ylabel('y')

        self.ax[1].plot(self.x_DWBL , self.y_DWBL, '.-')
        self.ax[1].set_title("dwb local planer")
        self.ax[1].set_xlabel('x')
        self.ax[1].set_ylabel('y')


        #self.ax[2].plot( self.odom_x + self.cur_position.x , self.odom_y + self.cur_position.y, 'r*')
        #self.ax[2].set_title("current position")
        #self.ax[2].set_xlabel('x')
        #self.ax[2].set_ylabel('y')

        return self.ax

    def spin(self):
        rospy.loginfo('ROS-preset has been activated!')
        rate = rospy.Rate(1)

        #------------------------
        # animation and plot

        animation_local = FuncAnimation(self.fig, self.anim_DWBL, interval = 10)

        plt.show(block = True)
         #-------------------------------

        while not rospy.is_shutdown():

            # print('in the while')

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('ROS_preset_node')
    rospy.loginfo("star of Node")

    ros_preset_task = ROS_preset()

    ros_preset_task.spin()
