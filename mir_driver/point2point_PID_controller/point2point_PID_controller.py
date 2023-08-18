#!/usr/bin/env python3

import rospy
import threading

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import tf

from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



class Task1:

    def __init__(self):
        print("initialization mode")
        self.lock = threading.Lock()
        self.RATE = rospy.get_param('/rate', 50)
        
        
        
        self.bTw = tf.transformations.euler_matrix(-np.pi, 0.0, 0.0, 'rxyz')
        self.dt = 0.0
        self.time_start = 0.0
        self.t = 0

        self.goal_x =0
        self.goal_y = 0
        self.goal_rot_z = 0
        self.pose_des = [0.0, 0.0, 0.0]
        self.pose_des_0 = [0.0, 0.0, 0.0]       
        self.x_DWBL = 0
        self.y_DWBL = 0
        self.z_DWBL = 0
        self.odom_x = 1000 
        self.odom_y = 1000
        self.time_prev = 0

        self.flag = True
        self.flag_goal = False

        # ======= set PID controller parameters

        self.kp_ang_d2g = 1.2
        self.kd_ang_d2g = 0.5
        self.ki_ang_d2g = 0.3

        self.kp_d = 0.7
        self.kd_d = 0.3
        self.ki_d = 0.2

        self.kp_ang = 1.5
        self.kd_ang = 0.5
        self.ki_ang = 0.5

        #  ======= initialization of values for controller

        self.dist_err = 0
        self.dist_err_prop = 0
        self.dist_err_integ = 0
        self.dist_err_derivat = 0
        self.dist_err_prev = 0

        self.angl_err = 0
        self.angl_err_prop = 0
        self.angl_err_integ = 0
        self.angl_err_derivat = 0
        self.angl_err_prev = 0

        self.ang_err_dis2g = 0
        self.ang_err_dis2g_prop = 0
        self.ang_err_dis2g_integ = 0
        self.ang_err_dis2g_derivat = 0
        self.ang_err_dis2g_prev = 0

        "ROS stuff"
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel_user_defined", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        self.sub_DWB_plan = rospy.Subscriber("/move_base_node/DWBLocalPlanner/local_plan", Path, self.DWB_callback, queue_size=10)
        self.sub_current_goal = rospy.Subscriber("move_base_node/current_goal", PoseStamped, self.current_goal_callback)
        
        self.listener = tf.TransformListener()  #  ======= to find distance to global goal base on odom pose

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  #  ======= to cancel global goal after reaching it
        self.client.wait_for_server()




    def init_cont_again(self):
             #  ======= this function initializes the controller paramters after each local planner update

        self.dist_err = 0
        self.dist_err_prop = 0
        self.dist_err_integ = 0
        self.dist_err_derivat = 0
        self.dist_err_prev = 0

        self.ang_err_dis2g = 0
        self.ang_err_dis2g_prop = 0
        self.ang_err_dis2g_integ = 0
        self.ang_err_dis2g_derivat = 0
        self.ang_err_dis2g_prev = 0
        


    def transform_pose(self, pose_w):
        # in 'body' frame
        pose_des = self.bTw * np.matrix([pose_w[0], pose_w[1], pose_w[2], 0.0]).T
        return np.array(pose_des[:3]).reshape(-1,).tolist()


    def DWB_callback(self, p):

        #  ======= DWB local planner updates the tempral goal for controller
       
        if len(p.poses) > 2:   #  ======= less than this number makes the controller unstable (very close local goal)

            
            self.x_DWBL = p.poses[-1 ].pose.position.x
            self.y_DWBL = p.poses[-1 ].pose.position.y
            self.t_DWBL = p.poses[-1 ].header.stamp.to_sec()
            self.orien_DWBL = p.poses[-1 ].pose.orientation
            self.z_DWBL = tf.transformations.euler_from_quaternion((self.orien_DWBL.x, self.orien_DWBL.y, self.orien_DWBL.z, self.orien_DWBL.w))[2]

            self.x_DWBL_0 = p.poses[0].pose.position.x
            self.y_DWBL_0 = p.poses[0].pose.position.y
            self.orien_DWBL_0 = p.poses[0].pose.orientation
            self.z_DWBL_0 = tf.transformations.euler_from_quaternion((self.orien_DWBL_0.x, self.orien_DWBL_0.y, self.orien_DWBL.z, self.orien_DWBL_0.w))[2]

            self.pose_des = [self.x_DWBL, self.y_DWBL, self.z_DWBL]
            self.pose_des_0 = [self.x_DWBL_0, self.y_DWBL_0, self.z_DWBL_0]

            self.flag = True       #  ======= gives signal to controller allowed to publish the velocity commands

        else:
            self.flag = False        #  ======= gives signal to the controller to stop publishing velocity commands

        self.init_cont_again() 


    def current_goal_callback(self, msg):
        goal_position = msg.pose.position
        goal_q = msg.pose.orientation
        goal_rpy = tf.transformations.euler_from_quaternion((goal_q.x, goal_q.y, goal_q.z, goal_q.w))  # roll pitch yaw
        
        self.goal_x = goal_position.x
        self.goal_y = goal_position.y
        self.goal_rot_z = goal_rpy[2] 

        self.flag_goal = True

        print("global goal x, y, orientation")
        print(self.goal_x, self.goal_y, self.goal_rot_z)   
        
                #  ======= reads the taransformation odom to find out pose in global frame   
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
            self.odom_x = trans[0]
            self.odom_y = trans[1]
            # print(self.odom_x , self.odom_y)

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        
 




    def odometry_callback(self, msg):
        self.lock.acquire()
        
        # read current robot state
        self.cur_position = msg.pose.pose.position
        cur_q = msg.pose.pose.orientation
        cur_rpy = tf.transformations.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
        cur_rot_z = cur_rpy[2]

        velocity = Twist() 
        velocity.linear.x = 0
        velocity.angular.z = 0



        if self.flag_goal == True:  #  ======= if there is a global goal the flag is True and controller is active

            #  ======= calculate the distance between current position and global goal
            curp2goal = sqrt((self.goal_x - self.odom_x- self.cur_position.x) ** 2 + (self.goal_y - self.odom_y- self.cur_position.y) ** 2)


            if curp2goal <= 0.3 and curp2goal > 0.1:   #  ======= consider a region around a global goal to change the local goal to global goal
                
                self.pose_des = [self.goal_x - self.odom_x, self.goal_y - self.odom_y, self.goal_rot_z] #  ======= change local goal to global goal

                #  ======= global goal used for the error angel 
                e_x_0 = self.pose_des[0] - self.cur_position.x
                e_y_0 = self.pose_des[1] - self.cur_position.y

                # self.flag = True

            else:

                #  ======= local goal used for the error angel 
                e_x_0 = self.pose_des[0] - self.pose_des_0[0]
                e_y_0 = self.pose_des[1] - self.pose_des_0[1]

    
                #  ======= calculate the distance between current position and local goal
            e_x = self.pose_des[0] - self.cur_position.x
            e_y = self.pose_des[1] - self.cur_position.y        

                    #  ======= calculate the the error angel between desired local angel and robot orientation [-pi , pi]
            self.ang_err_dis2g = (arctan2(e_y_0, e_x_0) - cur_rot_z + 4 * pi) % (2 * pi)
            if self.ang_err_dis2g > pi:
                self.ang_err_dis2g = self.ang_err_dis2g - 2 * pi

                    #  ======= distance to goal
            self.dist_err = abs(sqrt(e_x ** 2 + e_y ** 2) * cos(self.ang_err_dis2g))



            self.ang_err_dis2g_integ += self.ang_err_dis2g_integ * self.dt
            self.dist_err_derivat = (self.ang_err_dis2g - self.ang_err_dis2g_prev) / self.dt
            self.ang_err_dis2g_prev = self.ang_err_dis2g

            self.dist_err_integ += self.dist_err_integ * self.dt
            d_dist_err = self.dist_err - self.dist_err_prev
            self.dist_err_derivat = d_dist_err / self.dt
            self.dist_err_prev = self.dist_err

            if curp2goal >= 0.1 and self.flag == True:
                velocity.angular.z = self.kp_ang_d2g * self.ang_err_dis2g + + self.ki_ang_d2g * self.ang_err_dis2g_integ + self.kd_ang_d2g * self.ang_err_dis2g_derivat
  

                velocity.linear.x = (self.kp_d * self.dist_err + self.ki_d * self.dist_err_integ + self.kd_d * self.dist_err_derivat)* cos(self.ang_err_dis2g)

         
                #  ======= make sure without enough local planner data the velocity commads are zero
            if self.flag == False:
                velocity.linear.x = 0
                velocity.angular.z = 0

                    #  ======= when the distance to goal is less than 0.1 then control the global orientation
            if (curp2goal < 0.1):
                                        
                
                self.angl_err = (self.goal_rot_z - cur_rot_z + 4 * pi) % (2 * pi)

                if self.angl_err > pi:
                    self.angl_err = self.angl_err - 2 * pi

                self.angl_err_integ += self.angl_err_integ * self.dt
                d_angl_err = self.angl_err - self.angl_err_prev
                self.angl_err_derivat = d_angl_err / self.dt
                self.angl_err_prev = self.angl_err
            
                velocity.angular.z = self.kp_ang * self.angl_err + self.ki_ang * self.angl_err_integ + self.kd_ang * self. angl_err_derivat
                print("goal rotation error: ", str(self.angl_err ), "distance2goal error: ", str(curp2goal))

                if abs(self.angl_err) < 0.01:      #  ======= after reaching the goal pose, the global goal is cancelled and the controller stops
                    self.flag_goal = False
                    print("goal rotation error: ", str(self.angl_err ), "distance2goal error: ", str(curp2goal))
                    print("*********** this goal reached and goal cancelled  ***********")
                    self.client.cancel_goal()

        self.pub_cmd_vel.publish(velocity)
        
        # self.time_prev = self.t
        self.lock.release()


    def spin(self):
        rospy.loginfo('Task started!')
        rate = rospy.Rate(self.RATE)

        self.time_start = rospy.get_time()
        # rospy.sleep(0.01); rospy.loginfo('waiting once')

        while not rospy.is_shutdown():
            self.t = rospy.get_time() - self.time_start
            self.dt = self.t - self.time_prev if self.t - self.time_prev else 1e-16
            self.time_prev = self.t 
          
            rate.sleep()
        
        rospy.loginfo('Task completed!')

if __name__ == "__main__":
    rospy.init_node('task1_node')
    task1 = Task1()

    try:

        task1.spin()
    except:
        print("the program stopped")


