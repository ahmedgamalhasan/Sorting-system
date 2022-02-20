#!/usr/bin/env python
import math
import rospy
import sensor_msgs.msg
#from moveit_commander import *
import moveit_msgs
import geometry_msgs
import numpy as np
from std_msgs.msg import Float32MultiArray
import sys
import copy
import rospy
from moveit_commander import *
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander import RobotCommander, MoveGroupCommander, PoseStamped
import  moveit_commander


class MoveItPlanner_c:

    
    g =0

    def __init__(self):
        self.robot = RobotCommander()
        self.arm = MoveGroupCommander("arm")
        self.pose = PoseStamped()
        self.count = int(0)
        self.goal_or_not = False
        self.x_goal=float(0.0)
        self.y_goal=float(0.0)

    def calculate_goal_points(self, x, y, z):

        if x is not self.x_goal    or     y is not self.y_goal:
            self.x_goal = x
            self.y_goal = y
            
            self.pose.pose.position.x = self.x_goal
            self.pose.pose.position.y = self.y_goal
            self.pose.pose.position.z = z
            self.arm.set_joint_value_target(self.pose, self.arm.get_end_effector_link(), True)
            self.plan = self.arm.plan()
            self.l = (len(self.plan[1].joint_trajectory.points))-1

            self.First_joint_angels = []
            self.Second_joint_angels = []
            self.Third_joint_angels = []
            for i in range(self.l+1):
                self.First_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[0])
                self.Second_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[1])
                self.Third_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[2])

            self.count = 0
        
        next_first_joint = self.First_joint_angels[self.count]
        next_second_joint = self.Second_joint_angels[self.count]
        next_third_joint = self.Third_joint_angels[self.count]
        self.count = self.count +1

        if self.count >= (self.l+1):
            self.goal_or_not = True
            if self.count > self.l:
                self.count = self.l

        return next_first_joint, next_second_joint, next_third_joint, self.goal_or_not



    def calculate_goal_points(self, x, y, z, solve_or_not):
        
        if solve_or_not :
            self.pose.pose.position.x = x
            self.pose.pose.position.y = y
            self.pose.pose.position.z = z
            self.arm.set_joint_value_target(self.pose, self.arm.get_end_effector_link(), True)
            self.plan = self.arm.plan()
            self.l = (len(self.plan[1].joint_trajectory.points))-1

            self.First_joint_angels = []
            self.Second_joint_angels = []
            self.Third_joint_angels = []
            for i in range(self.l):
                self.First_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[0])
                self.Second_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[1])
                self.Third_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[2])

            self.count = 0

        if not solve_or_not :

            next_first_joint = self.First_joint_angels[self.count]
            next_second_joint = self.Second_joint_angels[self.count]
            next_third_joint = self.Third_joint_angels[self.count]
            self.count = self.count +1
            
        if self.count == self.l:
            self.goal_or_not = True

        return next_first_joint, next_second_joint, next_third_joint ,self.goal_or_not



class Publisher_c:
    def __init__(self):
        self.pub = rospy.Publisher('last_point', Float32MultiArray, queue_size=10)
        self.goal_msg = Float32MultiArray()
        self.counter=0
    def Publish_if_okay(self, First_waypoint, Second_waypoint,Third_waypoint,l,flag):
        if (flag==1 and self.counter<l):

            self.goal_msg[0] = np.rad2deg(First_waypoint[self.counter])
            self.goal_msg[1] = np.rad2deg(Second_waypoint[self.counter])
            self.goal_msg[2] = np.rad2deg(Third_waypoint[self.counter])
            self.counter = self.counter+1
            rospy.loginfo(self.goal_msg)
            self.pub.publish(self.goal_msg)

    def Publish(self,first_joint, second_joint, third_joint, fourth_joint):
        self.goal_msg[0] = np.rad2deg(first_joint)
        self.goal_msg[1] = (second_joint)*((1000*2*3.14)/2)*57.2958
        self.goal_msg[2] = np.rad2deg(third_joint)
        rospy.loginfo(self.goal_msg)
        self.pub.publish(self.goal_msg)

