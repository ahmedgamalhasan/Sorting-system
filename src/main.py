#!/usr/bin/env python
# license removed for brevity

#import enum
#from logging import fatal
#from re import X
from numpy.core.fromnumeric import std
#from pub import coin
import rospy
#from rospy.client import init_node
#from rospy.core import configure_logging
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from enum import Enum
import numpy as np
from rospy.numpy_msg import numpy_msg
from class_solver_publisher import *
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32
from os import linesep as endl, pread
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
import rospy
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image


goal = False
#global coins_arr
#coins_arr = [[0,0,0],[0,0,0]]

def per_callback(data):
    
    bridge = CvBridge()
    
    global coins_arr
    coins_arr = bridge.imgmsg_to_cv2(data)
    
    #rospy.loginfo(coins_arr)
    #print("x = " ,coins[0][0])
    #print("y = " ,coins[0][1])

testfuncount = 0
def testfun(index):
    print("test function,number : " , index)
    test=input()

def node_init():
        rospy.init_node('node', anonymous=True)
        rospy.Subscriber("mrcamera", Image, per_callback)
        rate = rospy.Rate(1) # 10hz


node_init()
rate = rospy.Rate(1) # 10hz




class system_status_enum(Enum):
    hold =0
    home = 1
    perceive = 2
    sort = 3
    confg = 4

class empty_or_not(Enum):
    empty = 0
    not_empty = 1

class pick_and_place_e(Enum):
    pick = 0
    place = 1

class magnet(Enum):
    pick = 0
    place = 1

def magnet_pick(magnet_state,current_x,current_y, z_level_down,z_level_up):
    while goal == False:
        if rospy.is_shutdown():
            rospy.on_shutdown(nodename)
        next_joint_0, next_joint_1, next_joint_2, goal = planner.calculate_goal_points(current_x,current_y, z_level_down)
        while not gpio == '1':  # to be the motor error state
            # print("picking")
            if rospy.is_shutdown():
                rospy.on_shutdown(nodename)
            if(magnet_state==magnet.pick):
                publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 1)
            if(magnet_state==magnet.place):
                publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 0)

            print(" 1 for motor pick ok !")
            # gpio = input()     # for testing
            gpio = '1'  # for testing

        gpio = '0'

    goal = False

    while goal == False:
        if rospy.is_shutdown():
            rospy.on_shutdown(nodename)
        next_joint_0, next_joint_1, next_joint_2, goal = planner.calculate_goal_points(current_x, current_y,z_level_up)
        while not gpio == '1':  # to be the motor error state
            # print("picking")
            if rospy.is_shutdown():
                rospy.on_shutdown(nodename)
            if (magnet_state == magnet.pick):
                publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 1)
            if (magnet_state == magnet.place):
                publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 0)

            print(" 1 for motor pick ok !")
            # gpio = input()     # for testing
            gpio = '1'  # for testing

        gpio = '0'

    goal = False

class MoveItPlanner_c:

    
    g =0

    def __init__(self):
        group_name = "arm"
        self.robot = RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.pose = PoseStamped()
        self.count = int(0)
        self.goal_or_not = False
        self.x_goal = float(100000.0)
        self.y_goal = float(100000.0)


    def calculate_goal_points2(self, x, y, z):  # last working


        if x is not self.x_goal or y is not self.y_goal:
            if 1:      # joint control
                joint_goal = self.move_group.get_current_joint_values()
                joint_goal[0] = 1.05
                joint_goal[1] = 0.12
                joint_goal[2] = 2.0
                joint_goal[3] = 0

                # The go command can be called with joint values, poses, or without any
                # parameters if you have already set the pose or joint target for the group
                self.move_group.go(joint_goal, wait=True)


                # Calling ``stop()`` ensures that there is no residual movement
                self.move_group.stop()
                testfun(0)

            self.x_goal = x
            self.y_goal = y

            #pose_goal = geometry_msgs.msg.Pose()
            #pose_goal.orientation.w = 0.0
            #pose_goal.orientation.x = 0.0
            #pose_goal.orientation.y = 0.0
            #pose_goal.orientation.z = 0.0
            #pose_goal.position.x = 0.35
            #pose_goal.position.y = 0.35
            #pose_goal.position.z = 0.1


            self.pose.pose.position.x = 0.35
            self.pose.pose.position.y = 0.35
            self.pose.pose.position.z = 0.1


            #self.pose.pose.position.x = 0.35
            #self.pose.pose.position.y = 0.35
            #self.pose.pose.position.z = 0.1

            #self.arm.set_joint_value_target(self.pose, self.arm.get_end_effector_link(), True)


            self.move_group.set_pose_target(self.pose,self.move_group.get_end_effector_link())

            testfun(1)
            self.plan = self.move_group.plan()
            testfun(2)
            self.move_group.go(wait=True)

            #self.plan = move_group.go(wait=True)
            testfun(3)
            # Calling `stop()` ensures that there is no residual movement
            self.move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            self.move_group.clear_pose_targets()

            #self.move_group.execute( wait=True)


            self.l = (len(self.plan[1].joint_trajectory.points)) - 1
            test = input()  # test
            print("x", x)  # test
            print("y", y)  # test
            print("z", z)  # test
            self.First_joint_angels = []
            self.Second_joint_angels = []
            self.Third_joint_angels = []
            for i in range(self.l + 1):
                self.First_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[0])
                self.Second_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[1])
                self.Third_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[2])

            self.count = 0

        next_first_joint = self.First_joint_angels[self.count]
        next_second_joint = self.Second_joint_angels[self.count]
        next_third_joint = self.Third_joint_angels[self.count]
        self.count = self.count + 1

        test = input()  # test
        print("next_first_joint", next_first_joint)  # test
        print("next_second_joint", next_second_joint)  # test
        print("next_third_joint", next_third_joint)  # test

        if self.count >= (self.l + 1):
            self.goal_or_not = True
            if self.count > self.l:
                self.count = self.l

        return next_first_joint, next_second_joint, next_third_joint, self.goal_or_not


    def calculate_goal_points(self, x, y, z,coordinates):   # last working

        target = 'old'

        if  not x == self.x_goal    or  not   y == self.y_goal:
            target = 'new'
            self.x_goal = x
            self.y_goal = y
            #print("x y")
            #x=input()
            #y=input()
            #x=float(x)
            #y=float(y)

            if coordinates == 0:

                self.pose.pose.position.x = x
                self.pose.pose.position.y = y
                self.pose.pose.position.z = z
                self.pose.pose.orientation.w = 1
                self.pose.pose.orientation.x = 0
                self.pose.pose.orientation.y = 0
                self.pose.pose.orientation.z = 0


            if coordinates == 1:
                Transpose = np.array([[0, -1, 0.10], [-1, 0, 0.444], [0, 0, 1]])
                point_camera = np.array([x, y, 1])[np.newaxis]
                point_camera = point_camera.T
                #print(1111111111111111111111111111111111111111)
                #print(point_camera)
                point_scara = Transpose @ point_camera
                print(point_scara)
                self.pose.pose.position.x = float(point_scara[0])
                self.pose.pose.position.y = float(point_scara[1])
                self.pose.pose.position.z = z
                self.pose.pose.orientation.w = 1
                self.pose.pose.orientation.x = 0
                self.pose.pose.orientation.y = 0
                self.pose.pose.orientation.z = 0
                #testfun(100)
                #print(" x :",self.pose.pose.position.x )
                #print(" y :", self.pose.pose.position.y)
                #testfun(200)


            print("start_calc")  # testing
            self.move_group.set_joint_value_target(self.pose.pose, self.move_group.get_end_effector_link(),False)




            #self.move_group.set_pose_target(self.pose.pose)

            self.plan = self.move_group.plan()


            self.move_group.go(wait=True)

            #testfun(100)
            self.move_group.stop()

            #self.move_group.clear_pose_targets()

            #display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            #display_trajectory.trajectory_start = self.robot.get_current_state()
            #display_trajectory.trajectory.append(self.plan)
            # Publish
            #self.display_trajectory_publisher.publish(display_trajectory)

            #testfun(9)

            #self.move_group.execute(self.plan , wait = True)

            #testfun(10)

            self.l = (len(self.plan[1].joint_trajectory.points))
            #test = input()              #test
            #print("x",x)         #test
            #print("y", y)  # test
            #print("z", z)  # test
            self.First_joint_angels = []
            self.Second_joint_angels = []
            self.Third_joint_angels = []
            self.fourth_joint_angels = []
            for i in range(self.l):
                self.First_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[0])
                self.Second_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[1])
                self.Third_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[2])
                self.fourth_joint_angels.append(self.plan[1].joint_trajectory.points[i].positions[3])

            self.count = 0
        
        #next_first_joint = self.First_joint_angels[self.count]         # for step by step
        #next_second_joint = self.Second_joint_angels[self.count]       #
        #next_third_joint = self.Third_joint_angels[self.count]
        #next_fourth_joint = self.fourth_joint_angels[self.count]#
        #self.count = self.count + 1                                    #

        next_first_joint = self.First_joint_angels[self.l-1]
        next_second_joint = self.Second_joint_angels[self.l-1]
        next_third_joint = self.Third_joint_angels[self.l-1]
        next_fourth_joint = self.fourth_joint_angels[self.l - 1]
        self.count = self.l




        if self.count >= (self.l) or self.l == 1:
            self.goal_or_not = True
            if self.count >= self.l:
                self.count = (self.l-1)
        else:
            self.goal_or_not = False

        #joint_goal = self.move_group.get_current_joint_values()
        #joint_goal[0] = next_first_joint
        #joint_goal[1] = next_second_joint
        #joint_goal[2] = next_third_joint
        #joint_goal[3] = next_fourth_joint

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        #self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        #self.move_group.stop()

        # test = input()  # test
        print("=========================================")
        print("x : ", x)  # test
        print("y : ", y)  # test
        print("z : ", z)  # test
        print("next_first_joint : ", next_first_joint)  # test
        print("next_second_joint : ", next_second_joint)  # test
        print("next_third_joint : ", next_third_joint)  # test
        print("goal_or_not : ", self.goal_or_not)
        print("l : ", self.l)
        print("count : ", self.count)
        print("target : ", target)
        print("=========================================")

        return next_first_joint, next_second_joint, next_third_joint, self.goal_or_not



    def calculate_goal_points22(self, x, y, z, solve_or_not):
        
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
        arr=[0,0,0,0]
        self.goal_msg = Float32MultiArray(data=arr)
        self.counter=0
    def Publish_if_okay(self, First_waypoint, Second_waypoint,Third_waypoint,l,flag):
        if (flag==1 and self.counter<l):

            self.goal_msg[0] = np.rad2deg(First_waypoint[self.counter])
            self.goal_msg[1] = np.rad2deg(Second_waypoint[self.counter])
            self.goal_msg[2] = np.rad2deg(Third_waypoint[self.counter])
            self.counter = self.counter+1
            rospy.loginfo(self.goal_msg)
            self.pub.publish(self.goal_msg)

    def Publish(self,first_joint, second_joint, third_joint, elctromagnet_state , z_state):         # last working
        #self.goal_msg[0] = np.rad2deg(first_joint)
        #self.goal_msg[1] = (second_joint)*((1000*2*3.14)/2)*57.2958
        #self.goal_msg[2] = np.rad2deg(third_joint)
        #self.goal_msg[3] = np.rad2deg(elctromagnet_state)

        first_joint = (first_joint*(180/3.14))
        second_joint = (second_joint*(180/3.14))
        third_joint = (third_joint*(180/3.14))
        #elctromagnet_state = (elctromagnet_state*(180/3.14))

        first_joint  =      int( first_joint)
        second_joint  =      int(second_joint)
        third_joint    =     int(third_joint )
        #elctromagnet_state = int(elctromagnet_state)

        first_joint = float(first_joint)
        second_joint = float(second_joint)
        third_joint = float(third_joint)
        #elctromagnet_state = float(elctromagnet_state)

        arr = [first_joint, second_joint, -1*(third_joint), elctromagnet_state, z_state]
        self.goal_msg = Float32MultiArray(data=arr)

        rospy.loginfo(self.goal_msg)
        self.pub.publish(self.goal_msg)


pick_or_place=pick_and_place_e.pick             # initial state

system_status = system_status_enum.hold         # initial state

homed = False
percived = False
sorted = False
restart = False
configured = False


planner = MoveItPlanner_c()
publisher = Publisher_c()

class sorting_area_1_postion(object):
    x = float
    y = float
    state = empty_or_not.empty

class sorting_area_2_postion(object):
    x = float
    y = float
    state = empty_or_not.empty
        
lll=input()
'''
sorting_area_1=np.ndarray(shape=(3,3),dtype=object)
sorting_area_2=np.ndarray(shape=(3,3),dtype=object)

sorting_area_1[:,:] = sorting_area_postion
sorting_area_2[:,:] = sorting_area_postion
'''
#for i in range(3):
#    for j in range(3):
#        sorting_area_1[i,j] = sorting_area_postion
#for k in range(3):
#    for s in range(3):
#        sorting_area_2[k,s] = sorting_area_postion

sorting_area_1 = []

for i in range(4):
    sorting_area_1.append(sorting_area_1_postion())

sorting_area_2 = []

for j in range(4):
    sorting_area_2.append(sorting_area_2_postion())

sorting_area_1[0].x=0.3
sorting_area_1[0].y=-0.07
sorting_area_1[0].state= empty_or_not.empty

sorting_area_1[1].x=0.3
sorting_area_1[1].y=0.0
sorting_area_1[1].state= empty_or_not.empty

sorting_area_1[2].x=0.25
sorting_area_1[2].y=-0.07
sorting_area_1[2].state= empty_or_not.empty

sorting_area_1[3].x=0.25
sorting_area_1[3].y=0.0
sorting_area_1[3].state= empty_or_not.empty

sorting_area_2[0].x=0.3
sorting_area_2[0].y=0.07
sorting_area_2[0].state= empty_or_not.empty

sorting_area_2[1].x=0.3
sorting_area_2[1].y=0.14
sorting_area_2[1].state= empty_or_not.empty

sorting_area_2[2].x=0.25
sorting_area_2[2].y=0.07
sorting_area_2[2].state= empty_or_not.empty

sorting_area_2[3].x=0.25
sorting_area_2[3].y=0.14
sorting_area_2[3].state= empty_or_not.empty


#print(sorting_area_1[0].x)
#print(sorting_area_1[1].x)

'''
m = 3
n = 3
sorting_area_1 = [[0 for x in range(n)] for x in range(m)]
sorting_area_2 = [[0 for x in range(n)] for x in range(m)] 

sorting_area_1[0][0] = sorting_area_postion
sorting_area_1[0][0].x = 200.0
sorting_area_1[0][1] = sorting_area_postion
sorting_area_1[0][1].x = 240.0
sorting_area_1[0][2] = sorting_area_postion
sorting_area_1[0][2].x = 280.0
sorting_area_1[1][0] = sorting_area_postion
sorting_area_1[1][0].x = 200.0
sorting_area_1[1][1] = sorting_area_postion
sorting_area_1[1][1].x = 240.0
sorting_area_1[1][2] = sorting_area_postion
sorting_area_1[1][2].x = 280.0
sorting_area_1[2][0] = sorting_area_postion
sorting_area_1[2][0].x = 200.0
sorting_area_1[2][1] = sorting_area_postion
sorting_area_1[2][1].x = 240.0
sorting_area_1[2][2] = sorting_area_postion
sorting_area_1[2][2].x = 280.0
sorting_area_1[0][0] = sorting_area_postion
sorting_area_1[0][0].y = 200.0
sorting_area_1[0][1] = sorting_area_postion
sorting_area_1[0][1].y = 200.0
sorting_area_1[0][2] = sorting_area_postion
sorting_area_1[0][2].y = 200.0
sorting_area_1[1][0] = sorting_area_postion
sorting_area_1[1][0].y = 240.0
sorting_area_1[1][1] = sorting_area_postion
sorting_area_1[1][1].y = 240.0
sorting_area_1[1][2] = sorting_area_postion
sorting_area_1[1][2].y = 240.0
sorting_area_1[2][0] = sorting_area_postion
sorting_area_1[2][0].y = 280.0
sorting_area_1[2][1] = sorting_area_postion
sorting_area_1[2][1].y = 280.0
sorting_area_1[2][2] = sorting_area_postion
sorting_area_1[2][2].y = 280.0

sorting_area_2[0][0] = sorting_area_postion
sorting_area_2[0][0].x = 100.0
sorting_area_2[0][1] = sorting_area_postion
sorting_area_2[0][1].x = 140.0
sorting_area_2[0][2] = sorting_area_postion
sorting_area_2[0][2].x = 180.0
sorting_area_2[1][0] = sorting_area_postion
sorting_area_2[1][0].x = 100.0
sorting_area_2[1][1] = sorting_area_postion
sorting_area_2[1][1].x = 140.0
sorting_area_2[1][2] = sorting_area_postion
sorting_area_2[1][2].x = 180.0
sorting_area_2[2][0] = sorting_area_postion
sorting_area_2[2][0].x = 100.0
sorting_area_2[2][1] = sorting_area_postion
sorting_area_2[2][1].x = 140.0
sorting_area_2[2][2] = sorting_area_postion
sorting_area_2[2][2].x = 180.0
sorting_area_2[0][0] = sorting_area_postion
sorting_area_2[0][0].y = 100.0
sorting_area_2[0][1] = sorting_area_postion
sorting_area_2[0][1].y = 100.0
sorting_area_2[0][2] = sorting_area_postion
sorting_area_2[0][2].y = 100.0
sorting_area_2[1][0] = sorting_area_postion
sorting_area_2[1][0].y = 140.0
sorting_area_2[1][1] = sorting_area_postion
sorting_area_2[1][1].y = 140.0
sorting_area_2[1][2] = sorting_area_postion
sorting_area_2[1][2].y = 140.0
sorting_area_2[2][0] = sorting_area_postion
sorting_area_2[2][0].y = 180.0
sorting_area_2[2][1] = sorting_area_postion
sorting_area_2[2][1].y = 180.0
sorting_area_2[2][2] = sorting_area_postion
sorting_area_2[2][2].y = 180.0

sorting_area_1_= np.array



print(sorting_area_1[0][1].x)
'''
nodename = rospy.get_name()
rospy.loginfo("%s started" % nodename)

if __name__ == '__main__':
    try:



        '''
        while  1 :
            if rospy.is_shutdown():
                rospy.on_shutdown(nodename)
            print("start")
            coins_arr_0 = 0.35
            coins_arr_1 = 0.35
            z_level = 0.00
            #coins_arr_0 = input()
            print("coins_arr_0", coins_arr_0)
            #coins_arr_1 = input()
            #z_level =   input()
            print("x y")
            x = input()
            y = input()
            x = float(x)
            y = float(y)
            next_joint_0, next_joint_1, next_joint_2, goal = planner.calculate_goal_points(x,y, 0.1)
            print("next_joint_0",next_joint_0)
            print("next_joint_1", next_joint_1)
            print("next_joint_2", next_joint_2)
            print("goal", goal)
        '''
        count2 = 0
        count2 = int(count2)
        while not rospy.is_shutdown():


            while(0):

                print("dooos")
                zero1 = input()
                first = 0
                second = input()
                third = 0

                zero1 = float(zero1)
                first = float(first)
                second = float(second)
                third = float(third)

                #planner.calculate_goal_points(zero1,second,third)

                publisher.Publish(zero1,first,second, third)  # publish zeros to all joints

            #while home_check is not 1:
            #    home_check =input()
            if rospy.is_shutdown():
                rospy.on_shutdown(nodename)

            if system_status == system_status_enum.hold:   # system control
                print("hold")
                if not configured and not homed and not percived and not sorted:
                    system_status = system_status_enum.confg
                if configured and homed and not percived and not sorted :
                    system_status = system_status_enum.perceive
                if configured and homed and percived and not sorted:
                    system_status = system_status_enum.sort
                if configured and homed and percived and sorted:
                    rospy.loginfo(" process finished, Enter 1 for restart without homing or 2 for restart with homeing or 3 for Exit")
                    order=input()
                    if order == 1 :
                        configured = True
                        homed = True
                        percived = False
                        sorted = False

                    if order == 2:
                        configured = True
                        homed = False
                        percived = False
                        sorted = False

                    if order == 3:
                        rospy.on_shutdown(nodename)
                    
                    

            
            if system_status == system_status_enum.confg:
                rospy.loginfo(" you are now running ''sorting system'' ")
                home_check = int(0)
                z_level = float(0.1)

                z_level_down = float(0.0)
                z_level_up = float(0.1)


                goal = False
                configured = True
                gpio = False  # rassperpy gpio from arduino
                system_status = system_status_enum.home
                
            
            if system_status == system_status_enum.home:
                planner.calculate_goal_points(-0.1,-0.2,0.1,0)

                rospy.loginfo(" please move joints to its limits positons, then press the botton and enter < 1 > ")
                while not home_check == '1':
                    home_check =input()
                    zero = 0.0
                    zero = float(zero)
                    
                #publisher.Publish(zero, zero, zero, zero) # publish zeros to all joints
                planner.calculate_goal_points(0.3,0.0,z_level,0)

                print("press enter if the arm at home position")
                kk =input()
                goal = False
                homed = True
                system_status = system_status_enum.hold
                

            if system_status == system_status_enum.perceive: 
                ##########################################################
                ########## perciption ###########
                #################################
                print("persived")
                #persived = input()
                no_of_coins = int(4)
                coins_arr=np.ndarray(shape=(no_of_coins,3),dtype=float)
                coins_arr[0][0]=0.199 #0.1   # test - coin postion
                coins_arr[0][1]= 0.043# 0.3  # test - coin postion
                coins_arr[0][2]=1.0     # test - coin cat

                coins_arr[1][0]=0.116 #  0.0   # test - coin postion
                coins_arr[1][1]=0.047 #0.20   # test - coin postion
                coins_arr[1][2]=0.0     # test - coin cat

                coins_arr[2][0] =0.204 #-0.11  # test - coin postion
                coins_arr[2][1] =0.134  # 0.26  # test - coin postion
                coins_arr[2][2] = 1.0  # test - coin cat

                coins_arr[3][0] = 0.106 # -0.03  # test - coin postion
                coins_arr[3][1] = 0.129 #0.31  # test - coin postion
                coins_arr[3][2] = 0.0  # test - coin cat

                #print(coins_arr[0][0])
                #no_of_coins = coins_arr.shape([0])
                #################################
                #################################
                percived = True
                system_status = system_status_enum.hold
                ##########################################################

            if system_status == system_status_enum.sort:
                ############################
                ######### sorting ##########
                ############################
                '''
                 for i in range(no_of_coins):
                    while not goal:
                        next_joint_1, next_joint_2, next_joint_3, goal =planner(coins_arr[i,0], coins_arr[i,1], z_level )  
                        while not gpio:         # to be the motor error state
                            publisher(next_joint_1, next_joint_2, next_joint_3, 0)
                            gpio = input()     # for testing
                 sorted =True
                '''
                #print("nocoin")
                #nocoin = input()  # for testing
                coins_arr_2 = coins_arr

                if not (pow((coins_arr_2[0][0]),2)+pow((coins_arr_2[0][1]),2) > pow(0.34,2)):


                    if  not coins_arr[0][0] == 0.0 or not coins_arr[0][1] == 0.0:
                     #   print("coin")
                        if pick_or_place ==pick_and_place_e.pick:
                            while  goal == False:
                                if rospy.is_shutdown():
                                    rospy.on_shutdown(nodename)
                                print("gpio_picking")
                                next_joint_0, next_joint_1, next_joint_2, goal =planner.calculate_goal_points(coins_arr_2[count2][0], coins_arr_2[count2][1], z_level,1 )
                                while not gpio== '1':         # to be the motor error state
                                    #print("picking")
                                    if rospy.is_shutdown():
                                        rospy.on_shutdown(nodename)
                                    publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 0.0,2.0)   # z hold

                                    rate.sleep()
                                    rate.sleep()
                                    rate.sleep()
                                    rate.sleep()
                                    print("4 sec")
                                    publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 1.0,0.0)    # z down
                                    rate.sleep()
                                    rate.sleep()
                                    print("2 sec")
                                    publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 1.0,1.0)    # z up
                                    rate.sleep()
                                    rate.sleep()
                                    print("2 sec")
                                    publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 1.0,2.0)   # z hold

                                    print(" 1 for motor pick ok !")
                                    #gpio = input()     # for testing
                                    gpio = '1'           # for testing
                                gpio = '0'
                            # delay
                            #magnet_pick(magnet.pick, coins_arr_2[count2][0], coins_arr_2[count2][1], z_level_down, z_level_up)
                            print("turn elctromagnit on")
                            pick_or_place = pick_and_place_e.place
                            # delay
                            goal = False







                        if pick_or_place ==pick_and_place_e.place:
                            if coins_arr_2[count2][2] == 1.0:       # for pound
                                count = 0
                                count = int(count)
                                while sorting_area_1[count].state != empty_or_not.empty:
                                    count = count + 1
                                 #   print("counttttttttttt_in: " ,  count)
                                #print("counttttttttttt_out: ", count)
                                while goal == False:
                                    if rospy.is_shutdown():
                                        rospy.on_shutdown(nodename)
                                    print("gpio_placing_pound")
                                    next_joint_0, next_joint_1, next_joint_2, goal =planner.calculate_goal_points(sorting_area_1[count].x, sorting_area_1[count].y, z_level,0 )
                                    while not gpio=='1':         # to be the motor error state
                                        if rospy.is_shutdown():
                                            rospy.on_shutdown(nodename)

                                        publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 1.0, 2.0)  # z hold

                                        rate.sleep()
                                        rate.sleep()
                                        rate.sleep()
                                        rate.sleep()
                                        publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 1.0, 0.0)  # z down
                                        rate.sleep()
                                        rate.sleep()
                                        #publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 0.0, 0.0)  # z down
                                        publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 0.0, 1.0)  # z up
                                        rate.sleep()
                                        rate.sleep()
                                        publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 0.0, 2.0)  # z hold

                                        print(" 1 for motor place pound ok !")
                                        #gpio = input()     # for testing
                                        gpio = '1'  # for testing
                                    gpio= "0"
                                #sorting_area_1[count].state == empty_or_not.not_empty
                                # delay
                                #print("turn elctromagnet off pound")
                                #pick_or_place = pick_and_place_e.pick
                                #goal = False
                                sorting_area_1[count].state = empty_or_not.not_empty
                                # delay
                                #magnet_pick(magnet.place, coins_arr_2[count2][0], coins_arr_2[count2][1], z_level_down,z_level_up)
                                print("turn elctromagnet off ")
                                pick_or_place = pick_and_place_e.pick
                                goal = False



                            if coins_arr_2[count2][2] == 0.0:        # for 0.5
                                count = 0
                                count = int(count)
                                while  sorting_area_2[count].state != empty_or_not.empty:
                                    count = count +1
                                    if rospy.is_shutdown():
                                        rospy.on_shutdown(nodename)
                                while goal == False:

                                    print("gpio_place_0.5")
                                    next_joint_0, next_joint_1, next_joint_2, goal =planner.calculate_goal_points(sorting_area_2[count].x, sorting_area_2[count].y, z_level,0 )
                                    while not gpio=='1':         # to be the motor error state
                                        if rospy.is_shutdown():
                                            rospy.on_shutdown(nodename)


                                        publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 1.0, 2.0)  # z hold

                                        rate.sleep()
                                        rate.sleep()
                                        rate.sleep()
                                        rate.sleep()
                                        publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 1.0, 0.0)  # z down
                                        rate.sleep()
                                        rate.sleep()
                                        # publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 0.0, 0.0)  # z down
                                        publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 0.0, 1.0)  # z up
                                        rate.sleep()
                                        rate.sleep()
                                        publisher.Publish(next_joint_0, next_joint_1, next_joint_2, 0.0, 2.0)  # z hold

                                        print(" 1 for motor place 0.5 ok !")
                                        #gpio = input()     # for testing
                                        gpio = '1'          # for testing
                                    gpio = '0'


                                sorting_area_2[count].state = empty_or_not.not_empty
                                # delay
                                #magnet_pick(magnet.place, coins_arr_2[count2][0], coins_arr_2[count2][1], z_level_down,z_level_up)
                                print("turn elctromagnet off ")
                                pick_or_place = pick_and_place_e.pick
                                goal = False

                            count2 = count2 +1

                    ############################
                    ############################
                    else:
                        print("7ot coins pls ")
                else:
                    print("point is outside robot area , (x,y) = ", coins_arr[0][0] , coins_arr[0][1]  )
                    print("zo2 al coin gowa shwya ")
    except rospy.ROSInterruptException:
        pass

