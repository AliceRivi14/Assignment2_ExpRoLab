#! /usr/bin/env python

"""
.. module:: RandomMovement
    :platform: Unix
    :synopsis: Python module for robot random movement
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

ROS node for implementing the RANDOM_MOVEMENT state of the finite state machine FSM.

Through this node, the movement of the robot in random positions is simulated.

Client:
    ArmorClient

    MoveBaseAction

Service:
    /Mapping_Switch to active the RANDOM_MOVEMENT state

    MoveBaseGoal

"""
import random
import roslib
import time
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from std_srvs.srv import *
from assignment2.srv import *
# from armor_api.armor_client import ArmorClient
import math

from moveit_ros_planning_interface import _moveit_move_group_interface
from std_msgs.msg import Int32MultiArray, Float64

import Functions as F
import StateMachine as SM

Active = False


class RandomMovement:
    def __init__(self):

        # Initialisation service, publisher and subscriber
        Mov_srv = rospy.Service('/Movement_Switch', SetBool, self.MovementSwitchCB)

        self.Pub_PoseCamera = rospy.Publisher('/A/jointC_position_controller/command', Float64, queue_size=1)
        self.Pub_BLevel = rospy.Publisher('/B_Level', Int32MultiArray, queue_size=1)

        self.Sub_Odom = rospy.Subscriber('/odom', Odometry, self.OdomCB)

        # Inialisation of the MoveBase action client
        self.MBClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.Goal = MoveBaseGoal()
        self.State = self.MBClient.get_state()

        # MoveIt
        self.Group = _moveit_move_group_interface.MoveGroupInterface("arm", "robot_description", "")
        self.Omega = Float64();

        self.previous_x = -6.0
        self.previous_y = 11.0

    # /Movement_Switch service callback
    def MovementSwitchCB(self,req):
        """
        Function to provide an implementation of an action which, given a position
        goal, will attempt to reach it.

        If the signal of battery low is sent, the goal is cancelled.
        """

        Active = req.data

        rospy.loginfo('Waiting for the MoveBase server ...')
        self.MBClient.wait_for_server()

        self.Goal.target_pose.header.frame_id = "map"
        self.Goal.target_pose.pose.orientation.w = 1.0;
        self.Goal.target_pose.header.stamp = rospy.Time.now()

        # Get list of room coordinates
        LocationCoord = rospy.get_param('Ids')
        # Get list of coordinates of each room
        CoordinatesLoc = rospy.get_param('Coordinate')


        RoomClient = rospy.ServiceProxy('/room_info', RoomInformation)
        rospy.loginfo('Waiting for the RoomInformation server ...')
        rospy.wait_for_service('/room_info')

        resp = RoomClient()
        resp.room = F.Destination()
        self.MoveBaseA(resp.room)

        res = SetBoolResponse()
        res.message = 'FINAL POSITION REACHED'
        res.success = True

        rospy.loginfo(f'{res.message}')

        Active = False

        return res


    def MoveBaseA(self,Loc):

        RoomClient = rospy.ServiceProxy('/room_info', RoomInformation)
        rospy.loginfo(f'Actual position: ({self.previous_x},{self.previous_y})')

        Loc = RoomClient()
        F.MoveRobot(Loc.room)
        self.Goal.target_pose.pose.position.x = Loc.x
        self.Goal.target_pose.pose.position.y = Loc.y

        rospy.loginfo(f'Moving to {Loc.room} at ({self.Goal.target_pose.pose.position.x},{self.Goal.target_pose.pose.position.y}) position')
        self.MBClient.send_goal(self.Goal)

        rospy.loginfo('Waiting for the result ...')
        self.MBClient.wait_for_result()

        dist = math.sqrt(pow(self.Goal.target_pose.pose.position.x - self.previous_x) +
                    pow(self.Goal.target_pose.pose.position.y - self.previous_y))
        self.Pub_BLevel.publish(dist)


        B_Low = SM.Battery_State()
        if B_Low == True:
            self.MBClient.cancel_goal()
            print('GOAL CANCELLED \n')
        elif self.State == GoalStatus.SUCCEEDED:
            # Positioning the robotic arm in the 'Home' configuration
            self.Group.set_named_target("HomePose")
            self.Group.move()
            # RGB camera rotation around the z-axis
            for self.Omega.data in range(0.0, math.pi/12, 2*math.pi):
                self.Pub_PoseCamera.publish(self.Omega)


    def OdomCB(self,data):
        self.previous_x = data.pose.pose.position.x
        self.previous_y = data.pose.pose.position.y


if __name__ == "__main__":

    # Initialisation node
    rospy.init_node('RandomMovement')
    # Initialisation service
    Bat_srv = rospy.Service('/B_Switch', BatteryLow, SM.Battery_State)

    # When the service /Movement_Switch is called, node manager class is instantiated
    while not rospy.is_shutdown():
        if Active == False:
            print("M")
            continue

        # Wait for ctrl-c to stop the application
        rospy.spin()
