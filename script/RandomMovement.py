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
import roslib
import time
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse
from assignment2.srv import *
from armor_api.armor_client import ArmorClient
import math

from moveit_ros_planning_interface import _moveit_move_group_interface
from std_msgs.msg import String, Float64

import Functions as F
Path = 'ERL_WS/src/assignment2/worlds/topological_map.owl'
IRI = 'http://bnc/exp-rob-lab/2022-23'
Robot = 'Robot1'

Armor_Client_ID = 'User'
Armor_ReferenceName = 'Ref'
Armor_Client = ArmorClient(Armor_Client_ID, Armor_ReferenceName)


class RandomMovement:
    def __init__(self):

        # Initialisation service
        Mov_srv = rospy.Service('/Movement_Switch', Trigger, self.MovementSwitchCB)

        self.Battery_Client = rospy.ServiceProxy('/Recharging_Switch', Trigger)
        self.RoomClient = rospy.ServiceProxy('/room_info', RoomInformation)
        self.Bat_Client = rospy.ServiceProxy('/BLevel', BatteryLow)

        self.Pub_PoseCamera = rospy.Publisher('/A/jointC_position_controller/command', Float64, queue_size=1)
        
        #self.Sub_Odom = rospy.Subscriber('/odom', Odometry, self.OdomCB)
        self.Sub_Room = rospy.Subscriber('/Room', String, self.RoomCB)

        # Inialisation of the MoveBase action client
        self.MBClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.Goal = MoveBaseGoal()
        self.State = self.MBClient.get_state()

        # MoveIt
        self.Group = _moveit_move_group_interface.MoveGroupInterface("arm", "robot_description", "")
        self.Omega = Float64();

        self.previous_x = -6.0
        self.previous_y = 11.0

        self.Room = 'E'

        self.DestReach = True

    def RoomCB(self,Location):
        self.Room = Location.data   # Choosen location

#    def OdomCB(self,data):
#        self.previous_x = data.pose.pose.position.x
#        self.previous_y = data.pose.pose.position.y

    # /Movement_Switch service callback
    def MovementSwitchCB(self,req):
        """
        Function to provide an implementation of an action which, given a position
        goal, will attempt to reach it.

        If the signal of battery low is sent, the goal is cancelled.
        """
        
        print('Waiting for the RoomInformation server ...')
        rospy.wait_for_service('/room_info')

        self.MoveBaseA(self.Room)       # Move the robot to the choosen location

        res = TriggerResponse()
        
        if self.DestReach == True:
            res.message = 'FINAL POSITION REACHED'
            res.success = True
        else:
            res.message = 'GOAL CANCELLED'
            res.success = False

        print(f'{res.message}')

        return res

# NAVIGATION
    def MoveBaseA(self,Loc):


        print('Waiting for the MoveBase server ...')
        self.MBClient.wait_for_server()

        self.Goal.target_pose.header.frame_id = "map"
        self.Goal.target_pose.pose.orientation.w = 1.0;
        self.Goal.target_pose.header.stamp = rospy.Time.now()

        print(f'Actual position: ({self.previous_x},{self.previous_y})')

        self.Goal.target_pose.pose.position.x = F.CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['hasCoordinatesX', Loc]))[0]
        self.Goal.target_pose.pose.position.y = F.CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['hasCoordinatesY', Loc]))[0]

        self.Goal.target_pose.pose.position.x = float(self.Goal.target_pose.pose.position.x)
        self.Goal.target_pose.pose.position.y = float(self.Goal.target_pose.pose.position.y)

        F.MoveRobot(Loc)

        print(f'Moving to ({self.Goal.target_pose.pose.position.x},{self.Goal.target_pose.pose.position.y}) position')
        
        self.MBClient.send_goal(self.Goal)

        print('Waiting for the result ...')

        dist = math.sqrt(pow(float(self.Goal.target_pose.pose.position.x) - self.previous_x, 2) +
                    pow(float(self.Goal.target_pose.pose.position.y) - self.previous_y, 2))
        print(f'To reach {Loc} I need to travel {round(dist)}m')
        resp = self.Bat_Client(round(dist))

        # DEBUG
        print(f'{self.State}')

        self.MBClient.wait_for_result()

        

        if resp.LevelF < 30:
            # Cancel goal
            self.MBClient.cancel_goal()
            self.DestReach = False
        #BUG: NON FUNZIONA --> robot non preciso
        elif self.State == GoalStatus.SUCCEEDED:
            print(f'{Loc} reached')
            self.DestReach = True
            # Positioning the robotic arm in the 'Home' configuration
            self.Group.set_named_target("HomePose")
            self.Group.move()

            # TODO: RGB camera rotation around the z-axis
            print('Inspection')
            for self.Omega.data in range(0.0, 2*math.pi, math.pi/12):
                self.Pub_PoseCamera.publish(self.Omega)
        else:
            # Cancel goal
            self.MBClient.cancel_goal()
            print('The robot failed to reach the goal for some reason')
            self.DestReach = False

        self.previous_x = F.CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['hasCoordinatesX', Loc]))[0]
        self.previous_y = F.CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['hasCoordinatesY', Loc]))[0]


        return self.DestReach


if __name__ == "__main__":

    # Initialisation node
    rospy.init_node('RandomMovement')

    RandMove = RandomMovement()

    # Wait for ctrl-c to stop the application
    rospy.spin()
