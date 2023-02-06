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
from std_srvs.srv import *

# from geometry_msgs.msg import Point

from assignment2.srv import *
from armor_api.armor_client import ArmorClient

from moveit_ros_planning_interface import _moveit_move_group_interface
from std_msgs.msg import Float64

import Functions as F
import StateMachine as SM

Active = False
B_Low = False

class RandomMovement:
    def __init__(self):

        self.Pub_PoseCamera = rospy.Publisher('/A/jointC_position_controller/command',Float64, queue_size=1)

        self.Armor_Client_ID = 'User'
        self.Armor_ReferenceName = 'Ref'
        self.Armor_Client = ArmorClient(self.Armor_Client_ID, self.Armor_ReferenceName)

        self.MBClient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.Goal = MoveBaseGoal()
        self.State = self.MBClient.get_state()

        # MoveIt
        self.Group = _moveit_move_group_interface.MoveGroupInterface("arm", "robot_description", "")
        self.Omega = Float64();

    def MoveBaseA(self):
        """
        Function to provide an implementation of an action which, given a position
        goal, will attempt to reach it.

        If the position is not reached within a certain time (3.0 seconds) or if the
        signal of battery low is sent, the goal is cancelled.
        """

        rospy.loginfo('Waiting for the MoveBase server ...')
        MBClient.wait_for_server()

        self.Goal.target_pose.header.frame_id = "map"
        self.Goal.target_pose.pose.orientation.w = 1.0;
        self.Goal.target_pose.header.stamp = rospy.Time.now()

        # Get list of room coordinates
        LocationCoord = rospy.get_param('Ids')
        # Get list of coordinates of each room
        CoordinatesLoc = rospy.get_param('Coordinate')

        rospy.loginfo('Waiting for the RoomInformation server ...')
        rospy.wait_for_service('/room_info')
        RoomClient = rospy.ServiceProxy('/room_info', RoomInformation)
        resp = RoomClient()
        resp.room = F.Destination()

        Goal.target_pose.pose.position.x = resp.x
        Goal.target_pose.pose.position.y = resp.y

        rospy.loginfo(f'Moving to {resp.room} at ({Goal.target_pose.pose.position.x},{Goal.target_pose.pose.position.y}) position')
        self.MBClient.send_goal(Goal)
        F.MoveRobot(resp.room)

        rospy.loginfo('Waiting for the result ...')
        self.MBClient.wait_for_result()

        if B_Low == True:
            self.MBClient.cancel_goal()
            print('GOAL CANCELLED \n')
        elif self.State == GoalStatus.SUCCEEDED:
            # Positioning the robotic arm in the 'Home' configuration
            self.Group.set_named_target("HomePose")
            # RGB camera rotation around the z-axis
            for self.Omega.data in range(0.0,M_PI/12,2*M_PI):
                self.Pub_PoseCamera.publish(Omega)

# Service callback
def MovementSwitchCB(req):
    """
    Service callback.

    Args:
        req (bool): for enabling/disabling the service related to moving simulation

    Returns:
        res.success (bool): indicates successful run of triggered service

        res.message (string): informational
    """
    global Active, res

    Active = req.data
    res = SetBoolResponse()
    res.message = 'RANDOM_MOVEMENT state'
    res.success = True # Service enable
    return res


if __name__ == "__main__":

    # Initialisation node
    rospy.init_node('RandomMovement')
    # Initialisation service
    Mov_srv = rospy.Service('/Movement_Switch', SetBool, MovementSwitchCB)
    Bat_srv = rospy.Service('/B_Switch', BatteryLow, SM.Battery_State)

    RandMov = RandomMovement()

    # When the service /Movement_Switch is called, node manager class is instantiated
    while not rospy.is_shutdown():
        if Active == False:
            continue
        else:
            rospy.loginfo('I AM LOOKING FOR A LOCATION')
            RandMov.MoveBaseA()
            Active = False

        # Wait for ctrl-c to stop the application
        rospy.spin()
