#! /usr/bin/env python

"""
.. module:: RandomMovement
    :platform: Unix
    :synopsis: Python module for robot random movement
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

ROS node for implementing the RANDOM_MOVEMENT state of the finite state machine FSM.

This node manages the movement of the robot based on information obtained about the 
target room chosen by the FSM. 
Based on the position and battery status it informs the FSM if the final position has 
been reached or if the goal has been cancelled.

Publishes to:
    /A/jointC_position_controller/command to publish the camera joint position 

Subscribes to:
    /Room to establish the target room

    /odom where the simulator publishes the robot position

Client:
    ArmorClient

    MoveBaseAction

    /BLevel to manage the robot's battery level

Service:
    /Mapping_Switch to active the RANDOM_MOVEMENT state

    MoveBaseGoal

"""
import roslib
import time
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.msg import Odometry
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
    """
    Class representing the robot's movement based on information about the target room 
    chosen by the FSM.
    Once the final position of the robot is defined, the distance between the starting 
    point and the goal is calculated and thus the battery consumption for that path is 
    defined.
    If the robot reaches the final position, the FSM is informed that the destination 
    has been reached, but if the battery level is not sufficient, the goal is cancelled 
    and the robot is sent to room E to recharge.
    ...
    Methods
    ----------
    __init__(self)
    RoomCB(self,Location)
    OdomCB(self,data)
    MovementSwitchCB(self,req)
    """
    def __init__(self):
        """
        Initialisation function
        """

        # Initialisation service
        Mov_srv = rospy.Service('/Movement_Switch', Trigger, self.MovementSwitchCB)
        # Initialisation client
        self.Bat_Client = rospy.ServiceProxy('/BLevel', BatteryLow)
        # Initialisation publisher and subscriber
        self.Pub_PoseCamera = rospy.Publisher('/A/jointC_position_controller/command', Float64, queue_size=1)
        self.Sub_Room = rospy.Subscriber('/Room', String, self.RoomCB)
        self.Sub_Odom = rospy.Subscriber('/odom', Odometry, self.OdomCB)

        # Inialisation of the MoveBase action client
        self.MBClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.Goal = MoveBaseGoal()

        # MoveIt
        self.Group = _moveit_move_group_interface.MoveGroupInterface("arm", "robot_description", "")
        self.Omega = Float64();
        self.Omega.data = 0.0

        self.previous_x = -6.0
        self.previous_y = 11.0

        self.Room = 'E'

        self.DestReach = True

    # Room callback
    def RoomCB(self,Location):
        """
        Callback to obtain information about the robot's target location.
        
        Args:
            Location (string): location chosen as destination of the robot via the ontology
        
        """
        self.Room = Location.data   # Choosen location

    # Odometry callback
    def OdomCB(self,data):
        """
        Callback to obtain the actual position of the robot via odometry. 
        
        Args:
            data (float): information about the odometry

        """
        global actual_x, actual_y

        actual_x = data.pose.pose.position.x
        actual_y = data.pose.pose.position.y

    # /Movement_Switch service callback
    def MovementSwitchCB(self,req):
        """
        Function to inform the finished state machine FSM if the robot has reached the final position.
       
        Returns:
            res.success (bool): indicates successful run of triggered service

            res.message (string): informational
        
        """
        print('RANDOM_MOVEMENT')

        self.MoveBaseA(self.Room)       # Move the robot to the choosen location

        res = TriggerResponse()
        
        if self.DestReach == True:
            res.message = '\033[92mFINAL POSITION REACHED\033[0m'
            res.success = True
        else:
            res.message = '\033[91mGOAL CANCELLED\033[0m'
            res.success = False

        print(f'{res.message}')

        return res

# NAVIGATION
    def MoveBaseA(self,Loc):
        """
        Function to control the movement of the robot.

        When the robot reaches the final position, it inspects the room.

        The goal is cancelled if the battery is less than 30%. The battery check is performed considering the distance the robot travels
        
        Args:
            Loc (string): location where the robot is to arrive

        Returns:
            DestReach (bool): reaching the final destination

        """
        global actual_x, actual_y

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

        dist = math.sqrt(pow(self.Goal.target_pose.pose.position.x - self.previous_x, 2) +
                    pow(self.Goal.target_pose.pose.position.y - self.previous_y, 2))
        print(f'To reach {Loc} I need to travel {round(dist)}m')

        # Positioning the robotic arm in the 'Home' configuration
        self.Group.set_named_target("HomePose")
        self.Group.move()
        
        print('Waiting for the result ...')

        self.MBClient.wait_for_result()
        resp = self.Bat_Client(round(dist))

        x = abs(self.Goal.target_pose.pose.position.x - actual_x)
        y = abs(self.Goal.target_pose.pose.position.y - actual_y)        

        if resp.LevelF < 30 and Loc != 'E':
            # Cancel goal
            self.MBClient.cancel_goal()
            self.DestReach = False
        elif (x <= 1 and y <= 1) or Loc == 'E':
            print(f'{Loc} reached')
            self.Omega.data = 0.0
            self.Pub_PoseCamera.publish(self.Omega)

            # RGB camera rotation around the z-axis
            print('Inspection')
            while self.Omega.data <= 2*math.pi:
                self.Omega.data += math.pi/4
                print(self.Omega.data)
                self.Pub_PoseCamera.publish(self.Omega)
                self.Group.move()   
                time.sleep(0.5)

            self.Pub_PoseCamera.publish(0.0)
            self.DestReach = True
        else:
            # Cancel goal
            self.MBClient.cancel_goal()
            print('The robot failed to reach the goal for some reason')
            self.DestReach = False


        self.previous_x = F.CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['hasCoordinatesX', Loc]))[0]
        self.previous_y = F.CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['hasCoordinatesY', Loc]))[0]

        self.previous_x = float(self.previous_x)
        self.previous_y = float(self.previous_y)

        return self.DestReach


if __name__ == "__main__":

    # Initialisation node
    rospy.init_node('RandomMovement')
    
    RandMove = RandomMovement()

    # Wait for ctrl-c to stop the application
    rospy.spin()
