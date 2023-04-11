#! /usr/bin/env python

"""
.. module:: RandomMovement
    :platform: Unix
    :synopsis: Python module for robot random movement
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

ROS node for implementing the RANDOM_MOVEMENT state of the finite state machine FSM.

???

Publishes to:
    /A/jointC_position_controller/command per pubblicare la posizione della camera

Subscribes to:
    /Room per determinare la stanza di destinazione

    /odom where the simulator publishes the robot position

Client:
    ArmorClient

    MoveBaseAction

    /BLevel per gestire il livello di batteria del robot

    /room_info ???

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
    Classe relativa al controllo del movimento del robot
    """
    def __init__(self):
        """
        Initialisation function
        """

        # Initialisation service
        Mov_srv = rospy.Service('/Movement_Switch', Trigger, self.MovementSwitchCB)
        # Initialisation client
        self.Bat_Client = rospy.ServiceProxy('/BLevel', BatteryLow)
        self.RoomClient = rospy.ServiceProxy('/room_info', RoomInformation)
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
        Callback per ottenere le informazioni riguardo alla location di destinazione del robot.
        
        Args:
            Location (string): location scelta come destinazione del robot attraverso l'ontologia
        """
        self.Room = Location.data   # Choosen location

    # Odometry callback
    def OdomCB(self,data):
        """
        Callback per ottenere l'effettiva posizione attuale del robot attraverso l'odometria.
        
        Args:
            data (float): informazione riguardo l'odometria
        
        """
        global actual_x, actual_y

        actual_x = data.pose.pose.position.x
        actual_y = data.pose.pose.position.y

    # /Movement_Switch service callback
    def MovementSwitchCB(self):
        """
        Funzione per informare la finita macchina a stati FSM se il robot ha raggiunto la posizione finale.
        
        Returns:
            res.success (bool): indicates successful run of triggered service

            res.message (string): informational
        
        """
        print('RANDOM_MOVEMENT')
        
        # BUG: NECESSARIO???
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
        """
        Funzione per controllare il movimentod del robot.

        Quando il robot raggiunge la posizione finale, esegue l'ispezione della stanza.

        Il goal viene cancellato nel caso in cui la batteria sia inferiore al 30%. Il controllo della batteria viene effettuato considerando la distanza che percorre il robot
        
        Args:
            Loc (string): location in cui deve arrivare il robot

        Returns:
            DestReach (bool): raggiungimento della destinazione finale
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

        dist = math.sqrt(pow(float(self.Goal.target_pose.pose.position.x) - self.previous_x, 2) +
                    pow(float(self.Goal.target_pose.pose.position.y) - self.previous_y, 2))
        print(f'To reach {Loc} I need to travel {round(dist)}m')

        # Positioning the robotic arm in the 'Home' configuration
        self.Group.set_named_target("HomePose")
        self.Group.move()
        
        print('Waiting for the result ...')
        self.MBClient.wait_for_result()
        resp = self.Bat_Client(round(dist))
        

        if resp.LevelF < 30 and Loc != 'E':
            # Cancel goal
            self.MBClient.cancel_goal()
            self.DestReach = False
        elif (actual_x >= self.Goal.target_pose.pose.position.x - 1.0 and actual_x <= self.Goal.target_pose.pose.position.x + 1.0) or (actual_y >= self.Goal.target_pose.pose.position.y - 1.0 and actual_y <= self.Goal.target_pose.pose.position.y + 1.0) or Loc == 'E':
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
