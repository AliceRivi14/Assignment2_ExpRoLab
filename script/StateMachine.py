#!/usr/bin/env python

"""
.. module:: StateMachine
    :platform: Unix
    :synopsis: Python module for the Finite State Machine
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

ROS node for implementing a finite state machine FSM.

Client:
    /Battery_Switch to active the ROOM_E state

    /Movement_Switch to active the RANDOM_MOVEMENT state

    /Mapping_Switch to active the TOPOLOGICAL_MAP state

Service:
    /B_Switch to communicate the need for recharging the battery

"""

import roslib
import rospy
import smach
import smach_ros
import time
import random

from assignment2.srv import *
# from armor_api.armor_client import ArmorClient
from std_srvs.srv import *
from std_msgs.msg import String

import Functions as F

Battery_Client = None
Movement_Client = None
Map_Client = None
B_Low = False

# Service callback
def Battery_State(req):
    """
    Service callback.

    Args:
        req (bool): notifies that the battery is low
    Returns:
        res (bool): indicates successful run of triggered service

    """
    global B_Low

    rospy.loginfo('RECHARGE REQUEST')
    B_Low = req.B_Low
    res = BatteryLowResponse()
    res.B_State = True  # Full battery
    return res

# State TOPOLOGICAL_MAP
class TOPOLOGICAL_MAP(smach.State):
    """
    Class implementing FSM state concerning the topological map.

    """
    def __init__(self):
        """
        Initialisation function

        """
        smach.State.__init__(self,
                             outcomes = ['map_OK','wait'])
    # Execution function
    def execute(self, userdata):
        """
        Function that executes the status of the FSM by calling the service
        related to the mapping situation.

        Returns:
            Transition of the FSM to be carried out
                - *wait*: if the map construction isn't finished
                - *map_OK*: when map construction ends

        """
        global Map_Client
        rospy.loginfo('Executing state TOPOLOGICAL_MAP')
        resp = Map_Client(True)
        if resp.success == True:    # Map construction ends
            return 'map_OK'
        else:
            time.sleep(5)
            return 'wait'

# State RANDOM_MOVEMENT
class CHOOSE_DESTINATION(smach.State):
    """
    Class implementing FSM sub state concerning the choice of location
    in which the robot is to move.

    """
    def __init__(self):
        """
        Initialisation function

        """
        smach.State.__init__(self,
                            outcomes = ['b_low', 'destination'])
    # Execution function
    def execute(self, userdata):
        """
        Function that executes the status of the FSM by executing a function
        to decide in which location the robot should move according to  the urgency.

        Returns:
            Transition of the FSM to be carried out
                - *b_low*: if the robot needs to be recharged
                - *destination*: when the location in which the robot is to move is chosen

        """
        global B_Low, Pub_Room
        rospy.loginfo('Executing state CHOOSE_DESTINATION')
        time.sleep(2)
        RoomID = F.Destination()        # Choice of the destination
        if B_Low == True:               # Recharging required
            return 'b_low'
        else:
            Pub_Room.publish(RoomID)    
            return 'destination'

# State RANDOM_MOVEMENT
class RANDOM_MOVEMENT(smach.State):
    """
    Class implementing FSM sub state concerning the random movement.

    """
    def __init__(self):
        """
        Initialisation function

        """
        smach.State.__init__(self,
                            outcomes = ['b_low', 'move', 'wait'])
    # Execution function
    def execute(self, userdata):
        """
        Function that executes the status of the FSM by calling the service
        related to the moving situation.

        Returns:
            Transition of the FSM to be carried out
                - *b_low*: if the robot needs to be recharged
                - *move*: if the robot can move between the rooms
                - *wait*: if the room is not reached yet or the goal has been cancelled
        """
        global B_Low
        global Movement_Client
        rospy.loginfo('Executing state RANDOM_MOVEMENT')
        resp = Movement_Client(True)
        if B_Low == True:               # Recharging required
            return 'b_low'
        elif resp.success == True:      # Room reached
            return 'move'
        else:
            time.sleep(2)
            return 'wait'

# State ROOM_E
class ROOM_E(smach.State):
    """
    Class implementing FSM state concerning the room E.

    """
    def __init__(self):
        """
        Initialisation function

        """
        smach.State.__init__(self,
                            outcomes = ['move', 'b_low'])
    # Execution function
    def execute(self, userdata):
        """
        Function that executes the status of the FSM by calling the service
        related to the recharging situation.

        Returns:
            Transition of the FSM to be carried out
                - *b_low*: if the robot needs to be recharged
                - *move*: if the robot can move between the rooms

        """
        global B_Low
        global Battery_Client
        rospy.loginfo('Executing state ROOM_E')
        resp = Battery_Client(True)
        if B_Low == True:       # Recharging required
            return 'b_low'
        else:
            return 'move'

def main():
    """
    This function initializes the ROS node, clients and service and waits for
    the creation and execution of the FSM.

    """
    global Battery_Client, Movement_Client, Map_Client

    # Initialisation node
    rospy.init_node('Robot_State_Machine')

    # Initialisation clients and service
    Battery_Client = rospy.ServiceProxy('/Recharging_Switch', SetBool)
    Movement_Client = rospy.ServiceProxy('/Movement_Switch', SetBool)
    Map_Client = rospy.ServiceProxy('/Mapping_Switch', SetBool)
    B_srv = rospy.Service('/BLevel_Switch', BatteryLow, Battery_State)

    Pub_Room = rospy.Publisher('/Room', String, queue_size=1)

    # Create a SMACH state machine
    SM = smach.StateMachine(outcomes = ['Container'])
    # Open the Container
    with SM:
        # Add states to the Container
        # Initial state
        smach.StateMachine.add('TOPOLOGICAL_MAP', TOPOLOGICAL_MAP(),
                               transitions = {'map_OK': 'SURVEILLANCE',
                                              'wait': 'TOPOLOGICAL_MAP'})

        # Create a sub SMACH state machine
        SubSM = smach.StateMachine(outcomes = ['recharging'])
        # Open the Sub container
        with SubSM:
        # Add states to the Sub Container
            smach.StateMachine.add('CHOOSE_DESTINATION', CHOOSE_DESTINATION(),
                                    transitions = {'b_low': 'recharging',
                                                   'destination': 'RANDOM_MOVEMENT'})

            smach.StateMachine.add('RANDOM_MOVEMENT', RANDOM_MOVEMENT(),
                                    transitions = {'b_low': 'recharging',
                                                   'move': 'CHOOSE_DESTINATION',
                                                   'wait': 'RANDOM_MOVEMENT'})

        smach.StateMachine.add('SURVEILLANCE', SubSM,
                                transitions = {'recharging': 'ROOM_E'})

        smach.StateMachine.add('ROOM_E', ROOM_E(),
                               transitions = {'move': 'SURVEILLANCE',
                                              'b_low': 'ROOM_E'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('Introspection', SM, '/SM_ROOT')
    sis.start() # Visualization

    # Execute the state machine
    outcome = SM.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
