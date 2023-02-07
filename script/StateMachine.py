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

import Functions as F

Battery_Client = None
Movement_Client = None
Map_Client = None
State_ = 1
B_Low = False

# 1 - TOPOLOGICAL_MAP state
# 2 - RANDOM_MOVEMENT state
# 3 - ROOM_E state

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

def ChangeState(State):
    """
    Function for communicating which node to execute based on the status of the FSM.

    Args:
        State (int): current status of the FSM

            1. TOPOLOGICAL_MAP
            2. RANDOM_MOVEMENT
            3. ROOM_E

    """
    global Battery_Client, Movement_Client, Map_Client
    global State_
    State_ = State
    # TOPOLOGICAL_MAP state
    if State_ == 1:
        resp = Map_Client(True)
        resp = Battery_Client(False)
        resp = Movement_Client(False)
    # RANDOM_MOVEMENT state
    if State_ == 2:
        resp = Map_Client(False)
        resp = Battery_Client(False)
        resp = Movement_Client(True)
    # ROOM_E state
    if State_ == 3:
        resp = Map_Client(False)
        resp = Battery_Client(True)
        resp = Movement_Client(False)

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
        rospy.loginfo('Executing state TOPOLOGICAL_MAP')
        ChangeState(1)
        time.sleep(1)
        resp = Map_Client()
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
        global B_Low
        rospy.loginfo('Executing state CHOOSE_DESTINATION')
        F.Destination()
        time.sleep(5)
        if B_Low == True:           # Recharging required
            return 'b_low'
        else:
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
        rospy.loginfo('Executing state RANDOM_MOVEMENT')
        ChangeState(2)
        time.sleep(5)
        resp = Movement_Client()
        if B_Low == True:               # Recharging required
            return 'b_low'
        elif resp.success == True:      # Room reached
            return 'move'
        else:
            time.sleep(5)
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
        rospy.loginfo('Executing state ROOM_E')
        ChangeState(3)
        time.sleep(5)
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
    B_srv = rospy.Service('/B_Switch', BatteryLow, Battery_State)

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
