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

from assignment2.srv import *
# from armor_api.armor_client import ArmorClient
from std_srvs.srv import Trigger
from std_msgs.msg import String

import Functions as F

Battery_Client = None
Movement_Client = None
Map_Client = None
Pub_Room = None
BLev = 100

# Service callback
def Battery_State(req):
    """
    Service callback.

    Args:
        req (float): notifies that the battery level
    Returns:
        res (bool): 

    """
    global BLev
    res = BatteryLowResponse()
    # BUG: calcolo da sistemare
    BLev = BLev - req.LevelI
    print(f'Battery level = {BLev}%')
  
    if BLev < 30:
        res.LevelF = BLev      # Battery level when the final destination is reached
        print('I NEED TO BE RECHARGED')
        time.sleep(3)
    else:
        res.LevelF = BLev

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

        print('Executing state TOPOLOGICAL_MAP')
        time.sleep(0.5)
        respM = Map_Client()

        if respM.success == True:    # Map construction ends
            return 'map_OK'
        else:
            time.sleep(1)
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
        global Pub_Room, BLev

        print('Executing state CHOOSE_DESTINATION')
        time.sleep(0.5)
        RoomID = F.Destination()        # Choice of the destination
        time.sleep(1)

        if BLev < 30:                   # Recharging required
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
        """

        global Movement_Client
        global BLev

        print('Executing state RANDOM_MOVEMENT')
        time.sleep(0.5)
        respR = Movement_Client()
        time.sleep(1)

        # BUG: Dopo essersi ricaricato, ricomincia a selezionare la destinazione, 
        # ma una volta arrivato qui invoca il server del movement ma risponde quello 
        # della batteria in modo strano
        if BLev < 30:                       # Recharging required
            return 'b_low'
        elif respR.success == True:          # Room reached
            return 'move'
        else:
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
                            outcomes = ['move', 'wait'])
    # Execution function
    def execute(self, userdata):
        """
        Function that executes the status of the FSM by calling the service
        related to the recharging situation.

        Returns:
            Transition of the FSM to be carried out
                - *wait*: if the battery isn't full yet
                - *move*: if the robot can move between the rooms

        """
        global Battery_Client
         
        print('Executing state ROOM_E')
        time.sleep(0.5)
        respB = Battery_Client()
        time.sleep(1)

        if respB.success == True:      # Battery full
            return 'move'
           # respB.success = False
        else:
            time.sleep(3)
            return 'wait'

def main():
    """
    This function initializes the ROS node, clients and service and waits for
    the creation and execution of the FSM.

    """
    global Battery_Client, Movement_Client, Map_Client
    global B_srv, Pub_Room

    # Initialisation node
    rospy.init_node('Robot_State_Machine')

    # Initialisation clients
    Battery_Client = rospy.ServiceProxy('/Recharging_Switch', Trigger)
    Movement_Client = rospy.ServiceProxy('/Movement_Switch', Trigger)
    Map_Client = rospy.ServiceProxy('/Mapping_Switch', Trigger)
    # Initilisation service
    B_srv = rospy.Service('/BLevel', BatteryLow, Battery_State)
    # Initialisation publisher
    Pub_Room = rospy.Publisher('/Room', String, queue_size=1)

    print('STATE MACHINE')
    
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
                                              'wait': 'ROOM_E'})

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
