#!/usr/bin/env python
"""
.. module:: Functions
    :platform: Unix
    :synopsis: Python module for the Finite State Machine
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

Script defining various functions used by ROS nodes

Client:
    ArmorClient

"""
import roslib
import rospy
import time
import random
from armor_api.armor_client import ArmorClient

Path = 'ERL_WS/src/assignment1/ontology/Map.owl'
IRI = 'http://bnc/exp-rob-lab/2022-23'
Robot = 'Robot1'

Armor_Client_ID = 'User'
Armor_ReferenceName = 'Ref'
Armor_Client = ArmorClient(Armor_Client_ID, Armor_ReferenceName)

def CleanList(res):
    """
    Function to clean outputs of the list.

    Args:
        res: Armor_Client.call response

    Returns:
        List (string): list of strings
    """
    List = res.queried_objects
    # Remove IRI
    List =  [Q.replace('<'+IRI+'#', '') for Q in List]
    List =  [Q.replace('>', '') for Q in List]
    # Remove timestamp
    List =  [Q.replace('"', '') for Q in List]
    List =  [Q.replace('^^xsd:long', '') for Q in List]

    return List

def Update_Time():
    """
    Function to update robot times.
    """
    Armor_Client.call('REASON', '', '', [''])
    # Robot time
    Old = str(CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['now', Robot]))[0])
    New = str(round(time.time()))
    # Replacing the previous robot-time with the actual one
    Armor_Client.call('REPLACE', 'OBJECTPROP', 'IND', ['now', Robot, 'Long', New, Old])

# REASONER
def MoveRobot(Location):
    """
    Function to update information about the chosen location where the robot is to move.

    Args:
        Location (string): location where the robot is to move
    """
    # Robot location
    Old_Loc = Armor_Client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', Robot])
    Old_Loc = CleanList(Old_Loc)[0] # First element of the location list
    # Replacing the previous location with the one the robot moved to
    Armor_Client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', Robot, Location, Old_Loc])
    # Time when the robot visited the last location
    Old = str(CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', Location]))[0])
    # Time in which the robot visits the current location
    New = str(round(time.time()))

    # Replacing the previous location-time with the one the robot moved to
    Armor_Client.call('REPLACE', 'DATAPROP', 'IND', ['visitedAt', Location, 'Long', New, Old])
    
    Old_Urg = CleanList(Armor_Client.call('REPLACE', 'DATAPROP', 'IND', ['UrgencyThreshold', Robot]))[0]
    Armor_Client.call('REPLACE', 'DATAPROP', 'IND', ['UrgencyThreshold', Robot, 'Long', UrgencyThreshold, Old_Urg])

    rospy.loginfo(f'Go to location {Location}')

    Update_Time()

def Destination():
    """
    Function to decide in which location the robot should move according to the Urgency.

    Returns:
        A number that corresponds to the marker id associated with the location Target
    """

    rospy.loginfo('Waiting for the Armor server ...')
    rospy.wait_for_service('armor_interface_srv')

    Rooms = CleanList(Armor_Client.call('QUERY', 'IND', 'CLASS', ['ROOM']))
    Corridors = CleanList(Armor_Client.call('QUERY', 'IND', 'CLASS', ['CORRIDOR']))
    Urgent = CleanList(Armor_Client.call('QUERY', 'IND', 'CLASS', ['URGENT']))

    # Urgent rooms
    Urgent = [Idx for Idx in Urgent if Idx not in Corridors]
    # Location reachable by the robot
    Reachable = CleanList(Armor_Client.call('QUERY', 'OBJECTPROP', 'IND', ['canReach', Robot]))
    # Urgent rooms reachable
    Urgent = [Value for Value in Urgent if Value in Reachable]

    # If several rooms are urgent, choose one randomly
    if Urgent:
        Target = random.choice(Urgent)
    # If no room is urgent, choose a reachable corridor randomly
    else:
        Target = [Idx for Idx in Reachable if Idx in Corridors]
        if Target:
            Target = random.choice(Target)
    # If no corridor is reachable, randomly choose a reachable location
        else:
            Target = random.choice(Reachable)
            if not Target:
                rospy.loginfo('ERROR')

    return Target

