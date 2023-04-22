#! /usr/bin/env python

"""
.. module:: TopologicalMap
    :platform: Unix
    :synopsis: Python module for topologic map construction
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

ROS node for implementing the TOPOLOGICAL_MAP state of the finite state machine FSM.

This node allows the map to be constructed by loading the ontology, based on the 
information obtained from the markers.
Once the map is constructed, a signal is sent to the FSM.

Subscribes to:
    /MarkerList to retrive information from ArUco markers

Client:
    ArmorClient

Service:
    /Mapping_Switch to active the TOPOLOGICAL_MAP state

"""

import roslib
import random
import time
import rospy
from std_srvs.srv import TriggerResponse, Trigger
from assignment2.srv import *
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Int32

Path = 'ERL_WS/src/assignment2/worlds/topological_map.owl'
IRI = 'http://bnc/exp-rob-lab/2022-23'
Robot = 'Robot1'


class Topological_Map:
    """
    Class representing the initial state of the FSM, which loads the ontology initialised 
    with the description of the environment and the robot's starting point. 
    Ontology initialised with the description of the environment and the starting point 
    of the robot.
    ...
    Methods
    ----------
    __init__(self)
    IDCallback(self,lst)
    MappingSwitchCB(self,req)
    """
    def __init__(self):
        """
        Initialisation function
        """
        # Initialisation service and subscriber
        rospy.Subscriber("/MarkerList", Int32, self.IDCallback)
        Map_srv = rospy.Service('/Mapping_Switch', Trigger, self.MappingSwitchCB)

        self.Armor_Client_ID = 'User'
        self.Armor_ReferenceName = 'Ref'
        self.Armor_Client = ArmorClient(self.Armor_Client_ID, self.Armor_ReferenceName)

        self.IdList = []            # List of markers
        self.Location = []          # List of rooms
        self.LocationDict = {}      # Dictionary of rooms and their connections
        self.LocationCoord = {}     # Dictionary of rooms and their coordinates
        self.CoordinatesLoc = {}    # Dictionary of coordinates and their rooms    

    # Marker service callback
    def IDCallback(self,lst):
        """
        Callback to retrive codes from ArUco markers and stuck them in a list

        Args:
            lst (int): marker number
        """
        if(lst.data not in self.IdList):
            self.IdList.append(lst.data)


    # /Mapping_Switch service callback
    def MappingSwitchCB(self,req):
        """
        Function to inform the finished FSM state machine that the ontology has been 
        loaded with the relevant information about the rooms and the initial position 
        of the robot.
        
        Returns:
            res.success (bool): indicates successful run of triggered service

            res.message (string): informational
        
        """
        print('TOPOLOGICAL_MAP')

        print('Waiting for the Armor server ...')
        rospy.wait_for_service('armor_interface_srv')

        print('Waiting for the RoomInformation server ...')
        rospy.wait_for_service('/room_info')
        RoomClient = rospy.ServiceProxy('/room_info', RoomInformation)

        # Construct lists and dictionaries from infos retrived from the room_info server
        while len(self.IdList) != 7:
            time.sleep(0.5)
        for i in self.IdList:
        #for i in range(11, 18): #DEBUG FINCHE' NON LEGGE I MARKER
            resp = RoomClient(i)
            self.Location.append(resp.room)
            self.LocationDict[resp.room] = resp.connections
            self.LocationCoord[resp.room] = [resp.x,resp.y]
            self.CoordinatesLoc[str(resp.x) + ',' + str(resp.y)] = resp.room

        time.sleep(0.5)
        CurrentTime = int(time.time())

        # Load ontology
        self.Armor_Client.utils.load_ref_from_file(Path, IRI, buffered_manipulation=False, reasoner='PELLET', buffered_reasoner=False, mounted=False)
        self.Armor_Client.call('MOUNT', '', '', [''])
        self.Armor_Client.call('LOG', 'TERMINAL', 'ON', [''])

        for i in self.Location:
            Connections = self.LocationDict[i]
            Coordinates = self.LocationCoord[i]
            print(f'{i} coordinates are {Coordinates}')
            self.Armor_Client.call('ADD', 'DATAPROP', 'IND', ['hasCoordinatesX', i, 'Float', str(Coordinates[0])])
            self.Armor_Client.call('ADD', 'DATAPROP', 'IND', ['hasCoordinatesY', i, 'Float', str(Coordinates[1])])
            for j in Connections:
                print(f'{i} connected through door {j.through_door}')
                self.Armor_Client.call('ADD', 'OBJECTPROP', 'IND', ['hasDoor', i, j.through_door])


        # Robot starting room
        self.Armor_Client.call('ADD', 'OBJECTPROP', 'IND', ['isIn', Robot, 'E'])
        print(f'{Robot} starting room is ROOM E')

        # Set all rooms and corridors visited at CurrentTime time instant
        for RC in self.Location:
            self.Armor_Client.call('ADD', 'DATAPROP', 'IND', ['visitedAt', RC, 'Long', str(CurrentTime)])
        
        # Disjoint for Individuals
        self.Armor_Client.call('DISJOINT','IND','CLASS', ['LOCATION'])
        self.Armor_Client.call('DISJOINT','IND','CLASS', ['DOOR'])

        # First Reasoning
        self.Armor_Client.utils.apply_buffered_changes()
        self.Armor_Client.utils.sync_buffered_reasoner()

        res = TriggerResponse()
        res.message = 'MAP BUILT'
        res.success = True

        print(f'{res.message}')

        return res

if __name__ == "__main__":

    # Initialisation ROS node
    rospy.init_node('TopologicalMap')
    
    TopMap = Topological_Map()

    # Wait for ctrl-c to stop the application
    rospy.spin()
