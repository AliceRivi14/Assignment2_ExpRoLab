#! /usr/bin/env python

"""
.. module:: TopologicalMap
    :platform: Unix
    :synopsis: Python module for topologic map construction
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

ROS node for implementing the TOPOLOGICAL_MAP state of the finite state machine FSM.

This node allows the map to be constructed by loading the ontology.

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
from std_msgs.msg import Int32MultiArray

Path = 'ERL_WS/src/assignment2/worlds/topological_map.owl'
IRI = 'http://bnc/exp-rob-lab/2022-23'
Robot = 'Robot1'

class Topological_Map:
    def __init__(self):

        # Initialisation service and subscriber
        rospy.Subscriber("/MarkerList", Int32MultiArray, self.IDCallback)
        Map_srv = rospy.Service('/Mapping_Switch', Trigger, self.MappingSwitchCB)

        self.Armor_Client_ID = 'User'
        self.Armor_ReferenceName = 'Ref'
        self.Armor_Client = ArmorClient(self.Armor_Client_ID, self.Armor_ReferenceName)

        self.IdList = []            # List of markers
        self.Location = []          # List of rooms
        self.LocationDict = {}      # Dictionary of rooms and their connections
        self.LocationCoord = {}     # Dictionary of rooms and their coordinates
        self.CoordinatesLoc = {}    # Dictionary of coordinates and their rooms    

    # /Mapping_Swit service callback
    def MappingSwitchCB(self,req):
        """
        Function to load the topological map using the aRMOR client.
        """

        print('Waiting for the Armor server ...')
        rospy.wait_for_service('armor_interface_srv')

        print('Waiting for the RoomInformation server ...')
        rospy.wait_for_service('/room_info')
        RoomClient = rospy.ServiceProxy('/room_info', RoomInformation)

        # Construct lists and dictionaries from infos retrived from the room_info srv
        #for i in self.IdList:
        for i in range(11, 18): #DEBUG FINCHE' NON LEGGE I MARKER
            resp = RoomClient(i)
            self.Location.append(resp.room)
            self.LocationDict[resp.room] = resp.connections
            self.LocationCoord[resp.room] = [resp.x,resp.y]
            self.CoordinatesLoc[str(resp.x) + ',' + str(resp.y)] = resp.room

        time.sleep(0.5)
        StartTime = str(0)

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
            self.Armor_Client.call('ADD', 'DATAPROP', 'IND', ['visitedAt', RC, 'Long', str(StartTime)])
        
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

    # Marker service callback
    def IDCallback(self,lst):
        """
        Callback of the ROS message on the topic /MarkerList to retrive codes from ArUco markers and stuck them in a list
        """
        if(lst.data not in self.IdList):
            self.IdList.append(lst.data)

if __name__ == "__main__":

    # Initialisation ROS node
    rospy.init_node('TopologicalMap')

    TopMap = Topological_Map()

    # Wait for ctrl-c to stop the application
    rospy.spin()
