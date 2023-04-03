#!/usr/bin/env python

"""
.. module:: Battery
    :platform: Unix
    :synopsis: Python module for battery control
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

ROS node for implementing the ROOM_E state of the finite state machine FSM.

Through this node, the waiting time for the robot to recharge is simulated.

Client:
    /B_Switch to communicate the need for recharging the battery

    ArmorClient

Service:
    /Recharging_Switch to active the ROOM_E state

"""

import roslib
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from assignment2.srv import *
#from armor_api.armor_client import ArmorClient

import RandomMovement as RM

BLev = 100

class Battery:
    def __init__(self):

        # Initialisation service and client
        self.Batt_srv = rospy.Service('/Recharging_Switch', Trigger, self.BatterySwitchCB)
        self.B_Client = rospy.ServiceProxy('/BLevel', BatteryLow)        

        


    # Service callback
    def BatterySwitchCB(self, req):
        """
        Service callback.

        Args:
            req (bool): for enabling/disabling the service related to battery charging simulation

        Returns:
            res.success (bool): indicates successful run of triggered service

            res.message (string): informational
        """
        print('ROOM_E')

        global BLev 

        if self.B_Client().LevelF < 30:
            print('\033[91mBATTERY LOW\033[0m')
            RM.RandomMovement().MoveBaseA('E')
            print('Charging ...')

            BLev = self.B_Client().LevelF

            for BLev in range(BLev, 101):
                ++BLev
                
            print(f'Battery_level = {BLev}%')
            self.B_Client(BLev)
        else:
            print('...')
        

        res = TriggerResponse()
        res.message = 'BATTERY FULL'
        res.success = True

        print(f'\033[92m{res.message}\033[0m')

        return res

    

if __name__ == "__main__":
    
    # Initialisation node
    rospy.init_node('Battery')
    
    Batt = Battery()

    # Wait for ctrl-c to stop the application
    rospy.spin()
