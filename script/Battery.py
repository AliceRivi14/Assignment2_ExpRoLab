#!/usr/bin/env python

"""
.. module:: Battery
    :platform: Unix
    :synopsis: Python module for battery control
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

ROS node for implementing the ROOM_E state of the finite state machine FSM.

This node allows the robot to recharge its battery in room E.
Once the battery is fully charged, the FSM is informed.

Client:
    /BLevel to manage the robot's battery level

    /Movement_Switch to move the robot into room E where it can recharge itself

Service:
    /Recharging_Switch to active the ROOM_E state

"""

import roslib
import time
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from assignment2.srv import *
#from armor_api.armor_client import ArmorClient

BLev = 100

class Battery:
    """
    Class that allows the robot to be directed to the charging room and complete battery 
    charging.Once the battery reaches 100 per cent, the FSM is informed that it can 
    continue the surveillance action.
    ...
    Methods
    ----------
    __init__(self)
    BatterySwitchCB(self,req)
    """
    def __init__(self):
        """
        Initialisation function
        """
        # Initialisation service and client
        self.Batt_srv = rospy.Service('/Recharging_Switch', Trigger, self.BatterySwitchCB)
        self.B_Client = rospy.ServiceProxy('/BLevel', BatteryLow)    
        self.Movement_Client = rospy.ServiceProxy('/Movement_Switch', Trigger)

    # /Recharging_switch service callback
    def BatterySwitchCB(self,req):
        """
        Function for recharging the battery and informing the finished state machine FSM.
       
        Returns:
            res.success (bool): indicates successful run of triggered service

            res.message (string): informational
        """
        print('ROOM_E')

        global BLev

        if self.B_Client().LevelF < 30:
            print('\033[91mBATTERY LOW\033[0m')
            respR = self.Movement_Client()
            if respR.success == True:
                print('Charging ...')

                BLev = self.B_Client().LevelF
                for BLev in range(BLev, 101):
                    ++BLe
                    print('\033[93m#\033[0m', end='')
                    
                print(f'\nBattery_level = {BLev}%')
                BTot = BLev + self.B_Client().LevelF
                self.B_Client(BTot)

                res = TriggerResponse()
                res.message = 'BATTERY FULL'
                res.success = True
            else:
                time.sleep(3)      

        print(f'\033[92m{res.message}\033[0m')

        return res

    

if __name__ == "__main__":
    
    # Initialisation node
    rospy.init_node('Battery')
    
    Batt = Battery()

    # Wait for ctrl-c to stop the application
    rospy.spin()
