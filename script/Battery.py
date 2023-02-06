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
import time
import random
from std_srvs.srv import *
from assignment1.srv import BatteryLow, BatteryLowResponse
from armor_api.armor_client import ArmorClient

import Functions as F

B_Client = None
Active = False

# Service callback
def BatterySwitchCB(req):
    """
    Service callback.

    Args:
        req (bool): for enabling/disabling the service related to battery charging simulation

    Returns:
        res.success (bool): indicates successful run of triggered service

        res.message (string): informational
    """
    global Active, res

    Active = req.data
    res = SetBoolResponse()
    res.message = 'ROOM_E state'
    res.success = True # Service enable
    return res

def main():
    """
    This function initializes the ROS node, client and service.

    A message is sent to the service /B_switch every random seconds to notify the need for recharging.

    When the service /Recharging_Switch is called, battery charging is simulated.
    """
    global B_Client
    global Active

    # Initialisation node
    rospy.init_node('Battery')

    # Initialisation clients and service
    srv = rospy.Service('/Recharging_Switch', SetBool, BatterySwitchCB)
    B_Client = rospy.ServiceProxy('/B_Switch', BatteryLow)


# MODIFICARE CODICE PER SIMULARE SCARICA DELLA BATTERIA IN MODO PIÙ REALISTICO
# IN BASE AL TEMPO OPPURE AL MOVIMENTO
    while not rospy.is_shutdown():

        if Active == False:
            time.sleep(random.uniform(5.0, 10.0))
            resp = B_Client(True) # Recharging required
            rospy.loginfo('I NEED TO BE RECHARGED')
            continue
        else:
            rospy.loginfo('BATTERY LOW')
            F.MoveRobot('E')
            time.sleep(1.5)
            rospy.loginfo(f'BATTERY FULL')
            resp = B_Client(False)

        # Wait for ctrl-c to stop the application
        rospy.spin()

if __name__ == "__main__":
    main()
