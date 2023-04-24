Patrolling Algorithm
================================
**A ROS-based assignment for the Experimental Robotics Laboratory course held at the University of Genoa.**

> 📑: [Code Documentation]()

Introduction 
-----------------

This software architecture has been developed by [Student Robotics](https://studentrobotics.org) in ROS to simulate a surveillance robot.

The software developed uses a [Smach Finite State Machine](http://wiki.ros.org/smach) FSM and builds an ontology with [aRMOR](https://github.com/EmaroLab/armor) based on the information obtained from the markers via the [ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) library.

The robotic arm of the robot is moved using the [MoveIt](https://moveit.ros.org/) software framework, while the surveillance action is performed using the [SLAM Gmapping](http://wiki.ros.org/gmapping) algortim and the [MoveBase Navigation](http://wiki.ros.org/move_base) package.
 

Software Architecture
------------------------
There are 5 nodes in this software architecture:
* [StateMachine](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/StateMachine.py)
* [TopologicalMap](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/TopologicalMap.py)
* [RandomMovement](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/RandomMovement.py)
* [Battery](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/Battery.py)

* [Marker Detector](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/src/marker_detector.cpp)
* [Marker Server](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/src/marker_server.cpp)

Here we can see the Component-Based Software Engineering diagram:
![UML.png](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/images/UML.png)


### StateMachine node

In this node, the state machine and the sub-state machine are initialised.

* Client:

    `/Battery_Switch` to active the ROOM_E state

    `/Movement_Switch` to active the RANDOM_MOVEMENT state

    `/Mapping_Switch` to active the TOPOLOGICAL_MAP state

* Service:

    `/BLevel` to communicate the battery level and the need for recharging the battery

There are 4 classes representing the states of the finite state machine. Each state is characterised by the initialisation function `__init__(self)` and the function representing the state execution `execute(self, userdata)`.

- `class TOPOLOGICAL_MAP(smach.State)`: Class implementing FSM state concerning the topological map.

    Within the execute function, a call is made to the server that connects the FSM to the TopologicalMap node and then waits until the map is constructed.

    Returns the transition of the FSM to be carried out:

    * `wait`: if the map construction isn't finished
    * `map_OK`: when map construction ends

- `class CHOOSE_DESTINATION(samch.State)`: Class implementing FSM state conserning the choise of the destination.

    Within the execute function, a call is made to the function `Destination()` and 1 second is waited.

    Returns the transition of the FSM to be carried out:
    
    * `b_low`: if the robot needs to be recharged
    * `destination`: when the location in which the robot is to move is chosen

This is a piece of code that explains how the location in which the robot moves is chosen:
```python
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

print(f'Choose location {Target}')
return Target
```
    
- `class RANDOM_MOVEMNT(smach.State)`: Class implementing FSM state concerning the random movement.

    Within the execute function, a call is made to the server connecting the FSM to the RandomMovement node, and then you wait until the robot arrives at its destination.

    Returns the transition of the FSM to be carried out:
    
    * `b_low`: if the robot needs to be recharged
    * `move`: if the robot can move between the rooms
    * `wait`: if the goal isn't reached yet
    
- `class ROOM_E(smach.State)`: Class implementing FSM state concerning the room E.

    Within the execute function, a call is made to the server connecting the FSM to the Battery node, and and then wait until the robot arrives in charging room E and the battery is fully recharged.

    Returns the transition of the FSM to be carried out:
    
    * `move`: if the robot can move between the rooms
    * `wait`: if the battery isn't full yet

    
Running the FSM node in the terminal should show similar output

![SM_Terminal.png](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/images/SM_Terminal.png)

### TopologicalMap node 

This node allows the map to be constructed by loading the ontology, based on the information obtained from the markers. Once the map is constructed, a signal is sent to the FSM.

* Subscribes to:

    `/MarkerList` to retrive information from ArUco markers

* Client:

    `ArmorClient`

The present class is characterised by 2 methods:

* `IDCallback(lst)`: 

    Callback to retrieve codes from ArUco markers and embed them in a list.

* `MappingSwitchCB(req)`:

    Function to inform the finished FSM state machine that the ontology has been with the relevant information about the robot's rooms and initial position. the robot.
        

Running the TopologicalMap node in the terminal should show similar output

![TM_Terminal.png](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/images/TM_Terminal.png)

### RandomMovement node 

This node manages the movement of the robot based on information obtained about the target room chosen by the FSM. 

Based on the position and battery status it informs the FSM if the final position has been reached or if the goal has been cancelled.

* Publishes to:

    `/A/jointC_position_controller/command` to publish the camera joint position 

* Subscribes to:

    `/Room` to establish the target room

    `/odom` where the simulator publishes the robot position

* Client:

    `ArmorClient`

    `MoveBaseAction`

    `/BLevel` to manage the robot's battery level

* Service:

    `/Mapping_Switch` to active the RANDOM_MOVEMENT state

    `MoveBaseGoal`

The present class is characterised by 4 methods:

* `RoomCB(Location)`:

    Callback to obtain information about the robot's target location.
    
* `OdomCB(data)`:

    Callback to obtain the actual position of the robot via odometry.

* `MovementSwitchCB(req)`:

    Function to inform the finished state machine FSM if the robot has reached the final position.
    
* `MoveBaseA(Loc)`:

    Function to control the movement of the robot.
    
    When the robot reaches the final position, it inspects the room.
    The goal is cancelled if the battery is less than 30%. The battery check is performed considering the distance the robot travels.

```python
print(f'Actual position: ({self.previous_x},{self.previous_y})')

self.Goal.target_pose.pose.position.x = F.CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['hasCoordinatesX', Loc]))[0]
self.Goal.target_pose.pose.position.y = F.CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['hasCoordinatesY', Loc]))[0]
self.Goal.target_pose.pose.position.x = float(self.Goal.target_pose.pose.position.x)
self.Goal.target_pose.pose.position.y = float(self.Goal.target_pose.pose.position.y)

F.MoveRobot(Loc)

print(f'Moving to ({self.Goal.target_pose.pose.position.x},{self.Goal.target_pose.pose.position.y}) position')
self.MBClient.send_goal(self.Goal)

dist = math.sqrt(pow(self.Goal.target_pose.pose.position.x - self.previous_x, 2) +
            pow(self.Goal.target_pose.pose.position.y - self.previous_y, 2))
print(f'To reach {Loc} I need to travel {round(dist)}m')

print('Waiting for the result ...')
self.MBClient.wait_for_result()
resp = self.Bat_Client(round(dist))

x = abs(self.Goal.target_pose.pose.position.x - actual_x)
y = abs(self.Goal.target_pose.pose.position.y - actual_y)        

if resp.LevelF < 30 and Loc != 'E':
    # Cancel goal
    self.MBClient.cancel_goal()
    self.DestReach = False
elif (x <= 1 and y <= 1) or Loc == 'E':
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
```

### Battery node 

This node allows the robot to recharge its battery in room E. Once the battery is fully charged, the FSM is informed.

* Client:

    `/BLevel` to manage the robot's battery level

    `/Movement_Switch` to move the robot into room E where it can recharge itself

* Service:
    
    `/Recharging_Switch` to active the ROOM_E state

The present class is characterised by 1 method:
    
* `BatterySwitchCB(req)`:

    Function for recharging the battery and informing the finished state machine FSM.
    
Running the Battery node in the terminal should show similar output

![.png]()

### Marker Detector node

Through this node, the initial room in which the robot is located is inspected in order to identify all markers and to be able to derive information about the locations in which the robot will later move.

* Subscribes to: 

    `RGB/RGB/image_raw` to retrieve the robot vision through the camera

* Publishes to: 

   `/A/jointC_position_controller/command` to pub the RGB camera joint angle for the robot arm

   `/MarkerList` in which it is published the markers detected by the robot

* Clients:

   `/room_info` to obtain information about the environment 
   
The present class is characterised by 3 methods:

* `CallbackCamera(ImFeed)`:

    It reads the information from the camera and thanks to the ArUco libraries processes it and detects the respective ArUco marker. It prints on the terminal the detected marker ID.
    
    Once all the markers are found their ID are published on the topic.

* `DetectMarker()`:

    This function is used to move the robot joint and to detect the markers.
    
    In particular the robot arm is set in the 'HomePose' configuration and the RGB joint is move looking at the ground from 0 to 3.14. In this way it looks at all markers placed on the ground.
    
### Marker Server node

This node contains all the information associated with each room.

* Service:

   `/room_info` to share information about the environment

The present class is characterised by 1 method:

* `markerCallback(req, res)`:

  This function provides information about a room in a building based on an input ID.

> :memo: **Note:** 
> 
> In some files, the python script [Functions.py](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/Functions.py) is imported. 
> 
> In this script the various functions used by nodes are defined.

Installation and running
-------------------------------

In a terminal type the following commands:
```bashscript
$ mkdir -p ROS_ws/src/assignment2
$ cd ROS_ws/src/assignment2
$ git clone https://github.com/AliceRivi14/Assignment2_ExpRoLab.git
$ cd script
$ chmod +x <file_name>
```
Before compiling the package, move the moveit folder outside, inside the src folder, and then:
```bashscript
$ cd ../../..
$ catkin_make
```
Add the line `‘source [ws_path]/devel/setup.bash’` in your `.bashrc` file.

If is not already installed, install smach with
```bashscript
$ sudo apt-get install ros-[ROS-DISTRO]-executive-smach*
```
For everything to work properly, the [MoveIt](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html), [SLAM Gmapping](https://github.com/CarmineD8/SLAM_packages), [ArUco](https://github.com/CarmineD8/aruco_ros), [aRMOR](https://github.com/EmaroLab/ros_multi_ontology_references) and [MoveBase](https://github.com/CarmineD8/planning) package must be compiled.

First, the simulation environment must be run:
```bashscript
$ roslaunch assignment2 moveit.launch
```
Then you can run the files for the execution of the algorithm:
```bashscript
$ roslaunch assignment2 assignment.launch
```
So that there are no errors, the aRMOR service is launched separately:
```bashscript
$ rosrun armor execute it.emarolab.armor.ARMORMainService
```

In addition to the terminal in which the file is launched, 5 terminals relating to the ROS nodes appear.

> 📝 **Note:**
> 
>   Change the Path `‘[ws_path]/assignment_2/world/topological_map.owl’` in the [TopologicalMap.py](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/TopologicalMap.py) script.


Working Hypothesis and Environment
------------------------------------

The scenario involves a survillance robot operating in a 2D indoor environment, without obstacles, made of 4 rooms (R1, R2, R3, R4), with only one door, and 3 corridors (E, C1, C2), with multiple doors.

![Environment.png](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/images/Environment.png)

The behavior of the robot is divided into 2 phases:

**Phase 1**:
1. The robot starts from room E;
2. The robot through the movement of the camera must detect the markers in the room and gathers information about the environment;
3. The robot constructs the "semantic" map of the environment.

![Marker.png](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/images/Marker.png)

**Phase 2**:
1. The robot moves around locations following a surveillance policy:
    1. It must stay mainly in the corridors,
    2. If a reachable room has not been visited for a certain period of time, it must visit it (*urgency*);
2. The robot moves to a new location and performs a complete scan of the room to visit another location. 
3. When the robot's battery is low, the robot goes to position E and waits for the battery to be recharged before starting the patrolling behaviour again.

## System Limitations

The main limitation concerns the detection of markers, as the camera does not have a smooth movement and it may happen that even if the marker is seen, its ID is not recognised.

The battery drain is calculated as if each metre travelled by the robot corresponds to a decrease. However, this calculation is made based on the distance between the start and end position, as the crow flies, without taking into account the presence of walls or obstacles, so it does not reflect the real situation.

## Possible Improvement

One improvement to be made is definitely to make the battery discharging and charging routine more realistic. 

In addition, all the parameters relating to the movement of the robot joints should also be adjusted in order to have a more linear and efficient marker detection.

Author
--------
 
 *Alice Rivi*
 
 📧: *<S5135011@studenti.unige.it>*
