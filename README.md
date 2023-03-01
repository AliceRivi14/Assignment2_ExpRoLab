Patrolling Algorithm
================================
**A ROS-based assignment for the Experimental Robotics Laboratory course held at the University of Genoa.**

> 📑: [Code Documentation]()

Introduction 
-----------------

This software architecture has been developed by [Student Robotics](https://studentrobotics.org) in ROS to simulate a surveillance robot.

The whole software is provided in Python 3.

The software developed uses a [Smach Finite State Machine](http://wiki.ros.org/smach) FSM and builds an ontology with aRMOR, using the [armor_py_api](https://github.com/EmaroLab/armor_py_api).

Software Architecture
------------------------
There are 4 nodes in this software architecture:
* [StateMachine](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/StateMachine.py)
* [TopologicalMap](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/TopologicalMap.py)
* [RandomMovement](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/RandomMovement.py)
* [Battery](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/Battery.py)

Here we can see the Component-Based Software Engineering diagram:
![UML]()


### StateMachine node

In this node, the state machine and the sub-state machine are initialised.

* Client:

    `/Battery_Switch` to active the ROOM_E state

    `/Movement_Switch` to active the RANDOM_MOVEMENT state

    `/Mapping_Switch` to active the TOPOLOGICAL_MAP state

* Service:

    `/B_Switch` to communicate the need for recharging the battery

There are 4 classes representing the states of the finite state machine. Each state is characterised by the initialisation function `__init__(self)` and the function representing the state execution `execute(self, userdata)`.

- `class TOPOLOGICAL_MAP(smach.State)`: Class implementing FSM state concerning the topological map.

    Within the execute function, a call is made to the server connecting the FSM to the TopologicalMap node, and 3 seconds are waited for the requested behaviour to be executed.

    Returns the transition of the FSM to be carried out:

    * `b_low`: if the robot needs to be recharged
    * `map_OK`: when map construction ends

- `class CHOOSE_DESTINATION(samch.State)`: Class implementing FSM state conserning the choise of the destination.

    Within the execute function, a call is made to the function `Destination()` and 5 seconds are waited.

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
    # If no room is urgent, choose an reachable corridor randomly
    else:
        Target = [Idx for Idx in Reachable if Idx in Corridors]
        if Target:
            Target = random.choice(Target)
    # If no corridor is reachable, randomly choose a reachable location
        else:
            Target = random.choice(Reachable)
            if not Target:
                print('ERROR')
```
    
- `class RANDOM_MOVEMNT(smach.State)`: Class implementing FSM state concerning the random movement.

    Within the execute function, a call is made to the server connecting the FSM to the RandomMovement node, and 5 seconds are waited for the requested behaviour to be executed.

    Returns the transition of the FSM to be carried out:
    
    * `b_low`: if the robot needs to be recharged
    *  `move`: if the robot can move between the rooms
    
- `class ROOM_E(smach.State)`: Class implementing FSM state concerning the room E.

    Within the execute function, a call is made to the server connecting the FSM to the Battery node, and 5 seconds are waited for the requested behaviour to be executed.

    Returns the transition of the FSM to be carried out:
    
    * `b_low`: if the robot needs to be recharged
    * `move`: if the robot can move between the rooms

Through the smach viewer it is possible to visualise the behaviour of the FSM:
![StateMachine](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/images/StateMachine.png)

    
Running the FSM node in the terminal should show similar output

![Terminal_StateMachine.png]()

### TopologicalMap node 

This node enables the behaviour associated with the TOPOLOGICAL_MAP state to be performed, simulating the construction of the topological map.

* Client:

    `ArmorClient`

* Service:

    `/Mapping_Switch` to active the TOPOLOGICAL_MAP state

There are 2 functions:

* `LoadMap()`:

    through the aRMOR client, the topology map can be created by loading the ontology.
    
    ???
    
```python
# Load ontology
Armor_Client.utils.load_ref_from_file(Path, IRI, buffered_manipulation=False, reasoner='PELLET', buffered_reasoner=False, mounted=False)

```
    
* `Mapping_Switch(req)`:

    service callback.


### RandomMovement node 

This node enables the behaviour associated with the RANDOM_MOVEMENT state to be performed, simulating the robot's movement between locations. 

* Client:

    `ArmorClient`
    
    `MoveBaseAction`

* Service:

    `/Mapping_Switch` to active the RANDOM_MOVEMENT state

    `MoveBaseGoal`

There are 2 functions:

* `MoveBaseA()`:

    provides an implementation of a [MoveBaseAction](http://docs.ros.org/en/fuerte/api/move_base_msgs/html/msg/MoveBaseAction.html) which, given a position goal, will attempt to reach it.

    If the position is not reached within a certain time (3.0 seconds) or if the signal of battery low is sent, the goal is cancelled.
    
```python    
MBClient = actionlib.SimpleActionClient('move_base',MoveBaseAction)

```
  
* `Movement_Switch(req)`:

    service callback.

### Battery node 

This node performs the behaviour associated with the ROOM_E state, simulating the charging of the robot's battery and randomly alerts when the robot needs to be recharged.

* Client:

    `/B_Switch` to communicate the need for recharging the battery
    
    `ArmorClient`

* Service:

    `/Recharging_Switch` to active the RANDOM_MOVEMENT state

There are 1 functions:
    
* `Battery_Switch(req)`:

    service callback.
    
Running the Battery node in the terminal should show similar output




> :memo: **Note:** 
> 
> In each file, the python script [Functions.py](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/Functions.py) is imported. 
> 
> In this script the various functions used by several nodes are defined.

Installation and running
-------------------------------

In a terminal type the following commands:
```bashscript
$ mkdir -p ROS_ws/src/assignment2
$ cd ROS_ws/src/assignment2
$ git clone https://github.com/AliceRivi14/Assignment2_ExpRoLab.git
$ cd script
$ chmod +x <file_name>
$ cd ../../..
$ catkin_make
```
Add the line `‘source [ws_path]/devel/setup.bash’` in your `.bashrc` file.

If is not already installed, install smach with
```bashscript
$ sudo apt-get install ros-[ROS-DISTRO]-executive-smach*
```

To be able to run the nodes you must first run the aRMOR service
```bashscript
$ roscore &
$ rosrun armor execute it.emarolab.armor.ARMORMainService
```
And the run the launch file:
```bashscript
$ roslaunch assignment2 
```
In addition to the terminal in which the file is launched, 4 terminals relating to the four ROS nodes appear.

> ⚠️ **Warning:** 
> 
> In case of problems due to some nodes crashing, run the aRMOR service and open 4 more terminals.
> 
> In these terminals, execute the nodes in the following order:
> ```bashscript
> # 1st terminal
> $ rosrun assignment2 TopologicalMap.py
> ```
> ```bashscript
> # 2nd terminal
> $ rosrun assignment2 RandomMovement.py
> ```
> ```bashscript
> # 3rd terminal
> $ rosrun assignment2 Battery.py
> ```
> ```bashscript
> # 4th terminal
> $ rosrun assignment2 StateMachine.py
> ```

> 📝 **Note:**
> 
>   Change the Path `‘[ws_path]/assignment_2/world/topological_map.owl’` in the [TopologicalMap.py](https://github.com/AliceRivi14/Assignment2_ExpRoLab/blob/main/script/TopologicalMap.py) script  and the import path in each [script](https://github.com/AliceRivi14/Assignment2_ExpRoLab/tree/main/script).


Working Hypothesis and Environment
------------------------------------

The scenario involves a survillance robot operating in a 2D indoor environment, without obstacles, made of 4 rooms (R1, R2, R3, R4), with only one door, and 3 corridors (E, C1, C2), with multiple doors.

![Map]()

The behavior of the robot is divided into 2 phases:

**Phase 1**:
1. The robot starts from room E;
2. The robot through the movement of the camera must detect the markers in the room and gathers information about the environment;
3. The robot constructs the "semantic" map of the environment.

**Phase 2**:
1. The robot moves around locations following a surveillance policy:
    1. It must stay mainly in the corridors,
    2. If a reachable room has not been visited for a certain period of time, it must visit it (*urgency*);
2. The robot moves to a new location and performs a complete scan of the room to visit another location. 
3. When the robot's battery is low, the robot goes to position E and waits for the battery to be recharged before starting the patrolling behaviour again.

## System Limitations

The main limitation of this software concerns the robot's ability to move for a random amount of time, as opposed to the actual amount of battery power.

The predefined environment can also create problems as it is difficult to manage. In addition, in the case of the presence of obstacles, a path planner based on a defined map is required.

## Possible Improvement

The battery issue can easily be solved by setting a parameter indicating the battery percentage as a function of the distance travelled. Also, to prevent the robot from wasting too much time searching for the room to recharge, a return path could be set, as efficiently as possible, connecting each location with room E.

As for the environment, one could create a real map associating each room with certain locations. Equipping the robot with a scanner would also solve the problem of obstacles.

Author
--------
 
 *Alice Rivi*
 
 📧: *<S5135011@studenti.unige.it>*
