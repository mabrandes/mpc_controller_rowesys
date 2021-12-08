# rowesys_mpc_planner

## Overview

TODO Timo


### License

**Author:** Timo Schönegg <br />
**Project:** Focusproject ROWESYS, ETH Zürich <br />
**Created:** Apr 2020

The rowesys_mpc_planner package has been tested under ROS Melodic and Ubuntu 18.04.

## Installation

You need to have python installed. You need to install the mpc libary

```
pip3 install mpc
```

## Usage

Make sure, that the python files are executables. To make a file executable run

```
sudo chmod +x [python filename]
```

TODO Timo
To test the simple state machine, the planner, the line_detection, the end_of_row_detection, the turn and the gamepad have to be launched, before the state machine can be started.

```
  roslaunch rowesys_launch rowesys_all.launch 
```



-----------------------
TODO TIMO
-----------------------

## State machines

### rowesys_states.py

There are 3 states: MANUAL, IN_FIELD_NAVIGATION, TURN.

Buttons on the gamepad trigger a state transition to IN_FIELD_NAVIGATION or TURN and back to MANUAL at any time.

In IN_FIELD_NAVIGATION a detected end_of_row triggers a state transition to TURN.

If the TURN has finished, a state transition back to IN_FIELD_NAVIGATION is triggered.

### geese_feet_state_machine.py

Additionally to rowesys_states.py there is a new state GEESE_FEET.

If a transition to MANUAL happens, the geese feet are lifted. Anyway, they can be lowered manually afterwards.

If a transition to IN_FIELD_NAVIGATION happens, the geese feet are lowered.

If a transition to TURN happens, the geese feet are lifted.

The GEESE_FEET state is between every state transition of the rowesys_states.py state machine.

### geese_feet_state_machine_extended.py

Still in developement.


### Subscribed Topics

* **`/rowesys/robot_autonomous_mode`** ([rowesys_navigation_msgs/AutonomousMode])

  Subscribe to the gamepad state which autonomous state should be active. This is needed to perform a state transition, if there is an appropriate gamepad input.

  All three state machines subscribe to this topic.