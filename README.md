# mpc_controller_rowesys


**Author:** Matthias Brandes, Timo Schönegg <br />
**Project:** ROWESYS, ETH Zürich <br />
**Created:** HS 2021

## Following packages need to be installed from rowesys project (and their dependencies):
rowesys_base_gazebo, rowesys_control

## Reference generator
The launch file for this project is located in the reference_generator package: 

```
test_reference_signal_w_rowesys.launch
```

The datasets/rosbags to generate the reference are located in the /data folder. Depending on the set (CW or CCW course on the field) the starting position and angle have to be adjusted in the code (angle+180°, position*(-1)) such that the gazebo model is facing in the same direction as the reference.

In Rviz the PosewithCovariance Marker belongs to the real position. The computed reference position and orientation (real position/orientation + offset) and the pos/orient of the robot (gazebo model) are despicted with arrows. 

The package 4 outputs states needed for the MPC controller.



 
