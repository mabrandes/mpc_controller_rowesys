# mpc_controller_rowesys


**Author:** Matthias Brandes, Timo Schönegg <br />
**Project:** ROWESYS, ETH Zürich <br />
**Created:** HS 2021

## Following packages need to be installed from rowesys project (and their dependencies):
rowesys_base_gazebo, rowesys_control

## reference_state_generator
The launch file for this project is located in the reference_state_generator package: 

```
test_reference_signal_w_rowesys.launch
```

The datasets/rosbags to generate the reference are located in the /data folder. Depending on the set (CW or CCW course on the field) the starting position and angle have to be adjusted in the code (angle+180°, position*(-1)) such that the gazebo model is facing in the same direction as the reference.

In Rviz the PosewithCovariance Marker belongs to the real position. The computed reference position and orientation (real position/orientation + offset) and the pos/orient of the robot (gazebo model) are despicted with arrows. 

The package outputs 4 states, ( rho , alpha,  beta and phi) as depicted below. Only alpha, beta and rho are needed for the MPC controller.

### Current model (Introduction Autonomous Mobile Robots p. 81 - 88):
file:///home/mbrandes/Pictures/Selection_015.png![image](https://user-images.githubusercontent.com/37098089/152987202-a865cbe2-29b5-42e0-a347-a8306c29337c.png)

Most important tuning parameter:
- ref_time_ahead: defining the time the reference pos/orient is kept constant

## mpc_planner

Mpc library only worked with Pytorch_version 1.4.0. More details in mpc_planner package.
```
pip torch == 1.4.0
pip show torch
```
If problems with python version occur, change: 
```
#!/usr/bin/env python3 to #!/usr/bin/env python2
```
or vice versa

### Performance
Performance of controller in general is quite bad. Same has been observed for a simple PID (even though not much time has been spent on tuning).
Shows a "small" run time until a certain point. As soon as angle gets to big, runtime increases??

file:///home/mbrandes/Pictures/Screenshot%20from%202021-12-23%2010-28-09.png![image](https://user-images.githubusercontent.com/37098089/152985868-60d0fc98-d122-45e3-a2c6-13cf363e23a1.png)

I have never really checked papers for best line following models. Current model might not be the best.

Most important parameters for solver:
- goal_weights: weigths of states
- lqr_iter: #iterations per optimization
- eps: threshold of solver
- steps, dt: defining horizon of look ahead





 
