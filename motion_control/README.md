# Motion control node
This node makes the husky robot move along the waypoints it receives by the navigation and planning node.

## Structure of the node
The node is made up out of a single class, ```Motion_control```. This class receives an array of waypoints from the navigation node which it uses as goals.
It takes the position of the robot and the position of the goal, and calculates the distance and the angle between the robot and the goal.
The node also knows the orientation of the robot and with that it calculates the angle between the orientation of the robot and the straight line to the goal.
According to this angle, the robot starts to move with a greater angular velocity or with a greater lateral velocity. 
If the angle is very large, the robot will stay in place and rotate on the spot, whereas if the angle is very small, the robot will only drive forward.


## Expected results
The result in the whole application should be the publish of cmd_vel messages, which the Husky robot uses to move along a line between its position and the goal.
An example of output is visible below:

![Alt text](README_videos/motion_control.gif)

This image shows the location, the distance and the angle to the goal in the terminal.
It is visible that the angle to goal is very small, which means the robot is just going to drive in a straight line.


## Launching the nodes
The node itself can simply be launched by launching the launch file
```bash
roslaunch motion_control motion_control.launch
```

