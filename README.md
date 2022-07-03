# Simple-Navigation-Goals
The developed application aims to operate a robot autonomously to move in a closed environment with different static and moving obstacles. The robot must be able to trace a path from a given starting point to another destination point in the shortest possible time, avoiding the different objects it encounters along the way and even changing the fixed route if necessary.
# Programs and components used
ROS (version Noetic)

RViz (version 1.9)

Gazebo (version 7)
# Usage
Create a new ROS package and put the script inside. Then, use the following command to compile:
```
catkin_make
```
And execute:
```
rosrun simple_navigation_goals simple_navigation_goals
```
