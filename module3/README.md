# C++ package to support Module 3 on mobile robots.
This package is implements the following node(s):

- goToPosition

Please refer to Lectures 4 and 5 for details on the functionality of each of these node(s).

## goToPosition
This node implements the divide-and-conquer go-to-position controller and provides a placeholder for the implementation of the MIMO controller.

The program reads an input file goToPositionInput.txt in the package data directory. This file contains a sequence of commands, one command per line.

Each command comprises seven fields:
- a key string  (either "goto1" or "goto2") followed by six floating point numbers.
- "goto1" invokes the divide-and-conquer algorithm
- "goto2" invokes the MIMO controller (not implemented)
 
The first three numbers give the start pose of the turtle (x, y, theta, respectively).
The second three numbers give the goal pose of the turtle (x, y, theta, respectively).

For each command, the turtle is positioned at the start pose and then drives to goal pose using the specified algorithm.

The current turtle pose is sensed by subscribing to the turtle1/pose topic. The turtle is driven by publishing velocity commands on the turtle1/cmd_vel topic.

### Sample Input

`goto1 2.0 1.0 3.14 8.0 8.0 0.0`

`goto2 2.0 1.0 3.14 8.0 9.0 0.0`


### Running the example code

As always, make sure the ROS master is running:

`roscore`

Open a second terminal and enter

`rosrun turtlesim turtlesim_node`

Open a third terminal and enter

`rosrun module3 goToPosition`

Observe the behavior of the turtle and follow the instruction printed to the terminal.
