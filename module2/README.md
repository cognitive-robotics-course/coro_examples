# C++ package to support Module 2 on writing ROS software in C++: publishers, subscribers, services.
This package is implements the four examples in Module 2, Lecture 2, which involves the creation of a new  package, agitr, with four ROS nodes:

- hello
- pubvel
- subpose
- useservices

Please refer to Lecture 2 for details on the functionality of each of these nodes.

## Running the example code

As always, make sure the ROS master is running:

`roscore`

Open a second terminal and enter

`rosrun module2 hello`

to see the Hello World message.

Then enter

`rosrun turtlesim turtlesim_node`

Open a third terminal and enter

`rosrun module2 pubvel`

to publish random linear and angular command velocities and see the turtle wander about the simulator environment.

Open a fourth terminal and enter

`rosrun module2 subpose`

to see the pose values published on the turtleX/Pose topic, where X stands for the turtle number.

Enter <ctrl>-c to stop the pubvel and subpose nodes.

Enter

`rosrun module2 useservices`

to use the example services to clear the simulator and teleport the turtle


