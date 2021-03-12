# C++ package used to control the Lynxmotion AL5D simulator in Gazebo
This package is used to control the simulated version of the [Lynxmotion AL5D robotic arm](http://www.lynxmotion.com/c-130-al5d.aspx) that can be used for various simulation exercises such as picking and placing [Lego bricks](https://www.lego.com/en-us/product/buildable-2x4-red-brick-5006085). For the simulated version of the robotic arm, the C++ package works in conjunction with the [lynxmotion_al5d_description](https://github.com/CRAM-Team/lynxmotion_al5d_description) package that contains the robot model which can be viewed by running a simulation in [Gazebo](http://gazebosim.org/). In order to view the robotic arm and to access services such as spwaning the Lego bricks please follow the documentation provided in the [lynxmotion_al5d_description](https://github.com/CRAM-Team/lynxmotion_al5d_description) package.

The image below shows the simulated Lynxmotion AL5D robotic arm grasping a red Lego brick.

![Lynxmotion AL5D robotic arm grasping action](screenshots/grasp.png?raw=true "Robotic arm grasping Lego brick")

## Table of contents
1. [Gazebo simulator](#gazebo-simulator)
2. [Input](#input)
   1. [Sample input](#sample-input)
3. [Running the example code](#running-the-example-code)

### Gazebo simulator
Please refer to the [lynxmotion_al5d_description](https://github.com/CRAM-Team/lynxmotion_al5d_description) package to launch the Gazebo simulation enviroment and to access the services in the simulator such as spawning a Lego brick or resetting the workspace.

### Input
The input files are found in the data directory. The first line of the input file comprises a filename for the robot configuration file. The second line contains four numbers corresponding to the x, y and z coordinates of the Lego brick and its orientation Ø in degrees. The final line contains the x, y and z coordinates of the destination location and its orientation Ø. The sample input is shown below:

#### Sample input
```markdown
robot_1_config.txt
-200 100 -7 60
100 150 -7 45
```


### Running the example code
Assuming that the Gazebo simulation world has been launched and services such as spawning a Lego brick have been accessed from the [lynxmotion_al5d_description](https://github.com/cognitive-robotics-course/lynxmotion_al5d_description) package, the following commands can be used to run the robotProgramming and pickAndPlace example code respectively:

`rosrun module4 robotProgramming`

`rosrun module4 pickAndPlace`

After either of the example code has been run, the workspace can be cleared by using the reset service found in the [lynxmotion_al5d_description](https://github.com/cognitive-robotics-course/lynxmotion_al5d_description) package.

