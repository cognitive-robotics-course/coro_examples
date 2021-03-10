/* This is a ROS version of the standard "Hello , World" program */

#include <ros/ros.h>                      // This header defines the standard ROS classes

int main(int argc, char **argv) {
   ros::init(argc, argv, "hello_world");  // Initialize the ROS system
   ros::NodeHandle nh;                    // Register this program as a ROS node
   ROS_INFO_STREAM("Hello World!");       // Send some output as a log message
}
