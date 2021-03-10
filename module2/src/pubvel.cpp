/* This program publishes randomly−generated velocity messages for turtlesim */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // For geometry_msgs::Twist 
#include <stdlib.h>              // For rand() and RAND_MAX

int main(int argc , char **argv) {
   ros::init(argc, argv, "publish_velocity"); // Initialize the ROS system
   ros::NodeHandle nh;                        // Become a node
   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000); 
   srand(time(0));    // Seed the random number generator
   ros::Rate rate(2); // Loop at 2Hz until the node is shut down

   while(ros::ok()) {
      
      geometry_msgs::Twist msg;                                    // Create the message
      msg.linear.x =  double(rand())/double(RAND_MAX);             // fill in the fields
      msg.angular.z = 2*double(rand())/double(RAND_MAX)-1;         // other fields default to 0
      pub.publish(msg);                                            // Publish the message

      /* Send a message to rosout with the details */
      ROS_INFO_STREAM("Sending random velocity command:" << " linear =" << msg.linear.x << " angular =" << msg.angular.z);
      rate.sleep(); // Wait until it's time for another iteration
   }
}
