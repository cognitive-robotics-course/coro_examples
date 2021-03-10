/* This client program uses a sample of turtlesim services */
/* /clear, /turtle1/set_pen, and /turtle1/telepor_absolute */

#include <ros/ros.h>
#include <turtlesim/TeleportAbsolute.h> // for turtle1/teleport_absolute service
#include <turtlesim/SetPen.h>           // for turtle1/set_pen service
#include <std_srvs/Empty.h>             // for reset and clear services

int main(int argc, char **argv) {

   bool success = true;

   /* Initialize the ROS system and become a node */
   ros::init(argc, argv, "dvernon"); // Initialize the ROS system
   ros::NodeHandle nh;               // Become a node

   /* Create client objects for the required services */
   ros::service::waitForService("turtle1/teleport_absolute");
   ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");

   ros::service::waitForService("turtle1/set_pen");
   ros::ServiceClient setpenClient = nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");

   ros::service::waitForService("clear");
   ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("clear");

   /* Create the request and response objects for the teleport and set_pen services */   
   turtlesim::TeleportAbsolute teleport_arguments; // reposition the turtle without locomotion
   turtlesim::SetPen           pen_arguments;      // turn the pen on/off and change colour
   std_srvs::Empty             clear_arguments;    // to clear the background

   /* clear the simulator background */
   success = clearClient.call(clear_arguments);
   if (!success) {
      ROS_ERROR_STREAM("Turtle failed to clear" );
   }

   /* turn the pen off so that we don't see a trace when the turtle teleports */
   pen_arguments.request.off = 1;
   success = setpenClient.call(pen_arguments);
   if (!success) {
      ROS_ERROR_STREAM("TurtlePen failed to switch off");
   }
      
   teleport_arguments.request.x     = 2.5;         // location
   teleport_arguments.request.y     = 3.5;         // coordinates
   teleport_arguments.request.theta = 3.14159 / 2; // facing up, i.e. 90 degrees

   success = teleportClient.call(teleport_arguments);
   if (!success) {
      ROS_ERROR_STREAM("Turtle failed to teleport" );
   }

   /* turn the pen again so that we do see a trace when the turtle moves later on */
   pen_arguments.request.off    = 0;    
   pen_arguments.request.r      = 255; // white
   pen_arguments.request.g      = 255; // pen
   pen_arguments.request.b      = 255; // colour
   pen_arguments.request.width = 1;    // narrow line
      
   success = setpenClient.call(pen_arguments);
   if (!success) {
      ROS_ERROR_STREAM("TurtlePen failed to switch off");
   }
}
