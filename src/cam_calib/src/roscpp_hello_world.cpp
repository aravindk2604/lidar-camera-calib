/*
 * Hello World Example using ROS and CPP
 */

// Include the ROS library
#include <ros/ros.h>

// Main function
int main(int argc, char** argv)
{ 
  // Initialize the ROS Node "roscpp_hello_world"
  ros::init(argc, argv, "roscpp_hello_world");

  // Instantiate the ROS Node Handler as nh
  ros::NodeHandle nh;
  
  // Print "Hello ROS!" to the terminal and ROS log file
  ROS_INFO_STREAM("Hello from ROS node " << ros::this_node::getName());
  
  // Program succesful
  return 0;
}