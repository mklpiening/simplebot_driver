#include "simplebot.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simplebot");
  ros::NodeHandle n;

  std::cout << "connecting to device ..." << std::endl;
  Simplebot simplebot;
  std::cout << "connected!" << std::endl;

  ros::spin();
}
