#include "simplebot.hpp"
#include "simplebot_ros_controller.hpp"

#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "simplebot");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    std::string port;
    np.param("port", port, std::string("/dev/ttyUSB0"));
    int baudrate;
    np.param("baudrate", baudrate, 115200);
    double maxSpeed;
    np.param("max_speed", maxSpeed, 1.0);

    std::cout << "connecting to device ..." << std::endl;
    Simplebot simplebot(port, baudrate, maxSpeed);
    sleep(2);
    std::cout << "connected!" << std::endl;

    SimplebotRosController controller(simplebot);
    ros::Timer timer = n.createTimer(ros::Duration(0.1), &SimplebotRosController::timerHandler, &controller);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, &SimplebotRosController::moveCallback, &controller);
    ros::spin();
}