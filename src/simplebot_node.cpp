#include "simplebot.hpp"
#include "simplebot_ros_controller.hpp"

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>

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
    int stepsPerRotation;
    np.param("steps_per_rotation", stepsPerRotation, 90);
    double wheelRadius;
    np.param("wheel_radius", wheelRadius, 0.025);
    double axisLength;
    np.param("axis_length", axisLength, 0.18);

    std::cout << "connecting to device ..." << std::endl;
    Simplebot simplebot(port, baudrate, axisLength, wheelRadius, stepsPerRotation, maxSpeed);
    std::cout << "connected!" << std::endl;

    static SimplebotRosController controller(simplebot, n, tf::getPrefixParam(np));
    ros::Timer timer = n.createTimer(ros::Duration(0.1), &SimplebotRosController::timerHandler, &controller);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, &SimplebotRosController::moveCallback, &controller);

    std::function<void(double, double, double, double, double, double, double)> onOdometryReceived;
    onOdometryReceived = [](double x, double y, double theta, double vLeft, double vRight, double wheelsLeft, double wheelsRight)
    {
        controller.sendOdometry(x, y, theta, vLeft, vRight, wheelsLeft, wheelsRight);
    };

    simplebot.setOnOdometryHandler(onOdometryReceived);

    ros::spin();
}
