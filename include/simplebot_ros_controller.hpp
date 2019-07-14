#ifndef SIMPLEBOT_ROS_CONTROLLER_HPP
#define SIMPLEBOT_ROS_CONTROLLER_HPP

#include "simplebot.hpp"

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class SimplebotRosController 
{
public:
    SimplebotRosController(Simplebot& simplebot);
    void moveCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void timerHandler(const ros::TimerEvent& event);

private:
    Simplebot& m_simplebot;
    ros::Time m_lastMoveCmdTime;
};

#endif