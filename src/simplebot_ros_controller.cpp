#include "simplebot_ros_controller.hpp"

SimplebotRosController::SimplebotRosController(Simplebot& simplebot) : 
        m_simplebot(simplebot), 
        m_lastMoveCmdTime(0.0) { }

void SimplebotRosController::moveCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    m_lastMoveCmdTime = ros::Time::now();

    double speed = std::max(-m_simplebot.m_maxSpeed, std::min(m_simplebot.m_maxSpeed, msg->linear.x));
    m_simplebot.setSpeed(speed - msg->angular.z * 0.5, speed + msg->angular.z * 0.5);
}

void SimplebotRosController::timerHandler(const ros::TimerEvent& event)
{
    if (ros::Time::now() - m_lastMoveCmdTime > ros::Duration(0.5)) 
    {
        m_simplebot.setSpeed(0.0, 0.0);
    }
}