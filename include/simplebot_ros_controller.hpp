#ifndef SIMPLEBOT_ROS_CONTROLLER_HPP
#define SIMPLEBOT_ROS_CONTROLLER_HPP

#include "simplebot.hpp"

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class SimplebotRosController 
{
public:
    SimplebotRosController(Simplebot& simplebot, ros::NodeHandle& n, std::string tfPrefix, bool publishTf);
    void moveCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void timerHandler(const ros::TimerEvent& event);

    void sendOdometry(double x, double y, double theta, double vX, double vTheta, double wheelsLeft, double wheelsRight);
private:
    Simplebot& m_simplebot;
    ros::NodeHandle m_n;
    std::string m_tfPrefix;
    ros::Time m_lastMoveCmdTime;
    bool m_publisTf;

    ros::Publisher m_odomPub;
    ros::Publisher m_jointPub;
    tf::TransformBroadcaster m_odomBroadcaster;
};

#endif
