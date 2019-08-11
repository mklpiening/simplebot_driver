#ifndef SIMPLEBOT_ROS_CONTROLLER_HPP
#define SIMPLEBOT_ROS_CONTROLLER_HPP

#include "simplebot.hpp"

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class SimplebotRosController 
{
public:
    SimplebotRosController(Simplebot& simplebot, ros::NodeHandle& n, std::string tfPrefix);
    void moveCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void timerHandler(const ros::TimerEvent& event);

    void sendOdometry(double x, double y, double theta, double vX, double vTheta, double wheelsLeft, double wheelsRight);
private:
    void populateCovariance(nav_msgs::Odometry &msg, double v_x, double v_theta);

    Simplebot& m_simplebot;
    ros::NodeHandle m_n;
    std::string m_tfPrefix;
    ros::Time m_lastMoveCmdTime;
    ros::Publisher m_odomPub;
    tf::TransformBroadcaster m_odomBroadcaster;
};

#endif
