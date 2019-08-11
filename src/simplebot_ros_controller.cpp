#include "simplebot_ros_controller.hpp"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

SimplebotRosController::SimplebotRosController(Simplebot& simplebot, ros::NodeHandle& n, std::string tfPrefix) :
        m_simplebot(simplebot),
        m_n(n),
        m_tfPrefix(tfPrefix),
        m_lastMoveCmdTime(0.0),
        m_odomPub(m_n.advertise<nav_msgs::Odometry> ("odom", 10)),
        m_jointPub(m_n.advertise<sensor_msgs::JointState> ("joint_states", 1)) { }

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

void SimplebotRosController::sendOdometry(double x, double y, double theta, double vX, double vTheta, double wheelsLeft, double wheelsRight)
{
    // publish odom
    nav_msgs::Odometry odom;
    odom.header.frame_id = tf::resolve(m_tfPrefix, "odom_combined");
    odom.child_frame_id = tf::resolve(m_tfPrefix, "base_footprint");

    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    odom.twist.twist.linear.x = vX;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vTheta;

    m_odomPub.publish(odom);

    // publish tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = tf::resolve(m_tfPrefix, "odom_combined");
    odom_trans.child_frame_id = tf::resolve(m_tfPrefix, "base_footprint");

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);

    m_odomBroadcaster.sendTransform(odom_trans);

    // publish wheel rotations
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(4);
    joint_state.position.resize(4);
    joint_state.name[0] = "left_front_wheel_joint";
    joint_state.name[1] = "left_rear_wheel_joint";
    joint_state.name[2] = "right_front_wheel_joint";
    joint_state.name[3] = "right_rear_wheel_joint";

    joint_state.position[0] = joint_state.position[1] = wheelsLeft;
    joint_state.position[2] = joint_state.position[3] = wheelsRight;

    m_jointPub.publish(joint_state);
}
