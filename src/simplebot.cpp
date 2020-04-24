#include "simplebot.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

Simplebot::Simplebot()
  : odom_pub_(nh_.advertise<nav_msgs::Odometry>("odom", 10))
  , joint_pub_(nh_.advertise<sensor_msgs::JointState>("joint_states", 1))
  , cmd_vel_sub_(nh_.subscribe("cmd_vel", 10, &Simplebot::moveCallback, this))
  , cmd_timeout_timer_(nh_.createTimer(ros::Duration(0.1), &Simplebot::timerHandler, this))
  , last_move_cmd_time_(0.0)
  , io_()
  , x_(0.0)
  , y_(0.0)
  , theta_(0.0)
{
  ros::NodeHandle nh_p("~");

  std::string port;
  int baudrate;

  nh_p.param("port", port, std::string("/dev/ttyUSB0"));
  nh_p.param("baudrate", baudrate, 115200);
  nh_p.param("max_speed", max_speed_, 1.0);
  nh_p.param("wheel_radius", wheel_radius_, 0.025);
  nh_p.param("axis_length", axis_length_, 0.18);
  nh_p.param("turning_adaption", turning_adaptation_, 0.85);
  nh_p.param("publish_tf", publish_tf_, true);

  tf_prefix_ = tf::getPrefixParam(nh_p);

  serial_ = std::make_unique<boost::asio::serial_port>(io_, port);
  serial_->set_option(boost::asio::serial_port_base::baud_rate(baudrate));

  // wait for connection
  sleep(2);

  readOdometry();

  boost::thread t(boost::bind(&boost::asio::io_service::run, &io_));
  io_thread_.swap(t);
}

Simplebot::~Simplebot()
{
}

void Simplebot::moveCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  last_move_cmd_time_ = ros::Time::now();

  double speedL = std::max(-max_speed_, std::min(max_speed_, msg->linear.x - msg->angular.z * 0.5));
  double speedR = std::max(-max_speed_, std::min(max_speed_, msg->linear.x + msg->angular.z * 0.5));

  setSpeed(speedL, speedR);
}

void Simplebot::timerHandler(const ros::TimerEvent&)
{
  if (ros::Time::now() - last_move_cmd_time_ > ros::Duration(0.5))
  {
    setSpeed(0.0, 0.0);
  }
}

bool checkParity(int64_t data)
{
  int numOnes = 0;
  for (int i = 0; i < 64; i++)
  {
    if (data & (1 << i))
    {
      numOnes++;
    }
  }

  return numOnes % 2 == 0;
}

void Simplebot::setSpeed(double vL, double vR)
{
  int64_t rotL = vL * 10000 / (2 * M_PI * wheel_radius_);
  int64_t rotR = vR * 10000 / (2 * M_PI * wheel_radius_);

  uint8_t msg[19];

  msg[0] = 0xFF;

  msg[1] = (uint8_t)((rotL)&0xFF);
  msg[2] = (uint8_t)((rotL >> 8) & 0xFF);
  msg[3] = (uint8_t)((rotL >> 16) & 0xFF);
  msg[4] = (uint8_t)((rotL >> 24) & 0xFF);
  msg[5] = (uint8_t)((rotL >> 32) & 0xFF);
  msg[6] = (uint8_t)((rotL >> 40) & 0xFF);
  msg[7] = (uint8_t)((rotL >> 48) & 0xFF);
  msg[8] = (uint8_t)((rotL >> 56) & 0xFF);

  msg[9] = (uint8_t)((rotR)&0xFF);
  msg[10] = (uint8_t)((rotR >> 8) & 0xFF);
  msg[11] = (uint8_t)((rotR >> 16) & 0xFF);
  msg[12] = (uint8_t)((rotR >> 24) & 0xFF);
  msg[13] = (uint8_t)((rotR >> 32) & 0xFF);
  msg[14] = (uint8_t)((rotR >> 40) & 0xFF);
  msg[15] = (uint8_t)((rotR >> 48) & 0xFF);
  msg[16] = (uint8_t)((rotR >> 56) & 0xFF);

  msg[17] = (uint8_t)(!checkParity(rotL)) | (uint8_t)(!checkParity(rotR)) << 1;

  msg[18] = 0xFF;

  boost::asio::write(*serial_, boost::asio::buffer(msg, 19));
}

void Simplebot::odometryCallback(const boost::system::error_code& error, std::size_t bytes_transferred)
{
  if (!error && bytes_transferred > 0)
  {
    if (receive_buffer_[0] == 0xFF)
    {
      long dRotL = 0;
      dRotL = dRotL | (receive_buffer_[1]) | (((long)receive_buffer_[2]) << 8) | (((long)receive_buffer_[3]) << 16) |
              (((long)receive_buffer_[4]) << 24) | (((long)receive_buffer_[5]) << 32) |
              (((long)receive_buffer_[6]) << 40) | (((long)receive_buffer_[7]) << 48) |
              (((long)receive_buffer_[8]) << 56);

      long dRotR = 0;
      dRotR = dRotR | (receive_buffer_[9]) | (((long)receive_buffer_[10]) << 8) | (((long)receive_buffer_[11]) << 16) |
              (((long)receive_buffer_[12]) << 24) | (((long)receive_buffer_[13]) << 32) |
              (((long)receive_buffer_[14]) << 40) | (((long)receive_buffer_[15]) << 48) |
              (((long)receive_buffer_[16]) << 56);

      double dRotLeft = (double)dRotL / 10000.0;
      double dRotRight = (double)dRotR / 10000.0;

      double distLeft = 2.0 * M_PI * wheel_radius_ * dRotLeft;
      double distRight = 2.0 * M_PI * wheel_radius_ * dRotRight;

      double dTheta = (distRight - distLeft) / axis_length_ * turning_adaptation_;
      double hypothenuse = 0.5 * (distLeft + distRight);

      if (abs(hypothenuse * cos(theta_ + dTheta * 0.5)) < 1)
      {
        x_ += hypothenuse * cos(theta_ + dTheta * 0.5);
        y_ += hypothenuse * sin(theta_ + dTheta * 0.5);

        theta_ += dTheta;
        if (theta_ > M_PI)
        {
          theta_ -= 2.0 * M_PI;
        }
        if (theta_ < -M_PI)
        {
          theta_ += 2.0 * M_PI;
        }
      }

      double wheelsLeft = 2.0 * M_PI * fmod(dRotLeft, 1);
      if (wheelsLeft > M_PI)
      {
        wheelsLeft -= 2.0 * M_PI;
      }
      if (wheelsLeft < -M_PI)
      {
        wheelsLeft += 2.0 * M_PI;
      }

      double wheelsRight = 2.0 * M_PI * fmod(dRotRight, 1);
      if (wheelsRight > M_PI)
      {
        wheelsRight -= 2.0 * M_PI;
      }
      if (wheelsRight < -M_PI)
      {
        wheelsRight += 2.0 * M_PI;
      }

      double vX = (distLeft + distRight) * 0.5;
      double vTheta = (distRight - distLeft) / axis_length_ * turning_adaptation_;

      // publish odom
      nav_msgs::Odometry odom;
      odom.header.frame_id = tf::resolve(tf_prefix_, "odom_combined");
      odom.child_frame_id = tf::resolve(tf_prefix_, "base_footprint");

      odom.header.stamp = ros::Time::now();
      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);

      odom.pose.covariance[0] = 1;
      odom.pose.covariance[7] = 1;
      odom.pose.covariance[14] = 1;
      odom.pose.covariance[21] = 1;
      odom.pose.covariance[28] = 1;
      odom.pose.covariance[35] = 1;

      odom.twist.twist.linear.x = vX;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.angular.z = vTheta;

      odom.twist.covariance[0] = 1;
      odom.twist.covariance[7] = 1;
      odom.twist.covariance[14] = 1;
      odom.twist.covariance[21] = 1;
      odom.twist.covariance[28] = 1;
      odom.twist.covariance[35] = 1;

      odom_pub_.publish(odom);

      // publish tf
      if (publish_tf_)
      {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.frame_id = tf::resolve(tf_prefix_, "odom_combined");
        odom_trans.child_frame_id = tf::resolve(tf_prefix_, "base_footprint");

        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta_);

        odom_broadcaster_.sendTransform(odom_trans);
      }

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

      joint_pub_.publish(joint_state);
    }
  }

  readOdometry();
}

void Simplebot::readOdometry()
{
  serial_->async_read_some(boost::asio::buffer(receive_buffer_, 17),
                           boost::bind(&Simplebot::odometryCallback, this, boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred));
}
