#include "simplebot.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

Simplebot::Simplebot()
  : odom_pub_(nh_.advertise<nav_msgs::Odometry>("odom", 10))
  , joint_pub_(nh_.advertise<sensor_msgs::JointState>("joint_states", 1))
  , cmd_vel_sub_(nh_.subscribe("cmd_vel", 1, &Simplebot::moveCallback, this))
  , cmd_timeout_timer_(nh_.createTimer(ros::Duration(0.1), &Simplebot::timeoutTimerHandler, this))
  , last_move_cmd_time_(0.0)
  , cmd_publish_timer_(nh_.createTimer(ros::Duration(0.1), &Simplebot::publishTimerHandler, this))
  , io_()
  , x_(0.0)
  , y_(0.0)
  , theta_(0.0)
  , vel_changed_(true)
  , vl_(0.0)
  , vr_(0.0)
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
  nh_p.param("pose_variance", pose_variance_, 0.0);
  nh_p.param("twist_variance", twist_variance_, 0.0);
  nh_p.param("publish_motor_stat", publish_motor_stats_, false);

  if (publish_motor_stats_)
  {
    front_left_speed_pub = nh_.advertise<std_msgs::Float32>("simplebot/speed_fl", 1);
    front_right_speed_pub = nh_.advertise<std_msgs::Float32>("simplebot/speed_fr", 1);
    rear_left_speed_pub = nh_.advertise<std_msgs::Float32>("simplebot/speed_rl", 1);
    rear_right_speed_pub = nh_.advertise<std_msgs::Float32>("simplebot/speed_rr", 1);
  }

  tf_prefix_ = tf::getPrefixParam(nh_p);

  serial_ = std::make_unique<boost::asio::serial_port>(io_, port);
  serial_->set_option(boost::asio::serial_port_base::baud_rate(baudrate));

  // wait for connection
  sleep(2);

  readOdometry();

  boost::thread t(boost::bind(&boost::asio::io_service::run, &io_));
  io_thread_.swap(t);

  reconfigure_callback_ = boost::bind(&Simplebot::reconfigureCallback, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_callback_);
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

void Simplebot::publishTimerHandler(const ros::TimerEvent&)
{
  if (vel_changed_)
  {
    int64_t rot_l = vl_ * 10000 / (2 * M_PI * wheel_radius_);
    int64_t rot_r = vr_ * 10000 / (2 * M_PI * wheel_radius_);

    uint8_t msg[20];

    msg[0] = 0xFF;
    msg[1] = 0x00;

    msg[2] = (uint8_t)((rot_l)&0xFF);
    msg[3] = (uint8_t)((rot_l >> 8) & 0xFF);
    msg[4] = (uint8_t)((rot_l >> 16) & 0xFF);
    msg[5] = (uint8_t)((rot_l >> 24) & 0xFF);
    msg[6] = (uint8_t)((rot_l >> 32) & 0xFF);
    msg[7] = (uint8_t)((rot_l >> 40) & 0xFF);
    msg[8] = (uint8_t)((rot_l >> 48) & 0xFF);
    msg[9] = (uint8_t)((rot_l >> 56) & 0xFF);

    msg[10] = (uint8_t)((rot_r)&0xFF);
    msg[11] = (uint8_t)((rot_r >> 8) & 0xFF);
    msg[12] = (uint8_t)((rot_r >> 16) & 0xFF);
    msg[13] = (uint8_t)((rot_r >> 24) & 0xFF);
    msg[14] = (uint8_t)((rot_r >> 32) & 0xFF);
    msg[15] = (uint8_t)((rot_r >> 40) & 0xFF);
    msg[16] = (uint8_t)((rot_r >> 48) & 0xFF);
    msg[17] = (uint8_t)((rot_r >> 56) & 0xFF);

    msg[18] = (uint8_t)(!checkParity(rot_l)) | (uint8_t)(!checkParity(rot_r)) << 1;

    msg[19] = 0xFF;

    boost::asio::write(*serial_, boost::asio::buffer(msg, 20));
    vel_changed_ = false;
  }
}

void Simplebot::timeoutTimerHandler(const ros::TimerEvent&)
{
  if (ros::Time::now() - last_move_cmd_time_ > ros::Duration(1.0))
  {
    setSpeed(0.0, 0.0);
  }
}

void Simplebot::setSpeed(double vL, double vR)
{
  vl_ = vL;
  vr_ = vR;
  vel_changed_ = true;
}

void Simplebot::setMotorParams(float fr_rps_to_pwm, float fr_kp, float fr_ki, float fr_kd, float br_rps_to_pwm,
                               float br_kp, float br_ki, float br_kd, float fl_rps_to_pwm, float fl_kp, float fl_ki,
                               float fl_kd, float bl_rps_to_pwm, float bl_kp, float bl_ki, float bl_kd)
{
  uint8_t msg[131];

  int i = 0;
  msg[i++] = 0xFF;
  msg[i++] = 0x01;

  int64_t kp;
  int64_t ki;
  int64_t kd;

  int64_t rps_to_pwm;

  kp = fr_kp * 10000;
  ki = fr_ki * 10000;
  kd = fr_kd * 10000;

  msg[i++] = (uint8_t)((kp)&0xFF);
  msg[i++] = (uint8_t)((kp >> 8) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 16) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 24) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 32) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 40) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 48) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 56) & 0xFF);

  msg[i++] = (uint8_t)((ki)&0xFF);
  msg[i++] = (uint8_t)((ki >> 8) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 16) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 24) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 32) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 40) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 48) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 56) & 0xFF);

  msg[i++] = (uint8_t)((kd)&0xFF);
  msg[i++] = (uint8_t)((kd >> 8) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 16) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 24) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 32) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 40) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 48) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 56) & 0xFF);

  rps_to_pwm = fr_rps_to_pwm * 10000;

  msg[i++] = (uint8_t)((rps_to_pwm)&0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 8) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 16) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 24) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 32) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 40) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 48) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 56) & 0xFF);

  kp = br_kp * 10000;
  ki = br_ki * 10000;
  kd = br_kd * 10000;

  msg[i++] = (uint8_t)((kp)&0xFF);
  msg[i++] = (uint8_t)((kp >> 8) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 16) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 24) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 32) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 40) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 48) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 56) & 0xFF);

  msg[i++] = (uint8_t)((ki)&0xFF);
  msg[i++] = (uint8_t)((ki >> 8) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 16) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 24) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 32) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 40) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 48) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 56) & 0xFF);

  msg[i++] = (uint8_t)((kd)&0xFF);
  msg[i++] = (uint8_t)((kd >> 8) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 16) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 24) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 32) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 40) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 48) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 56) & 0xFF);

  rps_to_pwm = br_rps_to_pwm * 10000;

  msg[i++] = (uint8_t)((rps_to_pwm)&0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 8) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 16) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 24) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 32) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 40) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 48) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 56) & 0xFF);

  kp = fl_kp * 10000;
  ki = fl_ki * 10000;
  kd = fl_kd * 10000;

  msg[i++] = (uint8_t)((kp)&0xFF);
  msg[i++] = (uint8_t)((kp >> 8) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 16) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 24) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 32) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 40) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 48) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 56) & 0xFF);

  msg[i++] = (uint8_t)((ki)&0xFF);
  msg[i++] = (uint8_t)((ki >> 8) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 16) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 24) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 32) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 40) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 48) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 56) & 0xFF);

  msg[i++] = (uint8_t)((kd)&0xFF);
  msg[i++] = (uint8_t)((kd >> 8) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 16) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 24) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 32) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 40) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 48) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 56) & 0xFF);

  rps_to_pwm = fl_rps_to_pwm * 10000;

  msg[i++] = (uint8_t)((rps_to_pwm)&0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 8) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 16) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 24) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 32) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 40) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 48) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 56) & 0xFF);

  kp = bl_kp * 10000;
  ki = bl_ki * 10000;
  kd = bl_kd * 10000;

  msg[i++] = (uint8_t)((kp)&0xFF);
  msg[i++] = (uint8_t)((kp >> 8) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 16) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 24) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 32) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 40) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 48) & 0xFF);
  msg[i++] = (uint8_t)((kp >> 56) & 0xFF);

  msg[i++] = (uint8_t)((ki)&0xFF);
  msg[i++] = (uint8_t)((ki >> 8) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 16) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 24) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 32) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 40) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 48) & 0xFF);
  msg[i++] = (uint8_t)((ki >> 56) & 0xFF);

  msg[i++] = (uint8_t)((kd)&0xFF);
  msg[i++] = (uint8_t)((kd >> 8) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 16) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 24) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 32) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 40) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 48) & 0xFF);
  msg[i++] = (uint8_t)((kd >> 56) & 0xFF);

  rps_to_pwm = bl_rps_to_pwm * 10000;

  msg[i++] = (uint8_t)((rps_to_pwm)&0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 8) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 16) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 24) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 32) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 40) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 48) & 0xFF);
  msg[i++] = (uint8_t)((rps_to_pwm >> 56) & 0xFF);

  msg[i++] = 0xFF;

  boost::asio::write(*serial_, boost::asio::buffer(msg, 131));
}

void Simplebot::odometryCallback(const boost::system::error_code& error, std::size_t bytes_transferred)
{
  if (!error && bytes_transferred > 0)
  {
    if (receive_buffer_[0] == 0xFF)
    {
      int16_t raw_d_rot_fl = 0;
      raw_d_rot_fl = raw_d_rot_fl | (receive_buffer_[1]) | (((int16_t)receive_buffer_[2]) << 8);
      float d_rot_fl = (float)raw_d_rot_fl / 10000.0;
      float d_dist_fl = 2.0 * M_PI * wheel_radius_ * d_rot_fl;

      int16_t raw_d_rot_rl = 0;
      raw_d_rot_rl = raw_d_rot_rl | (receive_buffer_[3]) | (((int16_t)receive_buffer_[4]) << 8);
      float d_rot_rl = (float)raw_d_rot_rl / 10000.0;
      float d_dist_rl = 2.0 * M_PI * wheel_radius_ * d_rot_rl;

      int16_t raw_d_rot_fr = 0;
      raw_d_rot_fr = raw_d_rot_fr | (receive_buffer_[5]) | (((int16_t)receive_buffer_[6]) << 8);
      float d_rot_fr = (float)raw_d_rot_fr / 10000.0;
      float d_dist_fr = 2.0 * M_PI * wheel_radius_ * d_rot_fr;

      int16_t raw_d_rot_rr = 0;
      raw_d_rot_rr = raw_d_rot_rr | (receive_buffer_[7]) | (((int16_t)receive_buffer_[8]) << 8);
      float d_rot_rr = (float)raw_d_rot_rr / 10000.0;
      float d_dist_rr = 2.0 * M_PI * wheel_radius_ * d_rot_rr;

      uint8_t d_time = receive_buffer_[9];

      double dRotLeft = (d_rot_fl + d_rot_rl) / 2;
      double dRotRight = (d_rot_fr + d_rot_rr) / 2;

      double distLeft = (d_dist_fl + d_dist_rl) / 2;
      double distRight = (d_dist_fr + d_dist_rr) / 2;

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

      odom.pose.covariance[0] = pose_variance_;
      odom.pose.covariance[7] = pose_variance_;
      odom.pose.covariance[14] = pose_variance_;
      odom.pose.covariance[21] = pose_variance_;
      odom.pose.covariance[28] = pose_variance_;
      odom.pose.covariance[35] = pose_variance_;

      odom.twist.twist.linear.x = vX;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.angular.z = vTheta;

      odom.twist.covariance[0] = twist_variance_;
      odom.twist.covariance[7] = twist_variance_;
      odom.twist.covariance[14] = twist_variance_;
      odom.twist.covariance[21] = twist_variance_;
      odom.twist.covariance[28] = twist_variance_;
      odom.twist.covariance[35] = twist_variance_;

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

      if (publish_motor_stats_)
      {
        std_msgs::Float32 speed_msg;

        speed_msg.data = d_dist_fl * 1000 / d_time;
        front_left_speed_pub.publish(speed_msg);

        speed_msg.data = d_dist_rl * 1000 / d_time;
        rear_left_speed_pub.publish(speed_msg);

        speed_msg.data = d_dist_fr * 1000 / d_time;
        front_right_speed_pub.publish(speed_msg);

        speed_msg.data = d_dist_rr * 1000 / d_time;
        rear_right_speed_pub.publish(speed_msg);
      }
    }
  }

  readOdometry();
}

void Simplebot::readOdometry()
{
  serial_->async_read_some(boost::asio::buffer(receive_buffer_, 10),
                           boost::bind(&Simplebot::odometryCallback, this, boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred));
}

void Simplebot::reconfigureCallback(simplebot_driver::SimplebotConfig& config, uint32_t level)
{
  setMotorParams(config.fr_rps_to_pwm, config.fr_kp, config.fr_ki, config.fr_kd, config.br_rps_to_pwm, config.br_kp,
                 config.br_ki, config.br_kd, config.fl_rps_to_pwm, config.fl_kp, config.fl_ki, config.fl_kd,
                 config.bl_rps_to_pwm, config.bl_kp, config.bl_ki, config.bl_kd);
}
