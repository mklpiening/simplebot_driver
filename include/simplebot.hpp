#ifndef SIMPLEBOT_HPP
#define SIMPLEBOT_HPP

#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

class Simplebot
{
public:
  Simplebot();
  ~Simplebot();

  void setSpeed(double vL, double vR);

  void odometryCallback(const boost::system::error_code& error, std::size_t bytes_transferred);

private:
  void moveCallback(const geometry_msgs::Twist::ConstPtr& msg);

  void test(int a);

  void timerHandler(const ros::TimerEvent& event);

  void readOdometry();

  ros::NodeHandle nh_;

  ros::Publisher odom_pub_;
  ros::Publisher joint_pub_;

  ros::Publisher front_left_speed_pub;
  ros::Publisher front_right_speed_pub;
  ros::Publisher rear_left_speed_pub;
  ros::Publisher rear_right_speed_pub;

  tf::TransformBroadcaster odom_broadcaster_;
  ros::Subscriber cmd_vel_sub_;
  ros::Timer cmd_timeout_timer_;

  ros::Time last_move_cmd_time_;

  boost::thread io_thread_;
  boost::asio::io_service io_;
  std::unique_ptr<boost::asio::serial_port> serial_;

  double max_speed_;
  double axis_length_;
  double turning_adaptation_;
  double wheel_radius_;
  bool publish_tf_;
  std::string tf_prefix_;
  double pose_variance_;
  double twist_variance_;
  bool publish_motor_stats_;

  double x_;
  double y_;

  double theta_;

  uint8_t receive_buffer_[10];
};

#endif
