#include "simplebot_ros_controller.hpp"

SimplebotRosController::SimplebotRosController(Simplebot& simplebot, ros::NodeHandle& n, std::string tfPrefix) :
        m_simplebot(simplebot),
        m_n(n),
        m_tfPrefix(tfPrefix),
        m_lastMoveCmdTime(0.0),
        m_odomPub(m_n.advertise<nav_msgs::Odometry> ("odom", 10)) { }

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

void SimplebotRosController::populateCovariance(nav_msgs::Odometry &msg, double v_x, double v_theta)
{
  double sigma_x_ = 0.002;
  double sigma_theta_ = 0.017;
  double cov_x_y_ = 0.0;
  double cov_x_theta_ = 0.0;
  double cov_y_theta_ = 0.0;

  double odom_multiplier = 1.0;

  if (fabs(v_x) <= 1e-8 && fabs(v_theta) <= 1e-8)
  {
    //nav_msgs::Odometry has a 6x6 covariance matrix
    msg.twist.covariance[0] = 1e-12;
    msg.twist.covariance[35] = 1e-12;

    msg.twist.covariance[30] = 1e-12;
    msg.twist.covariance[5] = 1e-12;
  }
  else
  {
    //nav_msgs::Odometry has a 6x6 covariance matrix
    msg.twist.covariance[0] = odom_multiplier * pow(sigma_x_, 2);
    msg.twist.covariance[35] = odom_multiplier * pow(sigma_theta_, 2);

    msg.twist.covariance[30] = odom_multiplier * cov_x_theta_;
    msg.twist.covariance[5] = odom_multiplier * cov_x_theta_;
  }

  msg.twist.covariance[7] = DBL_MAX;
  msg.twist.covariance[14] = DBL_MAX;
  msg.twist.covariance[21] = DBL_MAX;
  msg.twist.covariance[28] = DBL_MAX;

  msg.pose.covariance = msg.twist.covariance;

  if (fabs(v_x) <= 1e-8 && fabs(v_theta) <= 1e-8)
  {
    msg.pose.covariance[7] = 1e-12;

    msg.pose.covariance[1] = 1e-12;
    msg.pose.covariance[6] = 1e-12;

    msg.pose.covariance[31] = 1e-12;
    msg.pose.covariance[11] = 1e-12;
  }
  else
  {
    msg.pose.covariance[7] = odom_multiplier * pow(sigma_x_, 2) * pow(sigma_theta_, 2);

    msg.pose.covariance[1] = odom_multiplier * cov_x_y_;
    msg.pose.covariance[6] = odom_multiplier * cov_x_y_;

    msg.pose.covariance[31] = odom_multiplier * cov_y_theta_;
    msg.pose.covariance[11] = odom_multiplier * cov_y_theta_;
  }
}

void SimplebotRosController::sendOdometry(double x, double y, double theta, double vX, double vTheta, double wheelsLeft, double wheelsRight)
{
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
    //populateCovariance(odom, x, theta);

    m_odomPub.publish(odom);

    if (true)
    {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.frame_id = tf::resolve(m_tfPrefix, "odom_combined");
        odom_trans.child_frame_id = tf::resolve(m_tfPrefix, "base_footprint");

        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);

        m_odomBroadcaster.sendTransform(odom_trans);
    }

    /*sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = "left_front_wheel_joint";
    joint_state.name[1] = "left_middle_wheel_joint";
    joint_state.name[2] = "left_rear_wheel_joint";
    joint_state.name[3] = "right_front_wheel_joint";
    joint_state.name[4] = "right_middle_wheel_joint";
    joint_state.name[5] = "right_rear_wheel_joint";

    joint_state.position[0] = joint_state.position[1] = joint_state.position[2] = wheelpos_l;
    joint_state.position[3] = joint_state.position[4] = joint_state.position[5] = wheelpos_r;

    joint_pub_.publish(joint_state);*/
}
