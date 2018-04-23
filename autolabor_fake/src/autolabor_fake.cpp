#include "autolabor_fake/autolabor_fake.h"
#include <ros/ros.h>
#include <thread>

namespace autolabor_fake {

  AutolaborFake::AutolaborFake(ros::NodeHandle& nh, float rate, float max_x_accel, float max_theta_accel,
      float cmd_timeout) : nh_(nh), rate_(rate), bc_thread_(&AutolaborFake::broadcastThreadFunc, this),
      max_x_accel_(max_x_accel), max_theta_accel_(max_theta_accel), cmd_timeout_(cmd_timeout) {
    vx_ = 0., vy_ = 0.;
    vth_ = 0.;
    x_ = 0., y_ = 0., th_ = 0.;
    last_time_ = ros::Time::now();
    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, boost::bind(&AutolaborFake::cmdCB, this, _1));
  }

  AutolaborFake::~AutolaborFake() {
    ;
  }


  void AutolaborFake::broadcastThreadFunc() {
    ros::Publisher odom_pub = nh_.advertise< nav_msgs::Odometry >("odom", 10);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time last_time, current_time;
    bool timeout_flag = false;
    while(nh_.ok())
    {
      ros::Time current_time = ros::Time::now();

      if ( (current_time - last_cb_).toSec() > cmd_timeout_)
      {
        if (timeout_flag == false) {
          ROS_WARN( "subscriber timeout, set all velocity to Zero" );
          vx_ = 0.; vy_ = 0.; vth_ = 0.;
          timeout_flag = true;
        }
      } else {
        timeout_flag = false;
      }

      float dt       = (current_time - last_time).toSec();
      float delta_x  = vx_*cos(th_)*dt - vy_*sin(th_)*dt;
      float delta_y  = vx_*sin(th_)*dt + vy_*cos(th_)*dt;
      float delta_th = vth_*dt;

      x_ += delta_x;
      y_ += delta_y;
      th_ += delta_th;

      // setuo tf frame
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x_;
      odom_trans.transform.translation.y = y_;
      odom_trans.transform.translation.z = 0.0;

      geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(th_);
      odom_trans.transform.rotation      = quat;

      // broadcast tf frame
      odom_broadcaster.sendTransform(odom_trans);

      //set up odom frame
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "base_link";

      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = quat;

      odom.child_frame_id = "odom";
      odom.twist.twist.linear.x = vx_;
      odom.twist.twist.linear.y = vy_;
      odom.twist.twist.angular.z = vth_;

      // publish odom frame
      odom_pub.publish(odom);

      last_time = current_time;
      rate_.sleep();
    }

  }


  void AutolaborFake::cmdCB(geometry_msgs::Twist::ConstPtr cmd) {
    current_time_ = ros::Time::now();

    double dt = std::min(( current_time_ - last_time_ ).toSec(), 0.5);

    float max_delta_vx = max_x_accel_ * dt;
    float max_delta_vth = max_theta_accel_ * dt;

    ROS_INFO("I heard twist message, vx: %lf, vy: %lf, vtheta: %lf", cmd->linear.x, cmd->linear.y, cmd->angular.z);

    float tmp_vx = cmd->linear.x;
    float tmp_vth = cmd->angular.z;

    float d_vx  = tmp_vx - vx_;
    float d_vth = tmp_vth - vy_;

    vy_ = 0.;

    if ( fabs(d_vx) > max_delta_vx )
    {
      vx_ = vx_ + (max_delta_vx * (d_vx > 0. ? 1. : -1.));
      ROS_WARN("accelerat ax too large, wanted velocity is: %f, real velocity is: %f", tmp_vx, vx_.load());
    } else {
      vx_ = tmp_vx;
    }

    if ( fabs(d_vth) > max_delta_vth )
    {
      vth_ = vth_ + (max_delta_vth * (d_vth > 0. ? 1. : -1.));
      ROS_WARN("accelerat ath too large, wanted theta velocity is: %f, real velocity is: %f", tmp_vth, vth_.load());
    } else {
      vth_ = tmp_vth;
    }

    last_cb_ = last_time_ = current_time_;
  }


}



