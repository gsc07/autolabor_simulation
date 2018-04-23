#ifndef AUTOLABORFAKE_H_
#define AUTOLABORFAKE_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <atomic>
#include <thread>

namespace autolabor_fake {
  class AutolaborFake {
    public:
      /**
       * @brief Cosntructor for fake node, will launch a new thread to continuely
       * broadcast tf and odom msgs
       */
      AutolaborFake(ros::NodeHandle& nh, float rate, float max_x_accel,
                                    float max_theta_accel, float cmd_timeout);

      ~AutolaborFake();

      void cmdCB(geometry_msgs::Twist::ConstPtr cmd);

    private:

      void broadcastThreadFunc();

      ros::NodeHandle&              nh_;
      ros::Rate                     rate_;
      ros::Subscriber               cmd_sub_;
      // broadccast tf and odom message thread
      std::thread                   bc_thread_;
      std::atomic<float>            vx_, vy_;
      std::atomic<float>            vth_;
      ros::Time                     last_time_, current_time_;
      ros::Time                     last_cb_;    // TODO: need wrap it by atomic
      float                         max_x_accel_;
      float                         max_theta_accel_;
      float                         cmd_timeout_;
      float                         x_, y_, th_;

  };
}















#endif
