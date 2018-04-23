#include "autolabor_fake/autolabor_fake.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "autolabor_fake");

  ros::NodeHandle nh("autolabor");

  double rate, max_x_accel, max_theta_accel, cmd_timeout;
  nh.param( "rate", rate, 10.0 );
  nh.param( "max_x_accel", max_x_accel, 1.0 );
  nh.param( "max_theta_accel", max_theta_accel, 1.0 );
  nh.param( "cmd_timeout", cmd_timeout, 0.2 );

  autolabor_fake::AutolaborFake autolabor(nh, rate, max_x_accel, max_theta_accel, cmd_timeout);
  ros::spin();
}
