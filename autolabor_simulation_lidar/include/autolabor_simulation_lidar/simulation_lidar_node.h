#ifndef SIMULATION_LIDAR_NODE_H
#define SIMULATION_LIDAR_NODE_H

#include "ros/ros.h"

#include "tf/transform_listener.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

#define ERROR          -2
#define UNKNOWN_SPACE  -1
#define FREE_SPACE     0
#define OBS_SPACE      100

#define esp 1e-6

using namespace std;

namespace autolabor_simulation {
class SimulationLidar
{
public:
  SimulationLidar();
  ~SimulationLidar(){}

  void run();
private:
  double normalAngle(double theta);
  bool limit_range(int& mx, int& my);

  double gaussRand(double mu, double sigma);

  void mapToWorld(int mx, int my, float& wx, float& wy);
  void wordToMap(float wx, float wy, int& mx, int& my);

  char getValue(int mx, int my);
  char getValue(float wx, float wy);

  void initLaserScan(sensor_msgs::LaserScan& laser_scan);
  void getFrame(sensor_msgs::LaserScan& laser_scan);

  void getPose(tf::StampedTransform& transform, double& start_angle, double& reverse);

  void distance(int start_x, int start_y, double theta, int& end_x, int& end_y);
  void distance(float start_x, float start_y, double theta, float& end_x, float& end_y);

  void mapReceived(const nav_msgs::OccupancyGrid::ConstPtr& grid_map);
  void pubLaserCallback(const ros::TimerEvent& event);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

  string global_frame_, lidar_frame_, stage_map_topic_, odom_topic_;
  double min_angle_, max_angle_, step_;
  double min_dis_, max_dis_;
  double noise_;
  int point_size_;
  int rate_;
  bool use_topic_odom_;

  unsigned int data_length_;
  nav_msgs::OccupancyGrid local_map_;
  nav_msgs::Odometry odom_msg_;

  boost::mutex map_mutex_;

  tf::TransformListener tf_;
  ros::NodeHandle nh_;
  ros::Publisher lidar_pub_;
  ros::Subscriber map_sub_, odom_sub_;
  ros::Timer pub_laser_timer_;
};
}

#endif // SIMULATION_LIDAR_NODE_H
