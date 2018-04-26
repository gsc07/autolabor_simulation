#ifndef SIMULATION_STAGE_NODE_H_
#define SIMULATION_STAGE_NODE_H_

#include <map>
#include <vector>

#include "ros/ros.h"

#include "tf/tf.h"
#include "geometry_msgs/Polygon.h"
#include "nav_msgs/OccupancyGrid.h"

#include "autolabor_simulation_stage/Obstacle.h"

#define ERROR          -2
#define UNKNOWN_SPACE  -1
#define FREE_SPACE     0
#define OBS_SPACE      100

using namespace std;

struct Point
{
  float x;
  float y;
};

typedef vector<Point> footprint;
typedef vector<footprint> footprint_list;
typedef map<string, footprint_list> footprints_map;
typedef map<string, footprint> obstacle_points_map;

namespace autolabor_simulation {
class SimulationStage
{
public:
  SimulationStage();
  ~SimulationStage();

  void run();
private:
  bool obstacleHandleServer(autolabor_simulation_stage::Obstacle::Request &req, autolabor_simulation_stage::Obstacle::Response &res);
  void mapReceived(const nav_msgs::OccupancyGrid::ConstPtr& grid_map);
  void publishMap();

  void insertObstacle(string obstacle_name, vector<geometry_msgs::Polygon> polygon_list, float x, float y, float yaw);
  void transformObstacle(string obstacle_name, float x, float y, float yaw);
  void deleteObstacle(string obstacle_name);

  void updateObstacleData();

  void getObstacleRange(footprint& footprint, float& min_x, float& max_x, float& min_y, float& max_y);
  bool pnpoly(footprint& footprint, float& x, float& y);
  void getObstaclePoints(footprint& f, footprint& obs_points);
  void transformFootprint(footprint_list& source, footprint& target, float x, float y, float yaw);

  void setObstacleValue(int mx, int my, char value);
  void setObstacleValue(float wx, float wy, char value);

  void mapToWorld(int mx, int my, float& wx, float& wy);
  void wordToMap(float wx, float wy, int& mx, int& my);

  char getValue(int mx, int my);
  char getValue(float wx, float wy);
  char getValue(int index);

  bool limit_range(int& mx, int& my);

  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Publisher map_pub_;
  ros::ServiceServer obstacle_service_;

  std::string input_topic_, output_topic_, real_map_frame_;

  int size_x_, size_y_;
  double origin_x_, origin_y_, resolution_;
  unsigned int data_length_;
  char* map_data_;
  char* obstacle_data_;

  bool exist_obstacle_;

  boost::mutex map_mutex_;

  footprints_map footprints_map_;
  obstacle_points_map obstacle_points_map_;
};

inline bool SimulationStage::limit_range(int &mx, int &my){
  bool x_flag = (mx >= size_x_) || (mx <= 0);
  mx = x_flag ? std::max(std::min(mx, size_x_), 0) : mx;
  bool y_flag = (my >= size_y_) || (my <= 0);
  my = y_flag ? std::max(std::min(my, size_y_), 0) : my;
  return x_flag || y_flag;
}

}

#endif
