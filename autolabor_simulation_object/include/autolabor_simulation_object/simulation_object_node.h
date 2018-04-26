#ifndef SIMULATION_OBJECT_NODE_H
#define SIMULATION_OBJECT_NODE_H

#include "string"

#include "ros/ros.h"

#include "geometry_msgs/Polygon.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerControl.h"
#include "interactive_markers/interactive_marker_server.h"
#include "interactive_markers/menu_handler.h"
#include "visualization_msgs/Marker.h"
using namespace std;

namespace autolabor_simulation {
class SimulationObject
{
public:
  SimulationObject();
  ~SimulationObject() {}

  void run();
private:
  void getInnerPoint(geometry_msgs::Polygon& footprint, vector<geometry_msgs::Point>& point_list, double resolution);
  bool pnpoly(geometry_msgs::Polygon& footprint, float& x, float& y);
  void getFootprintsRange(geometry_msgs::Polygon& footprint, float& min_x, float& max_x, float& min_y, float& max_y);
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void menuFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void initMenu();


  string object_id_, object_frame_;
  double resolution_;
  bool transform_flag_;
  vector<geometry_msgs::Polygon> object_footprints_;
  double rgba_[4] = {0.0, 0.0, 1.0, 1.0};

  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

};
}


#endif // SIMULATION_OBJECT_NODE_H
