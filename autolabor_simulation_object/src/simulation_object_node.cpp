#include "tf/tf.h"

#include "autolabor_simulation_stage/Obstacle.h"
#include "autolabor_simulation_object/simulation_object_node.h"

namespace autolabor_simulation {

SimulationObject::SimulationObject(){
  ros::NodeHandle private_node("~");
  if (!private_node.getParam("object_id", object_id_)){
    throw runtime_error("obstacle_id is not exist");
  }

  server_.reset(new interactive_markers::InteractiveMarkerServer(object_id_));

  private_node.param("object_frame", object_frame_, string("real_map"));
  private_node.param("resolution", resolution_, 0.05);

  XmlRpc::XmlRpcValue xml_object_color;
  if (private_node.hasParam("object_color") && private_node.getParam("object_color", xml_object_color)){
    ROS_ASSERT(xml_object_color.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(xml_object_color.size() == 4);
    for (int i=0; i<4; i++){
      rgba_[i] = min(max(0.0, static_cast<double>(xml_object_color[i])), 1.0);
    }
  }

  XmlRpc::XmlRpcValue xml_object_footprints;
  if (private_node.getParam("object_footprints", xml_object_footprints)){
    ROS_ASSERT(xml_object_footprints.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i=0; i<xml_object_footprints.size(); i++){
      ROS_ASSERT(xml_object_footprints[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(xml_object_footprints[i].size() % 2 == 0);
      geometry_msgs::Polygon polygon;
      for (int j=0; j<xml_object_footprints[i].size(); j=j+2){
        ROS_ASSERT(xml_object_footprints[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(xml_object_footprints[i][j+1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        geometry_msgs::Point32 p;
        p.x = (float)(static_cast<double>(xml_object_footprints[i][j]));
        p.y = (float)(static_cast<double>(xml_object_footprints[i][j+1]));
        polygon.points.push_back(p);
      }
      object_footprints_.push_back(polygon);
    }
  }

  vector<geometry_msgs::Point> innerPoints;
  for (int i=0; i<object_footprints_.size(); i++){
    getInnerPoint(object_footprints_.at(i), innerPoints, resolution_);
  }

  visualization_msgs::InteractiveMarker interactive_marker_;
  interactive_marker_.header.frame_id = object_frame_;
  interactive_marker_.header.stamp = ros::Time::now();
  interactive_marker_.name = object_id_;

  visualization_msgs::Marker marker_;
  marker_.type = visualization_msgs::Marker::CUBE_LIST;
  marker_.scale.x = resolution_;
  marker_.scale.y = resolution_;
  marker_.scale.z = resolution_;
  marker_.color.r = rgba_[0];
  marker_.color.g = rgba_[1];
  marker_.color.b = rgba_[2];
  marker_.color.a = rgba_[3];
  marker_.points = innerPoints;

  visualization_msgs::InteractiveMarkerControl marker_control_;
  marker_control_.always_visible = true;
  marker_control_.markers.push_back(marker_);
  marker_control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  marker_control_.orientation.w = 0.7071;
  marker_control_.orientation.y = 0.7071;
  interactive_marker_.controls.push_back(marker_control_);

  visualization_msgs::InteractiveMarkerControl button_control_;
  button_control_.always_visible = true;
  button_control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  interactive_marker_.controls.push_back(button_control_);

  server_->insert(interactive_marker_, boost::bind(&SimulationObject::processFeedback, this, _1));
}

void SimulationObject::getFootprintsRange(geometry_msgs::Polygon& footprint, float& min_x, float& max_x, float& min_y, float& max_y){
  if (footprint.points.size() > 0){
    min_x = max_x = footprint.points.at(0).x;
    min_y = max_y = footprint.points.at(0).y;
    for (int i=1; i<footprint.points.size(); i++){
      min_x = footprint.points.at(i).x < min_x ? footprint.points.at(i).x : min_x;
      max_x = footprint.points.at(i).x > max_x ? footprint.points.at(i).x : max_x;
      min_y = footprint.points.at(i).y < min_y ? footprint.points.at(i).y : min_y;
      max_y = footprint.points.at(i).y > max_y ? footprint.points.at(i).y : max_y;
    }
  }
}

void SimulationObject::getInnerPoint(geometry_msgs::Polygon& footprint, std::vector<geometry_msgs::Point>& point_list, double resolution){
  float min_x, max_x, min_y, max_y;
  getFootprintsRange(footprint, min_x, max_x, min_y, max_y);
  for (float x = (std::ceil(min_x / resolution) * resolution); x <= max_x; x = x + resolution){
    for (float y = (std::ceil(min_y / resolution) * resolution); y <= max_y; y = y + resolution){
      if (pnpoly(footprint, x, y)){
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        point_list.push_back(p);
      }
    }
  }
}

bool SimulationObject::pnpoly(geometry_msgs::Polygon& footprint, float& x, float& y){
  int i,j;
  bool c = false;
  for (i=0, j=footprint.points.size()-1; i<footprint.points.size(); j = i++){
    if ( ( (footprint.points.at(i).y > y) != (footprint.points.at(j).y > y) ) &&
         (x < (footprint.points.at(j).x-footprint.points.at(i).x) * (y-footprint.points.at(i).y) / (footprint.points.at(j).y - footprint.points.at(i).y) + footprint.points.at(i).x) ){
      c = !c;
    }
  }
  return c;
}

void SimulationObject::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  if (transform_flag_){
    tf::Quaternion q;
    q.setX(feedback->pose.orientation.x);
    q.setY(feedback->pose.orientation.y);
    q.setZ(feedback->pose.orientation.z);
    q.setW(feedback->pose.orientation.w);

    autolabor_simulation_stage::Obstacle msg;
    msg.request.obstacle_id = object_id_;
    msg.request.type = autolabor_simulation_stage::Obstacle::Request::TRANSFORM;
    msg.request.transform[0] = feedback->pose.position.x;
    msg.request.transform[1] = feedback->pose.position.y;
    msg.request.transform[2] = tf::getYaw(q);
    client_.call(msg);
  }
}

void SimulationObject::menuFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  interactive_markers::MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  interactive_markers::MenuHandler::CheckState state;
  menu_handler_.getCheckState(handle, state);
  autolabor_simulation_stage::Obstacle msg;
  msg.request.obstacle_id = object_id_;
  if (state == interactive_markers::MenuHandler::CHECKED){
    msg.request.type = autolabor_simulation_stage::Obstacle::Request::DELETE;
    if (client_.call(msg)){
      menu_handler_.setCheckState(handle, interactive_markers::MenuHandler::UNCHECKED);
      transform_flag_ = false;
    }
  }else{
    tf::Quaternion q;
    q.setX(feedback->pose.orientation.x);
    q.setY(feedback->pose.orientation.y);
    q.setZ(feedback->pose.orientation.z);
    q.setW(feedback->pose.orientation.w);

    msg.request.type = autolabor_simulation_stage::Obstacle::Request::NEW;
    msg.request.vertex = object_footprints_;
    msg.request.transform[0] = feedback->pose.position.x;
    msg.request.transform[1] = feedback->pose.position.y;
    msg.request.transform[2] = tf::getYaw(q);

    if (client_.call(msg)){
      menu_handler_.setCheckState(handle, interactive_markers::MenuHandler::CHECKED);
      transform_flag_ = true;
    }
  }
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void SimulationObject::initMenu(){
  interactive_markers::MenuHandler::EntryHandle handle = menu_handler_.insert("Apply", boost::bind(&SimulationObject::menuFeedback, this, _1));
  menu_handler_.setCheckState(handle, interactive_markers::MenuHandler::UNCHECKED);
  transform_flag_ = false;
}

void SimulationObject::run(){
  initMenu();
  menu_handler_.apply(*server_, object_id_);
  client_ = nh_.serviceClient<autolabor_simulation_stage::Obstacle>("obstacle_handle");
  server_->applyChanges();
  ros::spin();
}

}


int main(int argc, char **argv){
  ros::init(argc, argv, "simulation_object_node");
  autolabor_simulation::SimulationObject simulation_object;
  simulation_object.run();
  return 0;
}
