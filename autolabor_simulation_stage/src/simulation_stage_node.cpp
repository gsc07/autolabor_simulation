#include "nav_msgs/GetMap.h"
#include "autolabor_simulation_stage/simulation_stage_node.h"

namespace autolabor_simulation {

SimulationStage::SimulationStage():exist_obstacle_(false){
  ros::NodeHandle private_node("~");
  private_node.param("input_topic", input_topic_, std::string("map"));
  private_node.param("output_topic", output_topic_, std::string("stage_map"));
  private_node.param("real_map_frame", real_map_frame_, std::string("real_map"));
}

SimulationStage::~SimulationStage(){
  if (data_length_ > 0){
    delete [] map_data_;
    delete [] obstacle_data_;
  }
}

char SimulationStage::getValue(int index){
  if (index < 0 && index > data_length_){
    return ERROR;
  }
  if (exist_obstacle_){
    return obstacle_data_[index] == OBS_SPACE ? OBS_SPACE : map_data_[index];
  }else{
    return map_data_[index];
  }
}

void SimulationStage::publishMap(){
  nav_msgs::OccupancyGrid pub_map_msg;
  ros::Time current_time = ros::Time::now();
  map_mutex_.lock();
  pub_map_msg.header.stamp = current_time;
  pub_map_msg.header.frame_id = real_map_frame_;
  pub_map_msg.info.map_load_time = current_time;
  pub_map_msg.info.resolution = resolution_;
  pub_map_msg.info.width = size_x_;
  pub_map_msg.info.height = size_y_;
  pub_map_msg.info.origin.position.x = origin_x_;
  pub_map_msg.info.origin.position.y = origin_y_;
  pub_map_msg.info.origin.orientation.w = 1.0;
  for (int i=0; i<data_length_; i++){
    pub_map_msg.data.push_back(getValue(i));
  }
  map_mutex_.unlock();
  map_pub_.publish(pub_map_msg);
}

void SimulationStage::mapReceived(const nav_msgs::OccupancyGrid::ConstPtr& grid_map){
  map_mutex_.lock();
  resolution_ = grid_map->info.resolution;
  origin_x_ = grid_map->info.origin.position.x;
  origin_y_ = grid_map->info.origin.position.y;
  size_x_ = grid_map->info.width;
  size_y_ = grid_map->info.height;
  data_length_ = size_x_ * size_y_;
  obstacle_data_ = new char[data_length_];
  map_data_ = new char[data_length_];
  for (int i=0; i<data_length_; i++){
    map_data_[i] = grid_map->data.at(i);
  }
  map_mutex_.unlock();
  publishMap();
}

void SimulationStage::mapToWorld(int mx, int my, float &wx, float &wy){
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

void SimulationStage::wordToMap(float wx, float wy, int &mx, int &my){
  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);
}

char SimulationStage::getValue(int mx, int my){
  int index = my * size_x_ + mx;
  if (index < 0 && index > data_length_){
    return ERROR;
  }else{
    if (exist_obstacle_){
      return obstacle_data_[index] == OBS_SPACE ? OBS_SPACE : map_data_[index];
    }else{
      return map_data_[index];
    }
  }
}

char SimulationStage::getValue(float wx, float wy){
  int mx, my;
  wordToMap(wx, wy, mx, my);
  return getValue(mx, my);
}

void SimulationStage::getObstacleRange(footprint& footprint, float& min_x, float& max_x, float& min_y, float& max_y){
  if (footprint.size() > 0){
    min_x = max_x = footprint.at(0).x;
    min_y = max_y = footprint.at(0).y;
    for (int i=1; i<footprint.size(); i++){
      min_x = footprint.at(i).x < min_x ? footprint.at(i).x : min_x;
      max_x = footprint.at(i).x > max_x ? footprint.at(i).x : max_x;
      min_y = footprint.at(i).y < min_y ? footprint.at(i).y : min_y;
      max_y = footprint.at(i).y > max_y ? footprint.at(i).y : max_y;
    }
    min_x = std::max(min_x, (float)origin_x_);
    max_x = std::min(max_x, (float)(origin_x_ + size_x_ * resolution_));
    min_y = std::max(min_y, (float)origin_y_);
    max_y = std::min(max_y, (float)(origin_y_ + size_y_ * resolution_));
  }
}

bool SimulationStage::pnpoly(footprint& footprint, float& x, float& y){
  int i,j;
  bool c = false;
  for (i=0, j=footprint.size()-1; i<footprint.size(); j = i++){
    if ( ( (footprint.at(i).y > y) != (footprint.at(j).y > y) ) &&
         (x < (footprint.at(j).x-footprint.at(i).x) * (y-footprint.at(i).y) / (footprint.at(j).y - footprint.at(i).y) + footprint.at(i).x) ){
      c = !c;
    }
  }
  return c;
}

void SimulationStage::getObstaclePoints(footprint& f, footprint& obs_points){
  float min_x, max_x, min_y, max_y;
  getObstacleRange(f, min_x, max_x, min_y, max_y);
  for (float x = (std::ceil(min_x / resolution_) * resolution_); x <= max_x; x = x + resolution_/2){
    for (float y = (std::ceil(min_y / resolution_) * resolution_); y <= max_y; y = y + resolution_/2){
      if (pnpoly(f, x, y)){
        Point p; p.x = x; p.y = y;
        obs_points.push_back(p);
      }
    }
  }
}

void SimulationStage::deleteObstacle(string obstacle_name){
  if (footprints_map_.find(obstacle_name) != footprints_map_.end()){
    footprints_map_.erase(obstacle_name);
  }

  if (obstacle_points_map_.find(obstacle_name) != obstacle_points_map_.end()){
    obstacle_points_map_.erase(obstacle_name);
  }

  if(obstacle_points_map_.size() == 0){
    exist_obstacle_ = false;
  }

}

void SimulationStage::transformFootprint(footprint_list &source, footprint &target, float x, float y, float yaw){
  float cos_th = cos(yaw);
  float sin_th = sin(yaw);
  footprint f;
  for (int i=0; i<source.size(); i++){
    f.clear();
    for (int j=0; j<source[i].size(); j++){
      Point p;
      p.x =  x + (source[i][j].x * cos_th - source[i][j].y * sin_th);
      p.y =  y + (source[i][j].x * sin_th + source[i][j].y * cos_th);
      f.push_back(p);
    }
    getObstaclePoints(f, target);
  }
}

void SimulationStage::transformObstacle(string obstacle_name, float x, float y, float yaw){
  footprints_map::iterator iter = footprints_map_.find(obstacle_name);
  if (iter == footprints_map_.end()){
    throw runtime_error("No specified obstacle");
  }

  if (obstacle_points_map_.find(obstacle_name) != obstacle_points_map_.end()){
    obstacle_points_map_.erase(obstacle_name);
  }
  footprint obs_points;
  transformFootprint(iter->second, obs_points, x, y, yaw);
  obstacle_points_map_[obstacle_name] = obs_points;
}

void SimulationStage::insertObstacle(string obstacle_name, vector<geometry_msgs::Polygon> polygon_list, float x, float y, float yaw){
  deleteObstacle(obstacle_name);
  footprint_list obstacle_footprints;
  for (int i=0; i<polygon_list.size(); i++){
    footprint f;
    for (int j=0; j<polygon_list.at(i).points.size(); j++){
      Point p;
      p.x = polygon_list.at(i).points.at(j).x;
      p.y = polygon_list.at(i).points.at(j).y;
      f.push_back(p);
    }
    obstacle_footprints.push_back(f);
  }
  footprints_map_[obstacle_name] = obstacle_footprints;

  footprint obstacle_points;
  transformFootprint(obstacle_footprints, obstacle_points, x, y, yaw);
  obstacle_points_map_[obstacle_name] = obstacle_points;
  exist_obstacle_ = true;
}

void SimulationStage::updateObstacleData(){
  int mx, my, index;
  memset(obstacle_data_, 0, data_length_);
  for (obstacle_points_map::iterator iter = obstacle_points_map_.begin(); iter!=obstacle_points_map_.end(); iter++){
    for (int i=0; i<(iter->second).size(); i++){
      wordToMap((iter->second)[i].x, (iter->second)[i].y, mx, my);
      index = my * size_x_ + mx;
      if (index >=0 && index <= data_length_){
        obstacle_data_[index] = OBS_SPACE;
      }
    }
  }
  publishMap();
}

bool SimulationStage::obstacleHandleServer(autolabor_simulation_stage::Obstacle::Request &req, autolabor_simulation_stage::Obstacle::Response &res){
  if (req.type == req.TRANSFORM){
    transformObstacle(req.obstacle_id, req.transform[0], req.transform[1], req.transform[2]);
  }else if (req.type == req.NEW){
    insertObstacle(req.obstacle_id, req.vertex, req.transform[0], req.transform[1], req.transform[2]);
  }else if (req.type == req.DELETE){
    deleteObstacle(req.obstacle_id);
  }else{
    return false;
  }
  updateObstacleData();
  return true;
}

void SimulationStage::run(){
  map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(input_topic_, 1, &SimulationStage::mapReceived, this);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(output_topic_, 1, true);
  obstacle_service_ = nh_.advertiseService("obstacle_handle", &SimulationStage::obstacleHandleServer, this);
  ros::spin();
}

}

int main(int argc, char **argv){
  ros::init(argc, argv, "simulation_stage_node");
  autolabor_simulation::SimulationStage simulation_stage;
  simulation_stage.run();
  return 0;
}
