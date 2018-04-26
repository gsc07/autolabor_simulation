#include "autolabor_simulation_lidar/simulation_lidar_node.h"

namespace autolabor_simulation {

SimulationLidar::SimulationLidar(){
  ros::NodeHandle private_node("~");
  private_node.param("min_angle", min_angle_, -M_PI);
  private_node.param("max_angle", max_angle_, M_PI);
  private_node.param("min_distance", min_dis_, 0.15);
  private_node.param("max_distance", max_dis_, 6.00);
  private_node.param("noise", noise_, 0.00);
  private_node.param("size", point_size_, 400);
  private_node.param("rate", rate_, 10);

  private_node.param("stage_map_topic", stage_map_topic_, string("stage_map"));
  private_node.param("global_frame", global_frame_, string("real_map"));
  private_node.param("lidar_frame", lidar_frame_, string("lidar"));
  step_ = (max_angle_ - min_angle_) / (point_size_ - 1);
}

double SimulationLidar::gaussRand(double mu, double sigma){
  static double V1, V2, S;
  static int phase = 0;
  double X;

  if ( phase == 0 ) {
    do {
      double U1 = (double)rand() / RAND_MAX;
      double U2 = (double)rand() / RAND_MAX;

      V1 = 2 * U1 - 1;
      V2 = 2 * U2 - 1;
      S = V1 * V1 + V2 * V2;
    } while(S >= 1 || S == 0);

    X = V1 * sqrt(-2 * log(S) / S);
  } else
    X = V2 * sqrt(-2 * log(S) / S);

  phase = 1 - phase;
  return (X*sigma + mu);
}

void SimulationLidar::initLaserScan(sensor_msgs::LaserScan &laser_scan){
  laser_scan.header.frame_id = lidar_frame_;
  laser_scan.angle_min = min_angle_;
  laser_scan.angle_max = max_angle_;
  laser_scan.angle_increment = step_;
  laser_scan.time_increment = 1 / point_size_ / rate_;
  laser_scan.range_min = min_dis_;
  laser_scan.range_max = max_dis_;
}

double SimulationLidar::normalAngle(double theta){
  if (theta > M_PI){
    theta = normalAngle(theta - 2*M_PI);
  }else if (theta <= -M_PI){
    theta = normalAngle(theta + 2*M_PI);
  }
  return theta;
}

void SimulationLidar::getPose(tf::StampedTransform &transform, double &start_angle, double &reverse){
  double roll, pitch, yaw;
  tf_.lookupTransform(global_frame_, lidar_frame_, ros::Time(), transform);
  transform.getBasis().getRPY(roll, pitch, yaw);
  if (pow(roll,2) + pow(pitch,2) > esp){
      start_angle = yaw + max_angle_;
      reverse = -1.0;
  }else{
      start_angle = yaw + min_angle_;
      reverse = 1.0;
  }
}

bool SimulationLidar::limit_range(int &mx, int &my){
  bool x_flag = (mx >= local_map_.info.width) || (mx <= 0);
  mx = x_flag ? std::max(std::min(mx, (int)local_map_.info.width), 0) : mx;
  bool y_flag = (my >= local_map_.info.height) || (my <= 0);
  my = y_flag ? std::max(std::min(my, (int)local_map_.info.height), 0) : my;
  return x_flag || y_flag;
}

void SimulationLidar::distance(int start_x, int start_y, double theta, int& end_x, int& end_y){
  theta = normalAngle(theta);
  double k, e=-0.5;
  int major_step, minor_step;
  if (limit_range(start_x, start_y)){
    end_x = start_x;
    end_y = start_y;
    return;
  }else if (std::abs(theta) <= (0.25 * M_PI) || std::abs(theta) >= (0.75 * M_PI)){
    k = std::abs(std::tan(theta));
    major_step = std::abs(theta) <= (0.25 * M_PI) ? 1 : -1;
    minor_step = theta >= 0 ? 1 : -1;
    while (true){
      start_x += major_step;
      e += k;
      if (e >= 0){
        start_y += minor_step;
        e -= 1;
      }
      if (limit_range(start_x, start_y) || getValue(start_x, start_y) == OBS_SPACE || getValue(start_x, start_y) == UNKNOWN_SPACE){
        end_x = start_x;
        end_y = start_y;
        return;
      }
    }
  }else{
    k = std::abs(tan(0.5 * M_PI - theta));
    major_step = theta > 0 ? 1 : -1;
    minor_step = std::abs(theta) <= (0.5 * M_PI) ? 1 : -1;
    while (true){
      start_y += major_step;
      e += k;
      if (e >= 0){
        start_x += minor_step;
        e -= 1;
      }
      if (limit_range(start_x, start_y) || getValue(start_x, start_y) == OBS_SPACE || getValue(start_x, start_y) == UNKNOWN_SPACE){
        end_x = start_x;
        end_y = start_y;
        return;
      }
    }
  }
}

void SimulationLidar::mapToWorld(int mx, int my, float &wx, float &wy){
  wx = local_map_.info.origin.position.x + (mx + 0.5) * local_map_.info.resolution;
  wy = local_map_.info.origin.position.y + (my + 0.5) * local_map_.info.resolution;
}

void SimulationLidar::wordToMap(float wx, float wy, int &mx, int &my){
  mx = (int)((wx - local_map_.info.origin.position.x) / local_map_.info.resolution);
  my = (int)((wy - local_map_.info.origin.position.y) / local_map_.info.resolution);
}

char SimulationLidar::getValue(int mx, int my){
  int index = my * local_map_.info.width + mx;
  if (index < 0 && index > data_length_){
    return ERROR;
  }else{
    return local_map_.data.at(index);
  }
}

char SimulationLidar::getValue(float wx, float wy){
  int mx, my;
  wordToMap(wx, wy, mx, my);
  return getValue(mx, my);
}

void SimulationLidar::distance(float start_x, float start_y, double theta, float &end_x, float &end_y){
  int start_mx, start_my, end_mx, end_my;
  wordToMap(start_x, start_y, start_mx, start_my);
  distance(start_mx, start_my, theta, end_mx, end_my);
  mapToWorld(end_mx, end_my, end_x, end_y);
}

void SimulationLidar::getFrame(sensor_msgs::LaserScan &laser_scan){
  tf::StampedTransform transform;
  double start_angle, reverse, angle;
  float start_wx, start_wy, end_wx, end_wy, dis;
  laser_scan.header.stamp = ros::Time::now();
  getPose(transform, start_angle, reverse);
  start_wx = (float)transform.getOrigin().getX();
  start_wy = (float)transform.getOrigin().getY();
  laser_scan.ranges.clear();
  for (int i=0; i<point_size_; i++){
    angle = start_angle + i * reverse * step_;
    distance(start_wx, start_wy, angle, end_wx, end_wy);
    dis = sqrt(pow(end_wx - start_wx, 2) + pow(end_wy - start_wy, 2));
    dis += dis * gaussRand(0, noise_);
    if (dis <= max_dis_ && dis >= min_dis_){
      laser_scan.ranges.push_back(dis);
    }else{
      laser_scan.ranges.push_back(numeric_limits<float>::infinity());
    }
  }
}

void SimulationLidar::pubLaserCallback(const ros::TimerEvent &event){
  sensor_msgs::LaserScan laserscan;
  initLaserScan(laserscan);
  map_mutex_.lock();
  getFrame(laserscan);
  map_mutex_.unlock();
  lidar_pub_.publish(laserscan);
}

void SimulationLidar::mapReceived(const nav_msgs::OccupancyGrid::ConstPtr &grid_map){
  map_mutex_.lock();
  local_map_ = *grid_map;
  data_length_ = grid_map->info.width * grid_map->info.height;
  map_mutex_.unlock();
}

void SimulationLidar::run(){
  tf_.waitForTransform(global_frame_, lidar_frame_, ros::Time(), ros::Duration(1.0));
  lidar_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
  map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(stage_map_topic_, 1, &SimulationLidar::mapReceived, this);
  pub_laser_timer_ = nh_.createTimer(ros::Duration(1.0/rate_), &SimulationLidar::pubLaserCallback, this);
  ros::spin();
}

}

int main(int argc, char **argv){
  ros::init(argc, argv, "simulation_lidar_node");
  autolabor_simulation::SimulationLidar simulation_lidar;
  simulation_lidar.run();
  return 0;
}
