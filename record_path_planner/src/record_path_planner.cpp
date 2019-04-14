
#include <record_path_planner/record_path_planner.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(autolabor_algorithm::RecordPathPlanner, nav_core::BaseGlobalPlanner)

namespace autolabor_algorithm {

    RecordPathPlanner::RecordPathPlanner() : initialized_(false) {

    }

    void RecordPathPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
        ros::NodeHandle private_nh("~/" + name);

        private_nh.param("map_frame", map_frame_, std::string("map"));
        private_nh.param("file_path", file_path_, std::string(""));

        full_path_pub_ = private_nh.advertise<nav_msgs::Path>("full_path", 1, true);
        sub_path_pub_ = private_nh.advertise<nav_msgs::Path>("sub_path", 1, true);

        if (file_path_.empty()) {
            ROS_ERROR_STREAM("default_path_name cannot be empty!");
            return;
        }
        std::ifstream is(file_path_);
        if (!is) {
            ROS_ERROR_STREAM("file : " + file_path_ + " not exist!");
        } else {
            record_path_.header.frame_id = map_frame_;
            record_path_.header.stamp = ros::Time::now();
            while (!is.eof()) {
                double x, y, yaw;
                geometry_msgs::PoseStamped poseStamped;
                is >> x >> y >> yaw;
                poseStamped.header.frame_id = map_frame_;
                poseStamped.header.stamp = ros::Time::now();
                poseStamped.pose.position.x = x;
                poseStamped.pose.position.y = y;
                poseStamped.pose.position.z = 0;
                poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                record_path_.poses.push_back(poseStamped);
            }
            full_path_pub_.publish(record_path_);
        }
        initialized_ = true;
    }

    bool RecordPathPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan) {
        plan.clear();
        if (record_path_.poses.empty()) {
            return false;
        } else {
            int start_index = min_distance_index(start, record_path_);
            int goal_index = min_distance_index(goal, record_path_);
            int dir = start_index < goal_index ? 1 : -1;
            for (auto i = static_cast<size_t>(start_index); i != static_cast<size_t>(goal_index); i = i + dir) {
                plan.push_back(record_path_.poses.at(i));
            }

            nav_msgs::Path sub_path;
            sub_path.header.frame_id = map_frame_;
            sub_path.header.stamp = ros::Time::now();
            sub_path.poses = plan;
            sub_path_pub_.publish(sub_path);
            return true;
        }
    }
}

//int main(int argc, char **argv) {
//    std::ifstream file;
//    file.open("/home/colin/RosProject/Workspace/catkin_simulation_dev/src/path_server/path_data/path.txt");
//    std::string tmp;
//    while (!file.eof()) {
//        double a, b, c;
//        file >> a >> b >> c;
//        std::cout << " -> " << a << " " << b << " " << c << std::endl;
//    }
//    file.close();
//    return 0;
//}