
#include <record_path_planner/record_path_planner.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(autolabor_algorithm::RecordPathPlanner, nav_core::BaseGlobalPlanner)

namespace autolabor_algorithm {

    RecordPathPlanner::RecordPathPlanner() : initialized_(false), start_index_(-1), goal_index_(-1) {

    }

    void RecordPathPlanner::receive_path_callback(const nav_msgs::Path::ConstPtr &msg) {
        ROS_INFO_STREAM("RECEIVE PATH!");
        record_path_ = *msg;
    }

    void RecordPathPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
        ros::NodeHandle private_nh("~/" + name);

        private_nh.param<std::string>("map_frame", map_frame_, std::string("map"));
        private_nh.param<double>("goal_change_threshold", goal_change_threshold_, 0.1);

        sub_path_pub_ = private_nh.advertise<nav_msgs::Path>("sub_path", 1, true);
        path_subscribe_ = nh_.subscribe<nav_msgs::Path>("recorded_path", 10, &RecordPathPlanner::receive_path_callback, this);
        initialized_ = true;
    }

    bool RecordPathPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan) {

        if (record_path_.poses.empty()) {
            ROS_INFO_STREAM("POSE EMPTY!");
            return false;
        } else if (goal.header.frame_id != map_frame_) {
            ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", map_frame_.c_str(), goal.header.frame_id.c_str());
            return false;
        } else if (start.header.frame_id != map_frame_) {
            ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", map_frame_.c_str(), start.header.frame_id.c_str());
            return false;
        }

        // 重新设置目标点
        if (goal_index_ == -1 || norm2(goal, goal_cache_) >= (goal_change_threshold_ * goal_change_threshold_)) {
            plan.clear();
            start_index_ = min_distance_index(start, record_path_);
            goal_index_ = min_distance_index(goal, record_path_);
            dir_ = start_index_ < goal_index_ ? 1 : -1;
            for (auto i = static_cast<size_t>(start_index_); i != static_cast<size_t>(goal_index_); i = i + dir_) {
                geometry_msgs::PoseStamped org = record_path_.poses.at(i);
                org.pose.orientation = tf::createQuaternionMsgFromYaw(dir_ == 1 ? tf::getYaw(org.pose.orientation) : (tf::getYaw(org.pose.orientation) + M_PI));
                plan.push_back(org);
            }
            plan_cache_ = plan;
            goal_cache_ = goal;
        } else {
            if (plan_cache_.size() > 1) {
                int tmp = 0;
                for (auto i = 1; i < plan_cache_.size(); i++) {
                    if (norm2(start, plan_cache_.at(static_cast<unsigned long>(i-1))) <= norm2(start, plan_cache_.at(static_cast<unsigned long>(i)))) {
                        tmp = i-1;
                        break;
                    }
                }
                plan_cache_.erase(plan_cache_.begin(), plan_cache_.begin() + tmp);
            }
            plan = plan_cache_;
        }

        nav_msgs::Path sub_path;
        sub_path.header.frame_id = map_frame_;
        sub_path.header.stamp = ros::Time::now();
        sub_path.poses = plan_cache_;
        sub_path_pub_.publish(sub_path);
        return true;
    }
}
