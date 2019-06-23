#include <pluginlib/class_list_macros.hpp>
#include <actionlib_msgs/GoalStatus.h>
#include <loop_path_planner/loop_path_planner.h>


PLUGINLIB_EXPORT_CLASS(autolabor_algorithm::LoopPathPlanner, nav_core::BaseGlobalPlanner)

namespace autolabor_algorithm {

    LoopPathPlanner::LoopPathPlanner() : initialized_(false), start_index_(-1), goal_index_(-1) {

    }

    void LoopPathPlanner::receive_path_callback(const nav_msgs::Path::ConstPtr &msg) {
        ROS_INFO_STREAM("RECEIVE PATH!");
        record_path_ = *msg;
    }

    void LoopPathPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
        if (!initialized_) {
            ros::NodeHandle private_nh("~/" + name);

            private_nh.param<std::string>("map_frame", map_frame_, std::string("map"));
            private_nh.param<double>("path_length", path_length_, 0.5);
            private_nh.param<int>("direction", direction_, 1);
            private_nh.param<bool>("loop", loop_, true); // 是否重复
            private_nh.param<bool>("round", round_, false); // true -> 来回走, false -> 循环走

            sub_path_pub_ = private_nh.advertise<nav_msgs::Path>("sub_path", 1, true);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

            result_subscribe_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1, &LoopPathPlanner::reach_goal, this);
            path_subscribe_ = nh_.subscribe<nav_msgs::Path>("recorded_path", 10, &LoopPathPlanner::receive_path_callback, this);
            initialized_ = true;
        }
    }

    void LoopPathPlanner::reach_goal(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg) {
        if (msg->status.status == 3 && loop_) {
            if (round_) {
                direction_ = -direction_;
                if (direction_ == 1) {
                    goal_pub_.publish(record_path_.poses.at(record_path_.poses.size() - 1));
                } else {
                    goal_pub_.publish(record_path_.poses.at(0));
                }
            }
        }
    }

    void LoopPathPlanner::pub_path(std::vector<geometry_msgs::PoseStamped> &plan) {
        nav_msgs::Path sub_path;
        sub_path.header.frame_id = map_frame_;
        sub_path.header.stamp = ros::Time::now();
        sub_path.poses = plan;
        sub_path_pub_.publish(sub_path);
    }

    bool LoopPathPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan) {
        if (record_path_.poses.size() <= 1) {
            ROS_INFO_STREAM("PATH TOO SHORT!");
            return false;
        } else if (goal.header.frame_id != map_frame_) {
            ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", map_frame_.c_str(), goal.header.frame_id.c_str());
            return false;
        } else if (start.header.frame_id != map_frame_) {
            ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", map_frame_.c_str(), start.header.frame_id.c_str());
            return false;
        }

        plan.clear();
        if (start_index_ == -1 || norm2(goal, goal_cache_) >= 0.01) {
            start_index_ = min_distance_index(start, record_path_);
            if (round_) {
                direction_ = start_index_ < (record_path_.poses.size() / 2) ? 1 : -1;
            }
            goal_cache_ = goal;
        } else {
            start_index_ = min_distance_index_inner(start, record_path_, static_cast<size_t>(start_index_), static_cast<size_t>(goal_index_));
        }

        int next_index = start_index_;
        double sum = 0;
        int from_index, to_index;
        for (;;) {
            if (direction_ > 0) {
                from_index = next_index;
                to_index = next_index + 1;
                if (to_index >= record_path_.poses.size()) {
                    if (loop_ && !round_) {
                        to_index = 0;
                    } else {
                        break;
                    }
                }
            } else {
                from_index = next_index;
                to_index = next_index - 1;
                if (to_index < 0) {
                    if (loop_ && !round_) {
                        to_index = static_cast<int>(record_path_.poses.size() - 1);
                    } else {
                        break;
                    }
                }
            }
            sum += std::sqrt(norm2(record_path_.poses.at(static_cast<unsigned long>(from_index)), record_path_.poses.at(static_cast<unsigned long>(to_index))));
            geometry_msgs::PoseStamped org = record_path_.poses.at(static_cast<unsigned long>(to_index));
            org.pose.orientation = tf::createQuaternionMsgFromYaw(direction_ > 0 ? tf::getYaw(org.pose.orientation) : (tf::getYaw(org.pose.orientation) + M_PI));
            plan.push_back(org);
            next_index = to_index;
            if (sum >= path_length_) {
                goal_index_ = to_index;
                break;
            }
        }
        pub_path(plan);
        return true;
    }
}