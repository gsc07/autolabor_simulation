#ifndef PROJECT_LOOP_PATH_PLANNER_H
#define PROJECT_LOOP_PATH_PLANNER_H

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>

namespace autolabor_algorithm {

    class LoopPathPlanner : public nav_core::BaseGlobalPlanner {
    public:
        LoopPathPlanner();

        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);

    private:
        void receive_path_callback(const nav_msgs::Path::ConstPtr &msg);

        void reach_goal(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);

        void pub_path(std::vector<geometry_msgs::PoseStamped> &plan);

        inline double norm2(const geometry_msgs::PoseStamped &from, geometry_msgs::PoseStamped &to) {
            double delta_x = from.pose.position.x - to.pose.position.x;
            double delta_y = from.pose.position.y - to.pose.position.y;
            return delta_x * delta_x + delta_y * delta_y;
        }

        inline int min_distance_index(const geometry_msgs::PoseStamped &from, nav_msgs::Path &path) {
            int index = -1;
            double min_value = std::numeric_limits<double>::max();
            double dis;
            for (size_t i = 0; i < path.poses.size(); i++) {
                dis = norm2(from, path.poses.at(i));
                if (dis < min_value) {
                    min_value = dis;
                    index = static_cast<int>(i);
                }
            }
            return index;
        }

        inline int positive_modulo(int i, int n) {
            return (i % n + n) % n;
        }

        inline int min_distance_index_inner(const geometry_msgs::PoseStamped &from, nav_msgs::Path &path, size_t from_index, size_t to_index) {
            int index = static_cast<int>(from_index);
            double min_value = std::numeric_limits<double>::max();
            double dis;
            for (int i = static_cast<int>(from_index); positive_modulo(i, static_cast<int>(path.poses.size())) != to_index; i = i + direction_) {
                auto tmp_index = static_cast<size_t>(positive_modulo(i, static_cast<int>(path.poses.size())));
                dis = norm2(from, path.poses.at(tmp_index));
                if (dis < min_value) {
                    min_value = dis;
                    index = static_cast<int>(tmp_index);
                }
            }
            return index;
        }

    private:
        bool initialized_;
        std::string map_frame_;
        double path_length_;
        bool loop_, round_;
        int direction_;

        nav_msgs::Path record_path_;
        geometry_msgs::PoseStamped goal_cache_;
        std::vector<geometry_msgs::PoseStamped> plan_cache_;

        int start_index_, goal_index_;

        ros::NodeHandle nh_;
        ros::Publisher sub_path_pub_;
        ros::Publisher goal_pub_;
        ros::Subscriber path_subscribe_;
        ros::Subscriber result_subscribe_;
    };

}


#endif //PROJECT_LOOP_PATH_PLANNER_H
