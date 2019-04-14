#ifndef PROJECT_RECORD_PATH_PLANNER_H
#define PROJECT_RECORD_PATH_PLANNER_H

#include <string>
#include <limits>
#include <fstream>
#include <stdlib.h>

#include <ros/ros.h>

#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>

namespace autolabor_algorithm {

    class RecordPathPlanner : public nav_core::BaseGlobalPlanner  {
    public:
        RecordPathPlanner();

        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);

    private:
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

    private:
        bool initialized_;
        std::string file_path_, map_frame_;
        nav_msgs::Path record_path_;

        ros::Publisher full_path_pub_;
        ros::Publisher sub_path_pub_;

    };

}

#endif //PROJECT_RECORD_PATH_PLANNER_H
