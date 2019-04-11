#ifndef PROJECT_RECORD_PATH_NODE_H
#define PROJECT_RECORD_PATH_NODE_H

#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <fstream>

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <path_server/SetPathName.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace autolabor_tools {

    class RecordPath {

    public:
        RecordPath();

        ~RecordPath();

        void run();

    private:
        void record_callback(const nav_msgs::Odometry::ConstPtr &msg);

        bool start_record(path_server::SetPathName::Request &req, path_server::SetPathName::Response &res);

        bool stop_record(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        void record_data(double x, double y, double yaw);

        inline std::string get_time_str() {
            char ch[64];
            time_t t = time(nullptr);
            strftime(ch, sizeof(ch), "%Y_%m_%d_%H_%M_%S", localtime(&t));
            return std::string(ch);
        }

        inline double norm2(double x, double y, double yaw) {
            return x * x + y * y + yaw * yaw;
        }

    private:
        bool record_flag_, first_record_;

        std::string map_frame_, base_link_frame_;
        std::string odom_topic_;
        double distance_interval_;

        std::ofstream output_file_;

        tf::TransformListener tf_;
        double cache_x_, cache_y_, cache_yaw_;
        nav_msgs::Path path_data_;

        ros::NodeHandle nh_;
        ros::Publisher path_pub_;
        ros::Subscriber odom_subscribe_;
        ros::ServiceServer start_record_server_, stop_record_server_;
    };

}

#endif //PROJECT_RECORD_PATH_NODE_H
