#ifndef PROJECT_LOAD_PATH_NODE_H
#define PROJECT_LOAD_PATH_NODE_H

#include <string>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Path.h>
#include "path_server/SetPathName.h"

namespace autolabor_tools {
    class LoadPath {
    public:
        LoadPath();

        ~LoadPath();

        void run();

    private:
        bool load_path(std::string path);

        bool load_path_callback(path_server::SetPathName::Request &req, path_server::SetPathName::Response &res);

    private:
        ros::NodeHandle nh_;
        tf::TransformListener tf_;

        std::string map_frame_, path_file_;
        ros::Publisher full_path_pub_;
        nav_msgs::Path record_path_;
        ros::ServiceServer load_path_server_;
    };
}


#endif //PROJECT_LOAD_PATH_NODE_H
