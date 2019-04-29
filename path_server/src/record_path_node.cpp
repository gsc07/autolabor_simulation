#include <record_path/record_path_node.h>

namespace autolabor_tools {


    RecordPath::RecordPath() : record_flag_(false), first_record_(true) {

    }

    RecordPath::~RecordPath() {
        output_file_.close();
    }


    void RecordPath::record_callback(const nav_msgs::Odometry::ConstPtr &msg) {
        if (record_flag_ && tf_.canTransform(map_frame_, base_link_frame_, ros::Time())) {
            tf::StampedTransform map_to_baselink;
            tf_.lookupTransform(map_frame_, base_link_frame_, ros::Time(), map_to_baselink);
            double x = map_to_baselink.getOrigin().getX();
            double y = map_to_baselink.getOrigin().getY();
            double yaw = tf::getYaw(map_to_baselink.getRotation());

            if (first_record_) {
                record_data(x, y, yaw);
                first_record_ = false;
            } else if (norm2(cache_x_ - x, cache_y_ - y, 0) > (distance_interval_ * distance_interval_)) {
                record_data(x, y, yaw);
            }
        }
    }

    void RecordPath::record_data(double x, double y, double yaw) {
        if (output_file_) {
            output_file_ << x << " " << y << " " << yaw << std::endl;
        }

        cache_x_ = x;
        cache_y_ = y;
        cache_yaw_ = yaw;

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = map_frame_;
        poseStamped.pose.position.x = x;
        poseStamped.pose.position.y = y;
        poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        path_data_.poses.push_back(poseStamped);
        path_pub_.publish(path_data_);
    }

    void RecordPath::run() {
        ros::NodeHandle private_node("~");

        private_node.param<std::string>("map_frame", map_frame_, "map");
        private_node.param<std::string>("base_link_frame", base_link_frame_, "base_link");
        private_node.param<std::string>("odom_topic", odom_topic_, "odom");

        private_node.param<double>("distance_interval", distance_interval_, 0.05);

        path_pub_ = nh_.advertise<nav_msgs::Path>("recorded_path", 1, true);
        odom_subscribe_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 10, &RecordPath::record_callback, this);
        start_record_server_ = nh_.advertiseService("start_record_path", &RecordPath::start_record, this);
        stop_record_server_ = nh_.advertiseService("stop_record_path", &RecordPath::stop_record, this);
        ros::spin();
    }

    bool RecordPath::start_record(path_server::SetPathName::Request &req, path_server::SetPathName::Response &res) {
        std::string filename = req.path_name;
        if (filename.empty()) {
            filename = get_time_str();
        }
        std::string path = ros::package::getPath("path_server") + "/path_data/";
        if (access(path.c_str(), 0) == -1) {
            mkdir(path.c_str(), 0777);
        }
        output_file_.close();
        output_file_ = std::ofstream(path + filename + ".path");

        path_data_.header.stamp = ros::Time::now();
        path_data_.header.frame_id = map_frame_;
        path_data_.poses.clear();

        record_flag_ = true;
        first_record_ = true;
        return true;
    }

    bool RecordPath::stop_record(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        if (record_flag_ && tf_.canTransform(map_frame_, base_link_frame_, ros::Time())) {
            tf::StampedTransform map_to_baselink;
            tf_.lookupTransform(map_frame_, base_link_frame_, ros::Time(), map_to_baselink);
            record_data(map_to_baselink.getOrigin().getX(), map_to_baselink.getOrigin().getY(), tf::getYaw(map_to_baselink.getRotation()));
        }
        output_file_.close();
        record_flag_ = false;
        return true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "record_path");
    autolabor_tools::RecordPath recordPath;
    recordPath.run();
    return 0;
}