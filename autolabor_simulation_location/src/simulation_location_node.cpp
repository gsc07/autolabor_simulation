

#include <autolabor_simulation_location/simulation_location_node.h>


namespace autolabor_simulation {

    SimulationLocation::SimulationLocation() {
        ros::NodeHandle private_node("~");
        private_node.param<std::string>("baselink_frame", baselink_frame_, "base_link");
        private_node.param<std::string>("real_map_frame", real_map_frame_, "real_map");
        private_node.param<std::string>("location_frame", location_frame_, "map");
        private_node.param<int>("rate", rate_, 10);

        //double location_to_real_map_x_, location_to_real_map_y_, location_to_real_map_yaw_;
        private_node.param<double>("location_to_real_map_x", location_to_real_map_x_, 0.0);
        private_node.param<double>("location_to_real_map_y", location_to_real_map_y_, 0.0);
        private_node.param<double>("location_to_real_map_yaw", location_to_real_map_yaw_, 0.0);
        location_to_realmap_.setRotation(tf::createQuaternionFromYaw(location_to_real_map_yaw_));
        location_to_realmap_.setOrigin(tf::Vector3(location_to_real_map_x_, location_to_real_map_y_, 0));
    }

    void SimulationLocation::pubLocationCallback(const ros::TimerEvent &event) {
        tf::StampedTransform realmap_to_baselink;
        if (tf_.canTransform(real_map_frame_, baselink_frame_, ros::Time())) {
            tf_.lookupTransform(real_map_frame_, baselink_frame_, ros::Time(), realmap_to_baselink);

            tf::Transform location_to_baselink = location_to_realmap_ * realmap_to_baselink;

            geometry_msgs::PointStamped location_to_baselink_msg;
            location_to_baselink_msg.header.stamp = ros::Time::now();
            location_to_baselink_msg.header.frame_id = location_frame_;

            location_to_baselink_msg.point.x = location_to_baselink.getOrigin().x();
            location_to_baselink_msg.point.y = location_to_baselink.getOrigin().y();
            location_to_baselink_msg.point.z = location_to_baselink.getOrigin().z();

            location_pub_.publish(location_to_baselink_msg);
        }
    }

    void SimulationLocation::run() {
        location_pub_ = nh_.advertise<geometry_msgs::PointStamped>("location_pos", 100);
        pub_location_timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &SimulationLocation::pubLocationCallback, this);
        ros::spin();
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simulation_location_node");
    autolabor_simulation::SimulationLocation simulation_location;
    simulation_location.run();
    return 0;
}