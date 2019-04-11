#ifndef PROJECT_SIMULATION_LOCATION_NODE_H
#define PROJECT_SIMULATION_LOCATION_NODE_H

#include <string>
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

namespace autolabor_simulation {

    class SimulationLocation {

    public:
        SimulationLocation();

        ~SimulationLocation() = default;

        void run();

    private:
        void pubLocationCallback(const ros::TimerEvent &event);

    private:
        std::string baselink_frame_, real_map_frame_, location_frame_;
        double location_to_real_map_x_, location_to_real_map_y_, location_to_real_map_yaw_;
        tf::Transform location_to_realmap_;
        int rate_;

        tf::TransformListener tf_;
        ros::NodeHandle nh_;

        ros::Publisher location_pub_;
        ros::Timer pub_location_timer_;
    };
}


#endif //PROJECT_SIMULATION_LOCATION_NODE_H
