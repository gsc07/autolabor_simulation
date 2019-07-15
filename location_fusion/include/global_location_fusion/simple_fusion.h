#ifndef PROJECT_SIMPLE_FUSION_H
#define PROJECT_SIMPLE_FUSION_H

#include <deque>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>


namespace autolabor_algorithm {

    class SimpleFusion {
    public:
        SimpleFusion();

        ~SimpleFusion() = default;

        void run();

    private:
        void position_callback(const geometry_msgs::PointStamped::ConstPtr &msg);

        void point_match();

        void publish_tf(const ros::TimerEvent &);

        inline void limit_deque(std::deque<Eigen::Vector2d> &deque_data) {
            while (deque_data.size() > buffer_size_) {
                deque_data.pop_front();
            }
        }

    private:
        std::string map_frame_, odom_frame_, base_link_frame_;

        tf::TransformListener tf_;
        tf::TransformBroadcaster tf_broadcaster_;

        ros::NodeHandle nh_;
        ros::Subscriber position_subscribe_;
        ros::Timer tf_timer_;

        bool start_flag_;
        int rate_;
        double distance_interval_;

        int buffer_size_;
        std::deque<Eigen::Vector2d> map_points_;
        std::deque<Eigen::Vector2d> odom_points_;

        tf::Transform map_to_odom_transform_;

        Eigen::Matrix3d rotation_;
        Eigen::Vector3d trans_;
        Eigen::Vector3d org_trans_;
    };

}


#endif //PROJECT_SIMPLE_FUSION_H
