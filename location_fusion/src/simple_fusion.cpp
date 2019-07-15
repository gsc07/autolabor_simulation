
#include <global_location_fusion/simple_fusion.h>


namespace autolabor_algorithm {

    SimpleFusion::SimpleFusion() : start_flag_(true) {
        map_to_odom_transform_ = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0));
    }

    void SimpleFusion::point_match() {
        // center of mass
        Eigen::Vector2d p1(0, 0), p2(0, 0);
        size_t point_size = map_points_.size();
        for (size_t i = 0; i < point_size; i++) {
            p1 += map_points_[i];
            p2 += odom_points_[i];
        }
        p1 = Eigen::Vector2d(p1 / point_size);
        p2 = Eigen::Vector2d(p2 / point_size);

        // remove the center
        std::vector<Eigen::Vector2d> q1(point_size), q2(point_size);
        for (size_t i = 0; i < point_size; i++) {
            q1[i] = map_points_[i] - p1;
            q2[i] = odom_points_[i] - p2;
        }

        // compute q1*q2^T
        Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < point_size; i++) {
            W += Eigen::Vector3d(q1[i].x(), q1[i].y(), 0) * Eigen::Vector3d(q2[i].x(), q2[i].y(), 0).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();


        rotation_ = U * (V.transpose());


        if (rotation_.determinant() > 0) {
            trans_ = Eigen::Vector3d(p1.x(), p1.y(), 0) - rotation_ * Eigen::Vector3d(p2.x(), p2.y(), 0);
        } else {
            trans_ = org_trans_;
            Eigen::Vector3d diff = Eigen::Vector3d(p1.x() - org_trans_.x(), p1.y() - org_trans_.y(), 0);
            // p2->1  diff->2
            double dot = p2.x() * diff.x() + p2.y() * diff.y();
            double det = p2.x() * diff.y() - p2.y() * diff.x();
            double angle = atan2(det, dot);
            double c = cos(angle);
            double s = sin(angle);
            rotation_ << c, -s, 0,
                s, c, 0,
                0, 0, 1;
        }

        tf::Vector3 map_to_odom_origin;
        tf::vectorEigenToTF(trans_, map_to_odom_origin);
        map_to_odom_transform_.setOrigin(map_to_odom_origin);

        tf::Matrix3x3 map_to_odom_rot;
        tf::matrixEigenToTF(rotation_, map_to_odom_rot);
        map_to_odom_transform_.setBasis(map_to_odom_rot);


//        std::cout << "trans : " << trans_.x() << " " << trans_.y() << " " << std::endl;
//        std::cout << "rot : " << tf::getYaw(map_to_odom_transform_.getRotation()) << " " << std::endl << std::endl;
    }

    void SimpleFusion::position_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
        if (tf_.canTransform(odom_frame_, base_link_frame_, ros::Time())) {
            tf::StampedTransform local_transform;
            tf_.lookupTransform(odom_frame_, base_link_frame_, ros::Time(), local_transform);

            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped.header.stamp = ros::Time::now();
            transform_stamped.header.frame_id = map_frame_;
            transform_stamped.child_frame_id = odom_frame_;

            if (start_flag_) {
                map_points_.emplace_back(msg->point.x, msg->point.y);
                odom_points_.emplace_back(local_transform.getOrigin().x(), local_transform.getOrigin().y());
                org_trans_ << msg->point.x - local_transform.getOrigin().x(), msg->point.y - local_transform.getOrigin().y(), 0;
                start_flag_ = false;
            } else {
                Eigen::Vector2d last_odom = odom_points_.back();
                double delta_x = local_transform.getOrigin().x() - last_odom.x();
                double delta_y = local_transform.getOrigin().y() - last_odom.y();
                if ((delta_x * delta_x + delta_y * delta_y) > (distance_interval_ * distance_interval_)) {
                    map_points_.emplace_back(msg->point.x, msg->point.y);
                    odom_points_.emplace_back(local_transform.getOrigin().x(), local_transform.getOrigin().y());
                    limit_deque(map_points_);
                    limit_deque(odom_points_);
                    point_match();
//                    ROS_INFO_STREAM(msg->point.x << " " << msg->point.y << " "
//                                                 << local_transform.getOrigin().x() << " " << local_transform.getOrigin().y() << " "
//                                                 << trans_.x() << " " << trans_.y() << " " << tf::getYaw(map_to_odom_transform_.getRotation()));
                }
            }

        }
    }

    void SimpleFusion::publish_tf(const ros::TimerEvent &) {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = map_frame_;
        transform_stamped.child_frame_id = odom_frame_;
        // todo:判断四元数是否归一化
        tf::transformTFToMsg(map_to_odom_transform_, transform_stamped.transform);
        tf_broadcaster_.sendTransform(transform_stamped);
    }

    void SimpleFusion::run() {
        ros::NodeHandle private_node("~");
        private_node.param<std::string>("map_frame", map_frame_, "map");
        private_node.param<std::string>("odom_frame", odom_frame_, "odom");
        private_node.param<std::string>("base_link_frame", base_link_frame_, "base_link");
        private_node.param<int>("buffer_size", buffer_size_, 50);
        private_node.param<double>("distance_interval", distance_interval_, 0.1);
        private_node.param<int>("rate", rate_, 20);

        position_subscribe_ = nh_.subscribe<geometry_msgs::PointStamped>("location_pos", 10, &SimpleFusion::position_callback, this);
        tf_timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &SimpleFusion::publish_tf, this);
        ros::spin();
    }


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_fusion_node");
    autolabor_algorithm::SimpleFusion fusion;
    fusion.run();
    return 0;
}