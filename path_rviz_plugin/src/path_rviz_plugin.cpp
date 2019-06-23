#include <tf/transform_datatypes.h>

#include <std_srvs/Empty.h>
#include <actionlib_msgs/GoalID.h>
#include <path_server/SetPathName.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_rviz_plugin/path_rviz_plugin.h>


namespace autolabor_plugin {

    PathRvizPlugin::PathRvizPlugin(QWidget *parent) : Panel(parent) {
        auto *button_layout = new QHBoxLayout;
        start_record_button_ = new QPushButton(tr("录制路径"), this);
        button_layout->addWidget(start_record_button_);
        stop_record_button_ = new QPushButton(tr("保存路径"), this);
        button_layout->addWidget(stop_record_button_);

        start_task_button_ = new QPushButton(tr("开始任务"), this);
        button_layout->addWidget(start_task_button_);
        stop_task_button_ = new QPushButton(tr("取消任务"), this);
        button_layout->addWidget(stop_task_button_);
        setLayout(button_layout);

        stop_record_button_->setEnabled(false);

        connect(start_record_button_, SIGNAL(clicked()), this, SLOT(start_record_callback()));
        connect(stop_record_button_, SIGNAL(clicked()), this, SLOT(stop_record_callback()));
        connect(start_task_button_, SIGNAL(clicked()), this, SLOT(start_task_callback()));
        connect(stop_task_button_, SIGNAL(clicked()), this, SLOT(stop_task_callback()));

        start_record_client_ = nh_.serviceClient<path_server::SetPathName>("start_record_path");
        stop_record_client_ = nh_.serviceClient<std_srvs::Empty>("stop_record_path");

        start_task_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        cancel_task_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    }

    void PathRvizPlugin::load(const rviz::Config &config) {
        Panel::load(config);
    }

    void PathRvizPlugin::save(rviz::Config config) const {
        Panel::save(config);
    }

    void PathRvizPlugin::start_record_callback() {
        path_server::SetPathName msg;
        msg.request.path_name = "default_path";
        if (!start_record_client_.call(msg)) {
            ROS_ERROR_STREAM("记录路径发生错误，请重试！");
        } else {
            stop_record_button_->setEnabled(true);
            start_task_button_->setEnabled(false);
        }
    }

    void PathRvizPlugin::stop_record_callback() {
        std_srvs::Empty msg;
        if (!stop_record_client_.call(msg)) {
            ROS_ERROR_STREAM("保存路径发生错误，请重试！");
        } else {
            start_task_button_->setEnabled(true);
        }
    }

    void PathRvizPlugin::start_task_callback() {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(generator_.uniformReal(-M_PI, M_PI));
        msg.pose.position.x = generator_.uniformReal(-100, 100);
        msg.pose.position.y = generator_.uniformReal(-100, 100);
        msg.pose.position.z = generator_.uniformReal(-100, 100);
        start_task_pub_.publish(msg);
    }

    void PathRvizPlugin::stop_task_callback() {
        actionlib_msgs::GoalID msg;
        msg.stamp = ros::Time::now();
        cancel_task_pub_.publish(msg);
    }
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(autolabor_plugin::PathRvizPlugin, rviz::Panel)