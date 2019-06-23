#ifndef PROJECT_PATH_RVIZ_PLUGIN_H
#define PROJECT_PATH_RVIZ_PLUGIN_H

# include <ros/ros.h>
# include <rviz/panel.h>

#include <QHBoxLayout>
#include <QPushButton>
#include <QString>

#include <random_numbers/random_numbers.h>

namespace autolabor_plugin {
    class PathRvizPlugin : public rviz::Panel {
    Q_OBJECT
    public:
        PathRvizPlugin(QWidget *parent = 0);

        virtual void load(const rviz::Config &config);

        virtual void save(rviz::Config config) const;

    protected Q_SLOTS:

        void start_record_callback();

        void stop_record_callback();

        void start_task_callback();

        void stop_task_callback();

    private:
        random_numbers::RandomNumberGenerator generator_;
        ros::NodeHandle nh_;
        ros::ServiceClient start_record_client_, stop_record_client_;
        ros::Publisher start_task_pub_, cancel_task_pub_;

        QPushButton *start_record_button_;
        QPushButton *stop_record_button_;
        QPushButton *start_task_button_;
        QPushButton *stop_task_button_;
    };
}


#endif //PROJECT_PATH_RVIZ_PLUGIN_H
