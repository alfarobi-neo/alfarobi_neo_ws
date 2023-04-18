#pragma once

#include <QtCore>
#include <QtGui>
#include <QtWidgets>

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "alfarobi_msgs_srvs_actions/Particles.h"
#include "alfarobi_msgs_srvs_actions/Features.h"

#include "v9_localization/v9_localization.h"

#include "localization_monitor/dataprocess.h"

#include <sstream>

class DataViewer:public QGraphicsView{
    Q_OBJECT
private:
    QGraphicsScene *data_scene_;

    DataProcess *data_process_;

    QImage field_background_;

    ros::NodeHandle nh_;

//    ros::Subscriber monitor_utils_sub_;
//    alfarobi_msgs_srvs_actions::MonitorUtils monitor_utils_;
//    void monitorUtilsCallback(const alfarobi_msgs_srvs_actions::MonitorUtilsConstPtr &_msg);

    message_filters::Subscriber<geometry_msgs::PoseStamped> robot_state_sub_;
    message_filters::Subscriber<alfarobi_msgs_srvs_actions::Particles > particles_state_sub_;
    message_filters::Subscriber<alfarobi_msgs_srvs_actions::Features > features_sub_;
    message_filters::Subscriber<geometry_msgs::PointStamped > projected_ball_sub_;
    message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, alfarobi_msgs_srvs_actions::Particles,
                                    alfarobi_msgs_srvs_actions::Features, geometry_msgs::PointStamped > input_sync_;
    void inputUtilsCallback(const geometry_msgs::PoseStampedConstPtr &_msg1,
                            const alfarobi_msgs_srvs_actions::ParticlesConstPtr &_msg2,
                            const alfarobi_msgs_srvs_actions::FeaturesConstPtr &_msg3,
                            const geometry_msgs::PointStampedConstPtr &_msg4);

    geometry_msgs::Pose2D robot_state_;
    alfarobi_msgs_srvs_actions::Particles particles_state_;
    alfarobi_msgs_srvs_actions::Features features_;
    geometry_msgs::Point projected_ball_;

    void drawRobot();
    void drawParticles();
    void drawProjectedLines();
    void drawProjectedBall();
    void drawGrid();

public:
    DataViewer(QObject *parent=0);
    ~DataViewer();

    void processBegin();
    void updateData();
    bool show_grid_;
    bool show_particles_;
signals:
    void updateViewer();

};

