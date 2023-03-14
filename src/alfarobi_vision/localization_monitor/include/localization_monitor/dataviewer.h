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

#include "vision_utils/Particles.h"
#include "vision_utils/Features.h"

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
//    v9_localization::MonitorUtils monitor_utils_;
//    void monitorUtilsCallback(const v9_localization::MonitorUtilsConstPtr &_msg);

    message_filters::Subscriber<geometry_msgs::PoseStamped> robot_state_sub_;
    message_filters::Subscriber<vision_utils::Particles > particles_state_sub_;
    message_filters::Subscriber<vision_utils::Features > features_sub_;
    message_filters::Subscriber<geometry_msgs::PointStamped > projected_ball_sub_;
    message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, vision_utils::Particles,
                                    vision_utils::Features, geometry_msgs::PointStamped > input_sync_;
    void inputUtilsCallback(const geometry_msgs::PoseStampedConstPtr &_msg1,
                            const vision_utils::ParticlesConstPtr &_msg2,
                            const vision_utils::FeaturesConstPtr &_msg3,
                            const geometry_msgs::PointStampedConstPtr &_msg4);

    geometry_msgs::Pose2D robot_state_;
    vision_utils::Particles particles_state_;
    vision_utils::Features features_;
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

