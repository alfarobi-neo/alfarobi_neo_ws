#ifndef HEAD_CONTROL_H
#define HEAD_CONTROL_H

#include <geometry_msgs/Point.h>
#include "alfarobi_dxlsdk/servo_controller.h"
#include "alfarobi_dxlsdk/joint_value.h"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

class HeadControl {
private:
    alfarobi::ServoController *temp_servo;
    ros::Subscriber ball_pos_sub;
    ros::NodeHandle nh_;
    double present_position[20];
    double absolute_position[20];
    double previous_position[20];
    geometry_msgs::Point ball_pos_;
    geometry_msgs::Point center_frame_;

    double p_gain, d_gain, i_gain;
    double H_FOV, V_FOV;
    int HOLD_THRESHOLD;

    bool in_action = false;
    bool ball_found;
    double time_start, time_now;
    bool is_moving = false;
    std::string comm = "";
    std::string state_now;

public:
    HeadControl();
    ~HeadControl();

    void goInitPose();
    void process(alfarobi::ServoController **serv);


    void calculation();

    void ballPosCallback(const geometry_msgs::Point::ConstPtr& msg);

    void write();
    void initialHead();
    
};

#endif