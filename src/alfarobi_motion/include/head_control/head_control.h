#ifndef HEAD_CONTROL_H
#define HEAD_CONTROL_H

#include <geometry_msgs/Point.h>
#include "alfarobi_dxlsdk/servo_controller.h"
#include "alfarobi_dxlsdk/joint_value.h"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <std_msgs/String.h>

class HeadControl {
private:
    alfarobi::ServoController *temp_servo;
    ros::Subscriber ball_pos_sub;
    ros::Publisher head_scan_pub_;
    ros::NodeHandle nh_;
    double present_position[20];
    double absolute_position[20];
    double previous_position[20];
    geometry_msgs::Point ball_pos_;
    geometry_msgs::Point center_frame_;

    double p_gain, d_gain, i_gain;
    double H_FOV, V_FOV;
    int HOLD_THRESHOLD;
    int scan_state;

    bool in_action = false;
    bool ball_found;
    double time_start, time_now;
    bool is_moving = false;
    std::string comm = "";
    std::string state_now;

    const double LEFT = 40.0;
    const double RIGHT = -40.0;
    const double MIDDLE = 0.0;
    const double UP = -30.0;
    const double DOWN = 40.0;

    enum {
        NOD = 0,
        SWEEP = 1,
        SQUARE = 2,
        NO_SCAN = 3
    };

    int nod_steps;
    int sweep_steps;
    int square_steps;

public:
    HeadControl();
    ~HeadControl();

    void goInitPose();
    void process(alfarobi::ServoController **serv);


    void calculation();

    void ballPosCallback(const geometry_msgs::Point::ConstPtr& msg);
    void ballSearchCallback(const std_msgs::String::ConstPtr& msg);
    void searching();

    void write();
    void write(int time);
    void initialHead();
    
};

#endif