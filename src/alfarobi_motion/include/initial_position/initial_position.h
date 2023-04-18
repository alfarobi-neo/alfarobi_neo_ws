#ifndef INITIAL_POSITION_H
#define INITIAL_POSITION_H

#include "alfarobi_web_gui/Torque.h"
#include "alfarobi_web_gui/Sequencer.h"
#include "alfarobi_dxlsdk/servo_controller.h"
#include "alfarobi_dxlsdk/joint_value.h"
#include "std_msgs/String.h"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

class InitialPosition {
private:
    alfarobi::ServoController *temp_servo;
    std::string current_seq_name;
    ros::NodeHandle nh_;
    double present_position[20];

    ros::Publisher joint_pub;
    ros::Subscriber joint_sub;
    ros::Subscriber torque_sub;
    ros::Subscriber button_sub;
    ros::Subscriber motion_state_sub;

    bool in_action = false;
    double time_start, time_now;
    bool is_moving = false;
    std::string comm = "";
    std::string state_now;

public:
    InitialPosition();
    ~InitialPosition();

    void loadParams();
    void goInitPose();
    void saveParams();
    void process(alfarobi::ServoController *serv);

    void applyCallback(const alfarobi_web_gui::Sequencer::ConstPtr& web_joint);
    void torqueCallback(const alfarobi_web_gui::Torque::ConstPtr& torque);
    void motionStateCallback(const std_msgs::String::ConstPtr& msg);
    void webButtonCallback(const std_msgs::String::ConstPtr& msg);

    void write();
    void refresh();
    void readAll();
    void read(int id);
    void enable(int id);
    void disable(int id);
};

#endif