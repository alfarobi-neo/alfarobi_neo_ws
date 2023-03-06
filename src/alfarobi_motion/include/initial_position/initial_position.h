#ifndef INITIAL_POSITION_H
#define INITIAL_POSITION_H

#include "alfarobi_web_gui/Torque.h"
#include "alfarobi_dxlsdk/servo_controller.h"
#include "std_msgs/String.h"

class InitialPosition {
private:
    alfarobi::ServoController *temp_servo;
    std::string current_seq_name;
    ros::NodeHandle nh_;

    ros::Subscriber torque_sub;
    ros::Subscriber motion_state_sub;

    bool in_action = false;

public:
    InitialPosition();
    ~InitialPosition();

    void loadParams();
    void goInitPose();
    void saveParams();
    void process();
    void torqueCallback(const alfarobi_web_gui::Torque::ConstPtr& torque);
    void motionStateCallback(const std_msgs::String::ConstPtr& msg);

    void write(alfarobi::joint_value *joints_);
    void read(alfarobi::joint_value *joints_);
    void enable(int id);
    void disable(int id);
};

#endif