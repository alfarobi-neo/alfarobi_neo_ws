#include "initial_position/initial_position.h"

InitialPosition::InitialPosition(){
    // temp_motion = new Motion();
    temp_servo = new alfarobi::ServoController();
}

InitialPosition::~InitialPosition() {
    // delete temp_motion;
    delete temp_servo;
}

void InitialPosition::process() {
    // if(!in_action) {
    //     // torque_sub = nh_.subscribe("/torqueState", 1000, &Sequencer::webButtonCallback, this);
    //     // motion_state_sub = nh_.subscribe("/motion_state", 1000, &Sequencer::motionStateCallback, this);

    //     // loadSequences();
    // }
    
    // if(state_now == "initial_pose") {

    //     in_action = true;
    //     // int input = 0;
    //     // std::cin>>input;
    //     // if(input == 999) {
    //     //     return;
    //     // }
    //     // loadParams("JATUH_DEPAN");

    //     // loop_rate.sleep();
    //     // ros::spin();
    // }
}