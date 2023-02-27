#ifndef ALFAROBI_MOTION_H
#define ALFAROBI_MOTION_H

#include <initializer_list>
#include <ros/ros.h>
#include "alfarobi_dxlsdk/servo_controller.h"

class Motion{
private:
    alfarobi::ServoController *serv;
public:
    Motion();
    enum status{
        INITIAL_POSE = 0,
        SEQUENCER = 1,
        KICKING = 2,
        WALKING = 3,
    };

    bool    
        init_pose = false,
        sequencer = false,
        kicking = false,
        walking = false;

   
    void write(alfarobi::joint_value joints_);
    void read(alfarobi::joint_value joints_);

};

#endif