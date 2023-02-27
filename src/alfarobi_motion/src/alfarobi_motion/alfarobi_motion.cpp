#include "alfarobi_motion/alfarobi_motion.h"

Motion::Motion() {
    serv = new alfarobi::ServoController();
}

void Motion::write(alfarobi::joint_value joints_) {
    for(int i=0; i<20; i++) {
        if(joints_.write[i]) {
            serv->write(i+1, joints_.val[i] , joints_.target_time[i]);
        }
    }
    
}

void Motion::read(alfarobi::joint_value joints_) {
    for(int i=0; i<20; i++) {
        if(joints_.read[i]) {
            serv->read(i+1);
        }
    }
}