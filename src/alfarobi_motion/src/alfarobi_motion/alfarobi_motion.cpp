#include "alfarobi_motion/alfarobi_motion.h"

Motion::Motion() {
    serv = new alfarobi::ServoController();
    serv->torqueEnable();
}

Motion::~Motion() {
    delete serv;
}
void Motion::write(alfarobi::joint_value* joints_) {
    for(int i=0; i<20; i++) {
        if(joints_->write[i]) {
            serv->write(i+1, joints_->val[i] , joints_->target_time[i]);
            std::cout<<"Writing\n";
        }
    }
    
}

void Motion::read(alfarobi::joint_value* joints_) {
    for(int i=0; i<20; i++) {
        if(joints_->read[i]) {
            serv->read(i+1);
        }
    }
}

void Motion::disable(int id) {
    serv->torqueDisableID(id);
}

void Motion::enable(int id) {
    serv->torqueEnableID(id);
}