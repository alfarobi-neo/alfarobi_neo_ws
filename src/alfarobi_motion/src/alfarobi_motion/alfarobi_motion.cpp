// ----------UNCOMMAND KALO UDAH TESTING---------//

// #include "alfarobi_motion/alfarobi_motion.h"
#include "sequencer/sequencer.h"
#include "initial_position/initial_position.h"
#include "ros/ros.h"

bool
    initial_position = false, 
    sequencer = false,
    kicking = false,
    walking = false;

void motionCallback(const std_msgs::String::ConstPtr &msg) {
    //buat testing doang di true in
    if(msg->data == "initial_position") {
        initial_position = true;
        sequencer = false;
        kicking = false;
        walking = false;
    }
    else if(msg->data == "sequencer") {
        initial_position = false;
        sequencer = true;
        kicking = false;
        walking = false;
    }
    else if(msg->data == "kicking") {
        initial_position = false;
        sequencer = false;
        kicking = true;
        walking = false;
    }
    else if(msg->data == "walking") {
        initial_position = false;
        sequencer = false;
        kicking = false;
        walking = true;
    }
    // std::cout<<"NOW: "<<msg->data;
    ROS_INFO("TUNING STATE NOW: %s", msg->data.c_str());
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "alfarobi_motion_node");

    bool init_process = false;

    ros::NodeHandle nh_m;

    ros::Rate loop_rate(5);
    ros::Subscriber motion_sub = nh_m.subscribe("/motion_state", 1000, motionCallback);

    alfarobi::ServoController *servo_temp = new alfarobi::ServoController();

    Sequencer *sequencer_temp = new Sequencer();
    InitialPosition *init_pose_temp = new InitialPosition();

    
    servo_temp->initialize();

    servo_temp->torqueEnable();

    while(ros::ok()) {
        
        if(sequencer) {
            sequencer_temp->process();
            // in_action = true;
        }
        else if(initial_position) {
            init_pose_temp->process();
        }
        else if(kicking) {
            //belum ada
        }
        else if(walking) {
            //belum ada
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    // servo_temp->write(20, 30, 4000);

    // double time_start, time_now;
    // bool moving = false, finish = false;

    // while(!finish){
    //     if(!moving) {
    //         time_start = ros::Time::now().toSec();
    //         moving = true;
    //         // init_pose_temp->write();
    //         servo_temp->write(20, servo_temp->deg2Bit(200), 5000);
    //     }
    //     time_now = ros::Time::now().toSec() - time_start;
    //     ROS_INFO("TIME_NOW: %f", time_now);
    //     if(time_now >= 5000/1000) {
    //         finish = true;
    //     }

        
    // }

    

    delete sequencer_temp, init_pose_temp;
    servo_temp->torqueDisable();
    servo_temp->dispose();

    return 0;
}