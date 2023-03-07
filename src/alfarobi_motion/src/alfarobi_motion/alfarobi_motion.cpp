//----------UNCOMMAND KALO UDAH TESTING---------//

// // #include "alfarobi_motion/alfarobi_motion.h"
// #include "sequencer/sequencer.h"
// #include "initial_position/initial_position.h"
// #include "ros/ros.h"

// bool
//     initial_position = false,
//     sequencer = false,
//     kicking = false,
//     walking = false;

// void motionCallback(const std_msgs::String::ConstPtr &msg) {
//     if(msg->data == "initial_position") {
//         initial_position = true;
//     }
//     else if(msg->data == "sequencer") {
//         sequencer = true;
//     }
//     else if(msg->data == "kicking") {
//         kicking = true;
//     }
//     else if(msg->data == "walking") {
//         walking = true;
//     }
// }

// int main(int argc, char** argv) {
//     ros::NodeHandle nh_m;
    
//     ros::init(argc, argv, "alfarobi_motion_node");

//     ros::Rate loop_rate(5);
//     ros::Subscriber motion_sub = nh_m.subscribe("/motion_state", 1000, motionCallback);

//     Sequencer *sequencer_temp = new Sequencer();

    

//     while(ros::ok()) {
        
//         if(sequencer) {
//             sequencer_temp->process();
//             // in_action = true;
//         }
//         else if(initial_position) {
//             //belum ada
//         }
//         else if(kicking) {
//             //belum ada
//         }
//         else if(walking) {
//             //belum ada
//         }

//         loop_rate.sleep();
//         ros::spin();
//     }

//     return 0;
// }