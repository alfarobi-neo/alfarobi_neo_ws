#include "head_control/head_control.h"

HeadControl::HeadControl() {
    ball_pos_sub = nh_.subscribe("/v9_ball_detector_node/ball_pos", 1000, &HeadControl::ballPosCallback, this);

    center_frame_.x = 320;
    center_frame_.y = 240;

    temp_servo = new alfarobi::ServoController();

    ball_found = false;

    p_gain = 0.5;
    d_gain = 0.005;

    H_FOV = 61.25; //61.25
    V_FOV = 41.60; //41.60

    HOLD_THRESHOLD = 10;
    temp_servo->initialize();
}

HeadControl::~HeadControl() {
    // delete temp_servo;
    temp_servo->dispose();
}

void HeadControl::process(alfarobi::ServoController **serv) {
    // if(state_now == "head_control") {

    //     in_action = true;
    // }
    ros::Rate loop_rate(5);
    while(ros::ok()) {
        if(!in_action){
        // enable();
            initialHead();
        // temp_servo = *serv;
            in_action = true;
        } 
        else if(in_action) {
            calculation();
        }

        loop_rate.sleep();
        ros::spinOnce();
        ROS_INFO("BALL TRACKER");
    }
    

    
}

void HeadControl::initialHead() {
    YAML::Node init_pose_params;
    try{
        init_pose_params = YAML::LoadFile(ros::package::getPath("alfarobi_motion") + "/config/initial_position.yaml");
    }catch(const std::exception &e){
        ROS_ERROR("[alfarobi_motion] Unable to load params file: %s", e.what());
    }
    
    try{
        present_position[18] = init_pose_params["head_pan"].as<double>();
        present_position[19] = init_pose_params["head_tilt"].as<double>();

        absolute_position[18] = init_pose_params["head_pan"].as<double>();
        absolute_position[19] = init_pose_params["head_tilt"].as<double>();
    }catch(const std::exception &e){
        ROS_ERROR("[alfarobi_motion]: %s", e.what());
    }

    ROS_INFO("head_pan: %f", present_position[18]);
    ROS_INFO("head_tilt: %f", present_position[19]);
    write();
}

void HeadControl::ballPosCallback(const geometry_msgs::Point::ConstPtr& msg) {
    if(msg->x != -1 && msg->y != -1){
        ball_pos_.x = msg->x;
        ball_pos_.y = msg->y;
        ball_pos_.z = msg->z;
        ball_found = true;
        ROS_INFO("BAGUS KONT");
    }else{
        ball_found = false;
        ROS_INFO("BAGUS KONT");
    }
}

void HeadControl::write() {
    if(!is_moving) {
        time_start = ros::Time::now().toSec();
        is_moving = true;
        for(uint8_t i=18; i<20; i++) {
            temp_servo->write(i+1, temp_servo->deg2Bit(present_position[i]) , 500); //5 detik
            
            ROS_INFO("WRITING");
        }
        // write(tempSeq->getJoint());
        // readAll();
    }
    // read(20);
    // ROS_INFO("JOINT VALUE: %f", present_position[19]);
    // temp_servo->write(20, temp_servo->deg2Bit(present_position[19]) , 3000); 
    // ROS_INFO("AAAAAAAa");
    time_now = ros::Time::now().toSec() - time_start;
    ROS_INFO("Time now: %f", time_now);
    if(time_now >= 500/1000) {
        is_moving = false;
        // tempSeq = tempSeq->next;
        // for(int i=0; i<20; i++) {
        //     tempSeq->getJoint()->write[i] = false;
        // }
    }
}

void HeadControl::calculation() {
    static int lost_count = 0;

    ROS_INFO("CALCULATION");

    if(!ball_found){
        present_position[18] = absolute_position[18];
        present_position[19] = absolute_position[19];

        if(lost_count < HOLD_THRESHOLD){
            lost_count++;
        }else{
            // std_msgs::String scan_msg;
            // scan_msg.data = "scan";
            // head_scan_pub_.publish(scan_msg);
        }
        return;
    }

    lost_count = 0;

    static double last_error_x = 0,last_error_y = 0;

    double error_x = center_frame_.x - ball_pos_.x;
    double error_y = center_frame_.y - ball_pos_.y;

    double offset_x = p_gain*error_x + d_gain*(error_x - last_error_x);
    double offset_y = p_gain*error_y + d_gain*(error_y - last_error_y);

    last_error_x = error_x;
    last_error_y = error_y;

    // setHeadJoint(offset_x * (H_FOV)/(2*center_frame_.x),offset_y * (V_FOV)/(2*center_frame_.y));
    present_position[18] = absolute_position[18] + (offset_x * (H_FOV)/(2*center_frame_.x));
    present_position[19] = absolute_position[19] + (offset_y * (V_FOV)/(2*center_frame_.y));

    
    ROS_INFO("head_pan: %f", present_position[18]);
    ROS_INFO("head_tilt: %f",present_position[19]);
    ball_found = false;

    write();
}