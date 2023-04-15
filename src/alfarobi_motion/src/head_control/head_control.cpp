#include "head_control/head_control.h"

HeadControl::HeadControl() {
    // ball_pos_sub = nh_.subscribe("/v9_ball_detector_node/ball_pos", 1000, &HeadControl::ballPosCallback, this);
    ball_pos_sub = nh_.subscribe("/python_image_sub/ball_pos", 1000, &HeadControl::ballPosCallback, this);
    // head_scan_pub_ = nh_.advertise<std_msgs::String>("/head_control/scan_type", 1000);

    center_frame_.x = 320;
    center_frame_.y = 240;

    temp_servo = new alfarobi::ServoController();

    ball_found = false;

    p_gain = 0.07;//0.08; //0.07; //0.075;//0.07 ;//0.7; //0.6
    i_gain = 0.02;//0.0225; //0.02; //0.07;//0.060; //0.040
    d_gain = 0.02;//0.01; //0.02; //0.009;//0.009; //0.050 /0.005

    H_FOV = 61.25; //61.25
    V_FOV = 41.60; //41.60

    HOLD_THRESHOLD = 5; //10
    scan_state = NO_SCAN;
    nod_steps = 0;
    sweep_steps = 0;
    square_steps = 0;
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
    ros::Rate loop_rate(30);
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

        previous_position[18] = init_pose_params["head_pan"].as<double>();
        previous_position[19] = init_pose_params["head_tilt"].as<double>();
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
        ROS_INFO_STREAM("X: "<<ball_pos_.x<<"\nY: "<<ball_pos_.y);
    }else{
        ball_found = false;
        ROS_INFO("NOT OK");
    }
}

void HeadControl::write() {
    if(!is_moving) {
        time_start = ros::Time::now().toSec();
        is_moving = true;
        for(uint8_t i=18; i<20; i++) {
            temp_servo->write(i+1, temp_servo->deg2Bit(present_position[i]) , 200); //5 detik
            
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
    if(time_now >= 200/1000) {
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
        // present_position[18] = absolute_position[18];
        // present_position[19] = absolute_position[19];

        if(lost_count < HOLD_THRESHOLD){
            lost_count++;
        }else{
            // std_msgs::String scan_msg;
            // scan_msg.data = "scan";
            // head_scan_pub_.publish(scan_msg);
            scan_state = SQUARE; //ngetest satu2
            searching();
        }
        return;
    }

    lost_count = 0;

    //baru
    // double i_gain = 0.1; //tambahan

    static double last_error_x = 0,last_error_y = 0;
    //baru
    static double error_x_sum = 0.0;
    static double error_y_sum = 0.0; 

    static double error_x_sub = 0.0;
    static double error_y_sub = 0.0; 
    //

    double error_x = center_frame_.x - ball_pos_.x;
    double error_y = center_frame_.y - ball_pos_.y;

    //baru
    error_x_sum += error_x;
    error_y_sum += error_y;
    
    error_x_sub -= error_x;
    error_y_sub -= error_y;
    //

    // double offset_x = p_gain*error_x + d_gain*(error_x - last_error_x);
    // double offset_y = p_gain*error_y + d_gain*(error_y - last_error_y);

    //baru
    // double offset_x = p_gain*error_x + d_gain*(error_x - last_error_x);
    // double offset_y = p_gain*error_y + d_gain*(error_y - last_error_y);
    //
    double offset_x = p_gain*error_x + i_gain*(error_x - last_error_x) + d_gain*(error_x - last_error_x);
    double offset_y = p_gain*error_y + i_gain*(error_y - last_error_y) + d_gain*(error_y - last_error_y);

    last_error_x = error_x;
    last_error_y = error_y;

    // setHeadJoint(offset_x * (H_FOV)/(2*center_frame_.x),offset_y * (V_FOV)/(2*center_frame_.y));
        
    present_position[18] = previous_position[18] + (offset_x * (H_FOV)/(2*center_frame_.x));
    present_position[19] = previous_position[19] - (offset_y * (V_FOV)/(2*center_frame_.y));

    ROS_INFO("head_pan: %f", present_position[18]);
    ROS_INFO("head_tilt: %f",present_position[19]);
    // ball_found = false;

    write();

    previous_position[18] = present_position[18];
    previous_position[19] = present_position[19];


    // static double last_error_x = 0,last_error_y = 0;

    // double error_x = center_frame_.x - ball_pos_.x;
    // double error_y = center_frame_.y - ball_pos_.y;

    // double offset_x = p_gain*error_x + d_gain*(error_x - last_error_x);
    // double offset_y = p_gain*error_y + d_gain*(error_y - last_error_y);

    // last_error_x = error_x;
    // last_error_y = error_y;

    // // setHeadJoint(offset_x * (H_FOV)/(2*center_frame_.x),offset_y * (V_FOV)/(2*center_frame_.y));
    // present_position[18] = offset_x * (H_FOV)/(2*center_frame_.x);
    // present_position[19] = offset_y * (V_FOV)/(2*center_frame_.y);
    // ball_found = false;

   // write();
}

// void HeadControl::nod(){
//     double nod_angle_up = UP; //-30.0
//     double nod_angle_down = DOWN; //40.0   
//     double time_interval = 0.7;
//     double pan_angle = present_position[18];
//     double tilt_angle = present_position[19];

//     // Nodding down
//     tilt_angle += nod_angle_down;
//     present_position[19] = tilt_angle;

//     // Nodding up
//     tilt_angle += nod_angle_up;
//     present_position[19] = tilt_angle;

//     // Rest position
//     present_position[18] = pan_angle;
//     present_position[19] = tilt_angle;
// }

// void HeadControl::sweep(){
//     double sweep_angle_l = LEFT; //40.0
//     double sweep_angle_r = RIGHT; //-40.0   
//     double time_interval = 0.7;
//     double pan_angle = present_position[18];
//     double tilt_angle = present_position[19];

//     // Sweep to the right
//     pan_angle += sweep_angle_r;
//     present_position[18] = pan_angle;

//     // Sweep to the left
//     pan_angle += sweep_angle_l;
//     present_position[18] = pan_angle;

//     // // Sweep to the right
//     // pan_angle -= sweep_angle_r;
//     // present_position[18] = pan_angle;

//     // Rest position
//     present_position[18] = pan_angle;
//     present_position[19] = tilt_angle;

// }

// void HeadControl::square(){
//     double top = UP + 10.0; //-20
//     double bottom = DOWN; //40
//     double left = LEFT + 10.0; //50
//     double right = RIGHT - 10.0; //-50
//     double time_interval = 0.7;
//     double pan_angle = present_position[18];
//     double tilt_angle = present_position[19];

//     // Move to top-left corner
//     pan_angle += top;
//     tilt_angle += left;
//     present_position[18] = pan_angle;
//     present_position[19] = tilt_angle;

//     // Move to top-right corner
//     pan_angle += top;
//     tilt_angle += right;
//     present_position[18] = pan_angle;
//     present_position[19] = tilt_angle;

//     // Move to bottom-right corner
//     pan_angle += bottom;
//     tilt_angle += right;
//     present_position[18] = pan_angle;
//     present_position[19] = tilt_angle;

//     // Move to bottom-left corner
//     pan_angle += bottom;
//     tilt_angle += left;
//     present_position[18] = pan_angle;
//     present_position[19] = tilt_angle;
// }


// void HeadControl::ballSearchCallback(const std_msgs::String::ConstPtr& msg){
//     if(msg->data == "NOD"){
//         scan_state = NOD;
//     }
//     else if(msg->data == "SWEEP") {
//         scan_state = SWEEP;
//     }
//     else if(msg->data == "SQUARE"){
//         scan_state = SQUARE;
//     }
//     else if(msg->data == "STOP") {
//         scan_state = NO_SCAN;
//     }
// }

void HeadControl::searching(){
    switch(scan_state) {
        case NOD:
            if(nod_steps == 0){
                present_position[19] = absolute_position[19] + DOWN;
            }
            else if(nod_steps == 1) {
                present_position[19] = absolute_position[19] + MIDDLE;
            }
            else if(nod_steps == 2) {
                present_position[19] = absolute_position[19] + UP;
            }
            else if(nod_steps == 3) {
                present_position[19] = absolute_position[19] + MIDDLE;
            }
            
            if(nod_steps)
                nod_steps = 0;
            else
                nod_steps++;
            break;
        case SWEEP:
            if(sweep_steps == 0){
                present_position[18] = absolute_position[18] + LEFT;
            }
            else if(sweep_steps == 1) {
                present_position[18] = absolute_position[18] + MIDDLE;
            }
            else if(sweep_steps == 2) {
                present_position[18] = absolute_position[18] + RIGHT;
            }
            else if(sweep_steps == 3) {
                present_position[18] = absolute_position[18] + MIDDLE;
            }
            
            if(sweep_steps == 3)
                sweep_steps = 0;
            else
                sweep_steps++;
            break;
        case SQUARE:
            if(square_steps == 0){
                present_position[18] = absolute_position[18] + LEFT;
                present_position[19] = absolute_position[19] + UP;    
            }
            else if(square_steps == 1) {
                present_position[18] = absolute_position[18] + RIGHT;
                present_position[19] = absolute_position[19] + UP;
            }
            else if(square_steps == 2) {
                present_position[18] = absolute_position[18] + RIGHT;
                present_position[19] = absolute_position[19] + DOWN;
            }
            else if(square_steps == 3) {
                present_position[18] = absolute_position[18] + LEFT;
                present_position[19] = absolute_position[19] + DOWN;
            }
            else if(square_steps == 4) {
                present_position[18] = absolute_position[18] + MIDDLE;
                present_position[19] = absolute_position[19] + MIDDLE;
            }
            
            if(square_steps == 4)
                square_steps = 0;
            else
                square_steps++;
            break;
        case NO_SCAN:
            break;
    }
    write(2000);
}

void HeadControl::write(int time) {
    if(!is_moving) {
        time_start = ros::Time::now().toSec();
        is_moving = true;
        for(uint8_t i=18; i<20; i++) {
            temp_servo->write(i+1, temp_servo->deg2Bit(present_position[i]) , time); //5 detik
            
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
    if(time_now >= time/1000) {
        is_moving = false;
        // tempSeq = tempSeq->next;
        // for(int i=0; i<20; i++) {
        //     tempSeq->getJoint()->write[i] = false;
        // }
    }
}
//square/zoro :
//kiri atas  : +50, -20 | kanan atas  : -50, -20 | tengah atas  : 0, -20 
//kiri bawah : +50, +40 | kanan bawah : -50, +40 | tengah bawah : 0, +40