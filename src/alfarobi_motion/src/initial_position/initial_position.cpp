#include "initial_position/initial_position.h"

InitialPosition::InitialPosition(){
    joint_pub = nh_.advertise<alfarobi_web_gui::Sequencer>("initial_pose/joint_value", 1000);
    button_sub = nh_.subscribe("initial_pose/web_button", 1000, &InitialPosition::webButtonCallback, this);
    joint_sub = nh_.subscribe("initial_pose/joint_value_web", 1000, &InitialPosition::applyCallback, this);
    motion_state_sub = nh_.subscribe("/motion_state", 1000, &InitialPosition::motionStateCallback, this);
    torque_sub = nh_.subscribe("/torque", 1000, &InitialPosition::torqueCallback, this);

    temp_servo = new alfarobi::ServoController();
    // temp_servo->initialize();
    // temp_servo->torqueEnable();
}

InitialPosition::~InitialPosition() {
    // delete temp_motion;
    // temp_servo->torqueDisable();
    // temp_servo->dispose();
    delete temp_servo;
}

void InitialPosition::loadParams() {
    YAML::Node init_pose_params;
    try{
        init_pose_params = YAML::LoadFile(ros::package::getPath("alfarobi_motion") + "/config/initial_position.yaml");
    }catch(const std::exception &e){
        ROS_ERROR("[alfarobi_motion] Unable to load params file: %s", e.what());
    }
    // const YAML::Node& temp = sequence_params;
    std::cout << "===================================\n";
    std::cout << "=== WELCOME TO INITIAL POSITION ===\n";
    std::cout << "===================================\n";
    
    try{
        present_position[0] = init_pose_params["r_sho_p"].as<double>();
        present_position[1] = init_pose_params["l_sho_p"].as<double>();
        present_position[2] = init_pose_params["r_sho_r"].as<double>();
        present_position[3] = init_pose_params["l_sho_r"].as<double>();
        present_position[4] = init_pose_params["r_el"].as<double>();
        present_position[5] = init_pose_params["l_el"].as<double>();
        present_position[6] = init_pose_params["r_hip_y"].as<double>();
        present_position[7] = init_pose_params["l_hip_y"].as<double>();
        present_position[8] = init_pose_params["r_hip_r"].as<double>();
        present_position[9] = init_pose_params["l_hip_r"].as<double>();
        present_position[10] = init_pose_params["r_hip_p"].as<double>();
        present_position[11] = init_pose_params["l_hip_p"].as<double>();
        present_position[12] = init_pose_params["r_knee"].as<double>();
        present_position[13] = init_pose_params["l_knee"].as<double>();
        present_position[14] = init_pose_params["r_ank_p"].as<double>();
        present_position[15] = init_pose_params["l_ank_p"].as<double>();
        present_position[16] = init_pose_params["r_ank_r"].as<double>();
        present_position[17] = init_pose_params["l_ank_r"].as<double>();
        present_position[18] = init_pose_params["head_pan"].as<double>();
        present_position[19] = init_pose_params["head_tilt"].as<double>();
    }catch(const std::exception &e){
        ROS_ERROR("[alfarobi_motion]: %s", e.what());
    }
    

    ROS_INFO("r_sho_p: %f", present_position[0]);
    ROS_INFO("l_sho_p: %f", present_position[1]);
    ROS_INFO("r_sho_r: %f", present_position[2]);
    ROS_INFO("l_sho_r: %f", present_position[3]);
    ROS_INFO("r_el: %f", present_position[4]);
    ROS_INFO("l_el: %f", present_position[5]);
    ROS_INFO("r_hip_y: %f", present_position[6]);
    ROS_INFO("l_hip_y: %f", present_position[7]);
    ROS_INFO("r_hip_r: %f", present_position[8]);
    ROS_INFO("l_hip_r: %f", present_position[9]);
    ROS_INFO("r_hip_p: %f", present_position[10]);
    ROS_INFO("l_hip_p: %f", present_position[11]);
    ROS_INFO("r_knee: %f", present_position[12]);
    ROS_INFO("l_knee: %f", present_position[13]);
    ROS_INFO("r_ank_p: %f", present_position[14]);
    ROS_INFO("l_ank_p: %f", present_position[15]);
    ROS_INFO("r_ank_r: %f", present_position[16]);
    ROS_INFO("l_ank_r: %f", present_position[17]);
    ROS_INFO("head_pan: %f", present_position[18]);
    ROS_INFO("head_tilt: %f", present_position[19]);

    alfarobi_web_gui::Sequencer joint_val;
    joint_val.r_sho_p = present_position[0];
    joint_val.l_sho_p = present_position[1];
    joint_val.r_sho_r = present_position[2];
    joint_val.l_sho_r = present_position[3];
    joint_val.r_el = present_position[4];
    joint_val.l_el = present_position[5];
    joint_val.r_hip_y = present_position[6];
    joint_val.l_hip_y = present_position[7];
    joint_val.r_hip_r = present_position[8];
    joint_val.l_hip_r = present_position[9];
    joint_val.r_hip_p = present_position[10];
    joint_val.l_hip_p = present_position[11];
    joint_val.r_knee = present_position[12];
    joint_val.l_knee = present_position[13];
    joint_val.r_ank_p = present_position[14];
    joint_val.l_ank_p = present_position[15];
    joint_val.r_ank_r = present_position[16];
    joint_val.l_ank_r = present_position[17];
    joint_val.head_pan = present_position[18];
    joint_val.head_tilt = present_position[19];

    joint_pub.publish(joint_val);
}

void InitialPosition::goInitPose() {
    write();
}

void InitialPosition::process(alfarobi::ServoController **serv) {
    if(!in_action){
        // enable();
        loadParams();
        temp_servo = *serv;
    }

    if(state_now == "initial_position") {

        in_action = true;
        // int input = 0;
        // std::cin>>input;
        // if(input == 999) {
        //     return;
        // }
        // loadParams("JATUH_DEPAN");

        // loop_rate.sleep();
        // ros::spin();
    }
}

void InitialPosition::motionStateCallback(const std_msgs::String::ConstPtr& msg) {
    state_now = msg->data;
    if(state_now != "initial_position"){
        in_action = false;
    }
}

void InitialPosition::webButtonCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "init_pose") {
        ROS_INFO("GO INITIAL POSITION");
        goInitPose();
        // comm = msg->data;
    }
    // else if(msg->data == "apply") {
    //     // comm = msg->data;
    //     ROS_INFO("APPLYING");
    // }
    else if(msg->data == "save") {
        // comm = msg->data;
        ROS_INFO("SAVING");
        saveParams();
    }
    else if(msg->data == "refresh") {
        // comm = msg->data;
        ROS_INFO("REFRESH");
        refresh(); //refresh sementara untuk ngeread dulu, uncommand untuk ngeread
    }
}

void InitialPosition::refresh() {
    for(int i=1; i<21; i++) {
        read(i);
        ROS_INFO("JOINT VALUE: %f", present_position[i-1]);
    }
    alfarobi_web_gui::Sequencer joint_val;
    joint_val.r_sho_p = present_position[0];
    joint_val.l_sho_p = present_position[1];
    joint_val.r_sho_r = present_position[2];
    joint_val.l_sho_r = present_position[3];
    joint_val.r_el = present_position[4];
    joint_val.l_el = present_position[5];
    joint_val.r_hip_y = present_position[6];
    joint_val.l_hip_y = present_position[7];
    joint_val.r_hip_r = present_position[8];
    joint_val.l_hip_r = present_position[9];
    joint_val.r_hip_p = present_position[10];
    joint_val.l_hip_p = present_position[11];
    joint_val.r_knee = present_position[12];
    joint_val.l_knee = present_position[13];
    joint_val.r_ank_p = present_position[14];
    joint_val.l_ank_p = present_position[15];
    joint_val.r_ank_r = present_position[16];
    joint_val.l_ank_r = present_position[17];
    joint_val.head_pan = present_position[18];
    joint_val.head_tilt = present_position[19];

    joint_pub.publish(joint_val);
}

void InitialPosition::torqueCallback(const alfarobi_web_gui::Torque::ConstPtr& torque){
    std::string torq_name = torque->joint_name;
    bool torq_state = torque->joint_state;

    alfarobi::joint_value *temp_joint = new alfarobi::joint_value();

    if(torq_state) {
        enable(temp_joint->getIdByString(torq_name));
        ROS_INFO("%s enabled", torq_name.c_str());
    }
    else if(!torq_state) {
        disable(temp_joint->getIdByString(torq_name));
        ROS_INFO("%s disabled", torq_name.c_str());
    }

    delete temp_joint;

    
}
void InitialPosition::applyCallback(const alfarobi_web_gui::Sequencer::ConstPtr& web_joint) {
    ROS_INFO("APPLYING");

    // double* present_position_temp = new double[20];
    present_position[0]  = web_joint->r_sho_p  ;
    present_position[1]  = web_joint->l_sho_p  ;
    present_position[2]  = web_joint->r_sho_r  ;
    present_position[3]  = web_joint->l_sho_r  ;
    present_position[4]  = web_joint->r_el     ;
    present_position[5]  = web_joint->l_el     ;
    present_position[6]  = web_joint->r_hip_y  ;
    present_position[7]  = web_joint->l_hip_y  ;
    present_position[8]  = web_joint->r_hip_r  ;
    present_position[9]  = web_joint->l_hip_r  ;
    present_position[10] = web_joint->r_hip_p  ;
    present_position[11] = web_joint->l_hip_p  ;
    present_position[12] = web_joint->r_knee   ;
    present_position[13] = web_joint->l_knee   ;
    present_position[14] = web_joint->r_ank_p  ;
    present_position[15] = web_joint->l_ank_p  ;
    present_position[16] = web_joint->r_ank_r  ;
    present_position[17] = web_joint->l_ank_r  ;
    present_position[18] = web_joint->head_pan ;
    present_position[19] = web_joint->head_tilt;  
     
    ROS_INFO("APPLIED");
    ROS_INFO("r_sho_p: %f", present_position[0]);
    ROS_INFO("l_sho_p: %f", present_position[1]);
    ROS_INFO("r_sho_r: %f", present_position[2]);
    ROS_INFO("l_sho_r: %f", present_position[3]);
    ROS_INFO("r_el: %f", present_position[4]);
    ROS_INFO("l_el: %f", present_position[5]);
    ROS_INFO("r_hip_y: %f", present_position[6]);
    ROS_INFO("l_hip_y: %f", present_position[7]);
    ROS_INFO("r_hip_r: %f", present_position[8]);
    ROS_INFO("l_hip_r: %f", present_position[9]);
    ROS_INFO("r_hip_p: %f", present_position[10]);
    ROS_INFO("l_hip_p: %f", present_position[11]);
    ROS_INFO("r_knee: %f", present_position[12]);
    ROS_INFO("l_knee: %f", present_position[13]);
    ROS_INFO("r_ank_p: %f", present_position[14]);
    ROS_INFO("l_ank_p: %f", present_position[15]);
    ROS_INFO("r_ank_r: %f", present_position[16]);
    ROS_INFO("l_ank_r: %f", present_position[17]);
    ROS_INFO("head_pan: %f", present_position[18]);
    ROS_INFO("head_tilt: %f", present_position[19]);
    write();
}

void InitialPosition::saveParams() {
    YAML::Emitter emitter;

    // Start the YAML document
    emitter << YAML::BeginDoc;

    // Write the data to the YAML document

    alfarobi::joint_value *joint_temp = new alfarobi::joint_value();
    emitter << YAML::Value << YAML::BeginMap;
    for(int i=0; i< 20; i++) {
        emitter << YAML::Key << joint_temp->name[i];
        {
            std::ostringstream oss;
            oss << std::setprecision(4) << std::fixed << present_position[i];
            emitter << YAML::Value << oss.str();
        }
    }
    emitter << YAML::EndMap;
    
    
    emitter << YAML::EndDoc;

    std::ofstream file_out(ros::package::getPath("alfarobi_motion") + "/config/initial_position.yaml");
    file_out << emitter.c_str();
    file_out.close();
    ROS_INFO("SAVED");
}

void InitialPosition::write() {
    if(!is_moving) {
        time_start = ros::Time::now().toSec();
        is_moving = true;
        for(uint8_t i=0; i<20; i++) {
            temp_servo->write(i+1, temp_servo->deg2Bit(present_position[i]) , 3000); //5 detik
            
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
    if(time_now >= 3000/1000) {
        is_moving = false;
        // tempSeq = tempSeq->next;
        // for(int i=0; i<20; i++) {
        //     tempSeq->getJoint()->write[i] = false;
        // }
    }
    
}

void InitialPosition::readAll() {
    for(int i=0; i<20; i++) {
        present_position[i] = temp_servo->read(i+1);
    }
}

void InitialPosition::read(int id) {
    present_position[id-1] = temp_servo->read(id);
}

void InitialPosition::enable(int id) {
    temp_servo->torqueEnableID(id+1);
    ROS_INFO("TORQ ID: %d", id+1);
    read(id);
}

void InitialPosition::disable(int id) {
    ROS_INFO("TORQ ID: %d", id+1);
    temp_servo->torqueDisableID(id+1);
}