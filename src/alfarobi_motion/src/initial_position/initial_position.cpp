#include "initial_position/initial_position.h"

InitialPosition::InitialPosition(){
    // temp_servo = new alfarobi::ServoController();
}

InitialPosition::~InitialPosition() {
    // delete temp_motion;
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
    
    present_position[0] = init_pose_params["R_SHO_P"].as<double>();
    present_position[1] = init_pose_params["L_SHO_P"].as<double>();
    present_position[2] = init_pose_params["R_SHO_R"].as<double>();
    present_position[3] = init_pose_params["L_SHO_R"].as<double>();
    present_position[4] = init_pose_params["R_ELB"].as<double>();
    present_position[5] = init_pose_params["L_ELB"].as<double>();
    present_position[6] = init_pose_params["R_HIP_Y"].as<double>();
    present_position[7] = init_pose_params["L_HIP_Y"].as<double>();
    present_position[8] = init_pose_params["R_HIP_P"].as<double>();
    present_position[9] = init_pose_params["L_HIP_P"].as<double>();
    present_position[10] = init_pose_params["R_HIP_R"].as<double>();
    present_position[11] = init_pose_params["L_HIP_R"].as<double>();
    present_position[12] = init_pose_params["R_KNEE"].as<double>();
    present_position[13] = init_pose_params["L_KNEE"].as<double>();
    present_position[14] = init_pose_params["R_ANK_R"].as<double>();
    present_position[15] = init_pose_params["L_ANK_R"].as<double>();
    present_position[16] = init_pose_params["R_ANK_P"].as<double>();
    present_position[17] = init_pose_params["L_ANK_P"].as<double>();
    present_position[18] = init_pose_params["HEAD_PAN"].as<double>();
    present_position[19] = init_pose_params["HEAD_TILT"].as<double>();

    alfarobi_web_gui::Sequencer joint_val;
    joint_val.r_sho_p = present_position[0];
    joint_val.l_sho_p = present_position[1];
    joint_val.r_sho_r = present_position[2];
    joint_val.l_sho_r = present_position[3];
    joint_val.r_el = present_position[4];
    joint_val.l_el = present_position[5];
    joint_val.r_hip_y = present_position[6];
    joint_val.l_hip_y = present_position[7];
    joint_val.r_hip_p = present_position[8];
    joint_val.l_hip_p = present_position[9];
    joint_val.r_hip_r = present_position[10];
    joint_val.l_hip_r = present_position[11];
    joint_val.r_knee = present_position[12];
    joint_val.l_knee = present_position[13];
    joint_val.r_ank_r = present_position[14];
    joint_val.l_ank_r = present_position[15];
    joint_val.r_ank_p = present_position[16];
    joint_val.l_ank_p = present_position[17];
    joint_val.head_pan = present_position[18];
    joint_val.head_tilt = present_position[19];

    joint_pub.publish(joint_val);
}

void InitialPosition::goInitPose() {
    write();
}

void InitialPosition::process() {
    joint_pub = nh_.advertise<alfarobi_web_gui::Sequencer>("/joint_value", 1000);
    button_sub = nh_.subscribe("initial_pose/web_button", 1000, &InitialPosition::webButtonCallback, this);
    temp_servo = new alfarobi::ServoController();
}

void InitialPosition::webButtonCallback(const std_msgs::String::ConstPtr& msg) {
    if(comm != msg->data) {
        if(comm == "init_pose") {
            goInitPose();
            comm = msg->data;
        }
        else if(comm == "apply") {
            comm = msg->data;
        }
        else if(comm == "save") {
            comm = msg->data;
        }
        else if(comm == "refresh") {
            comm = msg->data;
        }
    }
}

void InitialPosition::applyCallback(const alfarobi_web_gui::Sequencer::ConstPtr& web_joint) {
    // double* present_position_temp = new double[20];
    present_position[0]  = web_joint->r_sho_p  ;
    present_position[1]  = web_joint->l_sho_p  ;
    present_position[2]  = web_joint->r_sho_r  ;
    present_position[3]  = web_joint->l_sho_r  ;
    present_position[4]  = web_joint->r_el     ;
    present_position[5]  = web_joint->l_el     ;
    present_position[6]  = web_joint->r_hip_y  ;
    present_position[7]  = web_joint->l_hip_y  ;
    present_position[8]  = web_joint->r_hip_p  ;
    present_position[9]  = web_joint->l_hip_p  ;
    present_position[10] = web_joint->r_hip_r  ;
    present_position[11] = web_joint->l_hip_r  ;
    present_position[12] = web_joint->r_knee   ;
    present_position[13] = web_joint->l_knee   ;
    present_position[14] = web_joint->r_ank_r  ;
    present_position[15] = web_joint->l_ank_r  ;
    present_position[16] = web_joint->r_ank_p  ;
    present_position[17] = web_joint->l_ank_p  ;
    present_position[18] = web_joint->head_pan ;
    present_position[19] = web_joint->head_tilt;   
}

void InitialPosition::saveParams() {
    std::cout << "===SAVINGGGG===\n";
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
    std::cout << "===SAVED===\n";
}

void InitialPosition::write() {
    for(int i=0; i<20; i++) {
        temp_servo->write(i+1, present_position[i] , 5000); //5 detik
        std::cout<<"Writing\n";
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
    temp_servo->torqueEnableID(id);
    read(id);
}

void InitialPosition::disable(int id) {
    temp_servo->torqueDisableID(id);
}