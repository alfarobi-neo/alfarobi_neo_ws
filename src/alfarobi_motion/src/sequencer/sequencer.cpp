#include "sequencer/sequencer.h"


Sequencer::Sequencer(){

}

Sequencer::~Sequencer() {

}

std::string Sequencer::getName(int input) {
    return this->sequence_names[input];
}

void Sequencer::loadSequences() {
    YAML::Node sequence_params;
    try{
        sequence_params = YAML::LoadFile(ros::package::getPath("alfarobi_motion") + "/config/sequencer.yaml");
    }catch(const std::exception &e){
        ROS_ERROR("[alfarobi_motion] Unable to load params file: %s", e.what());
    }
    // const YAML::Node& temp = sequence_params;
    std::cout << "============================\n";
    std::cout << "=== WELCOME TO SEQUENCER ===\n";
    std::cout << "============================\n";
    
    int n = 0;
    for (YAML::const_iterator it = sequence_params.begin(); it != sequence_params.end(); it++) {
      std::string key= it->first.as<std::string>();
      std::cout<< n+1 <<". "<<key<< '\n';
      n += 1;

      sequence_names.push_back(key);
    }
}

void Sequencer::loadParams(std::string seq) {
    sequences_.clear();
    std::string seq_name = seq;
    YAML::Node sequence_params;
    try{
        sequence_params = YAML::LoadFile(ros::package::getPath("alfarobi_motion") + "/config/sequencer.yaml");
    }catch(const std::exception &e){
        ROS_ERROR("[alfarobi_motion] Unable to load params file: %s", e.what());
    }

    std::cout<<"==="<<seq_name<< "===\n";
    for(std::size_t i=0; i<sequence_params[seq_name].size(); i++) {
        sequence *temp = new sequence();
        
        temp->joint_val_.val[0]  = sequence_params[seq_name][i]["R_SHO_P"].as<double>();
        temp->joint_val_.val[1]  = sequence_params[seq_name][i]["L_SHO_P"].as<double>();
        temp->joint_val_.val[2]  = sequence_params[seq_name][i]["R_SHO_R"].as<double>();
        temp->joint_val_.val[3]  = sequence_params[seq_name][i]["L_SHO_R"].as<double>();
        temp->joint_val_.val[4]  = sequence_params[seq_name][i]["R_ELB"].as<double>();
        temp->joint_val_.val[5]  = sequence_params[seq_name][i]["L_ELB"].as<double>();
        temp->joint_val_.val[6]  = sequence_params[seq_name][i]["R_HIP_Y"].as<double>();
        temp->joint_val_.val[7]  = sequence_params[seq_name][i]["L_HIP_Y"].as<double>();
        temp->joint_val_.val[8]  = sequence_params[seq_name][i]["R_HIP_P"].as<double>();
        temp->joint_val_.val[9]  = sequence_params[seq_name][i]["L_HIP_P"].as<double>();
        temp->joint_val_.val[10] = sequence_params[seq_name][i]["R_HIP_R"].as<double>();
        temp->joint_val_.val[11] = sequence_params[seq_name][i]["L_HIP_R"].as<double>();
        temp->joint_val_.val[12] = sequence_params[seq_name][i]["R_KNEE"].as<double>();
        temp->joint_val_.val[13] = sequence_params[seq_name][i]["L_KNEE"].as<double>();
        temp->joint_val_.val[14] = sequence_params[seq_name][i]["R_ANK_R"].as<double>();
        temp->joint_val_.val[15] = sequence_params[seq_name][i]["L_ANK_R"].as<double>();
        temp->joint_val_.val[16] = sequence_params[seq_name][i]["R_ANK_P"].as<double>();
        temp->joint_val_.val[17] = sequence_params[seq_name][i]["L_ANK_P"].as<double>();
        temp->joint_val_.val[18] = sequence_params[seq_name][i]["HEAD_PAN"].as<double>();
        temp->joint_val_.val[19] = sequence_params[seq_name][i]["HEAD_TILT"].as<double>();

        // temp->time           = sequence_params[seq_name][i]["TIME"].as<double>();
        // Buat nge-test doang
        // const double temp2 =  sequence_params[seq_name][i]["R_SHO_P"].as<double>();
        // const double temp3 =  sequence_params[seq_name][i]["L_SHO_P"].as<double>();

        // std::cout << sequence_params[seq_name][i]["R_SHO_P"].as<int>() << '\n';
        // std::cout<<temp2<<temp3;

        sequences_.push_back(temp);

        delete temp;

    }
}

void Sequencer::apply() {

}

void Sequencer::saveParams() {

}