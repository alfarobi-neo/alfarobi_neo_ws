#include "sequencer/sequencer.h"


Sequencer::Sequencer(){

}

Sequencer::~Sequencer() {

}

void Sequencer::loadSequences() {
    YAML::Node sequence_params;
    try{
        sequence_params = YAML::LoadFile(ros::package::getPath("alfarobi_motion") + "/config/testing2.yaml");
    }catch(const std::exception &e){
        ROS_ERROR("[alfarobi_motion] Unable to load params file: %s", e.what());
    }
    // const YAML::Node& temp = sequence_params;
    std::cout << "============================\n";
    std::cout << "=== WELCOME TO SEQUENCER ===\n";
    std::cout << "============================\n";
    
    int n = 0;
    
    for (YAML::const_iterator it = sequence_params.begin(); it != sequence_params.end(); it++) {
        std::cout<< n+1 <<". "<<it->first<< '\n';
    }
    // JATUH DEPAN
    YAML::Node jatuh_depan = sequence_params["JATUH_DEPAN"];
    std::cout<<"JATUH DEPAN: "<<jatuh_depan.size() << '\n';
 
    SequenceList depan_temp;
    for(std::size_t j = 0; j < jatuh_depan.size(); j++) {
        
        Sequence seq_depan;
        seq_depan.insertVal(jatuh_depan[j]["R_SHO_P"].as<double>(), 0);
        seq_depan.insertVal(jatuh_depan[j]["L_SHO_P"].as<double>(), 1);
        seq_depan.insertVal(jatuh_depan[j]["R_SHO_R"].as<double>(), 2);
        seq_depan.insertVal(jatuh_depan[j]["L_SHO_R"].as<double>(), 3);
        seq_depan.insertVal(jatuh_depan[j]["R_ELB"].as<double>(), 4);
        seq_depan.insertVal(jatuh_depan[j]["L_ELB"].as<double>(), 5);
        seq_depan.insertVal(jatuh_depan[j]["R_HIP_Y"].as<double>(), 6);
        seq_depan.insertVal(jatuh_depan[j]["L_HIP_Y"].as<double>(), 7);
        seq_depan.insertVal(jatuh_depan[j]["R_HIP_P"].as<double>(), 8);
        seq_depan.insertVal(jatuh_depan[j]["L_HIP_P"].as<double>(), 9);
        seq_depan.insertVal(jatuh_depan[j]["R_HIP_R"].as<double>(), 10);
        seq_depan.insertVal(jatuh_depan[j]["L_HIP_R"].as<double>(), 11);
        seq_depan.insertVal(jatuh_depan[j]["R_KNEE"].as<double>(), 12);
        seq_depan.insertVal(jatuh_depan[j]["L_KNEE"].as<double>(), 13);
        seq_depan.insertVal(jatuh_depan[j]["R_ANK_R"].as<double>(), 14);
        seq_depan.insertVal(jatuh_depan[j]["L_ANK_R"].as<double>(), 15);
        seq_depan.insertVal(jatuh_depan[j]["R_ANK_P"].as<double>(), 16);
        seq_depan.insertVal(jatuh_depan[j]["L_ANK_P"].as<double>(), 17);
        seq_depan.insertVal(jatuh_depan[j]["HEAD_PAN"].as<double>(), 18);
        seq_depan.insertVal(jatuh_depan[j]["HEAD_TILT"].as<double>(), 19);
        seq_depan.insertPauseTime(jatuh_depan[j]["PAUSE_TIME"].as<double>());
        seq_depan.insertTargetTime(jatuh_depan[j]["TARGET_TIME"].as<double>());
        seq_depan.insertName("JATUH_DEPAN");
        

        depan_temp.insertSequence(seq_depan.getJoint(), seq_depan.getName());
    }
    
    //JATUH BELAKANG
    YAML::Node jatuh_belakang = sequence_params["JATUH_BELAKANG"];
    std::cout<<"JATUH BELAKANG: "<<jatuh_belakang.size() << '\n';
 
    SequenceList belakang_temp;
    for(std::size_t j = 0; j < jatuh_belakang.size(); j++) {
        
        Sequence seq_belakang;
        seq_belakang.insertVal(jatuh_belakang[j]["R_SHO_P"].as<double>(), 0);
        seq_belakang.insertVal(jatuh_belakang[j]["L_SHO_P"].as<double>(), 1);
        seq_belakang.insertVal(jatuh_belakang[j]["R_SHO_R"].as<double>(), 2);
        seq_belakang.insertVal(jatuh_belakang[j]["L_SHO_R"].as<double>(), 3);
        seq_belakang.insertVal(jatuh_belakang[j]["R_ELB"].as<double>(), 4);
        seq_belakang.insertVal(jatuh_belakang[j]["L_ELB"].as<double>(), 5);
        seq_belakang.insertVal(jatuh_belakang[j]["R_HIP_Y"].as<double>(), 6);
        seq_belakang.insertVal(jatuh_belakang[j]["L_HIP_Y"].as<double>(), 7);
        seq_belakang.insertVal(jatuh_belakang[j]["R_HIP_P"].as<double>(), 8);
        seq_belakang.insertVal(jatuh_belakang[j]["L_HIP_P"].as<double>(), 9);
        seq_belakang.insertVal(jatuh_belakang[j]["R_HIP_R"].as<double>(), 10);
        seq_belakang.insertVal(jatuh_belakang[j]["L_HIP_R"].as<double>(), 11);
        seq_belakang.insertVal(jatuh_belakang[j]["R_KNEE"].as<double>(), 12);
        seq_belakang.insertVal(jatuh_belakang[j]["L_KNEE"].as<double>(), 13);
        seq_belakang.insertVal(jatuh_belakang[j]["R_ANK_R"].as<double>(), 14);
        seq_belakang.insertVal(jatuh_belakang[j]["L_ANK_R"].as<double>(), 15);
        seq_belakang.insertVal(jatuh_belakang[j]["R_ANK_P"].as<double>(), 16);
        seq_belakang.insertVal(jatuh_belakang[j]["L_ANK_P"].as<double>(), 17);
        seq_belakang.insertVal(jatuh_belakang[j]["HEAD_PAN"].as<double>(), 18);
        seq_belakang.insertVal(jatuh_belakang[j]["HEAD_TILT"].as<double>(), 19);
        seq_belakang.insertPauseTime(jatuh_belakang[j]["PAUSE_TIME"].as<double>());
        seq_belakang.insertTargetTime(jatuh_belakang[j]["TARGET_TIME"].as<double>());
        seq_belakang.insertName("JATUH_BELAKANG");
        

        belakang_temp.insertSequence(seq_belakang.getJoint(), seq_belakang.getName());
    }

    // JATUH KIRI
    YAML::Node jatuh_kiri = sequence_params["JATUH_KIRI"];
    std::cout<<"JATUH KIRI: "<<jatuh_kiri.size() << '\n';
 
    SequenceList kiri_temp;
    for(std::size_t j = 0; j < jatuh_kiri.size(); j++) {
        
        Sequence seq_kiri;
        seq_kiri.insertVal(jatuh_kiri[j]["R_SHO_P"].as<double>(), 0);
        seq_kiri.insertVal(jatuh_kiri[j]["L_SHO_P"].as<double>(), 1);
        seq_kiri.insertVal(jatuh_kiri[j]["R_SHO_R"].as<double>(), 2);
        seq_kiri.insertVal(jatuh_kiri[j]["L_SHO_R"].as<double>(), 3);
        seq_kiri.insertVal(jatuh_kiri[j]["R_ELB"].as<double>(), 4);
        seq_kiri.insertVal(jatuh_kiri[j]["L_ELB"].as<double>(), 5);
        seq_kiri.insertVal(jatuh_kiri[j]["R_HIP_Y"].as<double>(), 6);
        seq_kiri.insertVal(jatuh_kiri[j]["L_HIP_Y"].as<double>(), 7);
        seq_kiri.insertVal(jatuh_kiri[j]["R_HIP_P"].as<double>(), 8);
        seq_kiri.insertVal(jatuh_kiri[j]["L_HIP_P"].as<double>(), 9);
        seq_kiri.insertVal(jatuh_kiri[j]["R_HIP_R"].as<double>(), 10);
        seq_kiri.insertVal(jatuh_kiri[j]["L_HIP_R"].as<double>(), 11);
        seq_kiri.insertVal(jatuh_kiri[j]["R_KNEE"].as<double>(), 12);
        seq_kiri.insertVal(jatuh_kiri[j]["L_KNEE"].as<double>(), 13);
        seq_kiri.insertVal(jatuh_kiri[j]["R_ANK_R"].as<double>(), 14);
        seq_kiri.insertVal(jatuh_kiri[j]["L_ANK_R"].as<double>(), 15);
        seq_kiri.insertVal(jatuh_kiri[j]["R_ANK_P"].as<double>(), 16);
        seq_kiri.insertVal(jatuh_kiri[j]["L_ANK_P"].as<double>(), 17);
        seq_kiri.insertVal(jatuh_kiri[j]["HEAD_PAN"].as<double>(), 18);
        seq_kiri.insertVal(jatuh_kiri[j]["HEAD_TILT"].as<double>(), 19);
        seq_kiri.insertPauseTime(jatuh_kiri[j]["PAUSE_TIME"].as<double>());
        seq_kiri.insertTargetTime(jatuh_kiri[j]["TARGET_TIME"].as<double>());
        seq_kiri.insertName("JATUH_KIRI");
        

        kiri_temp.insertSequence(seq_kiri.getJoint(), seq_kiri.getName());
    }

    // JATUH KANAN
    YAML::Node jatuh_kanan = sequence_params["JATUH_KANAN"];
    std::cout<<"JATUH KANAN: "<<jatuh_kanan.size() << '\n';
 
    SequenceList kanan_temp;
    for(std::size_t j = 0; j < jatuh_kanan.size(); j++) {
        
        Sequence seq_kanan;
        seq_kanan.insertVal(jatuh_kanan[j]["R_SHO_P"].as<double>(), 0);
        seq_kanan.insertVal(jatuh_kanan[j]["L_SHO_P"].as<double>(), 1);
        seq_kanan.insertVal(jatuh_kanan[j]["R_SHO_R"].as<double>(), 2);
        seq_kanan.insertVal(jatuh_kanan[j]["L_SHO_R"].as<double>(), 3);
        seq_kanan.insertVal(jatuh_kanan[j]["R_ELB"].as<double>(), 4);
        seq_kanan.insertVal(jatuh_kanan[j]["L_ELB"].as<double>(), 5);
        seq_kanan.insertVal(jatuh_kanan[j]["R_HIP_Y"].as<double>(), 6);
        seq_kanan.insertVal(jatuh_kanan[j]["L_HIP_Y"].as<double>(), 7);
        seq_kanan.insertVal(jatuh_kanan[j]["R_HIP_P"].as<double>(), 8);
        seq_kanan.insertVal(jatuh_kanan[j]["L_HIP_P"].as<double>(), 9);
        seq_kanan.insertVal(jatuh_kanan[j]["R_HIP_R"].as<double>(), 10);
        seq_kanan.insertVal(jatuh_kanan[j]["L_HIP_R"].as<double>(), 11);
        seq_kanan.insertVal(jatuh_kanan[j]["R_KNEE"].as<double>(), 12);
        seq_kanan.insertVal(jatuh_kanan[j]["L_KNEE"].as<double>(), 13);
        seq_kanan.insertVal(jatuh_kanan[j]["R_ANK_R"].as<double>(), 14);
        seq_kanan.insertVal(jatuh_kanan[j]["L_ANK_R"].as<double>(), 15);
        seq_kanan.insertVal(jatuh_kanan[j]["R_ANK_P"].as<double>(), 16);
        seq_kanan.insertVal(jatuh_kanan[j]["L_ANK_P"].as<double>(), 17);
        seq_kanan.insertVal(jatuh_kanan[j]["HEAD_PAN"].as<double>(), 18);
        seq_kanan.insertVal(jatuh_kanan[j]["HEAD_TILT"].as<double>(), 19);
        seq_kanan.insertPauseTime(jatuh_kanan[j]["PAUSE_TIME"].as<double>());
        seq_kanan.insertTargetTime(jatuh_kanan[j]["TARGET_TIME"].as<double>());
        seq_kanan.insertName("JATUH_KANAN");
        

        kanan_temp.insertSequence(seq_kanan.getJoint(), seq_kanan.getName());
    }

    sequences_list_.push_back(depan_temp);
    sequences_list_.push_back(belakang_temp);
    sequences_list_.push_back(kiri_temp);
    sequences_list_.push_back(kanan_temp);
        
    
}

void Sequencer::loadParams() {
    
}

void Sequencer::apply() {

}

void Sequencer::saveParams() {
    std::cout << "===SAVINGGGG===\n";
    YAML::Emitter emitter;

    // Start the YAML document
    emitter << YAML::BeginDoc;

    // Write the data to the YAML document
    SequenceList *current = new SequenceList();
    emitter << YAML::Value << YAML::BeginMap;
    for(int n=0; n<sequences_list_.size(); n++) {
        
        emitter << YAML::Key << sequences_list_[n].getSeq()->getName() << YAML::BeginSeq;
        

        
        // std::cout<<sequences_list_[n].getSeq()->joint_->target_time<<'\n';
        *current = sequences_list_[n];
        current->printList();

        Sequence *temp = new Sequence();
        temp = sequences_list_[n].getSeq();
        while(temp != NULL) {
            emitter << YAML::Value << YAML::BeginMap;
            std::cout<<sequences_list_[n].getSeq()->getName()<<" VALDII\n";
            for(int i=0; i< 20; i++) {
                emitter << YAML::Key << sequences_list_[n].getSeq()->joint_->name[i];
                {
                    std::ostringstream oss;
                    oss << std::setprecision(4) << std::fixed << temp->joint_->val[i];
                    emitter << YAML::Value << oss.str();
                }
            }
            emitter << YAML::Key << "TARGET_TIME";
            {
                std::ostringstream oss;
                oss << std::setprecision(4) << std::fixed << temp->joint_->target_time;
                emitter << YAML::Value << oss.str();
            }
            emitter << YAML::Key << "PAUSE_TIME";
            {
                std::ostringstream oss;
                oss << std::setprecision(4) << std::fixed << temp->joint_->pause_time;
                emitter << YAML::Value << oss.str();
            }
            temp = temp->next;
            emitter << YAML::EndMap;
        } if(temp != NULL) {
            std::cout<<sequences_list_[n].getSeq()->getName()<<" VALDII JUGAA\n";
        }
        
        
        emitter << YAML::EndSeq;
        
        delete temp;
        
    }
    emitter << YAML::EndMap;
    
    
    emitter << YAML::EndDoc;

    std::ofstream file_out(ros::package::getPath("alfarobi_motion") + "/config/sequencer.yaml");
    file_out << emitter.c_str();
    file_out.close();
    std::cout << "===SAVED===\n";
}