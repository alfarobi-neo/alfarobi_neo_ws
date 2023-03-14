#include "sequencer/sequencer.h"


Sequencer::Sequencer(){
    sequence_pub = nh_.advertise<alfarobi_web_gui::SequencerArr>("/SequenceArr", 1000);
    sequence_list_sub = nh_.subscribe("/SequenceList", 1000, &Sequencer::sequenceListCallback, this);
    apply_sub = nh_.subscribe("/SequenceArr", 1000, &Sequencer::applyCallback, this);
    web_button_sub = nh_.subscribe("/playButton", 1000, &Sequencer::webButtonCallback, this);
    torque_sub = nh_.subscribe("/torqueState", 1000, &Sequencer::webButtonCallback, this);
    motion_state_sub = nh_.subscribe("/motion_state", 1000, &Sequencer::motionStateCallback, this);

    temp_servo = new alfarobi::ServoController();
}

Sequencer::~Sequencer() {
    // delete temp_motion;
    delete temp_servo;
}

void Sequencer::process() {
    if(!in_action) {
        loadSequences();
    }
    
    if(state_now == "sequencer") {

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

void Sequencer::motionStateCallback(const std_msgs::String::ConstPtr& msg) {
    state_now = msg->data;
    if(state_now != "sequencer"){
        in_action = false;
    }
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
        for(int idx = 0; idx<20; idx++) {
            seq_depan.insertPauseTime(jatuh_depan[j]["PAUSE_TIME"].as<double>(), idx);
            seq_depan.insertTargetTime(jatuh_depan[j]["TARGET_TIME"].as<double>(), idx);
        }
        
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
        for(int idx = 0; idx<20; idx++) {
            seq_belakang.insertPauseTime(jatuh_belakang[j]["PAUSE_TIME"].as<double>(), idx);
            seq_belakang.insertTargetTime(jatuh_belakang[j]["TARGET_TIME"].as<double>(), idx);
        }
        
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
        for(int idx = 0; idx<20; idx++) {
            seq_kiri.insertPauseTime(jatuh_kiri[j]["PAUSE_TIME"].as<double>(), idx);
            seq_kiri.insertTargetTime(jatuh_kiri[j]["TARGET_TIME"].as<double>(), idx);
        }
        
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
        for(int idx = 0; idx<20; idx++) {
            seq_kanan.insertPauseTime(jatuh_kanan[j]["PAUSE_TIME"].as<double>(), idx);
            seq_kanan.insertTargetTime(jatuh_kanan[j]["TARGET_TIME"].as<double>(), idx);
        }
        
        seq_kanan.insertName("JATUH_KANAN");
        

        kanan_temp.insertSequence(seq_kanan.getJoint(), seq_kanan.getName());
    }

    sequences_list_.push_back(depan_temp);
    sequences_list_.push_back(belakang_temp);
    sequences_list_.push_back(kiri_temp);
    sequences_list_.push_back(kanan_temp);
}

void Sequencer::write(alfarobi::joint_value *joints_) {
    for(int i=0; i<20; i++) {
        if(joints_->write[i]) {
            temp_servo->write(i+1, joints_->val[i] , joints_->target_time[i]);
            std::cout<<"Writing\n";
        }
    }
}

void Sequencer::readAll() {
    for(int i=0; i<20; i++) {
        present_position[i] = temp_servo->read(i+1);
    }
}

void Sequencer::read(int id) {
    present_position[id - 1] = temp_servo->read(id);
}

void Sequencer::enable(int id) {
    temp_servo->torqueEnableID(id);
    read(id);
}

void Sequencer::disable(int id) {
    temp_servo->torqueDisableID(id);
}

void Sequencer::torqueCallback(const alfarobi_web_gui::Torque::ConstPtr& torque){
    std::string torq_name = torque->joint_name;
    bool torq_state = torque->joint_state;

    int name_index = 0;

    
    for(int i=0; i<sequences_list_.size(); i++) {
        if(sequences_list_[i].getSeq()->getName() == getCurrentName()){
            break;
        }
        else if(sequences_list_[i].getSeq()->getName() != getCurrentName() && sequences_list_.size() - i <= 1){
            ROS_ERROR("[alfarobi_motion] Sequence name doesn't exist");
            return;
        }
        name_index++;
    }

    Sequence *tempSeq = new Sequence();
    tempSeq = sequences_list_[name_index].getSeq();

    if(torq_state) {
        enable(tempSeq->getJoint()->getIdByString(torq_name));
    }
    else if(!torq_state) {
        disable(tempSeq->getJoint()->getIdByString(torq_name));
    }

    delete tempSeq;

    
}

void Sequencer::sequenceListCallback(const std_msgs::String::ConstPtr& msg) {
    setCurrentName(msg->data);
    try {
        loadParams(getCurrentName());
    }catch(const std::exception &e) {
        ROS_ERROR("[alfarobi_motion] Sequence name doesn't exist: %s", e.what());
    }
}

// callback saat 'apply' di web
void Sequencer::applyCallback(const alfarobi_web_gui::SequencerArr::ConstPtr& arr) {
    int name_index = 0;

    
    for(int i=0; i<sequences_list_.size(); i++) {
        if(sequences_list_[i].getSeq()->getName() == getCurrentName()){
            break;
        }
        else if(sequences_list_[i].getSeq()->getName() != getCurrentName() && sequences_list_.size() - i <= 1){
            ROS_ERROR("[alfarobi_motion] Sequence name doesn't exist");
            return;
        }
        name_index++;
    }

    Sequence *tempSeq = new Sequence();
    tempSeq = sequences_list_[name_index].getSeq();

    int i=0;

    std::cout<<"===APPLYING: "<<arr->SEQUENCE_NAME<<"======\n";

    while(arr->SEQUENCE[i].target_time != 0) {
        // std::cout << "AAAAAA\n";
        
        // std::cout<<"BBBB\n";
        tempSeq->getJoint()->setVal(0, arr->SEQUENCE[i].r_sho_p);
        tempSeq->getJoint()->setVal(1, arr->SEQUENCE[i].l_sho_p);
        tempSeq->getJoint()->setVal(2, arr->SEQUENCE[i].r_sho_r);
        tempSeq->getJoint()->setVal(3, arr->SEQUENCE[i].l_sho_r);
        tempSeq->getJoint()->setVal(4, arr->SEQUENCE[i].r_el);
        tempSeq->getJoint()->setVal(5, arr->SEQUENCE[i].l_el);
        tempSeq->getJoint()->setVal(6, arr->SEQUENCE[i].r_hip_y);
        tempSeq->getJoint()->setVal(7, arr->SEQUENCE[i].l_hip_y);
        tempSeq->getJoint()->setVal(8, arr->SEQUENCE[i].r_hip_p);
        tempSeq->getJoint()->setVal(9, arr->SEQUENCE[i].l_hip_p);
        tempSeq->getJoint()->setVal(10, arr->SEQUENCE[i].r_hip_r);
        tempSeq->getJoint()->setVal(11, arr->SEQUENCE[i].l_hip_r);
        tempSeq->getJoint()->setVal(12, arr->SEQUENCE[i].r_knee);
        tempSeq->getJoint()->setVal(13, arr->SEQUENCE[i].l_knee);
        tempSeq->getJoint()->setVal(14, arr->SEQUENCE[i].r_ank_r);
        tempSeq->getJoint()->setVal(15, arr->SEQUENCE[i].l_ank_r);
        tempSeq->getJoint()->setVal(16, arr->SEQUENCE[i].r_ank_p);
        tempSeq->getJoint()->setVal(17, arr->SEQUENCE[i].l_ank_p);
        tempSeq->getJoint()->setVal(18, arr->SEQUENCE[i].head_pan);
        tempSeq->getJoint()->setVal(19, arr->SEQUENCE[i].head_tilt);
        for(int j=0; j < 20; j++) {
            tempSeq->getJoint()->target_time[j] = arr->SEQUENCE[i].target_time;
            tempSeq->getJoint()->pause_time[j] = arr->SEQUENCE[i].pause_time;
        }
        i++;
        tempSeq = tempSeq->next;
    }
    std::cout<<"===APPLIED: "<<arr->SEQUENCE_NAME<<"======\n";
    delete tempSeq;
}

void Sequencer::loadParams(std::string name) {

    int name_index = 0;

    
    for(int i=0; i<sequences_list_.size(); i++) {
        if(sequences_list_[i].getSeq()->getName() == name){
            break;
        }
        else if(sequences_list_[i].getSeq()->getName() != name && sequences_list_.size() - i <= 1){
            ROS_ERROR("[alfarobi_motion] Sequence name doesn't exist");
            return;
        }
        name_index++;
    }
    // publish sequences to the web
    alfarobi_web_gui::SequencerArr arr;
    

    Sequence *tempSeq = new Sequence();
    tempSeq = sequences_list_[name_index].getSeq();

    int i=0;
    arr.SEQUENCE_NAME = sequences_list_[name_index].getSeq()->getName();

    std::cout<<"==="<<arr.SEQUENCE_NAME<<"======\n";

    while(tempSeq != NULL) {
        // std::cout << "AAAAAA\n";
        
        // std::cout<<"BBBB\n";
        arr.SEQUENCE[i].r_sho_p = tempSeq->getJoint()->getVal(0);
        arr.SEQUENCE[i].l_sho_p = tempSeq->getJoint()->getVal(1);
        arr.SEQUENCE[i].r_sho_r = tempSeq->getJoint()->getVal(2);
        arr.SEQUENCE[i].l_sho_r = tempSeq->getJoint()->getVal(3);
        arr.SEQUENCE[i].r_el    = tempSeq->getJoint()->getVal(4);
        arr.SEQUENCE[i].l_el    = tempSeq->getJoint()->getVal(5);
        arr.SEQUENCE[i].r_hip_y = tempSeq->getJoint()->getVal(6);
        arr.SEQUENCE[i].l_hip_y = tempSeq->getJoint()->getVal(7);
        arr.SEQUENCE[i].r_hip_p = tempSeq->getJoint()->getVal(8);
        arr.SEQUENCE[i].l_hip_p = tempSeq->getJoint()->getVal(9);
        arr.SEQUENCE[i].r_hip_r = tempSeq->getJoint()->getVal(10);
        arr.SEQUENCE[i].l_hip_r = tempSeq->getJoint()->getVal(11);
        arr.SEQUENCE[i].r_knee  = tempSeq->getJoint()->getVal(12);
        arr.SEQUENCE[i].l_knee  = tempSeq->getJoint()->getVal(13);
        arr.SEQUENCE[i].r_ank_r = tempSeq->getJoint()->getVal(14);
        arr.SEQUENCE[i].l_ank_r = tempSeq->getJoint()->getVal(15);
        arr.SEQUENCE[i].r_ank_p = tempSeq->getJoint()->getVal(16);
        arr.SEQUENCE[i].l_ank_p = tempSeq->getJoint()->getVal(17);
        arr.SEQUENCE[i].head_pan = tempSeq->getJoint()->getVal(18);
        arr.SEQUENCE[i].head_tilt = tempSeq->getJoint()->getVal(19);
        arr.SEQUENCE[i].target_time = tempSeq->getJoint()->target_time[0];
        arr.SEQUENCE[i].pause_time = tempSeq->getJoint()->pause_time[0];
        i++;
        tempSeq = tempSeq->next;
    }

    sequence_pub.publish(arr);

    delete tempSeq;
   
}

void Sequencer::webButtonCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "play"){
        is_playing = true;
        play();
    }
    else if(msg->data == "stop") {
        is_playing = false;
    }
    else if(msg->data == "refresh") {
        refresh();
    }
}

void Sequencer::play() {
    int name_index = 0;

    
    for(int i=0; i<sequences_list_.size(); i++) {
        if(sequences_list_[i].getSeq()->getName() == getCurrentName()){
            break;
        }
        else if(sequences_list_[i].getSeq()->getName() != getCurrentName() && sequences_list_.size() - i <= 1){
            ROS_ERROR("[alfarobi_motion] Sequence name doesn't exist");
            return;
        }
        name_index++;
    }

    Sequence *tempSeq = new Sequence();
    tempSeq = sequences_list_[name_index].getSeq();


    while(tempSeq != NULL) {
        if(is_playing){
            if(!is_moving) {
                time_start = ros::Time::now().toSec();
                is_moving = true;
                for(int i=0; i<20; i++) {
                    tempSeq->getJoint()->write[i] = true;
                }
                write(tempSeq->getJoint());
                readAll();
            }
            time_now = ros::Time::now().toSec() - time_start;
            std::cout<<"Time Now: "<<time_now<<'\n';
            if(time_now >= (tempSeq->getJoint()->target_time[0] + tempSeq->getJoint()->pause_time[0])) {
                is_moving = false;
                tempSeq = tempSeq->next;
                for(int i=0; i<20; i++) {
                    tempSeq->getJoint()->write[i] = false;
                }
            }
        }
        else if(!is_playing){
            break;
        }
    }

    delete tempSeq;
}

void Sequencer::refresh() {
    ROS_INFO("REFRESH");
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
                oss << std::setprecision(4) << std::fixed << temp->joint_->target_time[0];
                emitter << YAML::Value << oss.str();
            }
            emitter << YAML::Key << "PAUSE_TIME";
            {
                std::ostringstream oss;
                oss << std::setprecision(4) << std::fixed << temp->joint_->pause_time[0];
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