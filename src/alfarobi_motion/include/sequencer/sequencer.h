#ifndef SEQUENCER_H
#define SEQUENCER_H


// #include "alfarobi_motion/alfarobi_motion.h"
#include "alfarobi_web_gui/SequencerArr.h"
#include "alfarobi_web_gui/Torque.h"
#include "alfarobi_dxlsdk/servo_controller.h"
#include "std_msgs/String.h"
#include "sequencer/sequencer_list.h"

class Sequencer{
private:
    // Motion *temp_motion;
    double present_position[20];
    alfarobi::ServoController *temp_servo;
    std::string current_seq_name;
    // sequence sequences;
    ros::NodeHandle nh_;
    // alfarobi_motion::SequencerParams params_;
    std::vector<SequenceList> sequences_list_;
    // std::map<std::string, sequence*> sequences_map_;
    std::vector<std::string> sequence_names;

    ros::Publisher sequence_pub;
    ros::Subscriber apply_sub;
    ros::Subscriber web_button_sub;
    ros::Subscriber sequence_list_sub;
    ros::Subscriber torque_sub;
    ros::Subscriber motion_state_sub;

    double time_start;
    double time_now;
    bool is_playing = false;
    bool is_moving = false;
    bool in_action = false;
    std::string state_now;


public:
    Sequencer();
    ~Sequencer();

    void loadSequences();

    void apply(Sequence newSeq);
    void process(alfarobi::ServoController **serv);
    
    void loadParams(std::string name);
    void saveParams();

    void insert(double val, int index);

    std::string getCurrentName() {
        return current_seq_name;
    }
    void setCurrentName(std::string name) {
        current_seq_name = name;
    }

    void sequenceListCallback(const std_msgs::String::ConstPtr& msg);
    void applyCallback(const alfarobi_web_gui::SequencerArr::ConstPtr& arr);
    void webButtonCallback(const std_msgs::String::ConstPtr& msg);
    void torqueCallback(const alfarobi_web_gui::Torque::ConstPtr& torque);
    void motionStateCallback(const std_msgs::String::ConstPtr& msg);


    void play();
    
    void refresh();
    // void torqueDisable(int index);
    // void torqueEnable(int index);

    void write(alfarobi::joint_value *joints_);
    void readAll();
    void read(int id);
    void enable(int id);
    void disable(int id);
};
#endif