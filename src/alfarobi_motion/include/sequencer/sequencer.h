#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "sequencer/sequencer_list.h"
#include "alfarobi_motion/alfarobi_motion.h"
#include "alfarobi_web_gui/SequencerArr.h"
#include "std_msgs/String.h"

class Sequencer{
private:
    Motion *temp_motion;
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

    double time_start;
    double time_now;
    bool is_moving = false;


public:
    Sequencer();
    ~Sequencer();

    void loadSequences();

    void apply(Sequence newSeq);
    void process();
    
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


    void play();
    void torqueDisable(int index);
    void torqueEnable(int index);
};
#endif