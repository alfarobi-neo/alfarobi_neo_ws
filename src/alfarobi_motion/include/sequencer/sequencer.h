#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "sequencer/sequencer_list.h"
#include "alfarobi_web_gui/SequencerArr.h"
#include "std_msgs/String.h"

class Sequencer{
private:
    // sequence sequences;
    ros::NodeHandle nh_;
    // alfarobi_motion::SequencerParams params_;
    std::vector<SequenceList> sequences_list_;
    // std::map<std::string, sequence*> sequences_map_;
    std::vector<std::string> sequence_names;

    ros::Publisher sequence_pub;
    ros::Subscriber sequence_sub;


public:
    Sequencer();
    ~Sequencer();

    void loadSequences();

    void apply(Sequence newSeq);
    void process();
    // void torqueDisable();
    void loadParams(std::string name);
    void saveParams();

    void insert(double val, int index);

    std::string getName(int input);

    void sequenceCallBack(const std_msgs::String::ConstPtr& arr);
    // void testCB(const std_msgs::String::ConstPtr& msg);
};
#endif