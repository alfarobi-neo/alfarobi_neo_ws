#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "sequencer/sequencer_list.h"

class Sequencer{
private:
    // sequence sequences;
    ros::NodeHandle nh_;
    // alfarobi_motion::SequencerParams params_;
    std::vector<SequenceList> sequences_list_;
    // std::map<std::string, sequence*> sequences_map_;
    std::vector<std::string> sequence_names;


public:
    Sequencer();
    ~Sequencer();

    void loadSequences();

    void apply(Sequence newSeq);
    // void torqueDisable();
    void loadParams();
    void saveParams();

    void insert(double val, int index);

    std::string getName(int input);
};
#endif