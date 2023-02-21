#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "alfarobi_dxlsdk/joint_value.h"
// #include "sequencer/SequencerParamsConfig.h"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>

struct sequence{
    alfarobi::joint_value joint_val_;
    double time;
    // node* next;
};

class Sequencer{
private:
    // sequence sequences;
    ros::NodeHandle nh_;
    // alfarobi_motion::SequencerParams params_;
    std::vector<sequence*> sequences_;
    std::vector<std::string> sequence_names;


public:
    Sequencer();
    ~Sequencer();

    void loadSequences();

    void apply();
    // void torqueDisable();
    void loadParams(std::string seq);
    void saveParams();

    std::string getName(int input);
};
#endif