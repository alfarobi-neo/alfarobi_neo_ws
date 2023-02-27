#ifndef SEQUENCER_LIST_H
#define SEQUENCER_LIST_H

#include "alfarobi_dxlsdk/joint_value.h"
#include "alfarobi_motion/alfarobi_motion.h"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <map>

class Sequence {
public:
    std::string name_;
    alfarobi::joint_value *joint_;
    Sequence* next;
    // Sequence* prev;
  
    // Default constructor
    Sequence()
    {
        std::cout<<"Sequence Constructor!\n";
        joint_ = new alfarobi::joint_value();
        next = NULL;
    }
  
    // Parameterised Constructor
    Sequence(alfarobi::joint_value *data, std::string name)
    {
        
        this->name_ = name;
        
        joint_ = new alfarobi::joint_value();
        for(int i=0; i<20; i++) {
            this->joint_->setVal(i, data->getVal(i));
            this->joint_->target_time[i] = data->target_time[i];
            this->joint_->pause_time[i] = data->pause_time[i];
        }
        
        this->next = NULL;
    }

    void insertVal(double val, int index);

    void insertName(std::string name);

    void insertTargetTime(double tt, int index);

    void insertPauseTime(double pt, int index);

    alfarobi::joint_value * getJoint() {
        return joint_;
    }
    std::string getName() {
        return name_;
    }
};
  
// Linked list class to
// implement a linked list.
class SequenceList {
    Sequence* head;
  
public:
    // Default constructor
    SequenceList() { head = NULL; }
  
    // Function to insert a
    // node at the end of the
    // linked list.
    void insertSequence(alfarobi::joint_value *data, std::string name);
  
    // Function to print the
    // linked list.
    void printList();
  
    // Function to delete the
    // node at given position
    void deleteSequence(std::string name);

    Sequence* getSeq() {
        return head;
    };
};
#endif