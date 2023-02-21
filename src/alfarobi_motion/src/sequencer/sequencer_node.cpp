#include "sequencer/sequencer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "alfarobi_motion_node");

    Sequencer *test = new Sequencer();
    test->loadSequences();

    ros::Rate rate(2);

    int input;
    // while(ros::ok()) {
        
    // }
    std::cout <<"\nEnter : ";
    std::cin>>input;

    test->loadParams(test->getName(input-1));
}