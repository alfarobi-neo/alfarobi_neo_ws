#include "localization_monitor/dataprocess.h"

DataProcess::DataProcess(QObject *parent):QThread(parent),
    nh_(ros::this_node::getName()){

    running=false;
}

DataProcess::~DataProcess(){
    mutex_.lock();
    running=false;
    wait_cond_.wakeOne();
    mutex_.unlock();
    this->wait();
}

void DataProcess::run(){
    mutex_.lock();
    ros::Rate loop_rate(30);
    while(running && ros::ok()){
        ros::spinOnce();

        loop_rate.sleep();
//        this->msleep(1000/30);
    }
    mutex_.unlock();
}

void DataProcess::begin(){
    running=true;
    if(!this->isRunning()){
        this->start(LowPriority);
    }
}

void DataProcess::stop(){
    running=false;
}
