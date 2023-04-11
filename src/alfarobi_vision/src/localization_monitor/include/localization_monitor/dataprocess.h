#pragma once

#include <QtCore>

#include <ros/ros.h>

class DataProcess:public QThread {
    Q_OBJECT
private:
    ros::NodeHandle nh_;

    QMutex mutex_;
    QWaitCondition wait_cond_;
    bool running;
protected:
    void run();
public:

    void begin();
    void stop();
    DataProcess(QObject *parent=0);
    ~DataProcess();
};
