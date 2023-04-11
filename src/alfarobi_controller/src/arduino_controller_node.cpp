#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <robotis_math/robotis_linear_algebra.h>
#include <eigen_conversions/eigen_msg.h>

#include <QApplication>
#include <QMainWindow>

#include "controller.h"

int main(int argc, char **argv)
{
    
    QApplication a(argc, argv);
    Controller* controller = new Controller(argc, argv);

    QThread* thread = new QThread;

    QObject::connect(thread, &QThread::started, controller, &Controller::run);
    QObject::connect(controller, &Controller::finished, thread, &QThread::quit);
    QObject::connect(controller, &Controller::finished, controller, &QThread::deleteLater);
    QObject::connect(thread, &QThread::finished, thread, &QThread::deleteLater);
    
    thread->start();

    return a.exec();

}
