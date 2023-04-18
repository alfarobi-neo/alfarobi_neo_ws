#include "vision_monitor/mainwindow.h"
#include <QApplication>

#include "signal.h"

void sigHandler(int sig){
    (void)sig;
    QApplication::quit();
}

int main(int argc, char *argv[]){

    ros::init(argc,argv,"vision_monitor_node",ros::init_options::NoSigintHandler);

    signal(SIGINT,sigHandler);

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
