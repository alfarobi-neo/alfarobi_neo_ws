#ifndef MODEL_BASED_CONTROL_H_
#define MODEL_BASED_CONTROL_H_

#include "ros/ros.h"
#include <ros/time.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <iostream>
#include <cmath>
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <sstream>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <tf/tf.h>

#include "csvManager.h"

#define PI 3.14159265

using namespace Eigen;

namespace robotis_op
{
class modelBasedControl
{
public:
    modelBasedControl(std::string constr_modelParamPath_);
    // modelBasedControl();
    ~modelBasedControl();

    void initialize();
    int reset;
    void resetValue();

    bool m_start;
    double m_time_start, m_time;

    void resetServoValues();

    struct stateSpace
    {
        std::vector<std::vector<double>> A,B,C,D;       
        double Ts;   
    }sys_c,sys_d;

    std::vector<double> x_temp;
    std::vector<double> xsim_temp;
    std::vector<double> xhat_temp;
    std::vector<double> wholeBodyStates;
    std::vector<double> xhat;
    std::vector<double> xsim;
    std::vector<double> ysim;
    std::vector<double> ymeas;
    std::vector<double> yest;
    std::vector<double> y_temp;
    
    void stateSpaceSimulator(double u, std::vector<double> &x, std::vector<double> &y);
    // void wholeBodyStateObserver(double u, std::vector<double> &xhat, std::vector<double> &yh, std::vector<double> y);
    void wholeBodyStateObserver(double u, std::vector<double> &xhat, std::vector<std::vector<double>> kalmanK, std::vector<double> &yh, std::vector<double> y);
    double outputFeedback(double ref, std::vector<double> &x, std::vector<std::vector<double>> dlqrK, std::vector<std::vector<double>> dlqrKr);
    // double outputFeedback(double ref, std::vector<double> &x, double *controlEffort);

    struct gain
    {
        std::vector<std::vector<double>> Q,R;
        std::vector<std::vector<double>> K, Kr;
    }dlqr, kalman;

    void c2d(stateSpace *sys_c);
    std::vector<std::vector<double> > dlqrGain(stateSpace *sys_d, gain *dlqr);
    std::vector<std::vector<double> > feedforwardGain(stateSpace *sys_d, gain *dlqr);
    std::vector<std::vector<double> > feedforwardGain(stateSpace *sys_d, std::vector<std::vector<double>> K);//gain *dlqr);
    std::vector<std::vector<double> > kalmanGain(stateSpace *sys_d, gain *kalman);

    double COMrefX, COMrefY;
    double pitchRef, rollRef;

    bool tuneGain;
    bool testObserver;

    double GainIntegral, batas;

private:
    std::string modelParamPath_;
    void loadModelParam(std::string modelParamPath_);
    robotis_op::csvManager csv;


    double gyroThreshold;

};
}

#endif