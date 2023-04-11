#ifndef DENSIS_H_
#define DENSIS_H_

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
// #include "densis_msgs/densis.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <tf/tf.h>

#define PI 3.14159265

using namespace Eigen;

namespace robotis_op
{
class densis
{
public:
    // densis();
    densis(std::string densisParamPath_);
    ~densis();

    void initialize();
    int reset;
    void resetValue();

    bool m_start;
    double m_time_start, m_time;
    bool densisMode;
    double inputSignal;

    void resetServoValues();

    std::vector<double> ymeas;
    
    void densisPublish();
    // densis_msgs::densis densisMsgs;
    double generatePseudonoise(double t, double min, double max, double resolution, double holdTime);
    double densisInput(double m_delta_time);

private:
    std::string densis_path_,physicalParam_path_;

    void loadPhysicalParam();
      //Physical Parameters
    struct physical{
        double L;
        double I_xx;
        double I_yy;
    }phDSP,phLSSP,phRSSP;
    double m;
    double dT;
    double B;
    double F;
    double Is;

    void loadDensisParam();
    enum
    {
        densisDSP = 0,
        densisRAnkPitch = 1,
        densisLAnkPitch = 2,
        densisRAnkRoll = 3,
        densisLAnkRoll = 4
    };
    //Densis variables
    int inputType;
    double inputAmp;
    double freq;
    double time_densis;
    // double generatePseudonoise(double t, double min, double max, double resolution, double holdTime);
    double sineSweep(double t, double minAmp, double maxAmp, double fmin, double fmax, double sweepPeriod);
    double pseudonoise2PRBS(double pseudonoise);
    double pulsedRich(double t, double min, double max, double pseudonoise, double onTime, double offTime);
    double randomPulsedRich(double t, double min, double max, double pseudonoise, double onTime, double offTime);
    double pseudonoise, minPSN, maxPSN, resolutionPSN, holdTimePSN;
    double signalPSN;
    double prevSeconds;
    bool holdValue;
    double PRBS;

    // pulsed rich
    bool pulseState;
    double minPulsedRich, maxPulsedRich, onTime, offTime;
    double signalPulse, signalPulsedRich;
    double prevSquareSeconds;

    // random pulsed rich
    double minRandomPulsedRich, maxRandomPulsedRich;
    double signalRandomPulse, signalRandomPulsedRich;
    
    // sine sweep
    double fsweep, minSine, maxSine, fmin, fmax, sweepPeriod;
    double tSweepStart; 
    bool sweepState;

    //Kicking Mode
    bool liftFoot;
    bool holdPose;
};
}

#endif