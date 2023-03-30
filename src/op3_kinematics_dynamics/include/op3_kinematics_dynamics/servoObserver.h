#ifndef SERVOOBSERVER_H_
#define SERVOOBSERVER_H_

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

using namespace Eigen;

namespace robotis_op
{
class servoObserver
{
public:
    servoObserver();
    ~servoObserver();
	
    std::string config_path_;

    void initialize();
    int reset;
    void resetValue();

    bool m_start;
    double m_time_start, m_time;
    double inputSignal;

    struct servo {
        double torqueIn; //control effort
        double velocityNow; //present velocity
        double positionIK; //joints from inverse kinematics
        double positionNow; //present position
        double torqueEst; //estimated torque
        double refPosFromTrq; //position from torque control effort mode
        double refPosFromPos; //position from angle control effort mode
        std::vector<double> servoStates;
        std::string name;
        double servoOutputAngle;
        double Verr;
    }r_ank_pitch,l_ank_pitch,r_ank_roll,l_ank_roll;

    void convertTrq2Pos(servo *s);
    void convertTrq2PosZTransform();
    void convertTrq2PosDiscreteStateSpace();
    void convertTrq2PosContinuousStateSpace();
    void servoStateObserver(double u, std::vector<double> &x, double &y);
    void torqueObserver(servo *s);
    void servoTorqueFeedback(servo *s);
    double Kposition, Kvelocity, KrTrq;
    void resetServoValues();

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
    const double pwmToVoltage = (double)(12)/(double)(511);
    const double angleToPWM = (double)(4096/(2*M_PI));
    const double rpmToRad = (double)(2*M_PI/60);
    struct servoParams{
        double Kp;
        double Kt;
        double N;
        double R;
        double nominalVoltage;
        double omegaNoLoad;
        double iNoLoad;
        double Jm;
        double stallTorqueMotor;
        double stallTorqueServo;
        double eta;
        double Bm;
        double BmGz;
        double JmGz;
        double KLoadTorque;
        double speedToEMF;
        double a_11, a_12, a_21, a_22, b_1, b_2, c_1, c_2;  
    }MX64;

    Eigen::VectorXd x_next, x_prev, y_ss, u_ss;
    std::vector<double> x_temp;
    
    void loadConfig();

private:
    
};
}

#endif