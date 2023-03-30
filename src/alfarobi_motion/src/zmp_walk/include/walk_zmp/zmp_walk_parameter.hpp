#ifndef _ZMP_WALK_PARAMETER_H_
#define _ZMP_WALK_PARAMETER_H_

#include <Eigen/Core>

#include <boost/date_time/posix_time/posix_time.hpp>

static const double gyroFactor = 2000.0 / 32768.0; //0.273* degree_to_rad * 300 / 1024; //dps to rad/s conversion

namespace robotis_op{

struct ZMPParameter{
// Walk Parameters
// Stance and velocity limit values
Eigen::Vector2d stanceLimitX;
Eigen::Vector2d stanceLimitY;
Eigen::Vector2d stanceLimitA;
Eigen::Vector3d velDelta;

double velXHigh;
double velDeltaXHigh;

//Toe/heel overlap checking values
Eigen::Vector2d footSizeX;
double stanceLimitMarginY;

//Stance parameters
double bodyHeight;
double bodyTilt;
double footX;
double footY;
double supportX;
double supportY;
Eigen::Vector3d qLArm;
Eigen::Vector3d qRArm;

//Hardness parameters
double hardnessSupport;
double hardnessSwing;

double hardnessArm;
int pDefault;

//Gait parameters
double tStep;
double tZmp;
double stepHeight;
Eigen::Vector2d phSingle;

//Compensation parameters
double hipRollCompensation;
Eigen::Vector2d ankleMod;
double turnCompThreshold;
double turnComp;

//Gyro stabilization parameters
bool useGyro;
Eigen::Vector4d ankleImuParamX;
Eigen::Vector4d ankleImuParamY;
Eigen::Vector4d kneeImuParamX;
Eigen::Vector4d hipImuParamY;
Eigen::Vector4d armImuParamX;
Eigen::Vector4d armImuParamY;

//Support bias parameters to reduce backlash-based instability
double velFastForward;
double velFastTurn;
double supportFront;
double supportFront2;
double supportBack;
double supportSideX;
double supportSideY;
double supportTurn;
double ankleSupportFaktor;
double standOffset;

double frontComp;
double AccelComp;

double default_belly_pitch, default_belly_roll;

//Initial body swing
double supportModYInitial;

Eigen::Vector2d zPhase;
Eigen::Vector2d xPhase;
double bodyTilt_x_scaling;

EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ZMPParameter();

void setParameter(robotis_op::ZMPParameter param);
//robotis_op::ZMPParameter loadParameter(robotis_op::ZMPParameter param);

};

}//namespasce

robotis_op::ZMPParameter get_default_parameter();

#endif
