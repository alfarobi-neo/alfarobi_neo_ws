/*
 *   Kinematics.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include <quintic_walk/kinematics.h>

const double Kinematics::HIP_OFFSET_X = 0.02134;
const double Kinematics::LEG_SIDE_OFFSET = 0.055;//0.038525; //0.038;//m //50; //37.0; //mm
const double Kinematics::HIP_OFFSET_Z = 0.090436;//0.077;//0.096;//m
const double Kinematics::THIGH_LENGTH = 0.1168; //m //119;// 118; //116.83; //mm
const double Kinematics::CALF_LENGTH = 0.1168; //m //115;//117; //116.80; //mm
const double Kinematics::ANKLE_LENGTH = 0.0355;//0.0535;//m //38.5;//37;//35.5; //mm
const double Kinematics::LEG_LENGTH = 0.2691;//0.3641; //m //272.5; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)
const double Kinematics::BODY_LENGTH = 0.231;

Kinematics* Kinematics::m_UniqueInstance = new Kinematics();

Kinematics::Kinematics()
{
}

Kinematics::~Kinematics()
{
}
