/*
 *   Kinematics.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

//#include "matrix.h"
//#include "JointData.h"

class Kinematics
{
private:
    static Kinematics* m_UniqueInstance;
protected:

public:
    static const double HIP_OFFSET_X;
    static const double LEG_SIDE_OFFSET; 
    static const double HIP_OFFSET_Z;
    static const double THIGH_LENGTH; 
    static const double CALF_LENGTH; 
    static const double ANKLE_LENGTH; 
    static const double LEG_LENGTH; //(THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)
    static const double BODY_LENGTH;

    Kinematics();

    ~Kinematics();

    static Kinematics* GetInstance()			{ return m_UniqueInstance; }
};

#endif
