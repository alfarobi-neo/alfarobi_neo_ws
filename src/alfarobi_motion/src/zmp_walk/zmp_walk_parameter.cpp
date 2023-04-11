#include <Eigen/Core>
#include "walk_zmp/zmp_math_basics.h"
#include "walk_zmp/zmp_walk_parameter.hpp"


robotis_op::ZMPParameter::ZMPParameter()
  :
    //----------------------------------------------
    //-- Stance and velocity limit values
    //----------------------------------------------
    stanceLimitX(Eigen::Vector2d(-0.10,0.10)),
    stanceLimitY(Eigen::Vector2d(0.07,0.20)),
    stanceLimitA(Eigen::Vector2d(0* degree_to_rad,30* degree_to_rad)),

    velDelta(Eigen::Vector3d(0.02,0.02,0.15)),

    velXHigh (0.06), // default value from awesomewlk.lua because nciht in the config
    velDeltaXHigh (0.01), // default value from awesomewlk.lua because nciht in the config

    // Toe / heel overlap checking values
    footSizeX (Eigen :: Vector2d (-0.02,0.02)), // default value from awesomewlk.lua because nciht in the config
    stanceLimitMarginY (0.04), // default value from awesomewlk.lua because nciht in the config

    //OP default stance width: 0.0375*2 = 0.075
    //Heel overlap At radian 0.15 at each foot = 0.05*sin(0.15)*2=0.015
    //Heel overlap At radian 0.30 at each foot = 0.05*sin(0.15)*2=0.030

    //----------------------------------------------
    //-- Stance parameters
    //---------------------------------------------
    bodyHeight(0.295),
    bodyTilt(0* degree_to_rad),// Wird vom hipPitch überschrieben
    footX(0.020),//Moves the target pose of the upper body forward
    footY(0.035),
    supportX(0),
    supportY(0.010),
    ankleSupportFaktor(1),

    qLArm (degree_to_rad * Eigen :: Vector3d (90, -120, -120)),//(90,2, -20)), // Here you have to be able to set for Wheatley,
    qRArm (degree_to_rad * Eigen :: Vector3d (90, -30, -120)),//(90, 2, -20)), // that he pulls his arms further apart
    //Hardness parameters
    hardnessSupport(1),
    hardnessSwing(0.75),
    hardnessArm(0.5),//0.3
    pDefault(-1), // Wird im Basemotionserver auf den Default der mx28config.yaml gesetzt

    //---------------------------------------------
    //-- Gait parameters
    //---------------------------------------------
    tStep(0.25),//Zeit eines Schrittes
    tZmp(0.165),//Zeit, bis der ZMP erreicht sein muss, nach beginn eines Schrittes
    stepHeight(0.045),
    phSingle(Eigen::Vector2d(0.1, 0.9)),
    zPhase(Eigen::Vector2d(0.1,0.9)), //Früher gleich = phSingle
    xPhase(Eigen::Vector2d(0.1,0.9)),
    bodyTilt_x_scaling(0.5),

    //--------------------------------------------
    //-- Compensation parameters
    //--------------------------------------------
    hipRollCompensation(5* degree_to_rad),
    ankleMod(Eigen::Vector2d(-1,0)*1* degree_to_rad),

    turnCompThreshold(0.1),
    turnComp(0.003), //Lean front when turning

    //Gyro stabilization parameters
    useGyro(true),
    #if 1  //Depends somehow on the Servopid
    ankleImuParamX(Eigen::Vector4d(0.5,1.5,
                                   1, 20)),
    ankleImuParamY(Eigen::Vector4d(0.5,0.7, 0, 25)),
    kneeImuParamX(Eigen::Vector4d(0.5,1.5,
                                  1, 20)),
    hipImuParamY(Eigen::Vector4d(0.5,1.5,
                                 1, 20)),
    armImuParamX(Eigen::Vector4d(0.5,10,20,45)),
// TODO: the Y balancing of the poor is off! evaluate if you do not want to do that by default
    //DISABLE Y BALANCING
    armImuParamY(Eigen::Vector4d(0.5,0.3, -20, 20)),
    //ENABLE Y BALANCING
    //walk.armImuParamY={0.3,10*gyroFactor, 20* degree_to_rad, 45* degree_to_rad},
    #else
    ankleImuParamX(Eigen::Vector4d(0.9,0.3*gyroFactor, 0, 25* conversion_inverse));
kneeImuParamX(Eigen::Vector4d(0.9,1.2*gyroFactor, 0, 25* conversion_inverse));
ankleImuParamY(Eigen::Vector4d(0.9,0.7*gyroFactor, 0, 25* conversion_inverse));
hipImuParamY(Eigen::Vector4d(0.9,0.3*gyroFactor, 0, 25* conversion_inverse));
armImuParamX(Eigen::Vector4d(0.3,10*gyroFactor, 20* conversion_inverse, 45* conversion_inverse));
armImuParamY(Eigen::Vector4d(0.3,10*gyroFactor, 20* conversion_inverse, 45* conversion_inverse));
#endif



//Support bias parameters to reduce backlash-based instability
velFastForward(0.05),
velFastTurn(0.15),

//walk.supportFront = 0.01; --Lean back when walking fast forward
supportFront(0.04), //Lean kind of dynamic front when walking fast forward
supportFront2(0.03), //Lean front when accelerating forward
supportBack(-0.02), //Lean back when walking backward,
supportSideX(-0.005), //Lean back when sidestepping
supportSideY(0.01), //Lean sideways when sidestepping
supportTurn(0.02), //Lean front when turning
standOffset(0),


frontComp(0.003), //Lean static front when walking faster then 0.04 (6 in unserer REchnung)
//Verschiebt wohl den Torso nach Vorne, wenn beschleunigung größer als 0.2
AccelComp(0.003), //Lean static front wenn accelerating which is highter than 0.02 (3 in unserer Rechnung

//Initial body swing
supportModYInitial(0.0), //standartwert aus awesomewlk.lua weil nciht in der config
default_belly_pitch(0.0),
default_belly_roll(0.0)
{}

robotis_op::ZMPParameter get_default_parameter(){
  robotis_op::ZMPParameter p = robotis_op::ZMPParameter();
  return p;
}

void robotis_op::ZMPParameter::setParameter(robotis_op::ZMPParameter param)
{
  this->zPhase = param.zPhase;
  this->tStep = param.tStep;
  this->supportX = param.supportX;
  this->footY = param.footY;
}
