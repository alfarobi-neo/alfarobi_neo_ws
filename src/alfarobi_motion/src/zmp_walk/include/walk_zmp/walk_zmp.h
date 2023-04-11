#ifndef WALK_ZMP_H
#define WALK_ZMP_H

#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/thread.hpp>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "zmp_walking_module_msgs/ZMPWalkingParam.h"
#include "zmp_walking_module_msgs/GetZMPParam.h"
#include "zmp_walking_module_msgs/SetZMPParam.h"
#include <robot_state_publisher/robot_state_publisher.h>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "zmp_walk_parameter.hpp"
#include "Pose/pose.hpp"
#include "body.hpp"
#include "zmp_math_basics.h"
#include "zmp_team_darwin_kinematics.hpp"
#include "walk_zmp/kinematics.h"

#include "robotmodel.h"

#define G_CONST 9.81 //m/s^2
#define PI 3.14159265
#define WEIGHT 50 //kg*m/s^2 (Newton)
#define DT 0.016 //seconds

using namespace Eigen;

namespace robotis_op
{

class WalkZMP : public robotis_framework::MotionModule, public robotis_framework::Singleton<WalkZMP>
{
public:
  WalkZMP();
  virtual ~WalkZMP();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();
  void onModuleDisable();
  void onModuleEnable();

  /* ROS Topic Callback Functions */
  void walkingCommandCallback(const std_msgs::String::ConstPtr &msg);
  void walkingParameterCallback(const zmp_walking_module_msgs::ZMPWalkingParam::ConstPtr &msg);
  bool getWalkigParameterCallback(zmp_walking_module_msgs::GetZMPParam::Request &req,
                                  zmp_walking_module_msgs::GetZMPParam::Response &res);
  void publishStatusMsg(unsigned int type, std::string msg);
  void queueThread();

  //IMU
  void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

  void start();
  void set_velocity(double tvx, double vy, double va);
  double c_vx_manip, c_vy_manip, c_vphi_manip; //config velocity manipulation
  void loadConfig();
  void saveConfig(const std::string &path);

  zmp_walking_module_msgs::ZMPWalkingParam walking_param_;
  std::map<std::string, robotis_framework::DynamixelState *> current_;

  Eigen::Vector3d m_uTorso, m_uLeft,m_uRight;
  Eigen::Vector3d m_velCurrent, m_velCommand, m_velDiff;

  struct ZMP_coeff{
    //ZMP exponential coefficients:
    double aXP=0, aXN=0, aYP=0, aYN=0, m1X=0, m2X=0, m1Y=0, m2Y=0;
  } m_ZMP_coeff;

  double m_bodyTilt;

  //Gyro stabilization variables
  double m_kneeShift;
  Eigen::Vector2d m_ankleShift, m_hipShift, m_armShift;

  bool m_active, m_started;
  bool m_newStep;
  int m_stopRequest;
  int m_initial_step;
  int m_supportLeg;
  double m_shiftFactor;

  ZMPParameter m_parameter;
  Eigen::Vector2d m_ankleMod;
  boost::posix_time::ptime m_tLastStep;
  Eigen::Vector3d m_uLeft1, m_uLeft2, m_uRight1, m_uRight2, m_uTorso1, m_uTorso2;
  Eigen::Vector3d m_uTorsoActual;
  Eigen::Vector3d m_uSupport;
  Eigen::Vector3d m_imuGyr;
  Eigen::Vector3d m_imuOri;
//    Body m_body;
  std::string m_robottype;
  double m_toeTipCompensation;

private:
  enum
  {
    WalkingDisable = 0,
    WalkingEnable = 1,
    WalkingInitPose = 2,
    WalkingReady = 3
  };

  const bool DEBUG_PRINT;


  robotis_op::OP3KinematicsDynamics* op3_kd_;
  int control_cycle_msec_;
  std::string param_path_;
  boost::thread queue_thread_;
  boost::mutex publish_mutex_;

  ros::Publisher robot_pose_pub_;
  ros::Publisher status_msg_pub_;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher odom_pub_;
  ros::Publisher kick_status_pub_;
  ros::Publisher walking_status_pub_;

  //offset
  std::string joint[12], offset_;
  double offset[12];

  Eigen::MatrixXd calc_joint_tra_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd current_position_;
  Eigen::MatrixXd init_position_;
  Eigen::MatrixXi joint_axis_direction_;
  std::map<std::string, int> joint_table_;

  void wholeBodyCOM();
  double mass;
  Eigen::MatrixXd mc;
  Eigen::MatrixXd COM;

  int walking_state_;
  int cycleNumber;
  bool debug_print_;
  int init_pose_count_;
  ros::Time last_t;

  bool ctrl_running_;
  bool real_running_;
  double time_;
  bool useGyro;

  //stepController
  void StepController(bool kanan);

  //Support
  double IndexSupport;
  double imu_y, default_imu_y;
  double imu_x;


  //feedback Time Variable
  bool HIP = true;
  double TIME_;
  double TIME_START_;
  double TIME_START;
  double TIME_PAUSE;
  bool PAUSE = true;
  bool PLAY = true;
  bool startFeedback;
  double startTimeFeedback;
  double timeFeedback;
  double current_HIP_PITCH;
  double current_R_HIP_PITCH;
  double current_L_HIP_PITCH;
  double THETA_ERROR_;
  double A,B,C;
  double trap_func;

  //feedback Parameter
  double TH1;
  double TH2;
  double TH3;
  double Torque_Hip_Max;
  double Torque_Ankle_Max;
  double Acc;
  double Angle_Hip_Max;

  //feedback Error Variable
  int DIR_;
  double Z0;
  double OMEGA;
  double X_ERROR;
  double X_ERROR_PRE;
  double X_ERROR_DT;
  double PRE_THETA_ERROR;
  double THETA_ERROR_DT;
  double Angle_Ankle_Pitch;
  double Angle_Hip_Pitch = 0;
  double Angle_Hip_Pitch_Zero;
  Matrix<double, 18, 1> feedback;
  double theta1, thetaC, Xc, Zc, Xh, Zh, normPitchError;
  double normalizedIMU();

  double imu_vx;
  double imu_vy;
  double Angle_Ankle_Roll;
  double Angle_Hip_Roll;
  double current_HIP_ROLL;
  double current_R_HIP_ROLL;
  double current_L_HIP_ROLL;

  //ferdback Gain
  double Gain_Angle_Pitch;
  double Gain_Velocity_Pitch;

  //fall
  bool fall = true;
  double TIME_START_FALL;


  std::string robot_name;

  void calculateStepGoal();
  Matrix<double, 18, 1> advanceInStep(double ph, double phSingle);
  Matrix<double, 18, 1> getJoints;
  Eigen::Matrix<double, 6, 1> pLLeg, pRLeg, pTorso;
  Matrix<double, 18, 1> current_joint_pos;
  void update_velocity();
  void stance_reset();

  Eigen::Vector2d foot_phase(double ph, double ph_single);
  double mod_angle(double a);
  Eigen::Matrix<double, 6, 1> motion_arms();
  Eigen::Matrix<double, 3, 1> set_rarm_joint(const Eigen::Vector3d& qLArmActual);
  Eigen::Matrix<double, 3, 1> set_larm_joint(const Eigen::Vector3d& qLArmActual);
  void motion_legs(Eigen::Matrix<double, 12, 1>& qLegs, double phSingle);
  void setValue(); //set servo value
  void applyGyroStabilization(Eigen::Matrix<double, 12, 1>& qLegs);
  void pushRecovery(double phSingle);

  Eigen::Vector3d pose_global(const Eigen::Vector3d& pRelative, const Eigen::Vector3d& pose);
  Eigen::Vector3d pose_relative(const Eigen::Vector3d& uLeft1, const Eigen::Vector3d& uTorso1);

  double procFunc(double q, double s, double d);
  Eigen::Vector3d se2_interpolate(double t, const Eigen::Vector3d& u1, const Eigen::Vector3d& u2);
  Eigen::Vector3d foot_trajectory(double phSingle, const Eigen::Vector3d& u1, const Eigen::Vector3d& u2);

  Eigen::Vector3d step_left_destination(const Eigen::Vector3d& vel, const Eigen::Vector3d& uLeft, const Eigen::Vector3d& uRight);
  Eigen::Vector3d step_right_destination(const Eigen::Vector3d& vel, const Eigen::Vector3d& uLeft, const Eigen::Vector3d& uRight);
  Eigen::Vector3d step_torso(const Eigen::Vector3d& uLeft, const Eigen::Vector3d& uRight, double shiftFactor);

  Eigen::Vector3d zmp_com(double ph, const Eigen::Vector3d& m_uSupport,  const ZMP_coeff& zmp_coeff);
  Eigen::Vector2d zmp_solve(double zs, double z1, double z2, double x1, double x2);

  //Kicking
  Kinematics k;
  void kick(bool kanan);
  bool kanan;
  //  double current_T;
  double start_T; double deltaT;
  double current_TRetract; double tr;
  double current_TKick; double tk;
  bool startRetract; bool finishedRetract;
  bool startKick; bool finishedKick;
  double R_X, R_Y, R_Z, R_R, R_P, R_A;
  double L_X, L_Y, L_Z, L_R, L_P, L_A;
  double iR_X, iR_Y, iR_Z, iR_R, iR_P, iR_A;
  double iL_X, iL_Y, iL_Z, iL_R, iL_P, iL_A;
  double wR_X, wR_Y, wR_Z, wR_R, wR_P, wR_A;
  double wL_X, wL_Y, wL_Z, wL_R, wL_P, wL_A;

  // Boleh diubah
  //  Retract kick control point
  double P0x,   P0y,    P0z,//Titik Awal
         P1x,   P1y,    P1z,
         P2x,   P2y,    P2z,
         P3x,   P3y,    P3z;//Titik Akhir (Via Point)

  // Forward kick control point
  double  Q0x, Q0y, Q0z,//Titik Awal (Via Point)
          Q1x, Q1y, Q1z,
          Q2x, Q2y, Q2z,
          Q3x, Q3y, Q3z;//Titik Akhir


  bool m_StartKickCycle;
  bool m_StartKicking;
  bool m_FinishedKicking;
  bool firstTimeKick;

  //Position of Ball
  double m_ballPosX, m_ballPosY, m_ballPosZ;

  //Position of Goal
  double m_goalPosX, m_goalPosY, m_goalPosZ;
  double m_power;

  //Parameter Waktu
  double m_TRetract;   //Retract Period (seconds)
  double m_TKick;    //Kick Period (seconds)

  void inverseKinematic();
  void forwardKinematic();

  void printCurrentAngle();
  void saveCurrentPose();
  void saveWalkReadyPose();
  void saveLastKickPose();
  Eigen::Matrix<double, 6, 1> w_pLLeg, w_pTorso, w_pRLeg;
  Eigen::Matrix<double, 6, 1> i_pLLeg, i_pTorso, i_pRLeg;
   Eigen::Matrix<double, 6, 1> k_pLLeg, k_pTorso, k_pRLeg;

  Eigen::Affine3d BASE;
  Eigen::Affine3d R_HIP, R_KNEE, R_ANKLE, R_FOOT, m_R_FOOT;
  Eigen::Affine3d L_HIP, L_KNEE, L_ANKLE, L_FOOT, m_L_FOOT;
  Eigen::Vector3d L_FOOT_WORLD, R_FOOT_WORLD;
  Eigen::Vector3d L_ROT_WORLD, R_ROT_WORLD;

  // Eigen::VectorXd L_FOOT_WORLD, R_FOOT_WORLD;
  Eigen::VectorXd LFootWorld, RFootWorld;

  void forwardGoblok();

  //model
  double iTorsi(bool pitch, double angle, double velocityAngle, double integrator);
  double integrator(double state, double ref);
  double feedbackPID(bool pitch, double setpoint, double measured, double derivative);
  void resetFeedback();
  double setPointPitch, setPointRoll;
  double Pout, Iout, Dout, _integral, _pre_error, output;
  double feedbackPitch, feedbackRoll;


  void risingUp(bool kanan);
  void coolingDown(bool kanan);
  double hipToCOM;
  const double totalMass=5;
  double sidePercent, placeCOM,  sidePercentKick, sidePercentCool;
  double centerToHip, pendulumLength;
  double oTorquePitch, oTorqueRoll;
  double dAnglePitch, dAngleRoll;
  double incAnglePitch, incAngleRoll;
  double anglePitchGain, velocityPitchGain;
  double angleRollGain, velocityRollGain;
  double integratorGain;
  double highPassGyroX, highPassGyroY;
  bool flagRise, flagDown, flagSlide;

  //Rising Up
  bool firstTime;
  bool startRising, finishedRising;
  double TGeser, THalt, TRising, dTRising, timeRising, TGeserDown;

  //CooolingDown
  bool firstTimeDown;
  bool startDown, finishedDown;
  double TDown, dTDown, timeDown;

  double c_ballPosX, c_ballPosY, c_ballPosZ;
  double c_goalPosX, c_goalPosY, c_goalPosZ;
  double c_TRetract, c_TKick, c_TRising, c_TDown, c_TGeserDown, c_power;

  void kickStatus(const std::string &command);
  void walkingStatus(const std::string &command);
  void KickRequest(const std_msgs::String::ConstPtr &msg);
  bool walkKick;
  double tStop;

  ///////////////////////////////////////////ODOMETRY????????????????????????????????????????????????
  double hip_to_base_pitch, hip_to_base_roll, hip_to_base, camera_to_base, camera_to_base_r;
  ///////////////////////////////////////////ODOMETRY????????????????????????????????????????????????

};

}

#endif // WALK_ZMP_H
