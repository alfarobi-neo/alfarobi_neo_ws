#include <walk_zmp/walk_zmp.h>
#define troubleshootLift 0
#define troubleshootKick 1

using namespace Eigen;

namespace robotis_op
{

WalkZMP::WalkZMP()
  : control_cycle_msec_(8),
    debug_print_(false),
    m_robottype("Hambot"),
    DEBUG_PRINT(false)
{
    enable_ = false;
    module_name_ = "walk_zmp";
    control_mode_ = robotis_framework::PositionControl;

    init_pose_count_ = 0;
    walking_state_ = WalkingInitPose;

    op3_kd_ = new robotis_op::OP3KinematicsDynamics(robotis_op::WholeBody);

    // result
    result_["r_hip_yaw"] = new robotis_framework::DynamixelState();
    result_["r_hip_roll"] = new robotis_framework::DynamixelState();
    result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
    result_["r_knee"] = new robotis_framework::DynamixelState();
    result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
    result_["r_ank_roll"] = new robotis_framework::DynamixelState();

    result_["l_hip_yaw"] = new robotis_framework::DynamixelState();
    result_["l_hip_roll"] = new robotis_framework::DynamixelState();
    result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
    result_["l_knee"] = new robotis_framework::DynamixelState();
    result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
    result_["l_ank_roll"] = new robotis_framework::DynamixelState();

    result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
    result_["r_sho_roll"] = new robotis_framework::DynamixelState();
    result_["r_el"] = new robotis_framework::DynamixelState();

    result_["l_sho_pitch"] = new robotis_framework::DynamixelState();
    result_["l_sho_roll"] = new robotis_framework::DynamixelState();
    result_["l_el"] = new robotis_framework::DynamixelState();


    // joint table
    joint_table_["r_hip_yaw"] = 0;
    joint_table_["r_hip_roll"] = 1;
    joint_table_["r_hip_pitch"] = 2;
    joint_table_["r_knee"] = 3;
    joint_table_["r_ank_pitch"] = 4;
    joint_table_["r_ank_roll"] = 5;

    joint_table_["l_hip_yaw"] = 6;
    joint_table_["l_hip_roll"] = 7;
    joint_table_["l_hip_pitch"] = 8;
    joint_table_["l_knee"] = 9;
    joint_table_["l_ank_pitch"] = 10;
    joint_table_["l_ank_roll"] = 11;

    joint_table_["r_sho_pitch"] = 12;
    joint_table_["r_sho_roll"] = 13;
    joint_table_["r_el"] = 14;

    joint_table_["l_sho_pitch"] = 15;
    joint_table_["l_sho_roll"] = 16;
    joint_table_["l_el"] = 17;

    //offset
    joint[0]  = "r_hip_yaw";
    joint[1]  = "r_hip_roll";
    joint[2]  = "r_hip_pitch";
    joint[3]  = "r_knee";
    joint[4]  = "r_ank_pitch";
    joint[5]  = "r_ank_roll";
    joint[6]  = "l_hip_yaw";
    joint[7]  = "l_hip_roll";
    joint[8]  = "l_hip_pitch";
    joint[9]  = "l_knee";
    joint[10] = "l_ank_pitch";
    joint[11] = "l_ank_roll";

    target_position_ = Eigen::MatrixXd::Zero(1, result_.size());
    current_position_ = Eigen::MatrixXd::Zero(1, result_.size());
    init_position_ = Eigen::MatrixXd::Zero(1, result_.size());
    joint_axis_direction_ = Eigen::MatrixXi::Zero(1, result_.size());

    BASE     = Affine3d(Translation3d(Vector3d(0,0,0)));
    R_HIP    = Affine3d(Translation3d(Vector3d(0,-k.LEG_SIDE_OFFSET,-k.HIP_OFFSET_Z)));
    R_KNEE   = Affine3d(Translation3d(Vector3d(0,0,-k.THIGH_LENGTH)));
    R_ANKLE  = Affine3d(Translation3d(Vector3d(0,0,-k.CALF_LENGTH)));
    R_FOOT   = Affine3d(Translation3d(Vector3d(0,0,-k.ANKLE_LENGTH)));
    m_R_FOOT = Affine3d(Translation3d(Vector3d(0,0,0)));
    L_HIP    = Affine3d(Translation3d(Vector3d(0,k.LEG_SIDE_OFFSET,-k.HIP_OFFSET_Z)));
    L_KNEE   = Affine3d(Translation3d(Vector3d(0,0,-k.THIGH_LENGTH)));
    L_ANKLE  = Affine3d(Translation3d(Vector3d(0,0,-k.CALF_LENGTH)));
    L_FOOT   = Affine3d(Translation3d(Vector3d(0,0,-k.ANKLE_LENGTH)));
    m_L_FOOT = Affine3d(Translation3d(Vector3d(0,0,0)));

    // VectorXd L_FOOT_WORLD(4);
    // VectorXd R_FOOT_WORLD(4);

    VectorXd LFootWorld(4);
    VectorXd RFootWorld(4);

    useGyro = false;
}

WalkZMP::~WalkZMP()
{
  queue_thread_.join();
}

inline double time_to_double(const boost::posix_time::time_duration& t) {
  long l = t.total_nanoseconds();
  assert(((double)l )/ e9 == ((double)l )/ e9);
  assert(((double)l )/ e9 != 0);
  return ((double)l )/ e9;
}

void WalkZMP::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = boost::thread(boost::bind(&WalkZMP::queueThread, this));
  control_cycle_msec_ = control_cycle_msec;
  ros::Time::init();

  PLAY = true;
  ctrl_running_ = false;
  real_running_ = false;
  time_ = 0;

  //                     R_HIP_YAW,   R_HIP_ROLL, R_HIP_PITCH, R_KNEE,      R_ANKLE_PITCH, R_ANKLE_ROLL,
  //                     L_HIP_YAW,   L_HIP_ROLL, L_HIP_PITCH, L_KNEE,      L_ANKLE_PITCH, L_ANKLE_ROLL,
  //                     R_ARM_PITCH, R_ARM_ROLL, R_ARM_ELBOW, L_ARM_PITCH, L_ARM_ROLL,    L_ARM_ELBOW
  joint_axis_direction_ <<    -1,        -1,            1,          1,          -1,            1,
                              -1,        -1,           -1,         -1,           1,            1,
                              -1,         1,           -1,          1,          -1,            1;
  init_position_        <<    0.0,       0.0,          0.0,        0.0,         0.0,          0.0,
                              0.0,       0.0,          0.0,        0.0,         0.0,          0.0,
                              0.0,       0.0,          0.0,        0.0,         0.0,          0.0;
  init_position_ *= DEGREE2RADIAN;

  ros::NodeHandle ros_node;

  //asli bit bots
  m_uSupport = m_uTorso1 = m_uTorso2 =m_uTorso = Vector3d(walking_param_./*foot_x*/supp_x/2.0, 0, 0);
  m_uLeft1 = m_uLeft2 = m_uLeft = Vector3d(0, (walking_param_.foot_y), 0);
  m_uRight1 = m_uRight2 = m_uRight = Vector3d(0, -(walking_param_.foot_y), 0);

  m_velCurrent = Vector3d(0, 0, 0);
  m_velCommand = Vector3d(0, 0, 0);
  m_velDiff = Vector3d(0, 0, 0);

  //Gyro stabilization variables
  m_ankleShift = Vector2d(0, 0);
  m_kneeShift = 0;
  m_hipShift = Vector2d(0,0);
  m_armShift = Vector2d(0, 0);

  m_active = false;
  m_started = false;
  m_newStep = true;
  m_tLastStep = boost::posix_time::microsec_clock::local_time();

  m_stopRequest = 0;

  m_initial_step = 1;

  m_toeTipCompensation = 0;

  // Init Gyro to zero
  m_imuGyr = Vector3d::Zero();

  m_StartKickCycle = false;
  startRising = false;
  finishedRising = false;
  firstTime = false;

  //Kicking
  m_StartKicking = false;
  deltaT = 0;
  firstTimeKick = false;
  finishedKick = false;

  //CoolingDown
  startDown = false;
  finishedDown = false;
  firstTimeDown = false;


  ROS_DEBUG("Motion: Walk entry");

  param_path_ = ros::package::getPath("op3_manager") + "/config/GlobalConfig.yaml";
  offset_ = ros::package::getPath("op3_manager") + "/config/offset.yaml";

  stance_reset();

  loadConfig();

  last_t = ros::Time::now();

  this->stop();
}

void WalkZMP::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);
  robot_pose_pub_ = ros_node.advertise<geometry_msgs::Pose>("robotis/base_pose", 1);
  odom_pub_ = ros_node.advertise<nav_msgs::Odometry>("robotis/odometry", 50);
  kick_status_pub_ = ros_node.advertise<std_msgs::String>("/robotis/kicking/status",0);
  walking_status_pub_ = ros_node.advertise<std_msgs::String>("/robotis/walking/status",0);

  /* ROS Service Callback Functions */
  ros::ServiceServer get_walking_param_server = ros_node.advertiseService("/robotis/walking/zmp_get_params",
                                                                          &WalkZMP::getWalkigParameterCallback,
                                                                          this);
  /* sensor topic subscribe */
  ros::Subscriber walking_command_sub = ros_node.subscribe("/robotis/walking/zmp_command", 0,
                                                           &WalkZMP::walkingCommandCallback, this);
  ros::Subscriber walking_param_sub = ros_node.subscribe("/robotis/walking/zmp_set_params", 0,
                                                         &WalkZMP::walkingParameterCallback, this);
  // ros::Subscriber imu_sub = ros_node.subscribe("/alfarobi/imu", 0, &WalkZMP::ImuCallback, this);
  ros::Subscriber imu_sub = ros_node.subscribe("/arduino_controller/imu", 0, &WalkZMP::ImuCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

bool WalkZMP::isRunning()
{
  walkingStatus("otherModule");
  return real_running_ || (walking_state_ == WalkingInitPose);
}

void WalkZMP::start() {
  m_stopRequest = 0;
  //  walkingStatus("walkingStart"); //FAIL
  if(!m_active) {
    m_active = true;
    m_started = false;
    m_newStep = true;
    m_tLastStep = boost::posix_time::microsec_clock::local_time();
    m_initial_step = 1;
  }
}

void WalkZMP::stop() {
  //Always stops with feet together (which helps transition)
  m_stopRequest = fmax(1,m_stopRequest);
  m_active = false;
  m_started = true;
  m_newStep = false;
  // walkingStatus("disable"); //FAIL
  stance_reset();
}

void WalkZMP::stance_reset() {

  //ROS_DEBUG(2, "Stance Resetted");
  // walkingStatus("disable"); //FAIL
  m_StartKickCycle = false;
  startRising = false;

  m_uTorso1 = m_uTorso2 =m_uTorso = Vector3d(walking_param_./*foot_x*/supp_x/2.0, 0, 0);
  m_uLeft1 = m_uLeft2 = m_uLeft = Vector3d(0, (walking_param_.foot_y), 0);
  m_uRight1 = m_uRight2 = m_uRight = Vector3d(0, -(walking_param_.foot_y), 0);

  m_newStep = true;
  m_uSupport = m_uTorso;
  m_tLastStep = boost::posix_time::microsec_clock::local_time();
}

void WalkZMP::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Walking";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

void WalkZMP::walkingCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data == "start")
    start();
  else if (msg->data == "stop")
    stop();
  else if (msg->data == "reset")
    set_velocity(0,0,0);
  else if (msg->data == "save")
    saveConfig(param_path_);
  else if (msg->data == "kick_start")
  {
      stop();
      //////////std::cout<<"TENDANG CUUUUY!"<<std::endl;
      m_StartKickCycle = true;
  }
  else if (msg->data == "kick_reset")
  {
      //////////std::cout<<"RELOAD RELOAD!"<<std::endl;
      m_StartKickCycle = false;
      incAngleRoll = 0;
      incAnglePitch = 0;
      dTRising = 0;
      stop();
      feedbackRoll = 0;
      resetFeedback();
  }
  else if(msg->data == "right_kick")
  {
      kanan = 1;
      stop();
      //////////std::cout<<"TENDANG CUUUUY!"<<std::endl;
      m_StartKickCycle = true;
      //////////std::cout<<"TENDANG KANAN>>>>>>>>>>!"<<std::endl;
  }
  else if(msg->data == "left_kick")
  {
      kanan = 0;
      stop();
      //////////std::cout<<"TENDANG CUUUUY!"<<std::endl;
      m_StartKickCycle = true;
      //////////std::cout<<"<<<<<<<<<<<TENDANG KIRI!"<<std::endl;
  }
}

void WalkZMP::walkingParameterCallback(const zmp_walking_module_msgs::ZMPWalkingParam::ConstPtr &msg)
{
  walking_param_ = *msg;
}

bool WalkZMP::getWalkigParameterCallback(zmp_walking_module_msgs::GetZMPParam::Request &req, zmp_walking_module_msgs::GetZMPParam::Response &res)
{
  res.parameters = walking_param_;
  return true;
}

void WalkZMP::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  Vector3d imu;
  Quaterniond imu_orientation;

  imu(0) = msg->angular_velocity.x;
  imu(1) = msg->angular_velocity.y;
  imu(2) = msg->angular_velocity.z;

  imu_orientation.x() = msg->orientation.x;
  imu_orientation.y() = msg->orientation.y;
  imu_orientation.z() = msg->orientation.z;
  imu_orientation.w() = msg->orientation.w;

  m_imuGyr = imu;
  m_imuOri = robotis_framework::convertQuaternionToRPY(imu_orientation);
  op3_kd_->op3_link_data_[29]->orientation_=robotis_framework::convertQuaternionToRotation(imu_orientation);
}

void WalkZMP::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                      std::map<std::string, double> sensors)
{

  if (enable_ == false)
  {
    return;
  }

  // present angle
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string _joint_name = state_iter->first;
    int joint_index = joint_table_[_joint_name];

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(_joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    current_position_.coeffRef(0, joint_index) = dxl->dxl_state_->present_position_;
    uint data = dxl->dxl_state_->bulk_read_table_["hardware_error_status"];
    if(data != 0)
      ROS_ERROR("Device Error &s : &d", _joint_name, data);
    double valle = dxl->dxl_state_->present_position_;
    op3_kd_->setJointPos(_joint_name, valle);

  }

  if(!m_started) {
    m_started = true;
    m_tLastStep = boost::posix_time::microsec_clock::local_time();
    // timeFeedback = ros::Time::now().toSec();
  }

  if(!m_active)
  walkingStatus("disable");

//  if(startFeedback)
//  {
//    startFeedback = false;
//    startTimeFeedback = ros::Time::now().toSec();
//  }
//  timeFeedback = ros::Time::now().toSec()- startTimeFeedback ;

  boost::posix_time::ptime t = boost::posix_time::microsec_clock::local_time();

  double ph = time_to_double((t-m_tLastStep))/walking_param_.tstep;
  double phSingle = fmin(fmax(ph-m_parameter.zPhase.x(), 0)/(m_parameter.zPhase.y()-m_parameter.zPhase.x()),1);

  if(ph>=1) {
    ph = 0;
    m_newStep = true;
    m_tLastStep = boost::posix_time::microsec_clock::local_time();
  }

  //Stop when stopping sequence is done
  if(m_newStep && (m_stopRequest ==2)) {
    m_stopRequest = 0;
    m_active = false;
    // kickStatus("not_kicking");
  }

  // New step
  if(m_newStep && m_active) {  // m_newStep && m_active
    calculateStepGoal();
    m_StartKicking = false;
    finishedKick = true;
    kickStatus("not_kicking");
    walkingStatus("enable");
  } //end new step

  //Troubleshoot 2019_03_08
  //////std::cout<<"m_uTorsoActual:"<<std::endl;
  //////std::cout<<"X:"<<m_uTorsoActual.x()<<"\tY:"<<m_uTorsoActual.y()<<"\tZ:"<<m_uTorsoActual.z()<<std::endl;
  ////std::cout<<"pLLegX:"<<pLLeg.x()<<"\tpRLegX:"<<pRLeg.x()<<std::endl;
  ////std::cout<<"pLLegY:"<<pLLeg.y()<<"\tpRLegY:"<<pRLeg.y()<<std::endl;
  ////std::cout<<"pLLegZ:"<<pLLeg.z()<<"\tpRLegZ:"<<pRLeg.z()<<std::endl;
  ////std::cout<<"m_velCurrent.x(): "<<m_velCurrent.x() << "\tm_velDiff.x()" << m_velDiff.x() << std::endl;
  //std::cout<<"IMU_Roll(deg): "<<m_imuOri.x()*180/M_PI<<"\tIMU_Pitch(deg): "<<m_imuOri.y()*180/M_PI<<"\tIMU_Yaw(deg): "<<m_imuOri.z()*180/M_PI<<std::endl;
  //std::cout<<"gyro_Roll(deg/s): "<<m_imuGyr.x()*180/M_PI<<"\tgyro_Pitch(deg/s): "<<m_imuGyr.y()*180/M_PI<<"\tgyro_Yaw(deg/s): "<<m_imuGyr.z()*180/M_PI<<std::endl;
  // ////std::cout<<"BodyTilt:"<<walking_param_.bodytilt<<std::endl;
  ////std::cout<<std::endl;
  // wholeBodyCOM();
  getJoints = advanceInStep(ph,phSingle);
  // ////std::cout<<"Phase: "<<ph<<"\tM_SupportLeg"<<m_supportLeg<<std::endl;
  setValue(); //set value to servo
  // if(walking_param_.feedback_ == false)
  // {
  //   //////std::cout<<"Feedback NOT Active"<<std::endl;
  // }
  // else
  // {
  //   pushRecovery(phSingle);
  //   //////std::cout<<"Feedback Active"<<std::endl;
  // }

  pushRecovery(phSingle);

  for(int idx = 0; idx < 12; idx++)
  {
     current_joint_pos(idx) = joint_axis_direction_(idx)*(current_position_.coeffRef(0, idx)-offset[idx]);
  }
  //Troubleshoot 2019_03_11 Get Current Angle from Servo and Forward Kinematic
  printCurrentAngle();
  forwardKinematic();
  ////std::cout<<"IndexSupport: "<<IndexSupport<<std::endl;
  // forwardGoblok();
  // saveCurrentPose();
  setPointRoll = kanan?(0*M_PI/180):(0*M_PI/180);
  setPointPitch = 5*M_PI/180;
  feedbackRoll = feedbackPID(false,setPointRoll,m_imuOri.x(),-m_imuGyr.x());
  feedbackPitch = feedbackPID(true, setPointPitch, normalizedIMU(), -m_imuGyr.y());
  // ////std::cout<<"FeedbackRoll(DEG): "<<feedbackRoll*180/M_PI<<std::endl;

    //Kicking
  if(m_StartKickCycle == true && startRising == false)
  {
      resetFeedback();
      saveWalkReadyPose();

      //RisingUp
      TRising = ros::Time::now().toSec();
      dTRising = 0;
      startRising = true;
      finishedRising = false;
      firstTime = true;

      //Kicking
      m_StartKicking = false;
      deltaT = 0;
      firstTimeKick = false;
      finishedKick = true;

      //CoolingDown
      startDown = false;
      finishedDown = false;
      firstTimeDown = false;

      m_StartKickCycle = false;
      // kickStatus("kicking");
  }

  //Rising Up
  if(startRising == true && m_StartKickCycle == false)
  {
      //////std::cout << "Start Rising\t" << TRising << std::endl;
      dTRising = ros::Time::now().toSec()-TRising;
      //////std::cout << "Delta Rising\t" << dTRising << std::endl;
      //////std::cout << "ROS Time Now\t" << ros::Time::now().toSec() << std::endl;

      //        if(c_ballPosY<=0)
      risingUp(kanan);
      //        else
      //            risingUp(0);
      kickStatus("kicking");
  }

  if(troubleshootLift)
  {
    ////std::cout<<"troubleshootLift"<<std::endl;
  }
  else
  {
      //Kick
    if(m_StartKicking == true && finishedRising == true)
    {
        // saveCurrentPose();
        start_T = ros::Time::now().toSec();
        m_StartKicking = false;
        finishedKick = false;
        deltaT = 0;
        firstTimeKick = true;
        //////std::cout << "START " << m_StartKicking <<std::endl;
        //////std::cout << "Finished" << finishedKick << std::endl;
        kickStatus("kicking");
    }
    if(firstTimeKick == true) //&& m_StartKicking == false)
    {
        ////std::cout<< "startT\t" << start_T << std::endl;
        deltaT = ros::Time::now().toSec()-start_T;
        ////std::cout<< "deltaT\t" << deltaT << std::endl;
        ////std::cout << "ROSTimeNow\t" << ros::Time::now().toSec() << std::endl;
        kick(kanan);
    }

    // //Cooldown
    // if(startDown == true && firstTimeDown == true)
    // {
    //     ////std::cout << "Start Down\t" << TDown << std::endl;
    //     dTRising =0;
    //     deltaT = 0;
    //     dTDown = ros::Time::now().toSec()-TDown;
    //     ////std::cout << "Delta Down\t" << dTDown << std::endl;
    //     ////std::cout << "ROS Time Now\t" << ros::Time::now().toSec() << std::endl;

    //     coolingDown(kanan);
    //     //        if(c_ballPosY<=0)
    //     //            coolingDown(1);
    //     //        else
    //     //            coolingDown(0);
    // }

  }
    //  ////std::cout << "dt" << dt.toSec()  << " time : " << ros::Duration(ros::Rate(150)).toSec() << std::endl;
    //    return m_supportLeg ? FootPhase::right : FootPhase::left;
    // end motion_body
}

void WalkZMP::wholeBodyCOM()
{
  // mass = op3_kd_->calcTotalMass(0); //recursive, ambil joint awal saja
  // mc = op3_kd_->calcMC(0);
  op3_kd_->calcForwardKinematics(0);
  // op3_link_data_[joint_id]->position_
  COM = op3_kd_->calcCOM(0);
  ////std::cout<<"WholeBodyCOM x"<<COM(0)<<"\ty:"<<COM(1)<<"\tz"<<COM(2)<<std::endl;
}

void WalkZMP::calculateStepGoal()
{
  set_velocity(walking_param_.zmp_vx, walking_param_.zmp_vy, walking_param_.zmp_vphi);
  update_velocity();
  m_supportLeg = (m_supportLeg?0:1); // 0 for left support, 1 for right support
  m_uLeft1 = m_uLeft2;
  m_uRight1 = m_uRight2;
  m_uTorso1 = m_uTorso2;

  Vector2d supportMod = Vector2d(0,0); //Support Point modulation
  m_shiftFactor = 0.5; //How much should we shift final Torso pose?

  if(m_stopRequest ==1) {
    m_stopRequest = 2;
    m_velCurrent = Vector3d(0,0,0);
    m_velCommand = Vector3d (0,0,0);

    if(m_supportLeg == 0) {        // Left support
      m_uRight2 = pose_global(-2*Vector3d(walking_param_.foot_x,walking_param_.foot_y,0), m_uLeft1); //TODO Config
    } else {       // Right support
      m_uLeft2 = pose_global(2*Vector3d(walking_param_.foot_x,walking_param_.foot_y,0), m_uRight1);
    }
  }

  else { //Normal walk, advance steps
    if(m_supportLeg == 0) { // Left support
      m_uRight2 = step_right_destination(m_velCurrent, m_uLeft1, m_uRight1);
    } else { // Right support
      m_uLeft2 = step_left_destination(m_velCurrent, m_uLeft1, m_uRight1);
    }

    //Velocity-based support point modulation
    m_toeTipCompensation = 0;
    if((m_velDiff.x()>0) && ((m_velCurrent.x()-walking_param_.m_zmp_vx/100) > 0)) { //Accelerating to front
      supportMod.x() = walking_param_.supp_front2;//m_parameter.supportFront2;
    } else if((m_velCurrent.x()-walking_param_.m_zmp_vx/100)>walking_param_.velfast_forward){//m_parameter.velFastForward) {
      supportMod.x() = walking_param_.supp_front;//m_parameter.supportFront;
      m_toeTipCompensation = m_ankleMod.x();
    } else if((m_velCurrent.x()-walking_param_.m_zmp_vx/100) < 0) {
      supportMod.x() = m_parameter.supportBack;
    } else if(std::abs((m_velCurrent.z()-walking_param_.m_zmp_vphi/100)) > walking_param_.velfast_turn){//m_parameter.velFastTurn) {
      supportMod.x() = walking_param_.supp_turn;//m_parameter.supportTurn;
    } else {
      if((m_velCurrent.y()-walking_param_.m_zmp_vy/100)>0.02){//0.005){//0.015) {
        supportMod.x() = walking_param_.supp_side_x;//m_parameter.supportSideX;
        supportMod.y() = walking_param_.supp_side_y;//m_parameter.supportSideY;
      } else if((m_velCurrent.y()-walking_param_.m_zmp_vy/100)<-0.02){//-0.015) {
        supportMod.x() = walking_param_.supp_side_x;//m_parameter.supportSideX;
        supportMod.y() = -walking_param_.supp_side_y;//-m_parameter.supportSideY;
      }
    }
  }

  m_uTorso2 = step_torso(m_uLeft2, m_uRight2,m_shiftFactor);

  //Adjustable initial step body swing
  if(m_initial_step>0) {
    //ROS_DEBUG(" init steps") ;
    if(m_supportLeg == 0) { //LS
      supportMod.y() = m_parameter.supportModYInitial;
    } else { //RS
      supportMod.y() = -m_parameter.supportModYInitial;
    }
  }
  Eigen::Vector3d supportOffset = m_supportLeg?m_uRight1:m_uLeft1;

  if (m_robottype == "Darwin") {
    supportOffset.y() +=  (m_supportLeg?+1:-1)* walking_param_.foot_y;
  }

  //Apply velocity-based support point modulation for uSupport
  Vector3d uFootTorso = pose_relative(supportOffset,m_uTorso1); //Weg vom noch hinteren Beim zum AnfangsSchwerpunkt
  Vector3d uTorsoModded = pose_global(Vector3d (supportMod.x(),supportMod.y(),0),m_uTorso);
  Vector3d uFootModded = pose_global (uFootTorso,uTorsoModded);
  m_uSupport = pose_global(Vector3d (walking_param_.supp_x, (m_supportLeg?-1:1)*walking_param_.supp_y, 0),uFootModded);

  //Compute ZMP coefficients
  m_ZMP_coeff.m1X = (m_uSupport.x()-m_uTorso1.x())/(walking_param_.tstep*m_parameter.phSingle.x()); //Actually linear trajectory value
  m_ZMP_coeff.m2X = (m_uTorso2.x()-m_uSupport.x())/(walking_param_.tstep*(1-m_parameter.phSingle.y()));
  m_ZMP_coeff.m1Y = (m_uSupport.y()-m_uTorso1.y())/(walking_param_.tstep*m_parameter.phSingle.x());
  m_ZMP_coeff.m2Y = (m_uTorso2.y()-m_uSupport.y())/(walking_param_.tstep*(1-m_parameter.phSingle.y()));
  Vector2d zmp_solve_result = zmp_solve(m_uSupport.x(), m_uTorso1.x(), m_uTorso2.x(), m_uTorso1.x(), m_uTorso2.x());
  m_ZMP_coeff.aXP = zmp_solve_result.x();
  m_ZMP_coeff.aXN = zmp_solve_result.y();
  zmp_solve_result = zmp_solve(m_uSupport.y(), m_uTorso1.y(), m_uTorso2.y(), m_uTorso1.y(), m_uTorso2.y());

  m_ZMP_coeff.aYP = zmp_solve_result.x();
  m_ZMP_coeff.aYN = zmp_solve_result.y();

  m_newStep = false;
}

Matrix<double, 18, 1> WalkZMP::advanceInStep(double ph, double phSingle){

  Vector2d xzFoot = foot_phase(ph,phSingle);
  double xFoot = xzFoot.x();
  double zFoot = xzFoot.y();
  double turnCompX = 0;
  double frontCompX = 0;


  if(!m_active) zFoot=0;
  if(m_initial_step>0) { zFoot = 0; } //Don't lift foot at initial step

  if(m_active){
    if(m_supportLeg == 0) {    // Left support
      m_uRight = se2_interpolate(xFoot, m_uRight1, m_uRight2);
    } else {    // Right support
      m_uLeft = se2_interpolate(xFoot, m_uLeft1, m_uLeft2);
    }

    m_uTorso = zmp_com(ph, m_uSupport, m_ZMP_coeff);
  }

  //Turning
  if (std::abs(m_velCurrent.z())>m_parameter.turnCompThreshold && m_velCurrent.x()>-0.01) {
    turnCompX = m_parameter.turnComp;
  }

  //Walking front
  if((m_velCurrent.x()>0.04)) {
    frontCompX = walking_param_.front_comp;//m_parameter.frontComp;
  }
  if((m_velDiff.x()>0.02)) {
    frontCompX = frontCompX + m_parameter.AccelComp;
  }


  m_uTorsoActual = pose_global(Vector3d (walking_param_.foot_x+frontCompX+turnCompX,0,0), m_uTorso);
  //  m_uTorsoActual = pose_global(Vector3d (walking_param_.supp_x,0,0), m_uTorso);


  pTorso = Matrix<double, 6, 1>();
  pTorso.head<2>() = m_uTorsoActual.head<2>();
  pTorso(2) = walking_param_.body_height;
  pTorso.tail<3>() = Vector3d(0, walking_param_.bodytilt , m_uTorsoActual.z());
  //Arm movement compensation


  pLLeg = Matrix<double, 6, 1>();
  pLLeg.head<2>() = m_uLeft.head<2>();
  pLLeg(2) = 0; pLLeg(3) = 0; pLLeg(4) = 0;
  pLLeg(5) = m_uLeft.z();


  pRLeg = Matrix<double, 6, 1>();
  pRLeg.head<2>() = m_uRight.head<2>();
  pRLeg(2) = 0; pRLeg(3) = 0; pRLeg(4) = 0;
  pRLeg(5) = m_uRight.z();

  if(m_supportLeg == 0) {    // Left support
    pRLeg(2) = walking_param_.step_height*zFoot;
  } else {    // Right support
    pLLeg(2) = walking_param_.step_height*zFoot;
  }

  Matrix<double, 12, 1> qLegs;
  Eigen::Matrix<double, 6, 1> qArms;
  Matrix<double, 18, 1> qJoints;

  qLegs = inverse_legs(pLLeg, pRLeg, pTorso);
  motion_legs(qLegs, phSingle);

  qArms = motion_arms();

  qJoints.head<12>() = qLegs;
  qJoints.tail<6>() = qArms;

  return qJoints;
}

void WalkZMP::motion_legs(Matrix<double, 12, 1> &qLegs, double phSingle) {
  double phComp = fmin(1.0, fmin(phSingle / .1, (1 - phSingle) / .1));
  assert(phComp == phComp);
  if(false)
    applyGyroStabilization(qLegs);


  double belly_roll = m_parameter.default_belly_roll;
  double belly_pitch = m_parameter.default_belly_pitch;

  if (m_active) {
    if (m_supportLeg == 0) {
      //qLegs(Body::RANKLE_PITCH) += m_toeTipCompensation * phComp;//Lifting toetip //dicomment biar nggak ngganggu feedback
      qLegs(Body::LHIP_ROLL) += m_parameter.hipRollCompensation * phComp; //Hip roll compensation
    } else {
      //qLegs(Body::LANKLE_PITCH) += m_toeTipCompensation * phComp;//Lifting toetip
      qLegs(Body::RHIP_ROLL) -= m_parameter.hipRollCompensation * phComp;//Hip roll compensation TODO geändert
    }
  }
}

void WalkZMP::StepController(bool kanan){
//Dibuat utk kondisi active (walking) & statis
    //uRL2 = 0 IF STOP
    //Troubleshooting per komponen pers. 14
    //batasi panjang langkah uRL2
    //Langkah sesuai arah gangguan

    //double gyro_y = m_imuGyr.y();
    //double gyroOr = m_imuOr.y();
    double x_captureMax = 0.08;
    double a = 2 * acos(1- sqrt(abs((0.365 * pow(m_imuGyr.y(),2)/2+cos(m_imuOri.y()) - 1)/8)));
    double x_capture = fmin(fmax(2*cos(a/2), -x_captureMax),x_captureMax);
    double jarak = fmin(fmax(2*0.0365*sin(m_imuOri.y()), -x_captureMax),x_captureMax);

  //   ////std::cout<<"imuY: "<<m_imuOri.y()<<std::endl;
   //////std::cout<<"x_capture: "<<x_capture<<std::endl;
  //   ////std::cout<<"x_capture:  "<<jarak<<std::endl;
    if(m_imuOri.y()>M_PI/30 || m_imuOri.y()<-M_PI/30)
    {
    if(kanan)
    {

        m_uLeft2.x() += jarak;
        //////std::cout<<"LEFT2: "<<m_uLeft2.x()<<std::endl;
    }
    else
    {
        m_uRight2.x() += jarak;
        //////std::cout<<"RIGHT2: "<<m_uRight2.x()<<std::endl;
    }
    }
}

void WalkZMP::pushRecovery(double phSingle){
      // feedback << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
      double X_REFF = walking_param_.Acc;

      double phase_lift         = 0.15; //belum tau nilainya
      double phase_land         = 0.85; //belum tau nilainya

      // double R_Angle_Hip_Roll    = current_joint_pos(1) ;
      // double R_Angle_Hip_Pitch   = current_joint_pos(2) ;
      // double R_Angle_Knee        = current_joint_pos(3) ;
      // double R_Angle_Ankle_Pitch = current_joint_pos(4) ;
      // double R_Angle_Ankle_Roll  = current_joint_pos(5) ;

      // double L_Angle_Hip_Roll    = current_joint_pos(7) ;
      // double L_Angle_Hip_Pitch   = current_joint_pos(8) ;
      // double L_Angle_Knee        = current_joint_pos(9) ;
      // double L_Angle_Ankle_Pitch = current_joint_pos(10);
      // double L_Angle_Ankle_Roll  = current_joint_pos(11) ;


      A = 0.1168 ;//thigh
      B = 0.1168 ;//tibia
      imu_x = -m_imuOri.x() ;
      if(imu_y == 0)
      {
        imu_y = -default_imu_y;//30*PI/180;
      }else
      imu_y = -m_imuOri.y() ;

      imu_vx=  m_imuGyr.x() ;
      imu_vy=  m_imuGyr.y() ;
/*
      if(IndexSupport == 1){
        //SSP Kiri
        C = sqrt(pow(A,2)+pow(B,2) - 2*A*B * cos((PI-L_Angle_Knee)));//Length Hip To Ankle
        THETA_ERROR_ = asin (A * sin((PI - L_Angle_Knee)) / C)  + R_Angle_Hip_Pitch + imu_y;
      }else if(IndexSupport == 2){
        //SSP Kanan
        C = sqrt(pow(A,2)+pow(B,2) - 2*A*B * cos((PI-R_Angle_Knee)));//Length Hip To Ankle
        THETA_ERROR_ = asin (A * sin((PI - R_Angle_Knee)) / C)  + R_Angle_Hip_Pitch + imu_y;
      }else{
        //DSP
        C = sqrt(pow(A,2)+pow(B,2) - 2*A*B * cos((PI-R_Angle_Knee)));//Length Hip To Ankle
        THETA_ERROR_ = asin (A * sin((PI - R_Angle_Knee)) / C)  + R_Angle_Hip_Pitch + imu_y;
      }
      */
      THETA_ERROR_ = normalizedIMU();
      THETA_ERROR_DT = imu_vy;//(THETA_ERROR_-PRE_THETA_ERROR)/DT;//sqrt(2 * (9.8/C) * (1 - cos(THETA_ERROR_)) );
      TH3 = 3;

      Z0 = C * cos(THETA_ERROR_);
      X_ERROR = C * sin(THETA_ERROR_) - X_REFF;
      X_ERROR_DT = C *  cos(THETA_ERROR_) * THETA_ERROR_DT;
      OMEGA = sqrt(9.8 / Z0);

      TH1 = walking_param_.TH1;
      TH2 = walking_param_.TH2;
      Torque_Hip_Max = 2;
      Torque_Ankle_Max = 4;
      Angle_Hip_Max = PI/15;
      Acc = walking_param_.Acc;
      Gain_Angle_Pitch = walking_param_.Kp;
      Gain_Velocity_Pitch = walking_param_.Kd;

      double C1 = std::fabs(X_ERROR_DT / OMEGA + X_ERROR);
      double C2 = 0.04;//Torque_Ankle_Max / WEIGHT;
      double C3 =  (Torque_Ankle_Max + Torque_Hip_Max * pow((std::exp(OMEGA * TH1)-1),2)) / WEIGHT;

      //////////std::cout<<"------------------------------------------"<<std::endl;

      if(phSingle >= 0 && phSingle < phase_lift){
             trap_func = phSingle / phase_lift;
             ////std::cout<<"Trap 1"<<std::endl;
      }else if (phSingle >= phase_lift && phSingle < phase_land){
             trap_func = 1;
             ////std::cout<<"Trap 2"<<std::endl;
      }else if (phSingle >= phase_land && phSingle < 1){
             trap_func = (1 - phSingle) / (1 - phase_land);
             //////////std::cout<<"Trap 3"<<std::endl;
       }

      if(PAUSE){
        TIME_START_ = ros::Time::now().toSec();
        PAUSE = false;
      }

      TIME_PAUSE = ros::Time::now().toSec() - TIME_START_;

      if(TIME_PAUSE > TH3){
        PLAY = true;
        TIME_PAUSE = 0;
      }

      if(std::fabs(imu_x) < 0.4 && std::fabs(imu_x) > 0.175){

        // ////std::cout<<"ANKLE ROLL"<<std::endl;
        Angle_Ankle_Roll = (TH1 * imu_x + 0 * imu_vx);
        if(IndexSupport == 1){ //SSP Kiri
          feedback(5) = 0;
          feedback(11) = trap_func *  Angle_Ankle_Roll;
          setValue();
        }else if(IndexSupport == 2){ //ssp Kanan
          feedback(5) = trap_func * Angle_Ankle_Roll;
          feedback(11) = 0;
          setValue();
        }else{ //DSP
          feedback(5) = Angle_Ankle_Roll;
          feedback(11) = Angle_Ankle_Roll;
          setValue();
        }
       }

      // }else if(std::fabs(imu_x) < 10){

      //   ////std::cout<<"HIP ROLL"<<std::endl;
      //   Angle_Ankle_Roll = (Gain_Angle_Pitch * imu_x + Gain_Velocity_Pitch * imu_vx);

      //   if(IndexSupport == 1){
      //     feedback(5) = 0;
      //     feedback(11) = trap_func *  Angle_Ankle_Roll;
      //     setValue();
      //   }else if(IndexSupport == 2){
      //     feedback(5) = trap_func * Angle_Ankle_Roll;
      //     feedback(11) = 0;
      //     setValue();
      //   }else{
      //     feedback(5) = Angle_Ankle_Roll;
      //     feedback(11) = Angle_Ankle_Roll;
      //     setValue();
      //   }

      //   if(HIP){
      //       (X_ERROR < 0) ? (DIR_ = 1) : (DIR_ = -1);
      //       TIME_START = ros::Time::now().toSec();
      //       HIP = false;
      //       current_R_HIP_ROLL = joint_axis_direction_(1)*current_joint_pos(1);
      //       current_L_HIP_ROLL = joint_axis_direction_(7)*current_joint_pos(7);
      //       current_HIP_ROLL   = (current_R_HIP_ROLL + current_L_HIP_ROLL)/2;
      //       this->stop();
      //   }

      //   TIME_ = ros::Time::now().toSec() - TIME_START;

      //   if(TIME_ >= 0 && TIME_ <2*TH1){

      //     if(TIME_ < TH1){
      //       // X = X0 + 1/2*Acc*TIME^2 //dipercepat sejak 0 sampai TH1
      //       Angle_Hip_Roll = DIR_ * (current_HIP_ROLL + 0.5*Acc*TIME_*TIME_);//Angle_Hip_Max;
      //       feedback(1) = Angle_Hip_Roll;
      //       feedback(7) = Angle_Hip_Roll;
      //       setValue();
      //     }else if(TIME_ >= TH1){
      //       // X = X0 + (Acc*TH1)*(TIME-TH1) - 1/2*Acc*(TIME-TH1)^2 //diperlambat sejak TH1 sampai 2TH1
      //       Angle_Hip_Roll = DIR_ * ((current_HIP_ROLL + 0.5*Acc*TH1*TH1) + (Acc*TH1)*(TIME_-TH1) -0.5*Acc*pow((TIME_-TH1),2));//Angle_Hip_Max;
      //       feedback(1) = Angle_Hip_Roll;
      //       feedback(7) = Angle_Hip_Roll;
      //       setValue();
      //     }

      //   }else if(TIME_ >= 2*TH1 && TIME_ < 2*TH1+TH2){
      //      Angle_Hip_Roll = DIR_ * (Angle_Hip_Max * (2*TH1 + TH2 - TIME_) / TH2);
      //      feedback(1) = Angle_Hip_Roll;
      //      feedback(7) = Angle_Hip_Roll;
      //      setValue();
      //   }
      //   else
      //   {
      //     HIP = true;
      //     TIME_ = 0;
      //     PAUSE = true;
      //     PLAY = false;
      //     this->start();
      //   }

      // }

      // ////std::cout<<"IMU X\t"<<imu_x<<std::endl;
      // ////std::cout<<"IMU VX\t"<<imu_vx<<std::endl;

     // if((C1 < C2) && HIP)
     // {
      //  ////std::cout<<"ANKLE"<<std::endl;
       Angle_Ankle_Pitch = (Gain_Angle_Pitch * X_ERROR +  Gain_Velocity_Pitch * X_ERROR_DT);

       if(IndexSupport == 1){
         feedback(4) = 0;
         feedback(10) = trap_func *  Angle_Ankle_Pitch;
         setValue();
       }else if(IndexSupport == 2){
         feedback(4) =  trap_func * Angle_Ankle_Pitch;
         feedback(10) = 0;
         setValue();
       }else{
         feedback(4) = Angle_Ankle_Pitch;
         feedback(10) = Angle_Ankle_Pitch;
         setValue();
       }
     // }
     // else if(((C1 < C3) || !HIP) && PLAY && (IndexSupport == 0))
     // {
     //   ////std::cout<<"HIP"<<std::endl;
     //   Angle_Ankle_Pitch = trap_func * (Gain_Angle_Pitch * X_ERROR + Gain_Velocity_Pitch * X_ERROR_DT);

     //   if(IndexSupport == 1){
     //     feedback(4) = 0;
     //     feedback(10) = Angle_Ankle_Pitch;
     //     setValue();
     //   }else if(IndexSupport == 2){
     //     feedback(4) = Angle_Ankle_Pitch;
     //     feedback(10) = 0;
     //     setValue();
     //   }else{
     //     feedback(4) = Angle_Ankle_Pitch;
     //     feedback(10) = Angle_Ankle_Pitch;
     //     setValue();
     //   }

     //   if(HIP){
     //       (X_ERROR < 0) ? (DIR_ = 1) : (DIR_ = -1);
     //       TIME_START = ros::Time::now().toSec();
     //       HIP = false;
     //       current_R_HIP_PITCH = joint_axis_direction_(2)*current_joint_pos(2);
     //       current_L_HIP_PITCH = joint_axis_direction_(8)*current_joint_pos(8);
     //       current_HIP_PITCH = (current_R_HIP_PITCH + current_L_HIP_PITCH)/2;
     //       this->stop();
     //   }

     //   TIME_ = ros::Time::now().toSec() - TIME_START;

     //   if(TIME_ >= 0 && TIME_ <2*TH1){

     //     if(TIME_ < TH1){
     //       // X = X0 + 1/2*Acc*TIME^2 //dipercepat sejak 0 sampai TH1
     //       Angle_Hip_Pitch = DIR_ * (current_HIP_PITCH + 0.5*Acc*TIME_*TIME_);//Angle_Hip_Max;
     //       feedback(2) = Angle_Hip_Pitch;
     //       feedback(8) = Angle_Hip_Pitch;
     //       setValue();
     //     }else if(TIME_ >= TH1){
     //       // X = X0 + (Acc*TH1)*(TIME-TH1) - 1/2*Acc*(TIME-TH1)^2 //diperlambat sejak TH1 sampai 2TH1
     //       Angle_Hip_Pitch = DIR_ * ((current_HIP_PITCH + 0.5*Acc*TH1*TH1) + (Acc*TH1)*(TIME_-TH1) -0.5*Acc*pow((TIME_-TH1),2));//Angle_Hip_Max;
     //       feedback(2) = Angle_Hip_Pitch;
     //       feedback(8) = Angle_Hip_Pitch;
     //       setValue();
     //     }

     //   }else if(TIME_ >= 2*TH1 && TIME_ < 2*TH1+TH2){
     //      Angle_Hip_Pitch = DIR_ * (Angle_Hip_Max * (2*TH1 + TH2 - TIME_) / TH2);
     //      feedback(2) = Angle_Hip_Pitch;
     //      feedback(8) = Angle_Hip_Pitch;
     //      setValue();
     //   }
     //   else
     //   {
     //     HIP = true;
     //     TIME_ = 0;
     //     PAUSE = true;
     //     PLAY = false;
     //     this->start();
     //   }

     // }
     // else
     // {
     //   ////std::cout<<"ELSE\t"<<std::endl;
     //   Angle_Hip_Pitch = 0;
     //   Angle_Ankle_Pitch = 0;
     //   feedback(4) = Angle_Ankle_Pitch;
     //   feedback(10) = Angle_Ankle_Pitch;
     //   feedback(2) = Angle_Hip_Pitch;
     //   feedback(8) = Angle_Hip_Pitch;
     //   setValue();
     // }


      // std::cout<<"X ERROR\t"<<X_ERROR<<std::endl;
//      ////std::cout<<"THE ERROR\t"<<THETA_ERROR_<<std::endl;
//      ////std::cout<<"X ERROR DT\t"<<X_ERROR_DT<<std::endl;
//      ////std::cout<<"Z0\t"<<Z0<<std::endl;
//      ////std::cout<<"Feedback\t"<< Angle_Ankle_Pitch<<std::endl;
//      ////std::cout<<"THETA ERROR\t"<<THETA_ERROR_<<std::endl;
//      ////std::cout<<"THETA ERROR DT\t"<<THETA_ERROR_DT<<std::endl;
//      ////std::cout<<"OMEGA\t"<<OMEGA<<std::endl;
//      ////std::cout<<"C1\t"<<fabs(X_ERROR_DT / OMEGA + X_ERROR)<<std::endl;
//      ////std::cout<<"C2\t"<<Torque_Ankle_Max / WEIGHT<<std::endl;
//      ////std::cout<<"C3\t"<<(Torque_Ankle_Max + Torque_Hip_Max * pow((std::exp(OMEGA * TH1)-1),2)) / WEIGHT<<std::endl;

//      ////std::cout<<"C1\t"<<C1<<std::endl;
//      ////std::cout<<"C2\t"<<C2<<std::endl;
//      ////std::cout<<"C3\t"<<C3<<std::endl;

//      ////std::cout<<"PLAY_\t"<<PLAY<<std::endl;
//      ////std::cout<<"HIP_\t"<<HIP<<std::endl;
    //  ////std::cout<<"C1\t"<<C1<<std::endl;
//      ////std::cout<<"C2\t"<<C2<<std::endl;
//      ////std::cout<<"C3\t"<<C3<<std::endl;
    //  ////std::cout<<"ANKLE\t"<<Angle_Ankle_Pitch<<std::endl;
//      ////std::cout<<"HIP\t"<<Angle_Hip_Pitch<<std::endl;
//      ////std::cout<<"TIME\t"<<TIME_<<std::endl;
//      ////std::cout<<"TIME_PAUSE\t"<<TIME_PAUSE<<std::endl;
      PRE_THETA_ERROR = THETA_ERROR_;
}


void WalkZMP::applyGyroStabilization(Eigen::Matrix<double, 12, 1>& qLegs) {
  //Ankle stabilization using gyro feedback

  double gyro_roll0 = m_imuGyr.x(); //0 if without Gyrostabilisation
  double gyro_pitch0 = -m_imuGyr.y();



  //get effective gyro angle considering body angle offset
  double yawAngle = 0;
  if (!m_active) { //double support
    yawAngle = (m_uLeft.z() + m_uRight.z()) / 2.0 - m_uTorsoActual.z();
  } else if (m_supportLeg == 0) {  // Left support
    yawAngle = m_uLeft.z() - m_uTorsoActual.z();
  } else if (m_supportLeg == 1) {
    yawAngle = m_uRight.z() - m_uTorsoActual.z();
  }
  double gyro_roll = gyro_roll0 * cos(yawAngle) - gyro_pitch0 * sin(yawAngle);
  double gyro_pitch = gyro_pitch0 * cos(yawAngle) - gyro_roll0 * sin(yawAngle);

  ROS_INFO_STREAM_COND(DEBUG_PRINT, "gyro_roll : " << gyro_roll);
  ROS_INFO_STREAM_COND(DEBUG_PRINT, "gyro_pitch : " << gyro_pitch);

  double armShiftX = procFunc(gyro_pitch * walking_param_.armX_fact * gyroFactor, walking_param_.armX_deadband * DEGREE2RADIAN,
                              walking_param_.armX_maxVal * DEGREE2RADIAN);
  double armShiftY = procFunc(gyro_roll * walking_param_.armY_fact * gyroFactor, walking_param_.armY_deadband * DEGREE2RADIAN,
                              walking_param_.armY_maxVal * DEGREE2RADIAN);

  double ankleShiftX = procFunc(gyro_pitch *walking_param_.ankleX_fact * gyroFactor, walking_param_.ankleX_deadband * DEGREE2RADIAN,
                                walking_param_.ankleX_maxVal * DEGREE2RADIAN);
  double ankleShiftY = procFunc(gyro_roll * walking_param_.ankleY_fact * gyroFactor, walking_param_.ankleY_deadband * DEGREE2RADIAN,
                                walking_param_.ankleY_maxVal * DEGREE2RADIAN);
  double kneeShiftX = procFunc(gyro_pitch * walking_param_.kneeX_fact * gyroFactor, walking_param_.kneeX_deadband * DEGREE2RADIAN,
                               walking_param_.kneeX_maxVal * DEGREE2RADIAN);
  double hipShiftY = procFunc(gyro_roll * walking_param_.hipY_fact * gyroFactor, walking_param_.hipY_deadband * DEGREE2RADIAN,
                              walking_param_.hipY_maxVal * DEGREE2RADIAN);

  m_ankleShift.x() = m_ankleShift.x() + m_parameter.ankleImuParamX(0) * (ankleShiftX - m_ankleShift.x());
  m_ankleShift.y() = m_ankleShift.y() + m_parameter.ankleImuParamX(0) * (ankleShiftY - m_ankleShift.y());
  m_kneeShift = m_kneeShift + m_parameter.kneeImuParamX(0) * (kneeShiftX - m_kneeShift);
  m_hipShift.y() = m_hipShift.y() + m_parameter.hipImuParamY(0) * (hipShiftY - m_hipShift.y());
  m_armShift.x() = m_armShift.x() + m_parameter.armImuParamX(0) * (armShiftX - m_armShift.x());
  m_armShift.y() = m_armShift.y() + m_parameter.armImuParamY(0) * (armShiftY - m_armShift.y());


  //TODO: Toe/heel lifting

  if (!m_active) { //Double support, standing still
    //qLegs(2) = qLegs(2) + hipShift.y();    //Hip roll stabilization
    qLegs(Body::LKNEE) += m_kneeShift;    //Knee pitch stabilization
    qLegs(Body::LANKLE_PITCH) += m_ankleShift.y();    //Ankle pitch stabilization
    //qLegs(5) = qLegs(5) + ankleShift.y();    //Ankle roll stabilization

    //qLegs(7) = qLegs(7)  + hipShift.y();    //Hip roll stabilization
    qLegs(Body::RKNEE) += m_kneeShift;    //Knee pitch stabilization
    qLegs(Body::RANKLE_PITCH) += m_ankleShift.y();    //Ankle pitch stabilization
    //qLegs(11) = qLegs(11) + ankleShift.y();    //Ankle roll stabilization
  } else {
    if (m_supportLeg == 0) {  // Left support
      qLegs(Body::LHIP_ROLL) += m_hipShift.x();    //Hip roll stabilization
      qLegs(Body::LKNEE) += m_kneeShift;    //Knee pitch stabilization
      qLegs(Body::LANKLE_PITCH) += m_ankleShift.y();    //Ankle pitch stabilization
      qLegs(Body::LANKLE_ROLL) = (qLegs(Body::LANKLE_ROLL) + m_ankleShift.x()) *
          m_parameter.ankleSupportFaktor;    //Ankle roll stabilization
      qLegs(Body::LANKLE_ROLL) *= m_parameter.ankleSupportFaktor;
    } else {
      qLegs(Body::RHIP_ROLL) -= m_hipShift.x();    //Hip roll stabilization
      qLegs(Body::RKNEE) += m_kneeShift;    //Knee pitch stabilization
      qLegs(Body::RANKLE_PITCH) += m_ankleShift.y();    //Ankle pitch stabilization
      qLegs(Body::RANKLE_ROLL) = (qLegs(Body::RANKLE_ROLL) + m_ankleShift.x()) *
          m_parameter.ankleSupportFaktor;    //Ankle roll stabilization
      qLegs(Body::RANKLE_ROLL) *= m_parameter.ankleSupportFaktor;
    }
  }
}

Matrix<double,6,1> WalkZMP::motion_arms()
{

  Vector3d qLArmActual;
  Vector3d qRArmActual;

  qLArmActual.head<2>() = m_parameter.qLArm.head<2>()+m_armShift.head<2>();
  qRArmActual.head<2>() = m_parameter.qRArm.head<2>()+m_armShift.head<2>();

  //Check leg hitting
  double RotLeftA =  mod_angle(m_uLeft.z() - m_uTorso.z());
  double RotRightA =  mod_angle(m_uTorso.z() - m_uRight.z());

  Vector3d LLegTorso = pose_relative(m_uLeft,m_uTorso);
  Vector3d RLegTorso = pose_relative(m_uRight,m_uTorso);
  // qLArmActual.x() = 30*DEGREE2RADIAN*sin(RotRightA*PI);
  // qRArmActual.x() = 30*DEGREE2RADIAN*sin(RotLeftA*PI);
  //5 & 6
  qLArmActual.y() = fmax(2* DEGREE2RADIAN + fmax(0, RotLeftA)/2
                         + fmax(0,LLegTorso.y() - 0.04) /0.02 * 4* DEGREE2RADIAN,qLArmActual.y())-20*DEGREE2RADIAN;
  qRArmActual.y() = fmin(-2* DEGREE2RADIAN - fmax(0, RotRightA)/2
                         - fmax(0,-RLegTorso.y() - 0.04)/0.02 * 4* DEGREE2RADIAN, qRArmActual.y());

  qLArmActual.z() = m_parameter.qLArm.z();
  qRArmActual.z() = m_parameter.qRArm.z();

  Matrix<double, 6, 1> r;
  //The first 6 values of r represent the left leg, the last 6 values the right one
  r.head<3>() = set_rarm_joint((Vector3d)qRArmActual);
  r.tail<3>() = set_larm_joint((Vector3d)qLArmActual);
  //    r.head<3>() = set_rarm_joint(Vector3d(0,0,0));
  //    r.tail<3>() = set_larm_joint(Vector3d(0,0,0));

  return r;
}

Matrix<double, 3, 1> WalkZMP::set_larm_joint(const Eigen::Vector3d& qLArmActual){
  Matrix<double, 3, 1> qArm;
  qArm(0) = trim_to_range(qLArmActual(0) - walking_param_.arm_pitch *DEGREE2RADIAN);
  qArm(1) = trim_to_range(qLArmActual(1) + walking_param_.arm_roll*DEGREE2RADIAN);
  qArm(2) = trim_to_range(qLArmActual(2) - walking_param_.arm_elbow*DEGREE2RADIAN);

  return qArm;
}

Matrix<double, 3, 1> WalkZMP::set_rarm_joint(const Eigen::Vector3d& qLArmActual){
  Matrix<double, 3, 1> qArm;
  qArm(0) = trim_to_range(qLArmActual(0) - walking_param_.arm_pitch*DEGREE2RADIAN);
  qArm(1) = trim_to_range(qLArmActual(1) + walking_param_.arm_roll*DEGREE2RADIAN);
  qArm(2) = trim_to_range(qLArmActual(2) - walking_param_.arm_elbow*DEGREE2RADIAN);

  return qArm;
}

Vector3d WalkZMP::step_left_destination(const Vector3d& vel, const Vector3d& uLeft, const Vector3d& uRight) {
  Vector3d u0 = se2_interpolate(.5, uLeft, uRight);
  // Determine nominal midpoint position 1.5 steps in future
  Vector3d u1 = pose_global(1.5*vel, u0);
  Vector3d uLeftPredict = pose_global(Vector3d(0,walking_param_.foot_y,0), u1);
  Vector3d uLeftRight = pose_relative(uLeftPredict, uRight);

  // Do not pidgeon toe, cross feet:

  //Check toe and heel overlap
  //  double toeOverlap = -m_parameter.footSizeX.x()*uLeftRight.z();
  //  double heelOverlap = -m_parameter.footSizeX.y()*uLeftRight.z();
  //  double limitY = fmax(m_parameter.stanceLimitY.x(),2* walking_param_.foot_y-m_parameter.stanceLimitMarginY+fmax(toeOverlap,heelOverlap));


  //  uLeftRight.x() = fmin(fmax(uLeftRight.x(), m_parameter.stanceLimitX.x()), m_parameter.stanceLimitX.y());
  //  uLeftRight.y() = fmin(fmax(uLeftRight.y(), limitY),m_parameter.stanceLimitY.y());
  //  uLeftRight.z() = fmin(fmax(uLeftRight.z(), m_parameter.stanceLimitA.x()), m_parameter.stanceLimitA.y());


  Vector3d tmp = pose_global(uLeftRight, uRight);
  return tmp;
}

Vector3d WalkZMP::step_right_destination(const Vector3d& vel, const Vector3d& uLeft, const Vector3d& uRight) {
  Vector3d u0 = se2_interpolate(.5, uLeft, uRight);

  // Determine nominal midpoint position 1.5 steps in future
  Vector3d u1 = pose_global(1.5*vel, u0);
  Vector3d uRightPredict = pose_global(-1*Vector3d(0,walking_param_.foot_y,0), u1);
  Vector3d uRightLeft = pose_relative(uRightPredict, uLeft);
  // Do not pidgeon toe, cross feet:

  //Check toe and heel overlap
  //  double toeOverlap = m_parameter.footSizeX.x()*uRightLeft.z();
  //  double heelOverlap = m_parameter.footSizeX.y()*uRightLeft.z();
  //  double limitY = fmax(m_parameter.stanceLimitY.x(), 2* walking_param_.foot_y-m_parameter.stanceLimitMarginY+fmax(toeOverlap,heelOverlap));


  //  uRightLeft.x() = fmin(fmax(uRightLeft.x(), m_parameter.stanceLimitX.x()), m_parameter.stanceLimitX.y());
  //  uRightLeft.y() = fmin(fmax(uRightLeft.y(), -m_parameter.stanceLimitY.y()), -limitY);
  //  uRightLeft.z() = fmin(fmax(uRightLeft.z(), -m_parameter.stanceLimitA.y()), -m_parameter.stanceLimitA.x());

  Vector3d tmp = pose_global(uRightLeft, uLeft);
  return tmp;
}

Vector3d WalkZMP::step_torso(const Vector3d& uLeft, const Vector3d& uRight, double shiftFactor) {
  Vector3d uLeftSupport = pose_global(Vector3d(walking_param_.supp_x, walking_param_.supp_y, 0), uLeft);
  Vector3d uRightSupport = pose_global(Vector3d(walking_param_.supp_x, -walking_param_.supp_y, 0), uRight);
  return se2_interpolate(shiftFactor, uLeftSupport, uRightSupport);
}

void WalkZMP::set_velocity(double vx, double vy, double va) {
  //Filter the commanded speed
  m_velCommand.x() = (vx+walking_param_.m_zmp_vx) / 100; //to cm //dikurangi soalnya maju terus, jalan di tempat -0,5cm/s
  m_velCommand.y() = (vy+walking_param_.m_zmp_vy) / 100; //to cm //ditambah soalnya ke kanan terus, jalan di tempat 1,5cm/s
  m_velCommand.z() = (va+walking_param_.m_zmp_vphi) / 100; //to cm
}

void WalkZMP::update_velocity() {
  if(m_velCurrent.x()>m_parameter.velXHigh) {

    //Slower accelleration at high speed
    m_velDiff.x() = fmin(fmax((m_velCommand.x()/*-walking_param_.m_zmp_vx/100*/)-m_velCurrent.x(),-m_parameter.velDelta.x()),m_parameter.velDeltaXHigh);
  } else {
    m_velDiff.x() = fmin(fmax((m_velCommand.x()/*-walking_param_.m_zmp_vx/100*/)-m_velCurrent.x(), -m_parameter.velDelta.x()),m_parameter.velDelta.x());
  }
  m_velDiff.y() = fmin(fmax((m_velCommand.y()/*-walking_param_.m_zmp_vy/100*/)-m_velCurrent.y(), -m_parameter.velDelta.y()),m_parameter.velDelta.y());
  m_velDiff.z() = fmin(fmax((m_velCommand.z()/*-walking_param_.m_zmp_vphi/100*/)-m_velCurrent.z(), -m_parameter.velDelta.z()),m_parameter.velDelta.z());

  m_velCurrent.x() = m_velCurrent.x()+m_velDiff.x();
  m_velCurrent.y() = m_velCurrent.y()+m_velDiff.y();
  m_velCurrent.z() = m_velCurrent.z()+m_velDiff.z();

  if(m_initial_step>0) {
    m_velCurrent = Vector3d (0,0,0);
    m_initial_step = m_initial_step-1;
  }
}

//m_uSupport.x(), m_uTorso1.x(), m_uTorso2.x(), m_uTorso1.x(), m_uTorso2.x()
Eigen::Vector2d WalkZMP::zmp_solve(double zs, double z1, double z2, double x1, double x2) {
  /*
    Solves ZMP equation:
    x(t) = z(t) + aP*exp(t/tZmp) + aN*exp(-t/tZmp) - tZmp*mi*sinh((t-Ti)/tZmp)
    where the ZMP point is piecewise linear:
    z(0) = z1, z(T1 < t < T2) = zs, z(tStep) = z2
    */
  double T1 = walking_param_.tstep*m_parameter.phSingle.x();
  double T2 = walking_param_.tstep*m_parameter.phSingle.y();
  double m1 = (zs-z1)/T1;
  double m2 = -(zs-z2)/(walking_param_.tstep-T2);

  double c1 = x1-z1+walking_param_.tzmp*m1*sinh(-T1/walking_param_.tzmp);
  double c2 = x2-z2+walking_param_.tzmp*m2*sinh((walking_param_.tstep-T2)/walking_param_.tzmp);
  double expTStep = exp(walking_param_.tstep/walking_param_.tzmp);
  double aP = (c2 - c1/expTStep)/(expTStep-1/expTStep);
  double aN = (c1*expTStep - c2)/(expTStep-1/expTStep);
  return Vector2d(aP, aN);
}

//Finds the necessary COM for stability and returns it
Eigen::Vector3d WalkZMP::zmp_com(double ph, const Eigen::Vector3d& uSupport, const ZMP_coeff& zmp_coeff) {

  Vector3d com = Vector3d(0, 0, 0);
  double expT = exp(walking_param_.tstep*ph/walking_param_.tzmp);
  com.x() = uSupport.x() + zmp_coeff.aXP*expT + zmp_coeff.aXN/expT;
  com.y() = uSupport.y() + zmp_coeff.aYP*expT + zmp_coeff.aYN/expT;
  if(ph < m_parameter.phSingle.x()) {
    com.x() = com.x() + zmp_coeff.m1X*walking_param_.tstep*(ph-m_parameter.phSingle.x())
        - walking_param_.tzmp*zmp_coeff.m1X*sinh(walking_param_.tstep*(ph-m_parameter.phSingle.x())/walking_param_.tzmp);
    com.y() = com.y() + zmp_coeff.m1Y*walking_param_.tstep*(ph-m_parameter.phSingle.x())
        -walking_param_.tzmp*zmp_coeff.m1Y*sinh(walking_param_.tstep*(ph-m_parameter.phSingle.x())/walking_param_.tzmp);
  } else if(ph > m_parameter.phSingle.y()) {
    com.x() = com.x() + zmp_coeff.m2X*walking_param_.tstep*(ph-m_parameter.phSingle.y())
        -walking_param_.tzmp*zmp_coeff.m2X*sinh(walking_param_.tstep*(ph-m_parameter.phSingle.y())/walking_param_.tzmp);
    com.y() = com.y() + zmp_coeff.m2Y*walking_param_.tstep*(ph-m_parameter.phSingle.y())
        -walking_param_.tzmp*zmp_coeff.m2Y*sinh(walking_param_.tstep*(ph-m_parameter.phSingle.y())/walking_param_.tzmp);
  }
  com.z() = ph* (m_uLeft2.z()+m_uRight2.z())*0.5 + (1-ph)* (m_uLeft1.z()+m_uRight1.z())*0.5;
  return com;
}

Eigen::Vector2d WalkZMP::foot_phase(double ph, double phSingle) {
  // Computes relative x,z motion of foot during single support phase
  // phSingle = 0: x = 0, z = 0, phSingle = 1: x = 1,z = 0
  double phSingleSkew = pow(phSingle, 0.8) - 0.17*phSingle*(1-phSingle);
  double zf = 0.5*(1-cos(2*pi*phSingleSkew));

  //  double phSingleX = fmin(fmax(ph-m_parameter.xPhase.x(), 0)/(m_parameter.xPhase.y()-m_parameter.xPhase.x()),1);
  //  double phSingleSkewX = pow(phSingleX, 0.8) - 0.17*phSingle*(1-phSingle);
  double xf = 0.5*(1-cos(pi*phSingleSkew));

  return Vector2d(xf, zf);
}

/**
 * computes a mod, operation designed for angles in the range -pi, pi
 */
double WalkZMP::mod_angle(double a) {
  //Reduce angle to [-pi, pi)
  a = fmod(a, (2*pi));
  if (a >= pi) {
    a = a - 2*pi;
  }
  return a;
}

/**
 * Computes the sum out of two so called poses in an absolute way
 * @param pRelative the relative pose that is added on the other
 * @param pose the pose that is used as basis
 * The "relative" pose is rotated around their own z-component and then
 * added to the global other pose. Then the two rotation parts are summed up.
 * //Note from @Robert: I'm a bit confused why the rotation components are summed up.
 *      It still can be, that my imagination of the internal pose representation is
 *      still false, but this method would implicitly perform any kind of
 *      rotation of the original pose, without performing the maths to keep the
 *      components aligned with each other.
 * @return The summed up pose of the two input poses
 */
Eigen::Vector3d WalkZMP::pose_global(const Vector3d& pRelative, const Vector3d& pose) {
  double ca = cos(pose.z());
  double sa = sin(pose.z());
  return Vector3d(pose.x() + ca*pRelative.x() - sa*pRelative.y(),
                  pose.y() + sa*pRelative.x() + ca*pRelative.y(),
                  pose.z() + pRelative.z());
}

/**
 * Omnidirectional Walking Using ZMP and Preview Control
 * Translating between Coordinate Frames
 * Computes a relative sum of two poses
 * @param pGlobal The global pose of the update as reference
 * @param pose The pose to work on
 * First the difference between the two position parts of the poses is performed.
 * The pose is subtracted from the global goal. Then this difference is
 * rotated by the poses angle. Finally the resulting angle is the difference of
 * the two poses rotations.
 * @return The summed up pose of the two input poses
 */
// pose "+" returnValue = pGlobal
// pose_global_new( a, b) = c
// pose_relative(c, a) = b
// Computes the relative vector from pose to pGlobal, including the corresponding rotation
Vector3d WalkZMP::pose_relative(const Eigen::Vector3d& pGlobal, const Vector3d& pose) {
  double ca = cos(-pose.z());
  double sa = sin(-pose.z());
  double px = pGlobal.x()-pose.x();
  double py = pGlobal.y()-pose.y();
  double pa = pGlobal.z()-pose.z();
  return Vector3d(ca*px - sa*py, sa*px + ca*py, mod_angle(pa));
}

/**
 * This function looks like a deadband function, but behaves a little different.
 * The method reduces the norm of any input @param a by the parameter @param deadband
 * without switching signs. But the result will never be higher than @param maxvalue.
 * Of course this is meant in the means of the absolute value preserving the signs.
 */
double WalkZMP::procFunc(double a, double deadband, double maxvalue) {
  //Piecewise linear function for IMU feedback
  double b;
  if (a>0) {
    b=fmin(fmax(0,std::abs(a)-deadband), maxvalue);
  } else {
    b=-fmin(fmax(0,std::abs(a)-deadband), maxvalue);
  }
  return b;
}

/**
 * @param t a scaling parameter how to interpolate between u1 and u2
 * @param u1 the basic parameter. This is the basic part of the result
 * @param u2 the part to give the maximum interpolation target.
 * @return an interpolation between u1 and u2. t=0 => r=u1, t<1 => u1<r<u2 t=1 =>r=u2
 */
Vector3d WalkZMP::se2_interpolate(double t, const Vector3d& u1, const Vector3d& u2) {
  // helps smooth out the motions using a weighted average
  return Vector3d(u1.x()+t*(u2.x()-u1.x()),
                  u1.y()+t*(u2.y()-u1.y()),
                  u1.z()+t*mod_angle(u2.z()-u1.z()));
}

Vector3d WalkZMP::foot_trajectory(double phSingle, const Vector3d &u1, const Vector3d &u2)
{
  double phSingleSkew = pow(phSingle, 0.8) - 0.17*phSingle*(1-phSingle);

}

void WalkZMP::loadConfig()
{
  YAML::Node node;
  YAML::Node offset_node;
  YAML::Node kick_node;
  YAML::Node vel_node;
  try
  {
    // load yaml
    node = YAML::LoadFile(param_path_.c_str());
    offset_node = YAML::LoadFile(offset_.c_str());
    kick_node = YAML::LoadFile(param_path_.c_str());
    vel_node = YAML::LoadFile(param_path_.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  YAML::Node config_ = offset_node["offset"];

  for(int i=0; i<12; i++)
  {
      offset[i] = config_[joint[i]].as<double>();
  }

  YAML::Node doc = node["walk_zmp"];

  walking_param_.body_height = doc["bodyHeight"].as<double>();
  walking_param_.bodytilt = doc["bodyTilt"].as<double>()  * DEGREE2RADIAN;
  walking_param_.step_height = doc["stepHeight"].as<double>();
  walking_param_.foot_x = doc["footX"].as<double>();
  walking_param_.foot_y = doc["footY"].as<double>();
  walking_param_.tstep = doc["tStep"].as<double>();
  walking_param_.tzmp = sqrt(walking_param_.body_height/G_CONST);
  walking_param_.front_comp = doc["frontComp"].as<double>();
  walking_param_.supp_front = doc["supportFront"].as<double>();
  walking_param_.supp_front2 = doc["supportFront2"].as<double>();
  walking_param_.supp_side_x = doc["supportSideX"].as<double>();
  walking_param_.supp_side_y = doc["supportSideY"].as<double>();
  walking_param_.supp_turn = doc["supportTurn"].as<double>();
  walking_param_.supp_y = doc["supportY"].as<double>();
  walking_param_.supp_x = doc["supportX"].as<double>();
  walking_param_.arm_pitch = doc["ArmPitch"].as<double>();
  walking_param_.arm_roll = doc["ArmRoll"].as<double>();
  walking_param_.arm_elbow = doc["ArmElbow"].as<double>();

  walking_param_.hipY_fact = doc["hipY_fact"].as<double>();
  walking_param_.hipY_deadband = doc["hipY_deadband"].as<double>();
  walking_param_.hipY_maxVal = doc["hipY_maxVal"].as<double>();

  walking_param_.kneeX_fact = doc["kneeX_fact"].as<double>();
  walking_param_.kneeX_deadband = doc["kneeX_deadband"].as<double>();
  walking_param_.kneeX_maxVal = doc["kneeX_maxVal"].as<double>();

  walking_param_.ankleX_fact = doc["ankleX_fact"].as<double>();
  walking_param_.ankleX_deadband = doc["ankleX_deadband"].as<double>();
  walking_param_.ankleX_maxVal = doc["ankleX_maxVal"].as<double>();

  walking_param_.ankleY_fact = doc["ankleY_fact"].as<double>();
  walking_param_.ankleY_deadband = doc["ankleY_deadband"].as<double>();
  walking_param_.ankleY_maxVal = doc["ankleY_maxVal"].as<double>();

  walking_param_.armX_fact = doc["armX_fact"].as<double>();
  walking_param_.armX_deadband = doc["armX_deadband"].as<double>();
  walking_param_.armX_maxVal = doc["armX_maxVal"].as<double>();

  walking_param_.armY_fact = doc["armY_fact"].as<double>();
  walking_param_.armY_deadband = doc["armY_deadband"].as<double>();
  walking_param_.armY_maxVal = doc["armY_maxVal"].as<double>();

  walking_param_.Kp = doc["Kp"].as<double>();
  walking_param_.Kd = doc["Kd"].as<double>();

  walking_param_.TH1 = doc["TH1"].as<double>();
  walking_param_.TH2 = doc["TH2"].as<double>();
  walking_param_.Acc = doc["Acc"].as<double>();

  walking_param_.KpP    = doc["GainAnglePitch"].as<double>();
  walking_param_.KdP = doc["GainVelocityPitch"].as<double>();
  walking_param_.KpR     = doc["GainAngleRoll"].as<double>();
  walking_param_.KdR  = doc["GainVelocityRoll"].as<double>();
  walking_param_.Ki      = doc["GainIntegral"].as<double>();

  walking_param_.L_Shift = doc["L_Shift"].as<double>();
  walking_param_.L_Lift = doc["L_Lift"].as<double>();
  walking_param_.L_P_Kick = doc["L_P_Kick"].as<double>();
  walking_param_.L_P_Cool = doc["L_P_Cool"].as<double>();

  walking_param_.R_Shift = doc["R_Shift"].as<double>();
  walking_param_.R_Lift = doc["R_Lift"].as<double>();
  walking_param_.R_P_Kick = doc["R_P_Kick"].as<double>();
  walking_param_.R_P_Cool = doc["R_P_Cool"].as<double>();

  // YAML::Node vdoc = vel_node["velocity_params"];

  walking_param_.m_zmp_vx = doc["vx_manip"].as<double>();
  walking_param_.m_zmp_vy = doc["vy_manip"].as<double>();
  walking_param_.m_zmp_vphi = doc["vphi_manip"].as<double>();
  walking_param_.velfast_forward = doc["velfast_forward"].as<double>();
  walking_param_.velfast_turn = doc["velfast_turn"].as<double>();

  default_imu_y = PI/180*doc["default_imu_y"].as<double>();
  YAML::Node kdoc = kick_node["kick_params"];

  c_ballPosX = kdoc["ballPosX"].as<double>();
  c_ballPosY = kdoc["ballPosY"].as<double>();
  c_ballPosZ = kdoc["ballPosZ"].as<double>();

  c_goalPosX = kdoc["goalPosX"].as<double>();
  c_goalPosY = kdoc["goalPosY"].as<double>();
  c_goalPosZ = kdoc["goalPosZ"].as<double>();
  c_power    = kdoc["power"].as<double>();

  c_TRetract = kdoc["TRetract"].as<double>();
  c_TKick    = kdoc["TKick"].as<double>();
  c_TRising  = kdoc["TRising"].as<double>();
  c_TDown    = kdoc["TDown"].as<double>();
}

void WalkZMP::saveConfig(const std::string &path)
{
  YAML::Node node;
  YAML::Node vel_node;
  try
  {
    // load yaml
    node = YAML::LoadFile(param_path_.c_str());
    vel_node = YAML::LoadFile(param_path_.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }


  YAML::Node doc = node["walk_zmp"];

  //  doc["GyroStabilizer"] = walking_param_.zmp_useGyro;
  doc["bodyHeight"] = walking_param_.body_height ;
  doc["bodyTilt"] = walking_param_.bodytilt * RADIAN2DEGREE ;
  doc["stepHeight"] = walking_param_.step_height ;
  doc["footX"] = walking_param_.foot_x ;
  doc["footY"] = walking_param_.foot_y ;
  doc["tStep"] = walking_param_.tstep ;
  doc["tZmp"] = walking_param_.tzmp ;
  doc["frontComp"] = walking_param_.front_comp ;
  doc["supportFront"] = walking_param_.supp_front ;
  doc["supportFront2"] = walking_param_.supp_front2 ;
  doc["supportSideX"] = walking_param_.supp_side_x ;
  doc["supportSideY"] = walking_param_.supp_side_y ;
  doc["supportTurn"] = walking_param_.supp_turn ;
  doc["supportY"] = walking_param_.supp_y ;
  doc["supportX"] = walking_param_.supp_x  ;
  doc["ArmPitch"] = walking_param_.arm_pitch ;
  doc["ArmRoll"] = walking_param_.arm_roll  ;
  doc["ArmElbow"] = walking_param_.arm_elbow  ;


  doc["hipY_fact"] = walking_param_.hipY_fact;
  doc["hipY_deadband"] = walking_param_.hipY_deadband;
  doc["hipY_maxVal"] = walking_param_.hipY_maxVal;

  doc["kneeX_fact"] = walking_param_.kneeX_fact;
  doc["kneeX_deadband"] = walking_param_.kneeX_deadband;
  doc["kneeX_maxVal"] = walking_param_.kneeX_maxVal;

  doc["ankleX_fact"] = walking_param_.ankleX_fact;
  doc["ankleX_deadband"] = walking_param_.ankleX_deadband;
  doc["ankleX_maxVal"] = walking_param_.ankleX_maxVal;

  doc["ankleY_fact"] = walking_param_.ankleY_fact;
  doc["ankleY_deadband"] = walking_param_.ankleY_deadband;
  doc["ankleY_maxVal"] = walking_param_.ankleY_maxVal;

  doc["armX_fact"] = walking_param_.armX_fact;
  doc["armX_deadband"] = walking_param_.armX_deadband;
  doc["armX_maxVal"] = walking_param_.armX_maxVal;

  doc["armY_fact"] = walking_param_.armY_fact;
  doc["armY_deadband"] = walking_param_.armY_deadband;
  doc["armY_maxVal"] = walking_param_.armY_maxVal;

  doc["Kp"] =  walking_param_.Kp;
  doc["Kd"] =  walking_param_.Kd;
  doc["TH1"] =  walking_param_.TH1;
  doc["TH2"] =  walking_param_.TH2;
  doc["Acc"] =  walking_param_.Acc;

  doc["GainAnglePitch"] =  walking_param_.KpP;
  doc["GainVelocityPitch"] =  walking_param_.KdP;
  doc["GainAngleRoll"] =  walking_param_.KpR;
  doc["GainVelocityRoll"] =  walking_param_.KdR;
  doc["GainIntegral"] =  walking_param_.Ki;
  walking_param_.feedback_ = false;

  doc["L_Shift"] = walking_param_.L_Shift;
  doc["L_Lift"] = walking_param_.L_Lift;
  doc["L_P_Kick"] = walking_param_.L_P_Kick;
  doc["L_P_Cool"] = walking_param_.L_P_Cool;

  doc["R_Shift"] = walking_param_.R_Shift;
  doc["R_Lift"] = walking_param_.R_Lift;
  doc["R_P_Kick"] = walking_param_.R_P_Kick;
  doc["R_P_Cool"] = walking_param_.R_P_Cool;

  // YAML::Node vdoc = vel_node["velocity_params"];

  doc["vx_manip"] = walking_param_.m_zmp_vx;
  doc["vy_manip"] = walking_param_.m_zmp_vy;
  doc["vphi_manip"] = walking_param_.m_zmp_vphi;
  doc["velfast_forward"] = walking_param_.velfast_forward;
  doc["velfast_turn"] = walking_param_.velfast_turn;

  // output to file
  std::ofstream fout(param_path_.c_str());
  fout << node;
  // fout << vel_node;
}

void WalkZMP::onModuleEnable()
{
  walking_state_ = WalkingEnable;
  ROS_INFO("Walking Enable");
}

void WalkZMP::onModuleDisable()
{
  ROS_INFO("Walking Disable");
  walking_state_ = WalkingDisable;
}

void WalkZMP::setValue()
{
  for (int idx = 0; idx < 18; idx++)
  {
    double goal_position = 0.0;

    goal_position = init_position_.coeff(0, idx) + (getJoints(idx) * joint_axis_direction_(idx)) + (feedback(idx) * joint_axis_direction_(idx)) ;

    target_position_.coeffRef(0, idx) = goal_position;
  }
  // //Troubleshoot Servo Value
  // //////////std::cout<<"GOAL_L_HIP_PITCH(deg): "<<target_position_.coeffRef(0,8)*180/M_PI<<"\tGOAL_R_HIP_PITCH(deg): "<<target_position_.coeffRef(0,2)*180/M_PI<<std::endl;
  // //////////std::cout<<"GOAL_L_HIP_ROLL(deg): "<<target_position_.coeffRef(0,7)*180/M_PI<<"\tGOAL_R_HIP_ROLL(deg): "<<target_position_.coeffRef(0,1)*180/M_PI<<std::endl;
  // //////////std::cout<<"GOAL_L_ANKLE_PITCH(deg): "<<target_position_.coeffRef(0,10)*180/M_PI<<"\tGOAL_R_ANKLE_PITCH(deg): "<<target_position_.coeffRef(0,4)*180/M_PI<<std::endl;

  // set result
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int joint_index = joint_table_[joint_name];

    result_[joint_name]->goal_position_ = target_position_.coeff(0, joint_index);

  }
}

void WalkZMP::inverseKinematic()
{
    //deklarasi variable
    int i;
    pTorso = w_pTorso;
    pLLeg << L_X, L_Y, L_Z, L_R*DEGREE2RADIAN, L_P*DEGREE2RADIAN, L_A*DEGREE2RADIAN;
    pRLeg << R_X, R_Y, R_Z, R_R*DEGREE2RADIAN, R_P*DEGREE2RADIAN, R_A*DEGREE2RADIAN;
    getJoints.head<12>() = inverse_legs(pLLeg, pRLeg, pTorso);
    // for(int i=0; i<12; i++)
    // {
    //     if(i==3 || i==9)  getJoints(i)+=joint_axis_direction_(i)*(incAngleRoll); //gak tau orientasinya
    //     if(i==4 || i==10) getJoints(i)+=joint_axis_direction_(i)*(incAnglePitch); //gak tau orientasinya
    // }
}


double WalkZMP::iTorsi(bool pitch, double angle, double velocityAngle, double integrator)
{
    double u, i;
    i += integrator;
    if(pitch==true)
        u = (-1*(walking_param_.KpP*((angle)+i))) + (-1*(walking_param_.KdP*(velocityAngle)));
    else
        u = (-1*(walking_param_.KpR*((angle)+i))) + (-1*(walking_param_.KdR*(velocityAngle)));
    return u;
}

double WalkZMP::integrator(double state, double ref)
{
    double u;
    integratorGain = walking_param_.Ki;
    u=integratorGain*(state-ref)*DT;
    return u;
}

double WalkZMP::feedbackPID(bool pitch, double setpoint, double measured, double derivative)
{
    double error = setpoint - measured;
    _integral += error * DT;
    // double derivative = (error - _pre_error) / _dt;
    if(pitch)
    {
          // Calculate error
      // Proportional term
      Pout = walking_param_.KpP * error;

      // Integral term
      Iout = walking_param_.Ki * _integral;

      // Derivative terM
      Dout =  walking_param_.KdP* derivative;

      // Calculate total output
    }
    else
    {
      Pout = walking_param_.KpR * error;

      // Integral term
      Iout = walking_param_.Ki * _integral;

      // Derivative terM
      Dout =  walking_param_.KdR* derivative;
    }
    double output = Pout + Iout + Dout;
       // Restrict to max/min
      if( output > M_PI/6 )
          output = M_PI/6;
      else if( output < -M_PI/6 )
          output = -M_PI/6;

      // Save error to previous error
      _pre_error = error;

    return output;
}

double WalkZMP::normalizedIMU()
{
     double R_Angle_Hip_Pitch   = current_joint_pos(2) ;
      double R_Angle_Ankle_Pitch = current_joint_pos(4) ;
      double R_Angle_Knee        = current_joint_pos(3) ;

      double L_Angle_Hip_Pitch   = current_joint_pos(8) ;
      double L_Angle_Ankle_Pitch = current_joint_pos(10) ;
      double L_Angle_Knee        = current_joint_pos(9) ;
      // imu_y = -m_imuOri.y() ;
    if(IndexSupport == 1){
        //SSP Kiri
        C = sqrt(pow(k.THIGH_LENGTH,2)+pow(k.CALF_LENGTH,2) - 2*k.THIGH_LENGTH*k.CALF_LENGTH * cos((PI-L_Angle_Knee)));//Length Hip To Ankle
        theta1 = asin (k.THIGH_LENGTH * sin(PI - L_Angle_Knee)/C);
        thetaC = theta1 + L_Angle_Hip_Pitch + imu_y;
        Xc = C*sin(thetaC); Zc = C*cos(thetaC);
      }else if(IndexSupport == 2){
        //SSP Kanan
        C = sqrt(pow(k.THIGH_LENGTH,2)+pow(k.CALF_LENGTH,2) - 2*k.THIGH_LENGTH*k.CALF_LENGTH * cos((PI-R_Angle_Knee)));//Length Hip To Ankle
        theta1 = asin (k.THIGH_LENGTH * sin(PI - R_Angle_Knee)/C);
        thetaC = theta1 + R_Angle_Hip_Pitch + imu_y;
        Xc = C*sin(thetaC); Zc = C*cos(thetaC);
      }else{
        //DSP
       C = sqrt(pow(k.THIGH_LENGTH,2)+pow(k.CALF_LENGTH,2) - 2*k.THIGH_LENGTH*k.CALF_LENGTH * cos((PI-R_Angle_Knee)));//Length Hip To Ankle
       theta1 = asin (k.THIGH_LENGTH * sin(PI - R_Angle_Knee)/C);
       thetaC = theta1 + R_Angle_Hip_Pitch + imu_y;
       Xc = C*sin(thetaC); Zc = C*cos(thetaC);
      }
      Xh = k.HIP_OFFSET_Z*sin(imu_y); Zh = k.HIP_OFFSET_Z*cos(imu_y);
      // ////std::cout<<"C:"<<C<<std::endl;
      // ////std::cout<<"Xc: "<<Xc<<"\tXh: "<<Xh<<"\tZc: "<<Zc<<"\tZh: "<<Zh<<std::endl;
      // ////std::cout<<"XCOM: "<<Xc+Xh<<"\tZCOM: "<<Zc+Zh<<std::endl;
      // ////std::cout<<"normPitchError(DEG): "<<normPitchError*180/M_PI<<std::endl;
      return normPitchError = atan((Xc+Xh)/(Zc+Zh));
}

void WalkZMP::forwardKinematic()
{
    R_HIP.linear()   = Eigen::Matrix3d::Identity();
    R_KNEE.linear()  = Eigen::Matrix3d::Identity();
    R_ANKLE.linear() = Eigen::Matrix3d::Identity();
    L_HIP.linear()   = Eigen::Matrix3d::Identity();
    L_KNEE.linear()  = Eigen::Matrix3d::Identity();
    L_ANKLE.linear() = Eigen::Matrix3d::Identity();

    R_HIP.rotate(   AngleAxisd(current_joint_pos(0) , Vector3d( 0, 0, 1) * joint_axis_direction_(0)) *
                    AngleAxisd(current_joint_pos(1) , Vector3d( 1, 0, 0) * joint_axis_direction_(1)) *
                    AngleAxisd(current_joint_pos(2) , Vector3d( 0, 1, 0) * joint_axis_direction_(2)));
    R_KNEE.rotate(  AngleAxisd(current_joint_pos(3) , Vector3d( 0, 1, 0) * joint_axis_direction_(3)));
    R_ANKLE.rotate( AngleAxisd(current_joint_pos(4) , Vector3d( 0, 1, 0) * joint_axis_direction_(4)) *
                    AngleAxisd(current_joint_pos(5) , Vector3d( 1, 0, 0) * joint_axis_direction_(5)));
    L_HIP.rotate(   AngleAxisd(current_joint_pos(6) , Vector3d( 0, 0, 1) * joint_axis_direction_(6)) *
                    AngleAxisd(current_joint_pos(7) , Vector3d( 1, 0, 0) * joint_axis_direction_(7)) *
                    AngleAxisd(current_joint_pos(8) , Vector3d( 0, 1, 0) * joint_axis_direction_(8)));
    L_KNEE.rotate(  AngleAxisd(current_joint_pos(9) , Vector3d( 0, 1, 0) * joint_axis_direction_(9)));
    L_ANKLE.rotate( AngleAxisd(current_joint_pos(10), Vector3d( 0, 1, 0) * joint_axis_direction_(10)) *
                    AngleAxisd(current_joint_pos(11), Vector3d( 1, 0, 0) * joint_axis_direction_(11)));

    m_R_FOOT = BASE * R_HIP * R_KNEE * R_ANKLE * R_FOOT;
    m_L_FOOT = BASE * L_HIP * L_KNEE * L_ANKLE * L_FOOT;

    R_FOOT_WORLD = m_R_FOOT.translation();
    L_FOOT_WORLD = m_L_FOOT.translation();

    L_FOOT_WORLD(0) *= -1;

    R_ROT_WORLD = robotis_framework::convertRotationToRPY(m_R_FOOT.linear());
    L_ROT_WORLD = robotis_framework::convertRotationToRPY(m_L_FOOT.linear());

    ////std::cout<<"L_Z: "<<std::fixed << std::setprecision(4)<<L_FOOT_WORLD(2)<<"\tR_Z = "<<std::fixed << std::setprecision(4)<<R_FOOT_WORLD(2)<<std::endl;
    if(L_FOOT_WORLD(2) > R_FOOT_WORLD(2) && fabs(L_FOOT_WORLD(2)-R_FOOT_WORLD(2))>0.01){
        IndexSupport = 2;//1; //SSP Kanan
    }else if(L_FOOT_WORLD(2) < R_FOOT_WORLD(2) && fabs(L_FOOT_WORLD(2)-R_FOOT_WORLD(2))>0.01){
        IndexSupport = 1;//2; //SSP Kiri
    }else if(fabs(L_FOOT_WORLD(2)-R_FOOT_WORLD(2))<=0.01){
        IndexSupport = 0;
    }
}

void WalkZMP::forwardGoblok()
{
    // Eigen::MatrixXd RHA(4,4), RHR(4,4), RHP(4,4), RKP(4,4), RAP(4,4), RAR(4,4),
    //                 LHA(4,4), LHR(4,4), LHP(4,4), LKP(4,4), LAP(4,4), LAR(4,4),
    //                 RHipWorld(4,4), LHipWorld(4,4);
    // Eigen::VectorXd Ankle(4);

    // double right_rad[6];
    // double left_rad[6];
    // for(int i=0; i<5; i++)
    // {
    //   right_rad[i]=joint_axis_direction_(i)*current_joint_pos(i);
    // }
    // for(int i=6; i<11; i++)
    // {
    //   left_rad[i-6]=joint_axis_direction_(i)*current_joint_pos(i);
    // }

    // RHA << cos(right_rad[0]), -sin(right_rad[0]),  0, 0,
    //        sin(right_rad[0]),  cos(right_rad[0]),  0, 0,
    //                        0,                  0,  1, 0,
    //                        0,                  0,  0, 1;

    // RHR << 1,                  0,                 0,  0,
    //        0,  cos(right_rad[1]), sin(right_rad[1]),  0,
    //        0, -sin(right_rad[1]), cos(right_rad[1]),  0,
    //        0,                  0,                 0,  1;

    // RHP << cos(right_rad[2]), 0, sin(right_rad[2]),  0,
    //                        0, 1,                 0,  0,
    //       -sin(right_rad[2]), 0, cos(right_rad[2]),  0,
    //                        0, 0,                 0,  1;

    // RKP << cos(right_rad[3]), 0, sin(right_rad[3]),              0,
    //                        0, 1,                 0,              0,
    //       -sin(right_rad[3]), 0, cos(right_rad[3]),  -k.THIGH_LENGTH,
    //                        0, 0,                 0,              1;

    // RAP << cos(right_rad[4]), 0, sin(right_rad[4]),            0,
    //                          0, 1,                   0,            0,
    //       -sin(right_rad[4]), 0, cos(right_rad[4]), -k.CALF_LENGTH,
    //                          0, 0,                   0,            1;

    // RAR <<  1,                  0,                   0,  0,
    //         0, cos(right_rad[5]), sin(right_rad[5]),  0,
    //         0, -sin(right_rad[5]),  cos(right_rad[5]),  -k.ANKLE_LENGTH,
    //         0,                  0,                   0,  1;
    // RHipWorld<< 1, 0, 0, 0,
    //            0, 1, 0, -k.LEG_SIDE_OFFSET,
    //            0, 0, 1, 0,//k.THIGH_LENGTH+k.CALF_LENGTH,
    //            0, 0, 0, 1;
    // Ankle << 0,
    //          0,
    //          0,
    //          1;
    // LHA << cos(left_rad[0]), -sin(left_rad[0]),    0,                         0,
    //        sin(left_rad[0]),  cos(left_rad[0]),    0,                         0,
    //                      0,                0,    1,                         0,
    //                      0,                0,    0,                         1;

    // LHR << 1,                0,                 0,  0,
    //        0, cos(left_rad[1]), sin(left_rad[1]),  0,
    //        0, -sin(left_rad[1]),  cos(left_rad[1]),  0,
    //        0,                0,                 0,  1;

    // LHP << cos(left_rad[2]), 0, -sin(left_rad[2]),  0,
    //                        0, 1,                 0,  0,
    //       sin(left_rad[2]), 0, cos(left_rad[2]),  0,
    //                        0, 0,                 0,  1;

    // LKP << cos(left_rad[3]), 0, -sin(left_rad[3]),              0,
    //                        0, 1,                 0,               0,
    //       sin(left_rad[3]), 0, cos(left_rad[3]),  -k.THIGH_LENGTH,
    //                        0, 0,                 0,               1;

    // LAP << cos(left_rad[4]), 0, -sin(left_rad[4]),             0,
    //                        0, 1,                 0,                 0,
    //       sin(left_rad[4]), 0, cos(left_rad[4]),  -k.CALF_LENGTH,
    //                        0, 0,                 0,                 1;

    // LAR <<  1,                  0,                   0,  0,
    //         0, cos(left_rad[5]), sin(left_rad[5]),  0,
    //         0, -sin(left_rad[5]),  cos(left_rad[5]),  -k.ANKLE_LENGTH,
    //         0,                  0,                   0,  1;
    // LHipWorld<< 1, 0, 0, 0,
    //            0, 1, 0, k.LEG_SIDE_OFFSET,
    //            0, 0, 1, 0,//k.THIGH_LENGTH+k.CALF_LENGTH,
    //            0, 0, 0, 1;

    // RFootWorld = RHipWorld*RHA*RHR*RHP*RKP*RAP*RAR*Ankle;
    // LFootWorld = LHipWorld*LHA*LHR*LHP*LKP*LAP*LAR*Ankle;
    // //////std::cout<<"ForwardGoblok!"<<std::endl;
    // //////std::cout<<"gL_X = "<<std::fixed << std::setprecision(4)<<LFootWorld(0)<<"\tgR_X = "<<std::fixed << std::setprecision(4)<<RFootWorld(0)<<std::endl;
    // //////std::cout<<"gL_Y = "<<std::fixed << std::setprecision(4)<<LFootWorld(1)<<"\tgR_Y = "<<std::fixed << std::setprecision(4)<<RFootWorld(1)<<std::endl;
    // //////std::cout<<"gL_Z = "<<std::fixed << std::setprecision(4)<<LFootWorld(2)<<"\tgR_Z = "<<std::fixed << std::setprecision(4)<<RFootWorld(2)<<std::endl;
}

void WalkZMP::risingUp(bool kanan)
{
    double currentSidePercent;
    TGeser = 0.8, THalt = 0.8;
    sidePercent = 2.5; placeCOM = kanan?(0.00):(0.010);
    m_ballPosZ = c_ballPosZ;
    double dYL, dYR, dZ;
    double pendulumLength=walking_param_.body_height;
    timeRising = c_TRising;
    if(firstTime)
    {
        // saveCurrentPose();
        saveWalkReadyPose();
        firstTime = false;
    }

    if(dTRising <= TGeser && firstTime==false)
    {
        ////std::cout<<"GESEEEERRRRRRRRRRRRRR"<<std::endl;
        if(kanan)
        {
            dYL = (walking_param_.R_Shift - w_pLLeg(1))*(dTRising/TGeser);
            dYR = (-2*k.LEG_SIDE_OFFSET - w_pRLeg(1))*(dTRising/TGeser);
            L_X = w_pLLeg(0);       R_X = w_pRLeg(0);
            L_Y = w_pLLeg(1) + dYL; R_Y = w_pRLeg(1) + dYR;
            L_Z = w_pLLeg(2);       R_Z = w_pRLeg(2);
        }
        else
        {
            dYL = (2*k.LEG_SIDE_OFFSET - w_pLLeg(1))*(dTRising/TGeser);
            dYR = (walking_param_.L_Shift - w_pRLeg(1))*(dTRising/TGeser);
            L_X = w_pLLeg(0);       R_X = w_pRLeg(0);
            L_Y = w_pLLeg(1) + dYL; R_Y = w_pRLeg(1) + dYR;
            L_Z = w_pLLeg(2);       R_Z = w_pRLeg(2);
        }
        inverseKinematic();
        //Override R_HIP_PITCH
        // getJoints(2) =  -35*(M_PI/180);
        // //Override L_HIP_PITCH
        // getJoints(8) =  -35*(M_PI/180);
        setValue();
    }
    else if(dTRising > TGeser)
    {
      /*
        //pitch
        //          //bool pitch, angle,    velocityAngle, integrator(state, state_ref)
        oTorquePitch   = iTorsi(true, m_imuOri.y(), m_imuGyr.y(), integrator(m_imuOri.y(), 0));
        dAnglePitch    = asin(oTorquePitch / (totalMass * pendulumLength * 9.8));
        incAnglePitch += (dAnglePitch);
        //Roll


        oTorqueRoll    = iTorsi(false, m_imuOri.x(),  m_imuGyr.x(), integrator(m_imuOri.x(), setPointRoll*M_PI/18));
        dAngleRoll     = asin(oTorqueRoll / (totalMass * pendulumLength * 9.8));
        incAngleRoll  += (dAngleRoll);
        if(incAngleRoll > M_PI/9) incAngleRoll = M_PI/9;
        else if(incAngleRoll < -M_PI/9) incAngleRoll = -M_PI/9;
      */
        if(dTRising > TGeser && dTRising <= (TGeser+THalt))
        {
          if(kanan)
          {
              dYL = walking_param_.R_Shift;
              dYR = -2*k.LEG_SIDE_OFFSET;
              L_X = w_pLLeg(0); R_X = w_pRLeg(0);
              L_Y = dYL;        R_Y = dYR;
              L_Z = w_pLLeg(2); R_Z = w_pRLeg(2);
          }
          else
          {
              dYL = 2*k.LEG_SIDE_OFFSET;
              dYR = walking_param_.L_Shift;
              L_X = w_pLLeg(0); R_X = w_pRLeg(0);
              L_Y = dYL;        R_Y = dYR;
              L_Z = w_pLLeg(2); R_Z = w_pRLeg(2);
          }
          inverseKinematic();
          //Override R_HIP_PITCH
          // getJoints(2) =  -35*(M_PI/180);
          // //Override L_HIP_PITCH
          // getJoints(8) =  -35*(M_PI/180);
          setValue();
        }
        else if(dTRising > (TGeser+THalt) && dTRising <= (TGeser+THalt+timeRising))
        {
            dZ = m_ballPosZ * ((dTRising-TGeser-THalt)/timeRising);
            ////std::cout<<"NGANGKAAAAT BROOOOOOOOOOOOOOOOO"<<std::endl;
            // ////std::cout<<"FEEDBACK_PITCH(deg): " << incAnglePitch*180/M_PI << "\FEEDBACK_ROLL(deg): " << incAngleRoll*180/M_PI << std::endl;
            // ////std::cout<<"FEEDBACK_ROLL(deg): " << feedbackRoll*180/M_PI<<std::endl;
            if(kanan)
            {
                L_X = w_pLLeg(0); R_X = w_pRLeg(0);
                L_Y = walking_param_.R_Lift;  R_Y = -(sidePercent*k.LEG_SIDE_OFFSET);
                L_Z = w_pLLeg(2); R_Z = w_pRLeg(2) + dZ;
                inverseKinematic();
                //Override L_ANKLE_ROLL
                feedback(11) = feedbackRoll;
                //Override L_HIP_PITCH
                // getJoints(8) =  -35*(M_PI/180);
                setValue();
            }
            else
            {
                L_X = w_pLLeg(0);                      R_X = w_pRLeg(0);
                L_Y = (sidePercent*k.LEG_SIDE_OFFSET); R_Y = walking_param_.L_Lift;
                L_Z = w_pLLeg(2) + dZ;                 R_Z = w_pRLeg(2);
                inverseKinematic();
                //Override R_ANKLE_ROLL;
                feedback(5) = feedbackRoll;
                //Override R_HIP_PITCH
                // getJoints(2) =  -35*(M_PI/180);
                setValue();
            }
        }
        else if(dTRising > (TGeser+THalt+timeRising))
        {
            ////std::cout<<"TAHAAAAAAAAAN ANGKAT 1 KAKI"<<std::endl;
            // ////std::cout<<"FEEDBACK_PITCH(deg): " << incAnglePitch*180/M_PI << "\FEEDBACK_ROLL(deg): " << incAngleRoll*180/M_PI << std::endl;
            // ////std::cout<<"FEEDBACK_ROLL(deg): " << feedbackRoll*180/M_PI<<std::endl;
            if(kanan)
            {
                L_X = w_pLLeg(0); R_X = w_pRLeg(0);
                L_Y = walking_param_.R_Lift;  R_Y = -(sidePercent*k.LEG_SIDE_OFFSET);
                L_Z = w_pLLeg(2); R_Z = w_pRLeg(2) + m_ballPosZ;
                saveCurrentPose();
                inverseKinematic();
                //Override L_ANKLE_ROLL;
                feedback(11) = feedbackRoll;
                //Override L_HIP_PITCH
                // getJoints(8) =  -35*(M_PI/180);
                setValue();
            }
            else
            {
                L_X = w_pLLeg(0);                      R_X = w_pRLeg(0);
                L_Y = (sidePercent*k.LEG_SIDE_OFFSET); R_Y = walking_param_.L_Lift;
                L_Z = w_pLLeg(2) + m_ballPosZ;         R_Z = w_pRLeg(2);
                saveCurrentPose();
                inverseKinematic();
                //Override R_ANKLE_ROLL;
                feedback(5) = feedbackRoll;
                //Override R_HIP_PITCH
                // getJoints(2) =  -35*(M_PI/180);
                setValue();
            }
            finishedRising = true;
            m_StartKicking = true;
            finishedKick   = false;
            if(troubleshootLift)
            {
              ////std::cout<<"troubleshootLift"<<std::endl;
            }
            else
            {
              startRising    = false; //comment to troubleshoot lifting 1 foot
              dTRising = 0; //comment to troubleshoot lifting 1 foot
            }
            start_T = ros::Time::now().toSec();
        }

    }
}

void WalkZMP::kick(bool kanan)
{
    sidePercentKick = 0;//kanan?0.035:0;//0;//kanan?0.03:-0.03;
    sidePercentCool = kanan?(0.03):(-0.04);
    Eigen::MatrixXd PSR(1,4),PSK(1,4), CM(4,4), CPP(4,3), CPQ(4,3), retract(1,3), kick(1,3);
    Eigen::VectorXf v(3);
    m_ballPosX = c_ballPosX;
    m_ballPosY = kanan?(-c_ballPosY):(c_ballPosY);
    m_ballPosZ = c_ballPosZ;
    m_goalPosX = c_goalPosX;
    m_goalPosY = kanan?(-c_goalPosY):(c_goalPosY);
    m_goalPosZ = c_goalPosZ;
    m_power = c_power;
    m_TRetract = c_TRetract;
    m_TKick = c_TKick;
    timeDown = c_TDown;
    TGeserDown = 0.8;
    //Vector arah tembakan bola ke gawang
    v << m_goalPosX-m_ballPosX,
         m_goalPosY-m_ballPosY,
         m_goalPosZ-m_ballPosZ;

    //Control Matrix (jangan diubah)
    CM << 1,  0,  0,  0,
         -3,  3,  0,  0,
          3, -6,  3,  0,
         -1,  3, -3,  1;

    //  Retract kick control point (boleh diubah)
    P0x =  0;   P0y = 0;    P0z=0;//Titik Awal
    P1x = -0.3*m_power*v(0)/v.norm();   P1y = -m_power*v(1)/v.norm();    P1z=k.THIGH_LENGTH*0.4;
    P2x = -0.3*m_power*v(0)/v.norm();   P2y = -m_power*v(1)/v.norm();    P2z=k.THIGH_LENGTH*0.5;
    P3x = P2x+(m_ballPosX-P2x)/2;   P3y = P2y+(m_ballPosY-P2y)/2;    P3z=P2z+0.75*(m_ballPosZ-P2z);//Titik Akhir (Via Point)

    // Forward kick control point (boleh diubah)
    Q0x = m_ballPosX;                Q0y = m_ballPosY;                Q0z = m_ballPosZ;//Titik Awal (Via Point)
    Q1x = Q0x+m_power*v(0)/v.norm(); Q1y = Q0y+m_power*v(1)/v.norm();    Q1z = Q0z+m_power*v(2)/v.norm();
    Q2x = Q1x;                       Q2y = Q1y;                         Q2z = Q1z;
    Q3x = 0.3*m_ballPosX;            Q3y = kanan?(w_pRLeg(1)):(w_pLLeg(1));   Q3z = P0z;//Titik Akhir

    if(kanan)
    {
        CPP << P0x+i_pRLeg(0), P0y+i_pRLeg(1),      P0z+i_pRLeg(2),
               P1x+i_pRLeg(0), P1y+i_pRLeg(1),      P1z+i_pRLeg(2),
               P2x+i_pRLeg(0), P2y+i_pRLeg(1),      P2z+i_pRLeg(2),
               P3x+i_pRLeg(0), P3y+i_pLLeg(1),      P3z;
        CPQ << Q0x+i_pRLeg(0), Q0y+i_pRLeg(1),      Q0z,
               Q1x+i_pRLeg(0), Q1y+i_pRLeg(1),      Q1z,
               Q2x+i_pRLeg(0), Q2y+i_pRLeg(1),      Q2z,
               Q3x+i_pRLeg(0), Q3y+0.75*i_pRLeg(1), Q3z+i_pRLeg(2);
    }
    else
    {
        CPP << P0x+i_pLLeg(0), P0y+i_pLLeg(1),      P0z+i_pLLeg(2),
               P1x+i_pLLeg(0), P1y+i_pLLeg(1),      P1z+i_pLLeg(2),
               P2x+i_pLLeg(0), P2y+i_pLLeg(1),      P2z+i_pLLeg(2),
               P3x+i_pLLeg(0), P3y+i_pLLeg(1),      P3z;
        CPQ << Q0x+i_pLLeg(0), Q0y+i_pLLeg(1),      Q0z,
               Q1x+i_pLLeg(0), Q1y+i_pLLeg(1),      Q1z,
               Q2x+i_pLLeg(0), Q2y+i_pRLeg(1),      Q2z,
               Q3x+i_pLLeg(0), Q3y+0.75*i_pLLeg(1), Q3z+i_pLLeg(2);
    }
//    CPP << P0x, P0y, P0z,
//           P1x, P1y, P1z,
//           P2x, P2y, P2z,
//           P3x, P3y, P3z;

//    CPQ << Q0x, Q0y, Q0z,
//           Q1x, Q1y, Q1z,
//           Q2x, Q2y, Q2z,
//           Q3x, Q3y, Q3z;

    //Mulai Retract Kaki ke Belakang
    //Mulai Timer Retract
    startRetract = true;
    finishedRetract = false;
    startKick = false;
    finishedKick = false;
    // ////std::cout<< "StartRetract\t " << startRetract <<std::endl;
    // ////std::cout<< "FinishedRetract\t " << finishedRetract <<std::endl;
    if(startRetract && !finishedRetract)
    {
        current_TRetract = deltaT; //getMilliSpan(start_TRetract); //Elapsed Time
        tr = current_TRetract/m_TRetract; //Normalisasi parameter t
        PSR << 1, tr, pow(tr,2), pow(tr,3);
        retract = PSR * CM * CPP;
        if(kanan)
        {
              L_X = i_pLLeg(0);                    R_X = retract(0,0);
              L_Y = i_pLLeg(1)+walking_param_.R_P_Kick*(tr); R_Y = retract(0,1);
              L_Z = i_pLLeg(2);                    R_Z = retract(0,2);
              if(R_Y >= (L_Y-1.5*k.LEG_SIDE_OFFSET)) //Boundary condition so kicking foot doesn't hit support foot
              {
                R_Y = (L_Y-1.5*k.LEG_SIDE_OFFSET);
              }
              inverseKinematic();
              //Override L_HIP_ROLL
              // getJoints(7) =  7*(M_PI/180);
              //Override L_ANKLE_ROLL
              feedback(11)=feedbackRoll;
              setValue();
        }
        else
        {
              L_X = retract(0,0); R_X = i_pRLeg(0);
              L_Y = retract(0,1); R_Y = i_pRLeg(1)+walking_param_.L_P_Kick*tr;
              L_Z = retract(0,2); R_Z = i_pRLeg(2);
              if(L_Y <= (R_Y+1.5*k.LEG_SIDE_OFFSET)) //Boundary condition so kicking foot doesn't hit support foot
              {
                L_Y =  (R_Y+1.5*k.LEG_SIDE_OFFSET);
              }
              inverseKinematic();
              //Override R_HIP_ROLL
              // getJoints(1) = 20*(M_PI/180);
              // //Override R_HIP_ROLL
              // getJoints(1) += incAngleRoll;//10*(M_PI/180);
              //Override R_HIP_PITCH
              // getJoints(2) =  -40*(M_PI/180);
              //Override R_ANKLE_ROLL
              feedback(5)=feedbackRoll;
              setValue();
        }
        ////std::cout << "CurrentTRetract:\t" <<current_TRetract << std::endl;
        ////////std::cout << "R_X: " <<R_X<< "\tR_Y: " << R_Y << "\tR_Z: " << R_Z << std::endl;
        ////////std::cout << "L_X: " <<L_X<< "\tL_Y: " << L_Y << "\tL_Z: " << L_Z<< std::endl;
        ////////std::cout << "FinishedRetract" << finishedRetract << std::endl;
        if(tr >= 1)
        {
            startRetract = false;
            finishedRetract = true;
            startKick = true;
            //////std::cout << "FinishedRetract" << finishedRetract << std::endl;
            ////std::cout << "<<<<<<<<<<<DONE RETRACT!" << std::endl;
        }
        kickStatus("kicking");
        walkingStatus("enable");
    }

    //Mulai Kick Kaki ke Depan
    if(startKick && finishedRetract)
    {
        current_TKick = deltaT - m_TRetract; //getMilliSpan(start_TKick); //Elapsed Time
        tk = current_TKick/m_TKick; //Normalisasi parameter t
        PSK << 1, tk, pow(tk,2), pow(tk,3);
        kick = PSK * CM * CPQ;
        if(kanan)
        {
            L_X = i_pLLeg(0); R_X = kick(0,0);
            L_Y = i_pLLeg(1)+walking_param_.R_P_Kick; R_Y = kick(0,1);
            L_Z = i_pLLeg(2); R_Z = kick(0,2);
            if(R_Y >= (L_Y-1.5*k.LEG_SIDE_OFFSET))
            {
                R_Y = (L_Y-1.5*k.LEG_SIDE_OFFSET);
            }
            inverseKinematic();
            //Override L_HIP_ROLL
            // getJoints(7) =  6*(M_PI/180);
            // //Override L_HIP_PITCH
            // getJoints(8) =  -35*(M_PI/180);
            //Override L_ANKLE_ROLL
            feedback(11)=feedbackRoll;
            //Override L_ANKLE_PITCH
            getJoints(10)=-26*M_PI/180;
            setValue();
            if(current_TKick >= m_TKick && current_TKick < (m_TKick+0.3))
            {
                PSK << 1, 1, pow(1,2), pow(1,3);
                kick = PSK * CM * CPQ;
                L_X = i_pLLeg(0);                 R_X = kick(0,0);
                L_Y = i_pLLeg(1)+walking_param_.R_P_Kick; R_Y = kick(0,1);
                L_Z = i_pLLeg(2);                 R_Z = kick(0,2);
                if(R_Y >= (L_Y-1.5*k.LEG_SIDE_OFFSET))
                {
                    R_Y = (L_Y-1.5*k.LEG_SIDE_OFFSET);
                }
                inverseKinematic();
                // //Override L_HIP_PITCH
                // getJoints(8) =  -35*(M_PI/180);
                //Override L_ANKLE_ROLL
                feedback(11)=feedbackRoll;
                //Override L_ANKLE_PITCH
                getJoints(10)=-26*M_PI/180;
                setValue();
                ////std::cout<<"HOLD_KICK_HOLD_KICK"<<std::endl;
            }
        }
        else
        {
            L_X = kick(0,0); R_X = i_pRLeg(0);
            L_Y = kick(0,1); R_Y = i_pRLeg(1)+walking_param_.L_P_Kick;
            L_Z = kick(0,2); R_Z = i_pRLeg(2);
            if(L_Y <= (R_Y+1.5*k.LEG_SIDE_OFFSET))
            {
               L_Y =  (R_Y+1.5*k.LEG_SIDE_OFFSET);
            }
            inverseKinematic();
            //Override R_HIP_PITCH
            // getJoints(2) =  -35*(M_PI/180);
            //Override R_ANKLE_PITCH
            getJoints(4)=-26*M_PI/180;
            //Override R_ANKLE_ROLL
            feedback(5)=feedbackRoll;
            setValue();
            if(current_TKick >= m_TKick && current_TKick < (m_TKick+0.3))
            {
                PSK << 1, 1, pow(1,2), pow(1,3);
                kick = PSK * CM * CPQ;
                 L_X = kick(0,0); R_X = i_pRLeg(0);
                  L_Y = kick(0,1); R_Y = i_pRLeg(1)+walking_param_.L_P_Kick;
                  L_Z = kick(0,2); R_Z = i_pRLeg(2);
                  if(L_Y <= (R_Y+1.5*k.LEG_SIDE_OFFSET))
                  {
                    L_Y =  (R_Y+1.5*k.LEG_SIDE_OFFSET);
                  }
                inverseKinematic();
                //Override R_HIP_PITCH
                // getJoints(2) =  -26*(M_PI/180);
                //Override R_ANKLE_PITCH
                getJoints(4)=-26*M_PI/180;
                //Override R_ANKLE_ROLL
                feedback(5)=feedbackRoll;
                setValue();
                ////std::cout<<"HOLD_KICK_HOLD_KICK"<<std::endl;
            }
        }
        ////std::cout << "CurrentTKick:\t" <<current_TKick << std::endl;
        // setValue();
        ////////std::cout << "R_X: " <<R_X<< "\tR_Y: " << R_Y << "\tR_Z: " << R_Z<< std::endl;
        ////////std::cout << "L_X: " <<L_X<< "\tL_Y: " << L_Y << "\tL_Z: " << L_Z<< std::endl;
        ////////std::cout << "finishedKick" << finishedKick<< std::endl;
        //Napak turun
        kickStatus("kicking");
        walkingStatus("enable");
        if(current_TKick >= (m_TKick+0.3) && current_TKick < (m_TKick+0.3+timeDown))
        {
          if(kanan)
          {
            L_X = w_pLLeg(0); R_X = w_pRLeg(0);
            L_Y = i_pLLeg(1)+walking_param_.R_P_Cool; R_Y = i_pRLeg(1)+0.9*walking_param_.R_P_Cool;
            L_Z = i_pLLeg(2)+(w_pLLeg(2)-i_pLLeg(2))*(current_TKick-(m_TKick+0.3))/timeDown; R_Z = i_pRLeg(2)+(w_pRLeg(2)-i_pRLeg(2))*(current_TKick-(m_TKick+0.3))/timeDown;
            inverseKinematic();
          }
          else
          {
             L_X = w_pLLeg(0); R_X = w_pRLeg(0);
             L_Y = i_pLLeg(1)+0.9*walking_param_.L_P_Cool; R_Y = i_pRLeg(1)+walking_param_.L_P_Cool;
             L_Z = i_pLLeg(2)+(w_pLLeg(2)-i_pLLeg(2))*(current_TKick-(m_TKick+0.3))/timeDown; R_Z = i_pRLeg(2)+(w_pRLeg(2)-i_pRLeg(2))*(current_TKick-(m_TKick+0.3))/timeDown;
             inverseKinematic();
          }
          getJoints(4)=-26*M_PI/180;
          getJoints(10)=-26*M_PI/180;
          setValue();
          ////std::cout << "COOLINGDOWN" << finishedKick<< std::endl;
          kickStatus("kicking");
          walkingStatus("enable");
        }
        //Geser turun
        else if(current_TKick >= (m_TKick+0.3+timeDown) && current_TKick < (m_TKick+0.3+timeDown+TGeserDown))
        {

          if(kanan)
          {
            L_X = w_pLLeg(0); R_X = w_pRLeg(0);
            L_Y = (i_pLLeg(1)+walking_param_.R_P_Cool) + (w_pLLeg(1)-(i_pLLeg(1)+walking_param_.R_P_Cool))*(current_TKick-(m_TKick+0.3+timeDown))/TGeserDown;
            R_Y = (i_pRLeg(1)+0.9*walking_param_.R_P_Cool) + (w_pRLeg(1)-(i_pRLeg(1)+0.9*walking_param_.R_P_Cool))*(current_TKick-(m_TKick+0.3+timeDown))/TGeserDown;
            L_Z = w_pLLeg(2); R_Z = w_pRLeg(2);
            inverseKinematic();
          }
          else
          {
            L_X = w_pLLeg(0); R_X = w_pRLeg(0);
            L_Y = (i_pLLeg(1)+0.9*walking_param_.L_P_Cool)+(w_pLLeg(1)-(i_pLLeg(1)+0.9*walking_param_.L_P_Cool))*(current_TKick-(m_TKick+0.3+timeDown))/TGeserDown;
            R_Y = (i_pRLeg(1)+walking_param_.L_P_Cool) + (w_pRLeg(1)-(i_pRLeg(1)+walking_param_.L_P_Cool))*(current_TKick-(m_TKick+0.3+timeDown))/TGeserDown;
            L_Z = w_pLLeg(2); R_Z = w_pRLeg(2);
            inverseKinematic();
          }
          getJoints(4)=-26*M_PI/180;
          getJoints(10)=-26*M_PI/180;
          setValue();
          ////std::cout << "COOLINGDOWNKICK" << finishedKick<< std::endl;
          kickStatus("kicking");
          walkingStatus("enable");
        }
        else if(current_TKick >= (m_TKick+0.3+timeDown+TGeserDown))
        {
                  // saveCurrentPose();
                startKick = false;
                finishedKick = true;
                startDown = true;
                finishedDown = false;
                firstTimeKick = false;
                firstTimeDown = true;
                deltaT = 0;
                TDown = ros::Time::now().toSec();
                tk = 0;
                tr = 0;
                resetFeedback();
            ////////std::cout << "finishedKick" << finishedKick<< std::endl;
            kickStatus("finished_kick");
            ////std::cout << "DONE KICK!8=============D" << std::endl;
            //////////std::cout<<"startDown" << startDown << std::endl;
            //////////std::cout<<"finishedDown" << finishedDown << std::endl;
        }
        // else
            // kickStatus("finished_kick");
    }
}


void WalkZMP::coolingDown(bool kanan)
{
    sidePercentCool = 0.5;
    double dXL, dYL, dZL, dXR, dYR, dZR;
    timeDown = c_TDown;
    TGeserDown = 1.0;
    //Napak bawah
    if(dTDown <= timeDown)
    {
          ////std::cout<<"DOWNVVVVVVVVVVVVVVV: "<<dTDown<<std::endl;
          if(kanan)
          {
            // dYL = 0.5*k.LEG_SIDE_OFFSET;//0+0.01;
            //                 dXR = (w_pRLeg(0)-i_pRLeg(0))*dTDown/timeDown;
            //                  dYR = -1.8*k.LEG_SIDE_OFFSET;
            //                 dZR = (w_pRLeg(2)-i_pRLeg(2))*dTDown/timeDown+0.03;

            //   L_X = w_pLLeg(0); R_X = i_pRLeg(0)+dXR;
            //   L_Y = dYL;        R_Y = dYR;
            //   L_Z = w_pLLeg(2); R_Z = w_pRLeg(2)+dZR;
            L_X = k_pLLeg(0); R_X = k_pRLeg(0);
            L_Y = k_pLLeg(1); R_Y = k_pRLeg(1);
            L_Z = k_pLLeg(2); R_Z = k_pRLeg(2);
          }
          else
          {
              //                                               dYR = -0.5*k.LEG_SIDE_OFFSET;
              // dXL = (w_pLLeg(0)-i_pLLeg(0))*dTDown/timeDown;
              // dYL = 1.8*k.LEG_SIDE_OFFSET;
              // dZL = (w_pRLeg(2)-i_pRLeg(2))*dTDown/timeDown+0.03;
              // L_X = w_pLLeg(0)+dXL; R_X = w_pRLeg(0);
              // L_Y = dYL;            R_Y = dYR;
              // L_Z = w_pLLeg(2)+dZL; R_Z = w_pRLeg(2);
            L_X = k_pLLeg(0); R_X = k_pRLeg(0);
            L_Y = k_pLLeg(1); R_Y = k_pRLeg(1);
            L_Z = k_pLLeg(2); R_Z = k_pRLeg(2);
          }
          inverseKinematic();
          // //Override R_HIP_PITCH
          // getJoints(2) =  -40*(M_PI/180);
          // //Override L_HIP_PITCH
          // getJoints(8) =  -40*(M_PI/180);
          setValue();
    }
    /*
    //Geser badan ke walkready
    else if(dTDown > timeDown && (dTDown-timeDown) <= TGeserDown)
    {
        ////std::cout<<"<<<<<<<<<<<GESER>>>>>>>>>>: "<<dTDown<<std::endl;
        if(kanan)
        {
            dYL = (w_pLLeg(1)-0.01)*((dTDown-timeDown)/TGeserDown);
            dYR = (w_pRLeg(1)-(-sidePercent*k.LEG_SIDE_OFFSET+0.01))*((dTDown-timeDown)/TGeserDown); //0.04 -0.05
            L_X = w_pLLeg(0); R_X = w_pRLeg(0);
            L_Y = 0.01 + dYL; R_Y = -sidePercent*k.LEG_SIDE_OFFSET+0.01 + dYR;
            L_Z = w_pLLeg(2); R_Z = w_pRLeg(2);
        }
        else
        {
            dYL = (w_pLLeg(1)-(sidePercent*k.LEG_SIDE_OFFSET-0.01))*((dTDown-timeDown)/TGeserDown);//(sidePercent*k.LEG_SIDE_OFFSET - w_pLLeg(1))*(dTRising/2);
            dYR = (w_pRLeg(1)-(-0.01))*((dTDown-timeDown)/TGeserDown);//(sidePercent*k.LEG_SIDE_OFFSET - w_pRLeg(1))*(dTRising/2);
            L_X = w_pLLeg(0);                               R_X = w_pRLeg(0);
            L_Y = sidePercent*k.LEG_SIDE_OFFSET-0.01 + dYL; R_Y = -0.01 + dYR;
            L_Z = w_pLLeg(2);                               R_Z = w_pRLeg(2);
        }
        inverseKinematic();
        //Override R_HIP_PITCH
        // getJoints(2) =  -40*(M_PI/180);
        // //Override L_HIP_PITCH
        // getJoints(8) =  -40*(M_PI/180);
        setValue();
    }
    */
    else
    {
          L_X = w_pLLeg(0); R_X = w_pRLeg(0);
          L_Y = w_pLLeg(1); R_Y = w_pRLeg(1);
          L_Z = w_pLLeg(2); R_Z = w_pRLeg(2);
          inverseKinematic();
          feedbackRoll = 0;
          resetFeedback();
          setValue();
        startDown     = false;
        finishedDown  = true;
        firstTimeDown = false;
        // kickStatus("finished_kick");
        // ////std::cout << "finishedDown" << finishedDown<< std::endl;
        // ////std::cout << "~~~~~~~~~~~COOLED DOWN~~~~~~~~~~" << std::endl;
    }
}

void WalkZMP::saveCurrentPose()
{
    i_pLLeg << L_X, L_Y, L_Z, 0, 0, 0;
    i_pTorso = pTorso;
    i_pRLeg << R_X, R_Y, R_Z, 0, 0, 0;
    iR_X = R_FOOT_WORLD(0);
    iR_Y = R_FOOT_WORLD(1);
    iR_Z = R_FOOT_WORLD(2);
    iL_X = L_FOOT_WORLD(0);
    iL_Y = L_FOOT_WORLD(1);
    iL_Z = L_FOOT_WORLD(2);
    //////std::cout<<"SAVED_CURRENT_POSE"<<std::endl;
    //////std::cout<<"i_pLLegX:"<<i_pLLeg.x()<<"\ti_pTorsoX:"<<i_pTorso.x()<<"\ti_pRLegX:"<<i_pRLeg.x()<<std::endl;
    //////std::cout<<"i_pLLegY:"<<i_pLLeg.y()<<"\ti_pTorsoY:"<<i_pTorso.y()<<"\ti_pRLegY:"<<i_pRLeg.y()<<std::endl;
    //////std::cout<<"i_pLLegZ:"<<i_pLLeg.z()<<"\ti_pTorsoZ:"<<i_pTorso.z()<<"\ti_pRLegZ:"<<i_pRLeg.z()<<std::endl;
    //////std::cout << "iR_X: " <<iR_X<< "\tiR_Y: " << iR_Y << "\tiR_Z: " << iR_Z << std::endl;
    //////std::cout << "iL_X: " <<iL_X<< "\tiL_Y: " << iL_Y << "\tiL_Z: " << iL_Z<< std::endl;

}

void WalkZMP::saveLastKickPose()
{
  k_pLLeg << L_X, L_Y, L_Z, 0, 0, 0;
  k_pTorso = pTorso;
  k_pRLeg << R_X, R_Y, R_Z, 0, 0, 0;
}

void WalkZMP::saveWalkReadyPose()
{
    wR_X = R_FOOT_WORLD(0);
    wR_Y = R_FOOT_WORLD(1);
    wR_Z = R_FOOT_WORLD(2);
    wL_X = L_FOOT_WORLD(0);
    wL_Y = L_FOOT_WORLD(1);
    wL_Z = L_FOOT_WORLD(2);
    w_pLLeg = pLLeg; w_pTorso = pTorso; w_pRLeg = pRLeg;
    //////std::cout<<"SAVED_WALKREADY_POSE"<<std::endl;
    //////std::cout<<"w_pLLegX:"<<w_pLLeg.x()<<"\tw_pTorsoX:"<<w_pTorso.x()<<"\tw_pRLegX:"<<w_pRLeg.x()<<std::endl;
    //////std::cout<<"w_pLLegY:"<<w_pLLeg.y()<<"\tw_pTorsoY:"<<w_pTorso.y()<<"\tw_pRLegY:"<<w_pRLeg.y()<<std::endl;
    //////std::cout<<"w_pLLegZ:"<<w_pLLeg.z()<<"\tw_pTorsoZ:"<<w_pTorso.z()<<"\tw_pRLegZ:"<<w_pRLeg.z()<<std::endl;
    //////std::cout << "wR_X: " <<wR_X<< "\twR_Y: " << wR_Y << "\twR_Z: " << wR_Z << std::endl;
    //////std::cout << "wL_X: " <<wL_X<< "\twL_Y: " << wL_Y << "\twL_Z: " << wL_Z<< std::endl;
}

void WalkZMP::KickRequest(const std_msgs::String::ConstPtr &msg)
{
  if(msg->data == "kick")
    walkKick = true;
}

void WalkZMP::printCurrentAngle()
{
    //////std::cout<<"RADIANS"<<std::endl;
    // for(int i=0; i<12; i++)
    // {
        // ////std::cout<<i<<"\tgetJoints: "<<getJoints(i)<<"\tcurrentPostiton: "<<current_joint_pos(i)<<std::endl;
    // }
    ////std::cout<<"DEGREES"<<std::endl;
    for(int i=0; i<12; i++)
    {
        // std::cout<<i<<"\tgetJoints: "<<getJoints(i)*180/M_PI<<"\tcurrentPostiton: "<<current_joint_pos(i)*180/M_PI<<std::endl;
    }
    // for(int i=12; i<18; i++)
    // ////std::cout<<i<<"\tgetJoints: "<<getJoints(i)*180/M_PI<<std::endl; //print arm joint degrees
    //////std::cout<<"_"<<std::endl;
}

void WalkZMP::kickStatus(const std::string &command)
{
  std_msgs::String _commnd_msg;
  _commnd_msg.data = command;
  kick_status_pub_.publish(_commnd_msg);
  //////std::cout << "kickStatus : " << _commnd_msg.data << std::endl;
}

void WalkZMP::walkingStatus(const std::string &command)
{
  std_msgs::String _commnd_msg;
  _commnd_msg.data = command;
  walking_status_pub_.publish(_commnd_msg);
  //////std::cout << "kickStatus : " << _commnd_msg.data << std::endl;
}

void WalkZMP::resetFeedback()
{
  for(int i=0; i<18; i++)
  {
    feedback(i) = 0;
  }
}

}
