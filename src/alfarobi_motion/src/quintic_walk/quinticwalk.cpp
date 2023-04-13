#include "quintic_walk/quinticwalk.h"

using namespace boost::assign;
namespace robotis_op
{

QuinticWalk::QuinticWalk()
    :
      _footstep(0.14, true),
      _phase(0.0),
      _params(),
      _orders(0.0, 0.0, 0.0),
      _isEnabled(false),
      _wasEnabled(false),
      _isTrajsOscillating(false),
      _trunkPosAtLast(),
      _trunkVelAtLast(),
      _trunkAccAtLast(),
      _trunkAxisPosAtLast(),
      _trunkAxisVelAtLast(),
      _trunkAxisAccAtLast(),
      _trajs(),
      _compensate_gravity(false),
      debug_print_(false),
      mbc(ros::package::getPath("alfarobi_motion")),
      so(ros::package::getPath("alfarobi_motion")),
      densis(ros::package::getPath("alfarobi_motion")),
      gainFuzzy(1, std::vector<double> (2))
{
    ROS_INFO("TEST 1");
    enable_ = false;
    // module_name_ = "quintic_walk";
    // control_mode_ = robotis_framework::PositionControl;
    walking_state_ = WalkingInitPose;

    op3_kd_ = new robotis_op::OP3KinematicsDynamics(robotis_op::WholeBody);

     // result
    result_ = new alfarobi::ServoController();
    // result_["r_hip_yaw"] = new robotis_framework::DynamixelState();
    // result_["r_hip_roll"] = new robotis_framework::DynamixelState();
    // result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
    // result_["r_knee"] = new robotis_framework::DynamixelState();
    // result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
    // result_["r_ank_roll"] = new robotis_framework::DynamixelState();

    // result_["l_hip_yaw"] = new robotis_framework::DynamixelState();
    // result_["l_hip_roll"] = new robotis_framework::DynamixelState();
    // result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
    // result_["l_knee"] = new robotis_framework::DynamixelState();
    // result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
    // result_["l_ank_roll"] = new robotis_framework::DynamixelState();

    // result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
    // result_["r_sho_roll"] = new robotis_framework::DynamixelState();
    // result_["r_el"] = new robotis_framework::DynamixelState();

    // result_["l_sho_pitch"] = new robotis_framework::DynamixelState();
    // result_["l_sho_roll"] = new robotis_framework::DynamixelState();
    // result_["l_el"] = new robotis_framework::DynamixelState();

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

    // current_position
    current_position["r_hip_yaw"] = 0;
    current_position["r_hip_roll"] = 0;
    current_position["r_hip_pitch"] = 0;
    current_position["r_knee"] = 0;
    current_position["r_ank_pitch"] = 0;
    current_position["r_ank_roll"] = 0;

    current_position["l_hip_yaw"] = 0;
    current_position["l_hip_roll"] = 0;
    current_position["l_hip_pitch"] = 0;
    current_position["l_knee"] = 0;
    current_position["l_ank_pitch"] = 0;
    current_position["l_ank_roll"] = 0;

    current_position["r_sho_pitch"] = 0;
    current_position["l_sho_pitch"] = 0;

    base     = Affine3d(Translation3d(Vector3d(0,0,0)));
    r_hip    = Affine3d(Translation3d(Vector3d(0,-k.LEG_SIDE_OFFSET,0)));
    r_knee   = Affine3d(Translation3d(Vector3d(0,0,-k.THIGH_LENGTH)));
    r_ankle  = Affine3d(Translation3d(Vector3d(0,0,-k.CALF_LENGTH)));
    r_foot   = Affine3d(Translation3d(Vector3d(0,0,-k.ANKLE_LENGTH)));
    m_r_foot = Affine3d(Translation3d(Vector3d(0,0,0)));
    l_hip    = Affine3d(Translation3d(Vector3d(0,k.LEG_SIDE_OFFSET,0)));
    l_knee   = Affine3d(Translation3d(Vector3d(0,0,-k.THIGH_LENGTH)));
    l_ankle  = Affine3d(Translation3d(Vector3d(0,0,-k.CALF_LENGTH)));
    l_foot   = Affine3d(Translation3d(Vector3d(0,0,-k.ANKLE_LENGTH)));
    m_l_foot = Affine3d(Translation3d(Vector3d(0,0,0)));

    VectorXd L_FOOT_WORLD(4);
    VectorXd R_FOOT_WORLD(4);

    for(int i=0; i<CUPLIK_EMA; i++)
        bufferSkripsiEMA.push_back(0);

    target_position_ = Eigen::MatrixXd::Zero(1, 18 /*result_.size()*/);
    current_position_ = Eigen::MatrixXd::Zero(1, 18 /*result_.size()*/);
    goal_position_ = Eigen::MatrixXd::Zero(1, 18 /*result_.size()*/);
    init_position_ = Eigen::MatrixXd::Zero(1, 18 /*result_.size()*/);
    joint_axis_direction_ = Eigen::MatrixXi::Zero(1, 18 /*result_.size()*/);
    feedback = Eigen::MatrixXd::Zero(1, 18 /*result_.size()*/);

    integralPitch = 0;
}

QuinticWalk::~QuinticWalk()
{
    queue_thread_.join();
}

void QuinticWalk::initialize(const int control_cycle_msec) //8 di program lama
{
    queue_thread_ = boost::thread(boost::bind(&QuinticWalk::queueThread, this));
    control_cycle_msec_ = control_cycle_msec;

    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,
    //                     L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL,
    //                     R_ARM_SWING, L_ARM_SWING
    joint_axis_direction_ <<      -1,         -1,          -1,     -1,             1,            1,
                                  -1,         -1,           1,      1,            -1,            1,
                                   1,         -1,          -1,      1,            -1,            1;
    init_position_        <<     0.0,        0.0,         0.0,    0.0,           0.0,          0.0,
                                 0.0,        0.0,         0.0,    0.0,           0.0,          0.0,
                                 5.0,       -5.0,         0.0,    0.0,           0.0,          0.0;
    init_position_ *= DEGREE2RADIAN;

    _stopRequest = true;
    walkingReset();
    _isLeftSupport = true;
    _supportFootOdom = tf::Transform();
    tf::Quaternion quat = tf::Quaternion();
    quat.setRPY(0,0,0);
    _supportFootOdom.setRotation(quat);
    _supportFootOdom.setOrigin(tf::Vector3(0,0,0));

    joint_goals.resize(18);
    joint_goals += 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    // feedback.resize(18);
    // feedback += 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;


    //Full walk cycle frequency
    //(in Hz, > 0)
    walking_param_.freq = 1.5;
    //Length of double support phase in half cycle
    //(ratio, [0:1])
    walking_param_.doubleSupportRatio = 0.2;
    //Lateral distance between the feet center
    //(in m, >= 0)
    walking_param_.footDistance = 0.13;
    //Maximum flying foot height
    //(in m, >= 0)
    walking_param_.footRise = 0.035;
    //Let the foot's downward trajectory end above the ground
    //this is helpful if the support leg bends
    //(in m, >= 0)
    walking_param_.footPutDownZOffset = 0;
    //Phase time for moving the foot from Z offset to ground
    //(phase between apex and single support end [0:1])
    walking_param_.footPutDownPhase = 1;
    //Phase of flying foot apex
    //(single support cycle phase, [0:1])
    walking_param_.footApexPhase = 0.5;
    //Foot X/Y overshoot in ratio of step length
    //(ratio, >= 0)
    walking_param_.footOvershootRatio = 0.05;
    //Foot X/Y overshoot phase
    //(single support cycle phase, [footApexPhase:1]
    walking_param_.footOvershootPhase = 0.85;
    //Height of the trunk from ground
    //(in m, > 0)
    walking_param_.trunkHeight = 0.27;
    //Trunk pitch orientation
    //(in rad)
    walking_param_.trunkPitch = 0.2;
    //Phase offset of trunk oscillation
    //(half cycle phase, [0:1])
    walking_param_.trunkPhase = 0.4;
    //Trunk forward offset
    //(in m)
    walking_param_.trunkXOffset = 0.005;
    //Trunk lateral offset
    //(in m)
    walking_param_.trunkYOffset = 0.0;
    //Trunk lateral oscillation amplitude ratio
    //(ratio, >= 0)
    walking_param_.trunkSwing = 0.3;
    //Trunk swing pause length in phase at apex
    //(half cycle ratio, [0:1])
    walking_param_.trunkPause = 0.0;
    //Trunk forward offset proportional to forward step
    //(in 1)
    walking_param_.trunkXOffsetPCoefForward = 0.0;
    //Trunk forward offset proportional to rotation step
    //(in m/rad)
    walking_param_.trunkXOffsetPCoefTurn = 0.0;
    //Trunk pitch orientation proportional to forward step
    //(in rad/m)
    walking_param_.trunkPitchPCoefForward = 0.0;
    //Trunk pitch orientation proportional to rotation step
    //(in 1)
    walking_param_.trunkPitchPCoefTurn = 0.0;

    walking_param_.trunkYOnlyInDoubleSupport = false;

    ros::NodeHandle ros_node;

    std::string default_param_path = ros::package::getPath("alfarobi_motion") + "/config/quintic_walk/config.yaml";
    ros_node.param<std::string>("quintic_walk_param", param_path_, default_param_path);
    offset_ = ros::package::getPath("alfarobi_motion") + "/config/initial_position.yaml";
    // kickManip_path_ = ros::package::getPath("kicking") + "/config/kickManip.yaml";
    physicalParam_path_ = ros::package::getPath("op3_kinematics_dynamics") + "/config/physicalParam.yaml";
    fuzzy_path_ = ros::package::getPath("alfarobi_motion") + "/config/quintic_walk/fuzzy.yaml";

    loadParameter();
    loadOffset();
    //Initialize the footstep
    _footstep.setFootDistance(walking_param_.footDistance);
    _footstep.reset(true);
    //Reset the trunk saved state
    resetTrunkLastState();
    //Trajectories initialization
    buildTrajectories();

    firstExc = true;
}

void QuinticWalk::stop()
{
    _stopRequest = true;
    walkingReset();
    _isLeftSupport = true;
}

bool QuinticWalk::isRunning()
{
    return _walkActive;
}

void QuinticWalk::onModuleDisable()
{

    walking_state_ = WalkingDisable;
    ROS_INFO("Quintic Walk Disable");
    walkingReset();
}

void QuinticWalk::onModuleEnable()
{
    firstExc = true;
    walking_state_ = WalkingEnable;
    ROS_INFO("Quintic Walk Enable");
    walkingReset();
}

void QuinticWalk::queueThread()
{
    ros::NodeHandle ros_node;
    ros::CallbackQueue callback_queue;

    ros_node.setCallbackQueue(&callback_queue);

    /* publish topics */
    status_msg_pub_ = ros_node.advertise<alfarobi_msgs_srvs_actions::StatusMsg>("robotis/status", 1);
    walking_status_pub_ = ros_node.advertise<std_msgs::String>("/robotis/walking/status",0);

    /* ROS Service Callback Functions */
    // ros::ServiceServer get_walking_param_server = ros_node.advertiseService("/robotis/quintic_walk/get_params",
    //                                                                         &QuinticWalk::getWalkigParameterCallback,
    //                                                                         this);

    /* Sensor Topic Subscribe */
    ros::Subscriber walking_command_sub = ros_node.subscribe("/robotis/quintic_walk/command", 10,
                                                             &QuinticWalk::walkingCommandCallback, this);
    ros::Subscriber walking_param_sub = ros_node.subscribe("/robotis/quintic_walk/set_params", 10,
                                                           &QuinticWalk::walkingParameterCallback, this);

    ros::Subscriber imu_callback = ros_node.subscribe("/arduino_controller/imu", 10, &QuinticWalk::IMUCallback, this);

    densis_pub_ = ros_node.advertise<alfarobi_msgs_srvs_actions::densis>("/alfarobi/densis",0);

    ros::WallDuration duration(control_cycle_msec_ / 1000.0);
    while(ros_node.ok())
        callback_queue.callAvailable(duration);

}

void QuinticWalk::loadParameter()
{
    YAML::Node node;
    ////std::cout<<"LOAD PARAMETER"<<std::endl;
    try
    {
        // load yaml
        node = YAML::LoadFile(param_path_.c_str());

    } catch (const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return;
    }

    //    YAML::Node doc = node["walk_zmp"];

    walking_param_.freq = node["freq"].as<double>();
    walking_param_.doubleSupportRatio = node["doubleSupportRatio"].as<double>();
    walking_param_.footDistance = node["footDistance"].as<double>();
    if(walking_param_.footDistance < 2*0.04)
        walking_param_.footDistance = 2*0.04;
    walking_param_.footRise = node["footRise"].as<double>();
    walking_param_.footPutDownZOffset = node["footPutDownZOffset"].as<double>();
    walking_param_.footPutDownPhase = node["footPutDownPhase"].as<double>();
    walking_param_.footApexPhase = node["footApexPhase"].as<double>();
    walking_param_.footOvershootRatio = node["footOvershootRatio"].as<double>();
    walking_param_.footOvershootPhase = node["footOvershootPhase"].as<double>();
    walking_param_.trunkHeight = node["trunkHeight"].as<double>();
    walking_param_.trunkPitch = node["trunkPitch"].as<double>();
    walking_param_.trunkPhase = node["trunkPhase"].as<double>();
    walking_param_.trunkXOffset = node["trunkXOffset"].as<double>();
    walking_param_.trunkYOffset = node["trunkYOffset"].as<double>();
    walking_param_.trunkSwing = node["trunkSwing"].as<double>();
    walking_param_.trunkPause = node["trunkPause"].as<double>();
    walking_param_.trunkXOffsetPCoefForward = node["trunkXOffsetPCoefForward"].as<double>();
    walking_param_.trunkXOffsetPCoefTurn = node["trunkXOffsetPCoefTurn"].as<double>();
    walking_param_.trunkPitchPCoefForward = node["trunkPitchPCoefForward"].as<double>();
    walking_param_.trunkPitchPCoefTurn = node["trunkPitchPCoefTurn"].as<double>();

    walking_param_.KP_P_qw = node["KP_P_qw"].as<double>();
    walking_param_.KD_P_qw = node["KD_P_qw"].as<double>();
    walking_param_.KI_P_qw = node["KI_P_qw"].as<double>();

    walking_param_.Angle_0 = node["Angle_0"].as<double>();
    walking_param_.Angle_1 = node["Angle_1"].as<double>();
    walking_param_.Angle_2 = node["Angle_2"].as<double>();
    walking_param_.Angle_3 = node["Angle_3"].as<double>();
    walking_param_.Angle_4 = node["Angle_4"].as<double>();

    walking_param_.Gyro_0 = node["Gyro_0"].as<double>();
    walking_param_.Gyro_1 = node["Gyro_1"].as<double>();
    walking_param_.Gyro_2 = node["Gyro_2"].as<double>();
    walking_param_.Gyro_3 = node["Gyro_3"].as<double>();
    walking_param_.Gyro_4 = node["Gyro_4"].as<double>();

    walking_param_.KD_0 = node["KD_0"].as<double>();
    walking_param_.KD_1 = node["KD_1"].as<double>();
    walking_param_.KD_2 = node["KD_2"].as<double>();
    walking_param_.KD_3 = node["KD_3"].as<double>();
    walking_param_.KD_4 = node["KD_4"].as<double>();

    walking_param_.KP_0 = node["KP_0"].as<double>();
    walking_param_.KP_1 = node["KP_1"].as<double>();
    walking_param_.KP_2 = node["KP_2"].as<double>();
    walking_param_.KP_3 = node["KP_3"].as<double>();
    walking_param_.KP_4 = node["KP_4"].as<double>();

    walking_param_.setpointPitch = node["setpointPitch"].as<double>();
    ////std::cout<<"Done Loading Quintic Param"<<std::endl;
}

void QuinticWalk::loadOffset()
{
    YAML::Node offset_node;
    try
    {
        // load yaml
        offset_node = YAML::LoadFile(offset_.c_str());

    } catch (const std::exception& e)
    {
        ROS_ERROR("Fail to load Offset yaml file.");
        return;
    }
    // YAML::Node config_ = offset_node["offset"];

    // for(int i=0; i<12; i++)
    // {
    //     offset[i] = config_[joint[i]].as<double>();
    // }
    try{
        offset[0] = offset_node["r_hip_y"].as<double>();
        offset[1] = offset_node["r_hip_r"].as<double>();
        offset[2] = offset_node["r_hip_p"].as<double>();
        offset[3] = offset_node["r_knee"].as<double>();
        offset[4] = offset_node["r_ank_p"].as<double>();
        offset[5] = offset_node["r_ank_r"].as<double>();
        offset[6] = offset_node["l_hip_y"].as<double>();
        offset[7] = offset_node["l_hip_r"].as<double>();
        offset[8] = offset_node["l_hip_p"].as<double>();
        offset[9] = offset_node["l_knee"].as<double>();
        offset[10] = offset_node["l_ank_p"].as<double>();
        offset[11] = offset_node["l_ank_r"].as<double>();
    }catch(const std::exception &e){
        ROS_ERROR("[alfarobi_motion]: %s", e.what());
    }

    ROS_INFO("Success to load Initial Position file.");
    ////std::cout<<"Done Loading Quintic Offset"<<std::endl;
    offset[0] = (offset[0] - 180.0) * DEGREE2RADIAN;
    offset[1] = (offset[1] - 180.0) * DEGREE2RADIAN;
    offset[2] = (offset[2] - 180.0) * DEGREE2RADIAN;
    offset[3] = (offset[3] - 180.0) * DEGREE2RADIAN;
    offset[4] = (offset[4] - 180.0) * DEGREE2RADIAN;
    offset[5] = (offset[5] - 180.0) * DEGREE2RADIAN;
    offset[6] = (offset[6] - 180.0) * DEGREE2RADIAN;
    offset[7] = (offset[7] - 180.0) * DEGREE2RADIAN;
    offset[8] = (offset[8] - 180.0) * DEGREE2RADIAN;
    offset[9] = (offset[9] - 180.0) * DEGREE2RADIAN;
    offset[10] = (offset[10] -180.0) * DEGREE2RADIAN;
    offset[11] = (offset[11] -180.0) * DEGREE2RADIAN;

    for(int i=0; i<12;i++) {
        ROS_INFO("%d: %f", i, offset[i]);
    }
}

void QuinticWalk::loadFuzzy()
{
    YAML::Node config_;
    try
    {
        // load yaml
        config_ = YAML::LoadFile(fuzzy_path_.c_str());

    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load fuzzy.yaml file. '%s' are you sure that is the directory?", fuzzy_path_.c_str());
        return;
    }

    YAML::Node i1 = config_["Angle"];
    Angle.DataAmount = i1["amount"].as<int>();
    for(int i=0; i<Angle.DataAmount; i++)
    {
        Angle.Bottom1.push_back(0.0);
        Angle.Upper1.push_back(0.0);
        Angle.Upper2.push_back(0.0);
        Angle.Bottom2.push_back(0.0);
    }
    ROS_INFO("Test Fuzzy");
    Angle.Bottom1[0] = i1["bottom1_1"].as<double>();
    Angle.Upper1[0] = i1["upper1_1"].as<double>();
    Angle.Upper2[0] = i1["upper2_1"].as<double>();
    Angle.Bottom2[0] = i1["bottom2_1"].as<double>();
    Angle.Bottom1[1] = i1["bottom1_2"].as<double>();
    Angle.Upper1[1] = i1["upper1_2"].as<double>();
    Angle.Upper2[1] = i1["upper2_2"].as<double>();
    Angle.Bottom2[1] = i1["bottom2_2"].as<double>();
    Angle.Bottom1[2] = i1["bottom1_3"].as<double>();
    Angle.Upper1[2] = i1["upper1_3"].as<double>();
    Angle.Upper2[2] = i1["upper2_3"].as<double>();
    Angle.Bottom2[2] = i1["bottom2_3"].as<double>();
    Angle.Bottom1[3] = i1["bottom1_4"].as<double>();
    Angle.Upper1[3] = i1["upper1_4"].as<double>();
    Angle.Upper2[3] = i1["upper2_4"].as<double>();
    Angle.Bottom2[3] = i1["bottom2_4"].as<double>();
    Angle.Bottom1[4] = i1["bottom1_5"].as<double>();
    Angle.Upper1[4] = i1["upper1_5"].as<double>();
    Angle.Upper2[4] = i1["upper2_5"].as<double>();
    Angle.Bottom2[4] = i1["bottom2_5"].as<double>();
    Angle.Bottom1[5] = i1["bottom1_6"].as<double>();
    Angle.Upper1[5] = i1["upper1_6"].as<double>();
    Angle.Upper2[5] = i1["upper2_6"].as<double>();
    Angle.Bottom2[5] = i1["bottom2_6"].as<double>();
    Angle.Bottom1[6] = i1["bottom1_7"].as<double>();
    Angle.Upper1[6] = i1["upper1_7"].as<double>();
    Angle.Upper2[6] = i1["upper2_7"].as<double>();
    Angle.Bottom2[6] = i1["bottom2_7"].as<double>();


    YAML::Node i2 = config_["Gyro"];
    Gyro.DataAmount = i2["amount"].as<int>();
    for(int i=0; i<Gyro.DataAmount; i++)
    {
        Gyro.Bottom1.push_back(0.0);
        Gyro.Upper1.push_back(0.0);
        Gyro.Upper2.push_back(0.0);
        Gyro.Bottom2.push_back(0.0);
    }
    Gyro.Bottom1[0] = i2["bottom1_1"].as<double>();
    Gyro.Upper1[0] = i2["upper1_1"].as<double>();
    Gyro.Upper2[0] = i2["upper2_1"].as<double>();
    Gyro.Bottom2[0] = i2["bottom2_1"].as<double>();
    Gyro.Bottom1[1] = i2["bottom1_2"].as<double>();
    Gyro.Upper1[1] = i2["upper1_2"].as<double>();
    Gyro.Upper2[1] = i2["upper2_2"].as<double>();
    Gyro.Bottom2[1] = i2["bottom2_2"].as<double>();
    Gyro.Bottom1[2] = i2["bottom1_3"].as<double>();
    Gyro.Upper1[2] = i2["upper1_3"].as<double>();
    Gyro.Upper2[2] = i2["upper2_3"].as<double>();
    Gyro.Bottom2[2] = i2["bottom2_3"].as<double>();

    //output
    YAML::Node o1 = config_["Kp"];
    Kp.DataAmount = o1["amount"].as<int>();
    for(int i=0; i<Kp.DataAmount; i++)
    {
        Kp.Bottom1.push_back(0.0);
        Kp.Upper1.push_back(0.0);
        Kp.Upper2.push_back(0.0);
        Kp.Bottom2.push_back(0.0);
    }
    Kp.Bottom1[0] = o1["bottom1_1"].as<double>();
    Kp.Upper1[0] = o1["upper1_1"].as<double>();
    Kp.Upper2[0] = o1["upper2_1"].as<double>();
    Kp.Bottom2[0] = o1["bottom2_1"].as<double>();
    Kp.Bottom1[1] = o1["bottom1_2"].as<double>();
    Kp.Upper1[1] = o1["upper1_2"].as<double>();
    Kp.Upper2[1] = o1["upper2_2"].as<double>();
    Kp.Bottom2[1] = o1["bottom2_2"].as<double>();
    Kp.Bottom1[2] = o1["bottom1_3"].as<double>();
    Kp.Upper1[2] = o1["upper1_3"].as<double>();
    Kp.Upper2[2] = o1["upper2_3"].as<double>();
    Kp.Bottom2[2] = o1["bottom2_3"].as<double>();
    Kp.Bottom1[3] = o1["bottom1_4"].as<double>();
    Kp.Upper1[3] = o1["upper1_4"].as<double>();
    Kp.Upper2[3] = o1["upper2_4"].as<double>();
    Kp.Bottom2[3] = o1["bottom2_4"].as<double>();
    Kp.Bottom1[4] = o1["bottom1_5"].as<double>();
    Kp.Upper1[4] = o1["upper1_5"].as<double>();
    Kp.Upper2[4] = o1["upper2_5"].as<double>();
    Kp.Bottom2[4] = o1["bottom2_5"].as<double>();

    YAML::Node o2 = config_["Kd"];
    Kd.DataAmount = o2["amount"].as<int>();
    for(int i=0; i<Kd.DataAmount; i++)
    {
        Kd.Bottom1.push_back(0.0);
        Kd.Upper1.push_back(0.0);
        Kd.Upper2.push_back(0.0);
        Kd.Bottom2.push_back(0.0);
    }
    Kd.Bottom1[0] = o2["bottom1_1"].as<double>();
    Kd.Upper1[0] = o2["upper1_1"].as<double>();
    Kd.Upper2[0] = o2["upper2_1"].as<double>();
    Kd.Bottom2[0] = o2["bottom2_1"].as<double>();
    Kd.Bottom1[1] = o2["bottom1_2"].as<double>();
    Kd.Upper1[1] = o2["upper1_2"].as<double>();
    Kd.Upper2[1] = o2["upper2_2"].as<double>();
    Kd.Bottom2[1] = o2["bottom2_2"].as<double>();
    Kd.Bottom1[2] = o2["bottom1_3"].as<double>();
    Kd.Upper1[2] = o2["upper1_3"].as<double>();
    Kd.Upper2[2] = o2["upper2_3"].as<double>();
    Kd.Bottom2[2] = o2["bottom2_3"].as<double>();

    YAML::Node cp = config_["CapturePoint"];
    CP_active = cp["CP_active"].as<bool>();
    CP_GAIN = cp["CP_GAIN"].as<double>();
}

void QuinticWalk::saveParameter()
{

    YAML::Node node;
    try
    {
        // load yaml
        node = YAML::LoadFile(param_path_.c_str());
    } catch (const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return;
    }

    node["freq"] = walking_param_.freq;
    node["doubleSupportRatio"] = walking_param_.doubleSupportRatio;
    node["footDistance"] = walking_param_.footDistance;
    node["footRise"] = walking_param_.footRise;
    node["footPutDownZOffset"] = walking_param_.footPutDownZOffset;
    node["footPutDownPhase"] = walking_param_.footPutDownPhase;
    node["footApexPhase"] = walking_param_.footApexPhase;
    node["footOvershootRatio"] = walking_param_.footOvershootRatio;
    node["footOvershootPhase"] = walking_param_.footOvershootPhase;
    node["trunkHeight"] = walking_param_.trunkHeight;
    node["trunkPitch"] = walking_param_.trunkPitch;
    node["trunkPhase"] = walking_param_.trunkPhase;
    node["trunkXOffset"] = walking_param_.trunkXOffset;
    node["trunkYOffset"] = walking_param_.trunkYOffset;
    node["trunkSwing"] = walking_param_.trunkSwing;
    node["trunkPause"] = walking_param_.trunkPause;
    node["trunkXOffsetPCoefForward"] = walking_param_.trunkXOffsetPCoefForward;
    node["trunkXOffsetPCoefTurn"] = walking_param_.trunkXOffsetPCoefTurn;
    node["trunkPitchPCoefForward"] = walking_param_.trunkPitchPCoefForward;
    node["trunkPitchPCoefTurn"] = walking_param_.trunkPitchPCoefTurn;
    node["KP_P_qw"] = walking_param_.KP_P_qw;
    node["KD_P_qw"] = walking_param_.KD_P_qw;
    node["KI_P_qw"] = walking_param_.KI_P_qw;
    node["Angle_0"] = walking_param_.Angle_0;
    node["Angle_1"] = walking_param_.Angle_1;
    node["Angle_2"] = walking_param_.Angle_2;
    node["Angle_3"] = walking_param_.Angle_3;
    node["Angle_4"] = walking_param_.Angle_4;
    node["Gyro_0"] = walking_param_.Gyro_0;
    node["Gyro_1"] = walking_param_.Gyro_1;
    node["Gyro_2"] = walking_param_.Gyro_2;
    node["Gyro_3"] = walking_param_.Gyro_3;
    node["Gyro_4"] = walking_param_.Gyro_4;
    node["KD_0"] = walking_param_.KD_0;
    node["KD_1"] = walking_param_.KD_1;
    node["KD_2"] = walking_param_.KD_2;
    node["KD_3"] = walking_param_.KD_3;
    node["KD_4"] = walking_param_.KD_4;
    node["KP_0"] = walking_param_.KP_0;
    node["KP_1"] = walking_param_.KP_1;
    node["KP_2"] = walking_param_.KP_2;
    node["KP_3"] = walking_param_.KP_3;
    node["KP_4"] = walking_param_.KP_4;
    node["setpointPitch"] = walking_param_.setpointPitch;

    std::ofstream fout(param_path_.c_str());
    fout << node;

}

// bool QuinticWalk::getWalkigParameterCallback(quintic_walk_msgs::GetWalkingParam::Request &req,
//                                              quintic_walk_msgs::GetWalkingParam::Response &res)
// {
//     res.parameters = walking_param_;

//     return true;
// }

void QuinticWalk::IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    Quaterniond imu_orientation;

    imu_orientation.x() = msg->orientation.x;
    imu_orientation.y() = msg->orientation.y;
    imu_orientation.z() = msg->orientation.z;
    imu_orientation.w() = msg->orientation.w;

    m_imuOri = robotis_framework::convertQuaternionToRPY(imu_orientation);

    m_imuGyr.x() = msg->angular_velocity.x;
    m_imuGyr.y() = msg->angular_velocity.y;
    m_imuGyr.z() = msg->angular_velocity.z;

    m_imuAcc.x() = msg->linear_acceleration.x;
    m_imuAcc.y() = msg->linear_acceleration.y;
    m_imuAcc.z() = msg->linear_acceleration.z;
}

void QuinticWalk::walkingParameterCallback(const alfarobi_msgs_srvs_actions::WalkingParam::ConstPtr &msg)
{
//    if(walking_param_.footDistance != msg->footDistance)
//    {
        walking_param_ = *msg;
        if(walking_param_.footDistance < 2*0.04)
            walking_param_.footDistance = 2*0.04;
        _footstep.setFootDistance(walking_param_.footDistance);
//        _footstep.reset(true);
//    }
//    else
//    {
//       walking_param_ = *msg;
//       _footstep.setFootDistance(walking_param_.footDistance);
//       //        _footstep.reset(true);

//    }
}

void QuinticWalk::walkingCommandCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "start")
    {
        walkingStatus("enable");
        _walkActive = true;
        _stopRequest = false;
        robot_state = RobotWalk;
    }
    else if (msg->data == "stop")
    {
        stop();
        walkingStatus("disable");
    }
    else if (msg->data == "save")
        saveParameter();
}

void QuinticWalk::walkingSpeedCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "fast")
       walking_param_.freq = 1.700;
    else if (msg->data == "slow")
        walking_param_.freq = 1.600;
    std::cout<< "frequency = " << walking_param_.freq <<std::endl;
   
}

void QuinticWalk::publishStatusMsg(unsigned int type, std::string msg)
{
    alfarobi_msgs_srvs_actions::StatusMsg status_msg;
    status_msg.header.stamp = ros::Time::now();
    status_msg.type = type;
    status_msg.module_name = "quintic_walk";
    status_msg.status_msg = msg;

    status_msg_pub_.publish(status_msg);
}

void QuinticWalk::write() {
    if(!is_moving) {
        time_start = ros::Time::now().toSec();
        is_moving = true;
        for(uint8_t i=0; i<20; i++) {
            result_->write(i+1, result_->deg2Bit(180 + result_->getJointValue()->goal[i]/DEGREE2RADIAN) , 1000); //2 detik
            
            ROS_INFO("WRITING");
        }
        // write(tempSeq->getJoint());
        // readAll();
    }
    // read(20);
    // ROS_INFO("JOINT VALUE: %f", present_position[19]);
    // temp_servo->write(20, temp_servo->deg2Bit(present_position[19]) , 3000); 
    // ROS_INFO("AAAAAAAa");
    time_now = ros::Time::now().toSec() - time_start;
    ROS_INFO("Time now: %f", time_now);
    if(time_now >= 2000/1000) {
        is_moving = false;
        // tempSeq = tempSeq->next;
        // for(int i=0; i<20; i++) {
        //     tempSeq->getJoint()->write[i] = false;
        // }
    }
}

// double QuinticWalk::read(int id) {
//     return result->read(id);
// }

void QuinticWalk::process()
{
    // auto t1 = boost::chrono::high_resolution_clock::now();

    // command dlu
    // if (enable_ == false)
    //     return;

    // std::cout<<"WALKING PARAM"<<walking_param_.XMove<<" "<<walking_param_.YMove<<" "<<walking_param_
    // .ZMove<<std::endl;

    ROS_INFO("Processing Quintic Walk");

    loadFuzzy();

    ROS_INFO("Test 2");

    int joint_size = 18 /*result_.size()*/;

    // present angle
    for (int state_iter = 0; state_iter < 18; state_iter++)
    {
        std::string _joint_name = result_->getJointValue()->name[state_iter];
        int joint_index = joint_table_[_joint_name];

        // robotis_framework::Dynamixel *dxl = NULL;
        // std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(_joint_name);
        // if (dxl_it != dxls.end())
        //     dxl = dxl_it->second;
        // else
        //     continue;

        current_position[_joint_name] = offset[joint_index]; /*result_->read(state_iter);*/
        current_position_.coeffRef(0, joint_index) = current_position[_joint_name];
        // uint data = dxl->dxl_state_->bulk_read_table_["hardware_error_status"];
    //     if(data != 0)
    //         ROS_ERROR("Device Error %s : %d", _joint_name.c_str(), data);

    }

    ROS_INFO("Test 3");

    for(int idx = 0; idx < 12; idx++) {
        current_joint_pos(idx) = joint_axis_direction_(idx)*(current_position_.coeffRef(0, idx)-offset[idx]);
        ROS_INFO("current %d: %f", idx, current_joint_pos[idx]);
    }

    ROS_INFO("Test 4");

    // tambahan Bagas, feedback + Kalman Filter dari densis
    if(mbc.tuneGain == true)
    {
        so.resetServoValues();
        mbc.kalman.K = mbc.kalmanGain(&mbc.sys_d,&mbc.kalman);
        mbc.dlqr.K = mbc.dlqrGain(&mbc.sys_d,&mbc.dlqr);
        // Calculate Fuzzy Gain here ... (cc: Gaby)
        // mbc.dlqr.Kr = mbc.feedforwardGain(&mbc.sys_d, &mbc.dlqr);
        mbc.dlqr.Kr = mbc.feedforwardGain(&mbc.sys_d, mbc.dlqr.K);

        fuzzyQuintic();
    }
    else
    {
        mbc.ymeas[0] = COMDSPrpy(1)-mbc.pitchRef*M_PI/180;//asin(DSP_COMrefX/COM_DSP_Length); //pitch
        mbc.ymeas[1] = m_imuGyr(1);
        if(mbc.testObserver == true)
        {
            if(densis.densisMode == true)
                densisInput();
            mbc.wholeBodyStateObserver(so.r_ank_pitch.refPosFromPos, mbc.wholeBodyStates, mbc.kalman.K, mbc.yest, mbc.ymeas);
        }
        else
        {
            // for observers
            mbc.wholeBodyStateObserver(so.r_ank_pitch.refPosFromPos, mbc.wholeBodyStates, mbc.kalman.K, mbc.yest, mbc.ymeas);
            // for controller
            feedbackDSPAnklePitchPos2Pos();
        }
    }

    ROS_INFO("Test 5");

    if(firstExc)
        walking_state_ = WalkingInitPose;

    wholeBodyCOM();
    wholeBodyCOM_DSP();
    wholeBodyCOM_LSSP();
    wholeBodyCOM_RSSP();
    forwardKinematic();

    ROS_INFO("Test 6");

    if(walking_state_ == WalkingInitPose)
    {
        if(firstExc)
        {
            int max_=0;
            init_pose_count_ = 0;

            _orders = {0.0, 0.0, 0.0};
            if(walking_param_.YMove < 0 || walking_param_.ZMove > 0)
                setOrders(_orders, false, false);
            else
                setOrders(_orders,false, true);

            calculateWalking();
            motion_arms();

            for (int idx = 0; idx < 12; idx++)
                max_ = std::max((int)fabs(current_joint_pos(idx) - joint_goals.at(idx)), max_);

            double mov_time = (max_ / 30) < 0.5 ? 0.5 : (max_/30);
            double smp_time = control_cycle_msec_ * 0.001;
            int all_time_steps = int(mov_time / smp_time) + 2; //+1
            calc_joint_tra_.resize(all_time_steps, 12);
            ROS_INFO_STREAM("row: "<<calc_joint_tra_.rows() <<" col:"<< calc_joint_tra_.cols());

            // for (int idx = 0; idx < 12; idx++)
            //     calc_joint_tra_.block(0, idx, all_time_steps, 1) = robotis_framework::calcMinimumJerkTra(current_joint_pos(idx), 0.0, 0.0,
            //                                                                                              joint_goals.at(idx), 0.0, 0.0,
            //                                                                                              smp_time, mov_time);

            for (int idx = 0; idx < 12; idx++){
                // ROS_INFO_STREAM("Index: " << idx);s
                ROS_INFO("A");
                calc_joint_tra_.block(0, idx, all_time_steps, 1) = robotis_framework::calcMinimumJerkTra(current_joint_pos(idx), 0.0, 0.0,
                                                                                                         joint_goals.at(idx), 0.0, 0.0,
                                                                                                         smp_time, mov_time);
                ROS_INFO("Test EIGEN");
            }
            
            
        }
        firstExc = false;

        for (int idx = 0; idx < 12; idx++)
            joint_goals.at(idx) = calc_joint_tra_(init_pose_count_, idx);

        init_pose_count_ += 1;

        if(init_pose_count_ >= calc_joint_tra_.rows())
            walking_state_ = WalkingReady;

        if(_walkActive)
            robot_state = RobotWalk;
        else
            robot_state = RobotStop;
        
        ROS_INFO("Test 7");
    }
    else if (walking_state_ == WalkingReady || walking_state_ == WalkingEnable)
    {
        // simpleCapturePoint();

        switch(robot_state)
        {
            case RobotStop:
                _orders = {0.0, 0.0, 0.0};
                if(walking_param_.YMove < 0 || walking_param_.ZMove > 0)
                    setOrders(_orders, false, false);
                else
                    setOrders(_orders,false, true);

                if(fabs(mbc.ymeas[0]*180/PI)>3 && fabs(mbc.wholeBodyStates[1]*180/PI)>20 && CP_active==true)
                {
                    walkingParam = {calcCapturePoint()*CP_GAIN, walking_param_.YMove, walking_param_.ZMove};
                    robot_state = RobotCaptureStep;
                    robot_previous_state = RobotStop;
                    // std::cout<<"CAPTURE POINT : "<<walkingParam(0)*100<<" cm"<<std::endl;
                }
                break;
            case RobotWalk:
                if(walking_param_.YMove < 0 || walking_param_.ZMove > 0)
                    setOrders(_orders, true, false);
                else
                    setOrders(_orders, true, true);

                walkingParam = {walking_param_.XMove, walking_param_.YMove, walking_param_.ZMove};

                if(fabs(mbc.ymeas[0]*180/PI)>3 && fabs(mbc.wholeBodyStates[1]*180/PI)>20 && CP_active==true)
                {
                    walkingParam = {(calcCapturePoint()*CP_GAIN)+walking_param_.XMove, walking_param_.YMove, walking_param_.ZMove};
                    robot_state = RobotCaptureStep;
                    robot_previous_state = RobotWalk;
                    // std::cout<<"CAPTURE POINT : "<<walkingParam(0)*100<<" cm"<<std::endl;
                }
                break;
            case RobotCaptureStep:
                if(walking_param_.YMove < 0 || walking_param_.ZMove > 0)
                    setOrders(_orders, true, false);
                else
                    setOrders(_orders, true, true);

                if(robot_previous_state == RobotWalk)
                    walkingParam = {(calcCapturePoint()*CP_GAIN)+walking_param_.XMove, walking_param_.YMove, walking_param_.ZMove};
                else
                    walkingParam = {calcCapturePoint()*CP_GAIN, walking_param_.YMove, walking_param_.ZMove};
                // std::cout<<"CAPTURE POINT : "<<walkingParam(0)*100<<" cm"<<std::endl;

                if(fabs(mbc.ymeas[0]*180/PI)>10 && fabs(mbc.wholeBodyStates[1]*180/PI)>10)
                    CP_start = ros::Time::now().toSec();

                if(ros::Time::now().toSec()-CP_start>1)
                    robot_state = robot_previous_state;
                break;
        }

        // std::cout<<"Robot State : "<<robot_state<<std::endl;

        calculateWalking();
        motion_arms();

        ROS_INFO("Test 8");
    }

    // Servo Feedback Angle Limitter
    if(feedback(4) <= -30*DEGREE2RADIAN) //-60
        feedback(4)=-30*DEGREE2RADIAN; //Right_Ankle Pitch
    else if(feedback(4) > 30*DEGREE2RADIAN) //20
        feedback(4)=30*DEGREE2RADIAN; //Right_Ankle Pitch
    else if(feedback(5) <= -30*DEGREE2RADIAN) //-30
        feedback(5)=-30*DEGREE2RADIAN; //Right Ankle Roll
    else if(feedback(5) > 30*DEGREE2RADIAN) //30
        feedback(5)=30*DEGREE2RADIAN; //Right Ankle Roll
    else if(feedback(10) <= -30*DEGREE2RADIAN) //-60
        feedback(10)=-30*DEGREE2RADIAN; //Left Ankle Pitch
    else if(feedback(10) > 30*DEGREE2RADIAN) //20
        feedback(10)=30*DEGREE2RADIAN; //Left Ankle Pitch
    else if(feedback(11) <= -30*DEGREE2RADIAN) //-30
        feedback(11)=-30*DEGREE2RADIAN; //Left Ankle Roll
    else if(feedback(11) > 30*DEGREE2RADIAN) //30
        feedback(11)=30*DEGREE2RADIAN; //Left Ankle Roll

    ROS_INFO("Test 9");

    for (int idx = 0; idx < 18; idx++)
    {
        double goal_position = 0.0;
        goal_position = init_position_.coeff(0, idx) + (joint_goals.at(idx) * joint_axis_direction_(idx)) + (feedback(idx) * joint_axis_direction_(idx));

        target_position_.coeffRef(0, idx) = goal_position;
    }

    ROS_INFO("Test 10");

    so.r_ank_pitch.positionIK = target_position_.coeffRef(0, 4);
    so.r_ank_pitch.positionNow = target_position_.coeffRef(0, 5);
    so.l_ank_pitch.positionIK = target_position_.coeffRef(0, 10);
    so.l_ank_pitch.positionNow = target_position_.coeffRef(0, 11);

    ROS_INFO("Test 11");

    // set result
    for (int state_iter = 0; state_iter < 18; state_iter++)
    {
        std::string joint_name = result_->getJointValue()->name[state_iter];
        int joint_index = joint_table_[joint_name];

        

        // result_[joint_name]->goal_position_ = target_position_.coeff(0, joint_index);
        // double target = target_position_.coeff(0, joint_index);
        result_->getJointValue()->goal[joint_index] = target_position_.coeff(0, joint_index);
        op3_kd_->setJointPos(joint_name, result_->getJointValue()->goal[joint_index]);

        // write();
        ROS_INFO("%s: %f", joint_name.c_str(), result_->getJointValue()->goal[joint_index]);
    }
    ROS_INFO("Test 12");
    densisPublish();
  //   auto t2 = boost::chrono::high_resolution_clock::now();
  // auto elapsed_time_2 = boost::chrono::duration_cast<boost::chrono::milliseconds>(t2-t1).count();
  // std::cout << "[QUINTIC] Elapsed Process Time : " << elapsed_time_2 << "miliseconds"<<std::endl;
}

void QuinticWalk::fuzzyQuintic()
{
    // ANGLE.input = COMDSPrpy(1)*180/PI;
    ANGLE.input = mbc.ymeas[0]; // COMDSPrpy(1)*180/PI-mbc.pitchRef;
    GYRO.input = mbc.wholeBodyStates[1];

    //FUZZY
    ANGLE.setMembership(Angle.DataAmount, Angle.Bottom1, Angle.Upper1, Angle.Upper2, Angle.Bottom2);
    GYRO.setMembership(Gyro.DataAmount, Gyro.Bottom1, Gyro.Upper1, Gyro.Upper2, Gyro.Bottom2);
    KP.setMembership(Kp.DataAmount, Kp.Bottom1, Kp.Upper1, Kp.Upper2, Kp.Bottom2);
    KD.setMembership(Kd.DataAmount, Kd.Bottom1, Kd.Upper1, Kd.Upper2, Kd.Bottom2);

    Eigen::MatrixXi ruleKP(Angle.DataAmount, Gyro.DataAmount), ruleKD(Angle.DataAmount, Gyro.DataAmount);

    ruleKP << 3, 2, 3,
              2, 1, 2,
              1, 1, 1,
              2, 1, 2,
              3, 2, 3;
    ruleKD << 2, 2, 2,
              2, 2, 2,
              1, 1, 1,
              2, 2, 2,
              2, 2, 2;

    KP.setRule(Angle.DataAmount, Gyro.DataAmount, ruleKP);
    KD.setRule(Angle.DataAmount, Gyro.DataAmount, ruleKD);

    gainFuzzy[0][0] = KP.outputCentroid(KP.membership(ANGLE.predicate(), GYRO.predicate(), Angle.DataAmount, Gyro.DataAmount));
    gainFuzzy[0][1] = KD.outputCentroid(KD.membership(ANGLE.predicate(), GYRO.predicate(), Angle.DataAmount, Gyro.DataAmount));

    if(isnan(gainFuzzy[0][0]))
        gainFuzzy[0][0] = 0;
    if(isnan(gainFuzzy[0][1]))
        gainFuzzy[0][1] = 0;
}

void QuinticWalk::calculateWalking()
{
    /*
    This method computes the next motor goals as well as the odometry if the step was changed.
    */

    // save last step odometry if support foot changes
    // from max at 0.1
    // stepFeedback = 0;
    _orders = {walkingParam(0), walkingParam(1), walkingParam(2)}; //{walking_param_.XMove, walking_param_.YMove, walking_param_.ZMove};
    // std::cout<<walkingParam(0)<<" "<<walkingParam(1)<<" "<<walkingParam(2)<<std::endl;
    _orders /=10;
    _stepOdom = getFootstep().getNext();

    double dt = 0.01;
    std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();
    // only take real time difference if walking was not stopped before
    // using c++ time since it is more performant than ros time. We only need a local difference, so it doesnt matter
    if(! _just_started){
        auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - _last_update_time);
        dt = time_diff_ms.count() / 1000.0;
        if(dt == 0){
            ROS_WARN("dt was 0");
            dt = 0.001;
        }
    }
    _just_started = false;
    _last_update_time = current_time;
    //    ROS_INFO("calc1 dt %f", dt);
    // compute new values from splines
    update(dt); //0.005); //todo 1.0/_engineFrequency);
    // read the positions and orientations for trunk and fly foot
    computeCartesianPosition(_trunkPos, _trunkAxis, _footPos, _footAxis, _isLeftSupport);

    // check if support foot has changed
    if(_isLeftSupport != _wasLeftSupport){
        _wasLeftSupport = _isLeftSupport;
        // add odometry change of last step to trunk odom if step was completed
        // make transform
        tf::Transform step;
        step.setOrigin(tf::Vector3{_stepOdom[0], _stepOdom[1], 0.0});
        tf::Quaternion tf_quat = tf::Quaternion();
        tf_quat.setRPY(0, 0, _stepOdom[2]);
        step.setRotation(tf_quat);

        // transform global odometry
        _supportFootOdom = _supportFootOdom * step;

        //check if the walking came to a complete stop
        if(_stopRequest){
            _walkActive = false;
            robot_state = RobotStop;
            _just_started = true;
            return;
        }
    }

    // change goals from support foot based coordinate system to trunk based coordinate system
    tf::Vector3 tf_vec;
    tf::vectorEigenToTF(_trunkPos, tf_vec);
    tf::Quaternion tf_quat = tf::Quaternion();
    tf_quat.setRPY(_trunkAxis[0], _trunkAxis[1], _trunkAxis[2]);
    tf_quat.normalize();
    tf::Transform support_foot_to_trunk(tf_quat, tf_vec);
    tf::Transform trunk_to_support_foot_goal = support_foot_to_trunk.inverse();

    tf::vectorEigenToTF(_footPos, tf_vec);
    tf_quat.setRPY(_footAxis[0], _footAxis[1], _footAxis[2]);
    tf_quat.normalize();
    tf::Transform support_to_flying_foot(tf_quat, tf_vec);
    tf::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * support_to_flying_foot;

    // call ik solver
    bool success = false;
    std::vector<double> Leg_Pos;
    std::vector<double> L_Leg;

    success = ik_solver.solve(trunk_to_support_foot_goal, trunk_to_flying_foot_goal, Leg_Pos, getFootstep().isLeftSupport());


    //     publish goals if sucessfull
    if(success)
    {
        for(int i=0;i<12;i++)
        {
            joint_goals[i] = Leg_Pos[i];
        }

    }
}

void QuinticWalk::forwardKinematic()
{
    r_hip.linear()   = Eigen::Matrix3d::Identity();
    r_knee.linear()  = Eigen::Matrix3d::Identity();
    r_ankle.linear() = Eigen::Matrix3d::Identity();
    l_hip.linear()   = Eigen::Matrix3d::Identity();
    l_knee.linear()  = Eigen::Matrix3d::Identity();
    l_ankle.linear() = Eigen::Matrix3d::Identity();

    r_hip.rotate(   AngleAxisd(current_joint_pos(0) , Vector3d( 0, 0, 1) * joint_axis_direction_(0)) *
                    AngleAxisd(current_joint_pos(1) , Vector3d( 1, 0, 0) * joint_axis_direction_(1)) *
                    AngleAxisd(current_joint_pos(2) , Vector3d( 0, 1, 0) * joint_axis_direction_(2)));
    r_knee.rotate(  AngleAxisd(current_joint_pos(3) , Vector3d( 0, 1, 0) * joint_axis_direction_(3)));
    r_ankle.rotate( AngleAxisd(current_joint_pos(4) , Vector3d( 0, 1, 0) * joint_axis_direction_(4)) *
                    AngleAxisd(current_joint_pos(5) , Vector3d( 1, 0, 0) * joint_axis_direction_(5)));
    l_hip.rotate(   AngleAxisd(current_joint_pos(6) , Vector3d( 0, 0, 1) * joint_axis_direction_(6)) *
                    AngleAxisd(current_joint_pos(7) , Vector3d( 1, 0, 0) * joint_axis_direction_(7)) *
                    AngleAxisd(current_joint_pos(8) , Vector3d( 0, 1, 0) * joint_axis_direction_(8)));
    l_knee.rotate(  AngleAxisd(current_joint_pos(9) , Vector3d( 0, 1, 0) * joint_axis_direction_(9)));
    l_ankle.rotate( AngleAxisd(current_joint_pos(10), Vector3d( 0, 1, 0) * joint_axis_direction_(10)) *
                    AngleAxisd(current_joint_pos(11), Vector3d( 1, 0, 0) * joint_axis_direction_(11)));

    m_r_foot = base * r_hip * r_knee * r_ankle * r_foot;
    m_l_foot = base * l_hip * l_knee * l_ankle * l_foot;

    R_FOOT_WORLD = m_r_foot.translation();
    L_FOOT_WORLD = m_l_foot.translation();

    L_FOOT_WORLD(0) *= -1;

    R_ROT_WORLD = robotis_framework::convertRotationToRPY(m_r_foot.linear());
    L_ROT_WORLD = robotis_framework::convertRotationToRPY(m_l_foot.linear());

    //////std::cout<<"L_Z: "<<std::fixed << std::setprecision(4)<<L_FOOT_WORLD(2)<<"\tR_Z = "<<std::fixed << std::setprecision(4)<<R_FOOT_WORLD(2)<<std::endl;
    if(L_FOOT_WORLD(2) > R_FOOT_WORLD(2) && fabs(L_FOOT_WORLD(2)-R_FOOT_WORLD(2))>0.01){
        IndexSupport = 2;//1; //SSP Kanan
        IndexSupport_capture =2;
    }else if(L_FOOT_WORLD(2) < R_FOOT_WORLD(2) && fabs(L_FOOT_WORLD(2)-R_FOOT_WORLD(2))>0.01){
        IndexSupport = 1;//2; //SSP Kiri
        IndexSupport_capture = 1;
    }else if(fabs(L_FOOT_WORLD(2)-R_FOOT_WORLD(2))<=0.01){
        IndexSupport = 0;
    }

    ROS_INFO("TEST EIGEN");
}

void QuinticWalk::feedbackDSPAnklePitchPos2Pos()
{
    double input, input_temp;

    fuzzyQuintic(); //uncomment kl mau pakai fuzzy

    // if(fabs(mbc.wholeBodyStates[1]) > 20*PI/180)
    // {
    //     // std::cout<<"RAW DATA : "<<mbc.wholeBodyStates[1];
    //     mbc.wholeBodyStates[1] = skripsiEMA(mbc.wholeBodyStates[1]);
    //     // std::cout<<"\t EMA : "<<mbc.wholeBodyStates[1]<<std::endl;
    // }
    // else
    // {
    //     for(int i=0; i<bufferSkripsiEMA.size(); i++)
    //     // bufferSkripsiEMA.clear();
    //     bufferSkripsiEMA[i] = 0;
    // }

    // mbc.wholeBodyStates[0] = mbc.ymeas[0];
    mbc.dlqr.Kr = mbc.feedforwardGain(&mbc.sys_d, gainFuzzy); //uncomment kl mau pakai fuzzy
    input = mbc.outputFeedback(mbc.pitchRef*M_PI/180,mbc.wholeBodyStates,gainFuzzy,mbc.dlqr.Kr); //gain K fuzzy

    // input = mbc.outputFeedback(mbc.pitchRef*M_PI/180,mbc.wholeBodyStates,mbc.dlqr.K,mbc.dlqr.Kr); //murni gain K LQR
    /*
    if(COMDSPrpy(1)*180/PI > mbc.batas)
        integralPitch += (((mbc.pitchRef-mbc.batas)*PI/180-COMDSPrpy(1))*mbc.sys_d.Ts);
    else if(COMDSPrpy(1)*180/PI < -mbc.batas)
        integralPitch += (((mbc.pitchRef+mbc.batas)*PI/180-COMDSPrpy(1))*mbc.sys_d.Ts);
    else
        integralPitch = 0;

    // integralPitch += ((COMDSPrpy(1)-(mbc.pitchRef)*PI/180)*mbc.sys_d.Ts);
    // if(fabs(COMDSPrpy(1)*180/PI) > mbc.batas)
        input += (mbc.GainIntegral * integralPitch);

    std::cout<<"Integral Pitch : "<<integralPitch<<std::endl;

    if(fabs(COMDSPrpy(1)-mbc.pitchRef*PI/180) > 20*PI/180)
    {
        input = input_temp;
        // std::cout<<"GEDHE SUDUTNYA!"<<std::endl;
    }
    else
    {
        input_temp = input;
        // std::cout<<"SUDUT KECIL"<<std::endl;
    }
    */
    so.r_ank_pitch.refPosFromPos = input;
    so.l_ank_pitch.refPosFromPos = input;
    feedback(10) = so.l_ank_pitch.refPosFromPos;
    feedback(4) = so.r_ank_pitch.refPosFromPos;
    // std::cout<<"COMDSPpitch(DEG): "<<mbc.ymeas[0]*180/M_PI<<"\tgyroPitch(DEG/S): "<<mbc.ymeas[1]*180/M_PI<<"CtrlEffort(DEG):"<<so.l_ank_pitch.refPosFromPos*180/M_PI<<std::endl;
    // std::cout<<"L_Ankle_Pitch_Effort(DEG): "<<feedback(10)*180/PI<<std::endl;
    // std::cout<<"R_Ankle_Pitch_Effort(DEG): "<<feedback(4)*180/PI<<std::endl;
}

// Exponential Moving Average
double QuinticWalk::skripsiEMA(double rawData)
{
    double alpha = 0.4;
    double denominator = 0;
    double additionResult, filteredData;

    // Input from left(0)
    bufferSkripsiEMA.at(0) = rawData;

    // Average Filter with Buffer
    for(int i=0; i < CUPLIK_EMA; i++)
    {
        additionResult += (bufferSkripsiEMA[i] * pow((1-alpha),i));
        denominator += pow((1-alpha),i);
    }

    filteredData = additionResult/denominator;
    additionResult = 0;

    //Shift to right (+1)
    for (size_t i = 0; i < CUPLIK_EMA - 1; i++)
        bufferSkripsiEMA.at(i+1) = bufferSkripsiEMA.at(i);

    return filteredData;
}

double QuinticWalk::EMAV2(const std::vector<double> movAvg,double n ){
    double alpha = 2/(n+1);
    double sum, numerator, denominator, sum_numerator, sum_denominator;
    for (int i=0;i<=n;i++){
        if (i==0){
        numerator = movAvg[i];
        denominator = 1;
        }
        else {
        numerator = pow((1-alpha),i) * movAvg[i];
        denominator = pow((1-alpha),i);
        }
        sum_numerator += numerator;
        sum_denominator += denominator;
        // equation = pow((1-alpha),i+1)*args[i]/pow((1-alpha),i+1);
        sum = sum_numerator/sum_denominator;
    }
    return sum;
}

void QuinticWalk::wholeBodyCOM()
{
    rotIMU = robotis_framework::convertRPYToRotation(m_imuOri(0),m_imuOri(1),0); // Tanpa Yaw
    op3_kd_->op3_link_data_[0]->orientation_=rotIMU;
    op3_kd_->calcForwardKinematics(0);
    COM = op3_kd_->calcCOM(0);
    // std::cout<<"WholeBodyCOM x"<<COM(0)<<"\ty:"<<COM(1)<<"\tz"<<COM(2)<<std::endl;

    ROS_INFO("TEST EIGEN");
}


void QuinticWalk::wholeBodyCOM_DSP()
{
    // COM_DSP = (COM_LSSP+COM_RSSP)/2;
    // passive x,y,z = 23,24,25 //l_leg_end = 30
//   op3_kd_->op3_link_data_[0]->orientation_DSP_ = rotIMU;

  for(int i=0; i<=2; i++)
  {
      posLFootFromBase(i) = -op3_kd_->op3_link_data_[30]->position_(i);
      posRFootFromBase(i) = -op3_kd_->op3_link_data_[31]->position_(i);
  }
  posDSPFromBase = (posLFootFromBase + posRFootFromBase)/2;
//   std::cout<<"posDSPfromBase: "<<posDSPFromBase<<std::endl;
/*
  passiveDSP = rotIMU.inverse()*(posDSPFromBase);
  for(int i=0; i<=2; i++)
    op3_kd_->op3_link_data_[25]->relative_position_DSP_(i) = passiveDSP(i);

  op3_kd_->calcFKDSP(0);
  COM_DSP = op3_kd_->calcCOM_DSP(0);
  */
  COM_DSP = COM-posDSPFromBase;
  COM_DSP_Length = COM_DSP.norm();
  COMDSPrpy(0) = -atan(COM_DSP(1)/COM_DSP(2));
  COMDSPrpy(1) =  atan(COM_DSP(0)/COM_DSP(2));
//   std::cout<<"<<<DSP COM x"<<COM_DSP(0)<<"\ty:"<<COM_DSP(1)<<"\tz"<<COM_DSP(2)<<std::endl;
}

void QuinticWalk::wholeBodyCOM_LSSP()
{
  COM_LSSP = COM-posLFootFromBase;
  COMLSSPrpy(0) = -atan(COM_LSSP(1)/COM_LSSP(2));
  COMLSSPrpy(1) =  atan(COM_LSSP(0)/COM_LSSP(2));
//   std::cout<<"<<<LSSP COM x"<<COM_LSSP(0)<<"\ty:"<<COM_LSSP(1)<<"\tz"<<COM_LSSP(2)<<std::endl;
}

void QuinticWalk::wholeBodyCOM_RSSP()
{
  COM_RSSP = COM-posRFootFromBase;
  COMRSSPrpy(0) = -atan(COM_RSSP(1)/COM_RSSP(2));
  COMRSSPrpy(1) =  atan(COM_RSSP(0)/COM_RSSP(2));
//   std::cout<<"<<<RSSP COM x"<<COM_RSSP(0)<<"\ty:"<<COM_RSSP(1)<<"\tz"<<COM_RSSP(2)<<std::endl;
}

void QuinticWalk::walkingReset(){
    /*
    Resets the walking and stops it *imediatly*. This means that it can also stop during a step, thus in an
    unstable position. Should be normally used when the robot is already falling.
    */
    ////std::cout<< "resetting" << std::endl;

    _orders = {0.0, 0.0, 0.0};
    setOrders(_orders, false, true);
    _walkActive = false;
    robot_state = RobotStop;
    _just_started = true;
    ////std::cout<< "finished" << std::endl;

}

void QuinticWalk::motion_arms()
{
    joint_goals[12]=0;
    joint_goals[13]=0*DEGREE2RADIAN;
    joint_goals[14]=0*DEGREE2RADIAN;
    joint_goals[15]=0;
    joint_goals[16]=0*DEGREE2RADIAN;
    joint_goals[17]=0*DEGREE2RADIAN;
}

void QuinticWalk::walkingStatus(const std::string &command)
{
  std_msgs::String _commnd_msg;
  _commnd_msg.data = command;
  walking_status_pub_.publish(_commnd_msg);
  //////////std::cout << "kickStatus : " << _commnd_msg.data << std::endl;
}

void QuinticWalk::densisPublish()
{
    tf::pointEigenToMsg(m_imuOri,densisMsgs.IMUorientation);
    tf::pointEigenToMsg(m_imuGyr,densisMsgs.IMUgyro);
    tf::pointEigenToMsg(COM_LSSP, densisMsgs.COMLSSP);
    tf::pointEigenToMsg(COM_RSSP, densisMsgs.COMRSSP);
    tf::pointEigenToMsg(COM_DSP, densisMsgs.COMDSP);
    tf::pointEigenToMsg(COMLSSPrpy, densisMsgs.COMLSSPrpy);
    tf::pointEigenToMsg(COMRSSPrpy, densisMsgs.COMRSSPrpy);
    tf::pointEigenToMsg(COMDSPrpy, densisMsgs.COMDSPrpy);
    // tf::pointEigenToMsg(ZMPLSSP_accFil, densisMsgs.ZMPLSSP);
    // tf::pointEigenToMsg(ZMPRSSP_accFil, densisMsgs.ZMPRSSP);
    // tf::pointEigenToMsg(ZMPDSP_accFil, densisMsgs.ZMPDSP);

    // Servo Data Log Messages
    densisMsgs.LAnklePitch_velocityNow   = so.l_ank_pitch.velocityNow;
    densisMsgs.LAnklePitch_positionIK    = so.l_ank_pitch.positionIK;
    densisMsgs.LAnklePitch_positionNow   = so.l_ank_pitch.positionNow;
    densisMsgs.LAnklePitch_refPosFromPos = so.l_ank_pitch.refPosFromPos;

    densisMsgs.RAnklePitch_velocityNow   = so.r_ank_pitch.velocityNow;
    densisMsgs.RAnklePitch_positionIK    = so.r_ank_pitch.positionIK;
    densisMsgs.RAnklePitch_positionNow   = so.r_ank_pitch.positionNow;
    densisMsgs.RAnklePitch_refPosFromPos = so.r_ank_pitch.refPosFromPos;

    densisMsgs.COMDSPrpy_pitch_Est = mbc.yest[0];//wholeBodyStates[0];
    densisMsgs.gyro_pitch_Est = mbc.yest[1];//wholeBodyStates[1];

    densisMsgs.COMDSPrpy_pitch_Meas = mbc.ymeas[0];//ymeas[0];
    densisMsgs.gyro_pitch_Meas = mbc.ymeas[1];//ymeas[1];
    densis_pub_.publish(densisMsgs);
}

double QuinticWalk::calcCapturePoint()
{
    double capturePoint;

    Eigen::Vector3d omega_dot;
    Eigen::Vector3d accel_COM;
    Eigen::MatrixXd omega_cross;
    Eigen::Vector3d pelvisToCOM;

    Eigen::Vector3d gyro;
    gyro(0) = mbc.wholeBodyStates[0];
    gyro(1) = mbc.wholeBodyStates[1];
    gyro(2) = mbc.wholeBodyStates[2];

    if(IndexSupport==2) //SSP Kanan
    {
        omega_dot(0) = -(m_imuAcc(1))/posRFootFromBase.norm();
        omega_dot(1) = (m_imuAcc(0))/posRFootFromBase.norm();
        omega_dot(2) = m_imuAcc(2);
        pelvisToCOM = COM-posRFootFromBase;
    }
    else if(IndexSupport==1) //SSP Kiri
    {
        omega_dot(0) = -(m_imuAcc(1))/posLFootFromBase.norm();
        omega_dot(1) = (m_imuAcc(0))/posLFootFromBase.norm();
        omega_dot(2) = m_imuAcc(2);
        pelvisToCOM = COM-posLFootFromBase;
    }
    else
    {
        omega_dot(0) = -(m_imuAcc(1))/posDSPFromBase.norm();
        omega_dot(1) = (m_imuAcc(0))/posDSPFromBase.norm();
        omega_dot(2) = m_imuAcc(2);
        pelvisToCOM = COM-posDSPFromBase;
    }
    omega_cross = robotis_framework::calcHatto(gyro);
    accel_COM = m_imuAcc + omega_cross*(omega_cross*pelvisToCOM) + robotis_framework::calcHatto(omega_dot)*pelvisToCOM;

    // capturePoint -> x_dot . sqrt(z_com / g)
    if(IndexSupport==2) //SSP Kanan
        capturePoint = gyro(1) * COM_DSP(2)  * sqrt(COM_DSP(2) / G_CONSTANT);
    else //SSP Kiri dan DSP
        capturePoint = gyro(1) * COM_DSP(2) * sqrt(COM_DSP(2) / G_CONSTANT);

    return capturePoint;
}

//tambahin first active timer jgn lgsg on
void QuinticWalk::simpleCapturePoint()
{
  orientation_y_capture = COMDSPrpy(1) - (walking_param_.setpointPitch * DEGREE2RADIAN);

  if(firstCond){
    firstTime = ros::Time::now().toSec();
    firstCond = false;
  }

  timeNow = ros::Time::now().toSec() - firstTime;

  if(timeNow > 3){
    firstActive = true;
  }

  if ((orientation_y_capture > (7 * DEGREE2RADIAN) ) /*|| (orientation_y_capture < -(8 * DEGREE2RADIAN)))*/ && firstActive)
  {
      firstActive = false;
      lastXMove = walking_param_.XMove;
      Index_tmp = IndexSupport_capture;
      supportCount = 0;

      if (orientation_y_capture > (0 * DEGREE2RADIAN))
      {
          walking_param_.XMove = 0.350;
      }
      // else (orientation_y < (0 * RADIAN2DEGREE))
      // {
      //     walking_param_.XMove = -0.200;
      // }
      countCapture = true;
  }


  if (Index_tmp != IndexSupport_capture && countCapture)
  {

        // std::cout<<"HUAWAWADADADDADADADADADADAD\t"<<std::endl;
      supportCount++;
      Index_tmp = IndexSupport_capture;
  }

  if (supportCount == 2)
  {
    // std::cout<<"HUAWAWADADADDADADADADADADAD2222222222222222\t"<<std::endl;
      walking_param_.XMove = 0.2;
      firstActive = true;
      countCapture = false;
  }

  if (orientation_y_capture > 6 * DEGREE2RADIAN)
  {
        walking_param_.trunkPitch = 7 * DEGREE2RADIAN;
  }
  else
  {
        walking_param_.trunkPitch = 12 * DEGREE2RADIAN;
  }
  // std::cout<<"<<<<<==========orientation_y=========>>>>>>>\t"<<orientation_y_capture * RADIAN2DEGREE<<std::endl;
  // std::cout<<"<<<<<==========XMove=================>>>>>>>\t"<<walking_param_.XMove<<std::endl;
  // std::cout<<"<<<<<==========HIP=================>>>>>>>\t"<<walking_param_.trunkPitch<<std::endl;
}

void QuinticWalk::densisInput()
{
    densis.m_time = ros::Time::now().toSec() - densis.m_time_start;
    densis.inputSignal = densis.densisInput(densis.m_time);
    so.r_ank_pitch.refPosFromPos = densis.inputSignal;
    so.l_ank_pitch.refPosFromPos = densis.inputSignal;
    feedback(4) = so.r_ank_pitch.refPosFromPos; //R_Ankle_Pitch
    feedback(5) = 0; //R_Ankle_Roll
    feedback(10) = so.l_ank_pitch.refPosFromPos; //L_Ankle_Pitch
    feedback(11) = 0; //L_Ankle_Roll
}

}
