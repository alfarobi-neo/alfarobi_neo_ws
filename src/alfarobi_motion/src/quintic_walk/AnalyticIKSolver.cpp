#include "quintic_walk/AnalyticIKSolver.hpp"
namespace robotis_op{

AnalyticIKSolver::AnalyticIKSolver(){

}

//AnalyticIKSolver::AnalyticIKSolver(std::string robot_type, const robot_state::JointModelGroup &lleg_joints_group, const robot_state::JointModelGroup &rleg_joints_group){
//    _robot_type = robot_type;
//    _lleg_joints_group = &lleg_joints_group;
//    _rleg_joints_group = &rleg_joints_group;
//}

bool AnalyticIKSolver::solve(tf::Transform& trunk_to_support_foot, tf::Transform& trunk_to_flying_foot, std::vector<double>& positions, bool is_left_support) {
    std::vector<double> support_joints;
    std::vector<double> fly_joints;
    tf::Transform goal = tf::Transform();
    tf::Quaternion tf_quat = tf::Quaternion();
    tf::Vector3 tf_vec;

    //todo read from URDF
    tf::Vector3 trunk_to_LHipYaw;
    tf::Vector3 trunk_to_RHipYaw;
    trunk_to_LHipYaw.setValue(0.02134, 0.055, -0.090436);
    trunk_to_RHipYaw.setValue(0.02134, -0.055, -0.090436);


    // compute joint positions for fly and support leg
    bool success = true;
    if(is_left_support){
        // move from the trunk frame to the hip yaw frame and compute legIK
        goal.setRotation(trunk_to_support_foot.getRotation());
        goal.setOrigin(trunk_to_support_foot.getOrigin() - trunk_to_LHipYaw);
        success = success && legIK(goal, support_joints, true);
        goal.setRotation(trunk_to_flying_foot.getRotation());
        goal.setOrigin(trunk_to_flying_foot.getOrigin() - trunk_to_RHipYaw);
        success = success && legIK(goal, fly_joints, false);
    }else{
        goal.setRotation(trunk_to_flying_foot.getRotation());
        goal.setOrigin(trunk_to_flying_foot.getOrigin() - trunk_to_LHipYaw);
        success = success && legIK(goal, fly_joints, true);
        goal.setRotation(trunk_to_support_foot.getRotation());
        goal.setOrigin(trunk_to_support_foot.getOrigin() - trunk_to_RHipYaw);
        success = success && legIK(goal, support_joints, false);
    }
    //success = success && legIK(trunk_to_support_foot, support_joints);
    //success = success && legIK(trunk_to_flying_foot, fly_joints);

    // if IK was successful, asign positions to legs
    if(success){
        if(is_left_support) {
            positions.insert(positions.begin(), fly_joints.begin(), fly_joints.end());
            positions.insert(positions.end(), support_joints.begin(), support_joints.end());
        }else{
            positions.insert(positions.begin(), support_joints.begin(), support_joints.end());
            positions.insert(positions.end(), fly_joints.begin(), fly_joints.end());
        }
    }
    return success;
}

bool AnalyticIKSolver::legIK(tf::Transform& goal, std::vector<double>& positions, bool isLeftLeg){
    // this method works only for Darwin-OP like kinematics
    // the roll and pitch joints have to rotate around the same axis
    // the hip yaw has to be before the others

    // transform to foot from hip_yaw motor
    // todo read from URDF
    double hipYawToPitchX;
    double hipYawToPitchY;
    double hipYawToPitchZ;
    double hip_pitch_to_knee;
    double knee_to_ankle;
    double ankle_to_sole;
    hipYawToPitchX = -0.003;
    hipYawToPitchY = 0.0;
    hipYawToPitchZ = -0.0335;
    hip_pitch_to_knee = 0.1168;
    knee_to_ankle = 0.1168;
    ankle_to_sole = -0.0355;

    //ROS_INFO("Foot goal length %f", goal.getOrigin().length());
    // get RPY values for foot
    double foot_roll, foot_pitch, foot_yaw;
    tf::Matrix3x3(goal.getRotation()).getRPY(foot_roll, foot_pitch, foot_yaw);
    // yaw can only be set with hip yaw. Compute it from goal
    double hip_yaw = foot_yaw;
    //ROS_WARN("Goal from trunk  x:%f y:%f z:%f  OOO  r:%f, p:%f, y:%f", goal.getOrigin()[0], goal.getOrigin()[1], goal.getOrigin()[2], foot_roll, foot_pitch, foot_yaw);
    
    // translate the goal vector so that it starts at rotate point of hip_pitch and hip_roll
    tf::Vector3 hip_yaw_to_pitch {hipYawToPitchX, hipYawToPitchY, hipYawToPitchZ};
    hip_yaw_to_pitch = hip_yaw_to_pitch.rotate(tf::Vector3{0.0, 0.0, 1.0}, foot_yaw);
    goal.setOrigin(goal.getOrigin() - hip_yaw_to_pitch);
    //ROS_WARN("Goal from hip  x:%f y:%f z:%f", goal.getOrigin()[0], goal.getOrigin()[1], goal.getOrigin()[2]);

    //ROS_INFO("goal from hip length %f", goal.getOrigin().length());

    // we compute the goal vector to the ankle joint
    // transform from goal position to ankle is given by ankle_sole distance in z direction
    tf::Transform goal_to_ankle;
    goal_to_ankle.setOrigin(tf::Vector3{0.0, 0.0, -1* ankle_to_sole});
    tf::Transform to_ankle = goal * goal_to_ankle;
    tf::Vector3 ankle_goal = to_ankle.getOrigin();
    //ROS_WARN("Ankle Goal from hip  x:%f y:%f z:%f", ankle_goal[0], ankle_goal[1], ankle_goal[2]);
    //ROS_INFO("Ankle goal length %f", ankle_goal.length());

    // we can only find a solution if the ankle_goal is not too far away
    if(hip_pitch_to_knee + knee_to_ankle <= ankle_goal.length()){
        //the leg is not long enough to reach this
        ROS_ERROR("Analytic IK got no solution.");
        return false;
    }

    // we compute the hip roll
    // lookin from front, we can see that this is triangle between hip_roll and ankle_roll
    double hip_roll = atan2(ankle_goal[2], ankle_goal[1]);
    hip_roll = hip_roll + M_PI/2;
    // the ankle roll can now be computed
    // to keep the ankle parallel to the ground it has to be the inverse of the hip roll
    // we have to add the angle from the goal
    double ankle_roll = -1* hip_roll + foot_roll;

    // we can now compute the remaining pitch values
    // the hip pitch value can be computed similar to the hip roll, but in the XZ plane
    double hip_pitch = atan2(ankle_goal[2], ankle_goal[0]);
    hip_pitch = hip_pitch + M_PI/2;
    // the ankle pitch is again similar to the ankle roll
    double ankle_pitch = -1*hip_pitch - foot_pitch;
    // now only the knee is left
    // we have a triangle where two sides are the links from hip to knee and from knee to ankle
    // the thrid is the length of the ankle goal vector
    // we can use the law of cosines to compute the angle oposed to the ankle goal vector
    double beta = acos((pow(hip_pitch_to_knee,2) + pow(knee_to_ankle, 2) - pow(ankle_goal.length(), 2))/(2*hip_pitch_to_knee*knee_to_ankle));
    // the angle of the knee joint is 180- beta, since the knee is elongated at 0
    double knee = (M_PI - beta)*-1;

    //todo this is only true for legs with same length in upper and lower leg
    hip_pitch = hip_pitch - knee*0.5;
    ankle_pitch = ankle_pitch - knee*0.5;
//        if(!isLeftLeg){
//            hip_pitch = hip_pitch *-1;
//            knee = knee * -1;
//            ankle_pitch = ankle_pitch * -1;
//            //hip_yaw = hip_yaw * -1;
//        }
    //    if(_robot_type=="Minibot"){
    //        ankle_pitch = ankle_pitch *-1;
    //        hip_roll = hip_roll * -1;
    //        hip_yaw = hip_yaw * -1;
    //    }
    //    if(_robot_type=="Wolfgang"){
    //        ankle_pitch = ankle_pitch *-1;
    //        ankle_roll = ankle_roll * -1;
    //    }
    positions = std::vector<double>  {hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll};

    return true;
}
}
