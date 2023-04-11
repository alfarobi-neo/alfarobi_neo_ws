#include <walk_zmp/robotmodel.h>

using namespace Eigen;
using namespace KDL;

namespace robotis_op {


RobotModel::RobotModel()
{

  hipOffsetY = .038;    //OP, measured
  hipOffsetZ = 0.077;    //OP, Calculated from spec
  thighLength = .1168;//0.1168;//.1168;  //OP, spec //Für angepassten Darwin überschreibbar machen
  tibiaLength = .1168;//0.093;//.1168;  //OP, spec //Für angepassten Darwin überschreibbar machen
  footHeight = .0355;   //OP, spec

  InitializeModel();


}


void RobotModel::InitializeModel()
{
  base = Affine3d(Translation3d(Vector3d(0, 0, 0)));
  hipLeft = Affine3d(Translation3d(Vector3d(0, hipOffsetY, -hipOffsetZ)));
  hipRight = Affine3d(Translation3d(Vector3d(0, -hipOffsetY, -hipOffsetZ)));
  kneeRight = Affine3d(Translation3d(Vector3d(0, 0, -thighLength)));
  kneeLeft = Affine3d(Translation3d(Vector3d(0, 0, -thighLength)));
  ankleRight = Affine3d(Translation3d(Vector3d(0, 0, -tibiaLength)));
  ankleLeft = Affine3d(Translation3d(Vector3d(0, 0, -tibiaLength)));
  footRight = Affine3d(Translation3d(Vector3d(0, 0, -footHeight)));
  footLeft = Affine3d(Translation3d(Vector3d(0, 0, -footHeight)));

  robot = KDL::Tree("body");

  leftLeg.addSegment(KDL::Segment("l_hip_yaw",KDL::Joint(Joint::RotZ),
                                  Frame(KDL::Vector(0.0, hipOffsetY, -hipOffsetZ))));
  leftLeg.addSegment(KDL::Segment("l_hip_roll", KDL::Joint(Joint::RotX),
                                  Frame(KDL::Vector(0,0,0))));
  leftLeg.addSegment(KDL::Segment("l_hip_pitch", KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                  Frame(KDL::Vector(0,0,0))));
  leftLeg.addSegment(KDL::Segment("l_knee", KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                  Frame(KDL::Vector(0,0,-thighLength))));
  leftLeg.addSegment(KDL::Segment("l_ank_pitch", Joint(Joint::RotY),
                                  Frame(KDL::Vector(0,0,-tibiaLength))));
  leftLeg.addSegment(KDL::Segment("l_ank_roll", KDL::Joint("minus_RotX", KDL::Vector(0,0,0), KDL::Vector(-1,0,0), KDL::Joint::RotAxis),
                                  Frame(KDL::Vector(0,0, 0))));
  leftLeg.addSegment(KDL::Segment("l_foot", KDL::Joint(Joint::None),
                                  Frame(KDL::Vector(0, 0, -footHeight))));

  rightLeg.addSegment(KDL::Segment("r_hip_yaw",KDL::Joint(Joint::RotZ),
                                   Frame(KDL::Vector(0.0, -hipOffsetY, -hipOffsetZ))));
  rightLeg.addSegment(KDL::Segment("r_hip_roll", KDL::Joint(Joint::RotX),
                                   Frame(KDL::Vector(0,0,0))));
  rightLeg.addSegment(KDL::Segment("r_hip_pitch", KDL::Joint(Joint::RotY),
                                   Frame(KDL::Vector(0,0,0))));
  rightLeg.addSegment(KDL::Segment("r_knee", KDL::Joint(Joint::RotY),
                                   Frame(KDL::Vector(0,0,-thighLength))));
  rightLeg.addSegment(KDL::Segment("r_ank_pitch", KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                   Frame(KDL::Vector(0,0,0))));
  rightLeg.addSegment(KDL::Segment("r_ank_roll", KDL::Joint("minus_RotX", KDL::Vector(0,0,0), KDL::Vector(-1,0,0), KDL::Joint::RotAxis),
                                   Frame(KDL::Vector(0,0,0))));
  rightLeg.addSegment(KDL::Segment("r_foot", KDL::Joint(Joint::None),
                                   Frame(KDL::Vector(0,0,-footHeight))));

  robot.addChain(rightLeg, "body");
  robot.addChain(leftLeg, "body");
}

void RobotModel::updateModel(KDL::JntArray &jointpositions)
{
  int kinematic_left, kinematic_right;

  TreeFkSolverPos_recursive treesolver = TreeFkSolverPos_recursive(robot);


  kinematic_left = treesolver.JntToCart(jointpositions, com_left_stance, "l_foot");
  kinematic_right = treesolver.JntToCart(jointpositions, com_right_stance, "r_foot");
}

void RobotModel::updateModel(std::map<std::string, double> joint_state_map)
{
  KDL::TreeFkSolverPosFull_recursive fk_solver_(robot);

  if (fk_solver_.JntToCart(joint_state_map, fk_results, false) != 0)
    throw std::runtime_error("FK solver returned with an error.");

}

KDL::Frame RobotModel::getTrunkPose(int stance)
{
  if (stance == LEG_LEFT)
    return com_left_stance.Inverse();
  else
    return com_left_stance.Inverse();
}

std::vector<geometry_msgs::TransformStamped> RobotModel::getRobotPose()
{
  std::vector<geometry_msgs::TransformStamped> full_transform;
  for (std::map<std::string, tf2::Stamped<KDL::Frame> >::const_iterator it=fk_results.begin(); it!=fk_results.end(); ++it)
  {
    geometry_msgs::TransformStamped transform = tf2::kdlToTransform(it->second);
    transform.header.stamp = it->second.stamp_;
    transform.header.frame_id = it->second.frame_id_;
    transform.child_frame_id = it->first;
    full_transform.push_back(transform);
  }

  return full_transform;

}

void RobotModel::rightStance()
{

}

void RobotModel::leftStance()
{

}

}
