#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include <eigen3/Eigen/Eigen>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
// #include <robot_state_publisher/treefksolverposfull_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <tf2_kdl/tf2_kdl.h>
//#include <walk_zmp/zmp_team_darwin_kinematics.hpp>

namespace robotis_op {

class RobotModel
{

public:
  RobotModel();


  void InitializeModel();

  void updateModel(KDL::JntArray &jointpositions);
  void updateModel(std::map<std::string, double> joint_state_map);

  KDL::Frame getTrunkPose(int stance);

   std::vector<geometry_msgs::TransformStamped> getRobotPose();

   KDL::Tree getTree() { return this->robot;}


private:
  Eigen::Affine3d base;
  Eigen::Affine3d hipLeft;
  Eigen::Affine3d hipRight;
  Eigen::Affine3d kneeRight;
  Eigen::Affine3d kneeLeft;
  Eigen::Affine3d ankleLeft;
  Eigen::Affine3d ankleRight;
  Eigen::Affine3d footRight;
  Eigen::Affine3d footLeft;

  KDL::Chain leftLeg, rightLeg, head;

  KDL::Frame com_left_stance;
  KDL::Frame com_right_stance;

  KDL::Tree robot;

  std::map<std::string, tf2::Stamped<KDL::Frame> > fk_results;

  void leftStance();
  void rightStance();

  enum {LEG_LEFT = 0, LEG_RIGHT = 1};

  double hipOffsetY;    //OP, measured
  double hipOffsetZ;    //OP, Calculated from spec
  double thighLength;//0.1168;//.1168;  //OP, spec //Für angepassten Darwin überschreibbar machen
  double tibiaLength;//0.093;//.1168;  //OP, spec //Für angepassten Darwin überschreibbar machen
  double footHeight;   //OP, spec


};

}

#endif // ROBOTMODEL_H
