/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: SCH, Jay Song, Kayman */

#ifndef OP3_KINEMATICS_DYNAMICS_H_
#define OP3_KINEMATICS_DYNAMICS_H_

#include <vector>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include "op3_kinematics_dynamics_define.h"
#include "link_data.h"

namespace robotis_op
{

enum TreeSelect
{
  Manipulation,
  Walking,
  WholeBody
};

class OP3KinematicsDynamics
{

 public:
  OP3KinematicsDynamics();
  ~OP3KinematicsDynamics();
  OP3KinematicsDynamics(TreeSelect tree);

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);
  int findLinkId(std::string link_name);
  void setJointPos( std::string link_name, double value);
  void setOrientation( int id, Eigen::MatrixXd orientation);

  double calcTotalMass(int joint_id);
  double calcTotalMass_LSSP(int joint_id);
  double calcTotalMass_RSSP(int joint_id);
  double calcTotalMass_DSP(int joint_id);
  Eigen::MatrixXd calcMC(int joint_id);
  Eigen::MatrixXd calcMC_LSSP(int joint_id);
  Eigen::MatrixXd calcMC_RSSP(int joint_id);
  Eigen::MatrixXd calcMC_DSP(int joint_id);
  Eigen::MatrixXd calcCOM(int joint_id);
  Eigen::MatrixXd calcCOM_LSSP(int joint_id);
  Eigen::MatrixXd calcCOM_RSSP(int joint_id);
  Eigen::MatrixXd calcCOM_DSP(int joint_id);

  void calcGlobalInertia(int joint_id);
  void calcGlobalInertiaLSSP(int joint_id);
  void calcGlobalInertiaRSSP(int joint_id);
  void calcGlobalInertiaDSP(int joint_id);
  Eigen::MatrixXd calcTotalInertia(int joint_id);
  Eigen::MatrixXd calcTotalInertiaLSSP(int joint_id);
  Eigen::MatrixXd calcTotalInertiaRSSP(int joint_id);
  Eigen::MatrixXd calcTotalInertiaDSP(int joint_id);

  void calcForwardKinematics(int joint_ID);
  void calcFKLeftSSP(int joint_ID);
  void calcFKRightSSP(int joint_ID);
  void calcFKDSP(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                            Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter,
                             double ik_err);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                             int max_iter, double ik_err);

  // with weight
  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter,
                             double ik_err, Eigen::MatrixXd weight);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                             int max_iter, double ik_err, Eigen::MatrixXd weight);

  bool calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  bool calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch,
                                        double yaw);
  bool calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch,
                                       double yaw);

  LinkData *op3_link_data_[ ALL_JOINT_ID + 1];

  LinkData *getLinkData(const std::string link_name);
  LinkData *getLinkData(const int link_id);
  Eigen::MatrixXd getJointAxis(const std::string link_name);
  double getJointDirection(const std::string link_name);
  double getJointDirection(const int link_id);

  Eigen::MatrixXd calcPreviewParam(double preview_time, double control_cycle,
                                   double lipm_height,
                                   Eigen::MatrixXd K, Eigen::MatrixXd P);

  void loadConfig();
  std::string offset_path_;

  double body_length_m_;
  double hip_length_m_;
  double thigh_length_m_;
  double calf_length_m_;
  double ankle_length_m_;
  double leg_side_offset_m_;
};

}

#endif /* OP3_KINEMATICS_DYNAMICS_H_ */
