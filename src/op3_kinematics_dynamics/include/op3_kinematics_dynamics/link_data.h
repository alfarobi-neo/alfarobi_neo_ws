/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: sch */

#ifndef LINK_DATA_H_
#define LINK_DATA_H_

#include <robotis_math/robotis_math.h>
#include <eigen3/Eigen/Eigen>

namespace robotis_op
{

class LinkData
{
 public:
  LinkData();
  ~LinkData();

  std::string name_;

  int parent_;
  int sibling_;
  int child_;
  int id;

  double mass_;

  Eigen::MatrixXd relative_position_;
  Eigen::MatrixXd joint_axis_;
  Eigen::MatrixXd center_of_mass_;
  Eigen::MatrixXd inertia_;

  Eigen::MatrixXd global_inertia_;
  Eigen::MatrixXd global_inertia_LSSP_;
  Eigen::MatrixXd global_inertia_RSSP_;
  Eigen::MatrixXd global_inertia_DSP_;

  Eigen::MatrixXd relative_position_LSSP_;
  Eigen::MatrixXd relative_position_RSSP_;
  Eigen::MatrixXd relative_position_DSP_;

  double joint_limit_max_;
  double joint_limit_min_;

  double joint_angle_;
  double joint_velocity_;
  double joint_acceleration_;
  double offset_angle_;

  Eigen::MatrixXd position_;
  Eigen::MatrixXd orientation_;
  Eigen::Affine3d transformation_;

  //Left SSP
  Eigen::MatrixXd position_LSSP_;
  Eigen::MatrixXd orientation_LSSP_;

  //Right SSP
  Eigen::MatrixXd position_RSSP_;
  Eigen::MatrixXd orientation_RSSP_;

  //DSP
  Eigen::MatrixXd position_DSP_;
  Eigen::MatrixXd orientation_DSP_;

  Eigen::MatrixXd linier_velocity_;
  Eigen::MatrixXd angular_velocity_;
};

}

#endif /* LINK_DATA_H_ */
