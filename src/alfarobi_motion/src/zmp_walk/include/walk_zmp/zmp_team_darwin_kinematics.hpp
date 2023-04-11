#ifndef ZMP_INVERSE_LEGS_HPP_
#define ZMP_INVERSE_LEGS_HPP_

#include <cmath>
#include <Eigen/Core>

namespace robotis_op{

class Transform {
private:
  Eigen::Matrix4d t;
public:
  Transform():t(Eigen::Matrix4d::Zero()){t(3,3) = 1;} //TODO Richtig initialsieren
  Transform(const Eigen::Matrix4d& m):t(m){}
  ~Transform() {}

  Transform& rotateX(double a = 0);
  Transform& rotateY(double a = 0);
  Transform& rotateZ(double a = 0);

  double& operator() (int i, int j){
    return t(i,j);
  }
  const double& operator() (int i, int j) const{
    return t(i,j);
  }
  void apply(Eigen::Vector3d& v);
  Transform operator * (const Transform& t1) const;
  Transform inv() const;
};

Transform transform6D(const Eigen::Matrix<double, 6, 1>& p);
void set_long_leg_adjusted_values(double thigh, double tibia, double hip_y_offset, double hip_z_offset, double foot_height);

Eigen::Matrix<double, 12, 1> inverse_legs(
    const Eigen::Matrix<double, 6, 1>& pLLeg,
    const Eigen::Matrix<double, 6, 1>& pRLeg,
    const Eigen::Matrix<double, 6, 1>& pTorso);

Eigen::Matrix<double, 6, 1>
darwinop_kinematics_inverse_leg(
    const Transform& trLeg,
    int leg);

Eigen::Matrix<double, 6, 1>
darwinop_kinematics_inverse_lleg(const Transform& trLeg);

Eigen::Matrix<double, 6, 1>
darwinop_kinematics_inverse_rleg(const Transform& trLeg);

}//namespace

#endif
