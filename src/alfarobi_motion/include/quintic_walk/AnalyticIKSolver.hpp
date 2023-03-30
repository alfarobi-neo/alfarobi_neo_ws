#ifndef ANALYTICIKSOLVER_HPP
#define ANALYTICIKSOLVER_HPP

#include <math.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
//#include <moveit/move_group_interface/move_group_interface.h>

namespace robotis_op{
class AnalyticIKSolver{
    public:
        AnalyticIKSolver();
//        AnalyticIKSolver(std::string robot_type, const robot_state::JointModelGroup &lleg_joints_group, const robot_state::JointModelGroup &rleg_joints_group);
        bool solve(tf::Transform& trunk_to_support_foot, tf::Transform& trunk_to_flying_foot, std::vector<double> &positions, bool is_left_support);
        bool legIK(tf::Transform& goal, std::vector<double>& positions, bool isLeftLeg);
    private:
        std::string _robot_type;
//        const robot_state::JointModelGroup *_lleg_joints_group;
//        const robot_state::JointModelGroup *_rleg_joints_group;
};

}

#endif
