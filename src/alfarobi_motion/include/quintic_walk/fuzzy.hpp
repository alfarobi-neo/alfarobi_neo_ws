#ifndef FUZZY_HPP
#define FUZZY_HPP

#include <iostream>
#include <sstream>
#include <cmath>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <std_msgs/Float32.h>

#define COM_Y_TRIANGLE 0.33333333

class Fuzzy
{
public:
    double input;

    void setMembership(int amountData, std::vector<double> &Bottom1, std::vector<double> &Upper1, std::vector<double> &Upper2, std::vector<double> &Bottom2);
    void setRule(int row, int col, const Eigen::MatrixXi &data);
    void printRule();

    Eigen::ArrayXi checkLevel();
    Eigen::ArrayXd calcValue(const Eigen::ArrayXi &level);
    double outputCentroid(const Eigen::ArrayXd &membership);

    Eigen::ArrayXd predicate();
    Eigen::ArrayXd membership(const Eigen::ArrayXd &predicate1, const Eigen::ArrayXd &predicate2, int &dataAmount1, int &dataAmount2);

    Eigen::ArrayXd calcUnion(int section, double value);
    Eigen::ArrayXd calcIntersection(const Eigen::ArrayXi &section, const Eigen::ArrayXd &value);

private:
    int dataAmount;
    double bottom1[10];
    double upper1[10];
    double upper2[10];
    double bottom2[10];
    int rule[10][10];

    double calcX(double y, double y1, double y2, double x1, double x2);
    double calcY(double x, double x1, double x2, double y1, double y2);

    double calcHypotenuse(double x1, double y1, double x2, double y2);
    double calcGradien(double x1, double y1, double x2, double y2);
};

#endif