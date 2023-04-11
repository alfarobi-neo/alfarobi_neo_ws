#ifndef csvManager_H_
#define csvManager_H_

#include "ros/ros.h"
#include <ros/time.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <cmath>
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <sstream>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <tf/tf.h>

#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

using namespace Eigen;
using namespace std;

namespace robotis_op
{
/*
 * A class to read data from a csv file.
 */
class csvManager
{
 
public:
	csvManager();
 	~csvManager();
	std::vector<std::vector<double>> getMatrix(std::string fileName);
private:
	std::vector<std::vector<double> > transpose(const std::vector<std::vector<double> > data);
};
 
}

#endif