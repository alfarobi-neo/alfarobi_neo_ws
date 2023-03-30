#include "modelBasedControl.h"
#include "densis.h"
#include "servoObserver.h"
#include "densis_msgs/densis.h"

using namespace robotis_op; 

// typedef std::vector<double> Vec;
// typedef std::vector<Vec> Vec2D;
double input;
double balance_angle[12];

int main(int argc, char **argv)
{
  // robotis_op::modelBasedControl mbc;
  // robotis_op::csvManager csv;
  ros::init(argc, argv, "model_based_control_node");
  ros::NodeHandle n;

  robotis_op::modelBasedControl mbc(ros::package::getPath("model_based_control")); 
  robotis_op::densis densis(ros::package::getPath("model_based_control"));
  // robotis_op::servoObserver so;
  densis_msgs::densis densisMsgs;
  ros::Publisher densis_pub_ = n.advertise<densis_msgs::densis>("/modelBasedControl/densis_log",0);

  // ros::Publisher data_log_pub_ =  n.advertise<filter_msgs::filter>("/cobaFilter/data_log",0);
  ros::Rate loop_rate(30);
  // ros::Rate loop_rate(80);
  // ros::Rate loop_rate(120);
    
  densis.m_time_start = ros::Time::now().toSec();

  while (ros::ok())
  {
    auto t1 = boost::chrono::high_resolution_clock::now();
    // // data_log_pub_.publish(fil.dataLogMsgs);
    // csv.create(ros::package::getPath("model_based_control") + "/config/reportcard.csv");
    // csv.read_record(ros::package::getPath("model_based_control") + "/config/reportcard.csv");
    // csv.read_csv(ros::package::getPath("model_based_control") + "/config/data.csv");
    input = densis.densisInput(ros::Time::now().toSec()-densis.m_time_start,balance_angle);
    mbc.wholeBodyStateObserver(input, mbc.wholeBodyStates, mbc.ymeas);
    densisMsgs.LAnklePitch_refPosFromPos = input;
    densisMsgs.RAnklePitch_refPosFromPos = input;

    densisMsgs.COMDSPrpy_pitch_Est = mbc.wholeBodyStates[0];
    densisMsgs.gyro_pitch_Est = mbc.wholeBodyStates[1];

    densisMsgs.COMDSPrpy_pitch_Meas = mbc.ymeas[0];
    densisMsgs.gyro_pitch_Meas = mbc.ymeas[1];
    // densisMsgs.gyro_pitch_Fil = gyro_y_;
    densis_pub_.publish(densisMsgs);
    ros::spinOnce();
    /* //Mastah Lintang
    ::Vec2D A(csv.getMatrix(ros::package::getPath("model_based_control") + "/config/A.csv"));
    ::Vec2D B(csv.getMatrix(ros::package::getPath("model_based_control") + "/config/B.csv"));
    mbc.sys_c.A.clear();
    mbc.sys_c.B.clear();
    for(std::size_t r(0); r < A.size(); r++){
      ::Vec temp;
      for(std::size_t c(0); c < A[r].size(); c++){
        temp.push_back(A[r][c]);
      }
      mbc.sys_c.A.push_back(temp);
    }
    for(std::size_t r(0); r < B.size(); r++){
      ::Vec temp;
      for(std::size_t c(0); c < B[r].size(); c++){
        temp.push_back(B[r][c]);
      }
      mbc.sys_c.B.push_back(temp);
    }
    */
    // mbc.sys_c.A = csv.getMatrix(ros::package::getPath("model_based_control") + "/config/stateSpace/A.csv");
    // mbc.sys_c.B = csv.getMatrix(ros::package::getPath("model_based_control") + "/config/stateSpace/B.csv");
    // mbc.sys_c.C = csv.getMatrix(ros::package::getPath("model_based_control") + "/config/stateSpace/C.csv");
    loop_rate.sleep();
    auto t2 = boost::chrono::high_resolution_clock::now();
    auto elapsed_time = boost::chrono::duration_cast<boost::chrono::milliseconds>(t2-t1).count();
    std::cout << "Elapsed Node Time : " << elapsed_time << " miliseconds" << std::endl;
    
  }
   
  return 0;
}
