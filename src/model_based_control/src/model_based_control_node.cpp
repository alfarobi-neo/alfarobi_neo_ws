#include "model_based_control/modelBasedControl.h"
#include "model_based_control/densis.h"
#include "model_based_control/servoObserver.h"
#include "densis_msgs/densis.h"

using namespace robotis_op;

double input;
double noise;
double balance_angle[12];

int main(int argc, char **argv)
{
  ros::init(argc, argv, "model_based_control_node");
  ros::NodeHandle n;

  robotis_op::modelBasedControl mbc(ros::package::getPath("model_based_control"));
  robotis_op::densis densis(ros::package::getPath("model_based_control"));
  // robotis_op::servoObserver so;
  densis_msgs::densis densisMsgs;
  ros::Publisher densis_pub_ = n.advertise<densis_msgs::densis>("/modelBasedControl/densis_log",0);

  // ros::Publisher data_log_pub_ =  n.advertise<filter_msgs::filter>("/cobaFilter/data_log",0);
  ros::Rate loop_rate(25);
  // ros::Rate loop_rate(30);
  // ros::Rate loop_rate(80);
  // ros::Rate loop_rate(120);

  densis.m_time_start = ros::Time::now().toSec();

  while (ros::ok())
  {
    auto t1 = boost::chrono::high_resolution_clock::now();
    if(mbc.tuneGain == true)
    {
      // std::cout<<"Tuning Gain, input set to zero"<<std::endl;
      input = 0;
      mbc.kalman.K = mbc.kalmanGain(&mbc.sys_d,&mbc.kalman);
      mbc.dlqr.K = mbc.dlqrGain(&mbc.sys_d,&mbc.dlqr);
      // Calculate Fuzzy Gain here ... (cc: Gaby)
      mbc.dlqr.Kr = mbc.feedforwardGain(&mbc.sys_d, &mbc.dlqr);
      mbc.stateSpaceSimulator(input, mbc.xsim, mbc.ysim);
      for(int i=0;i<mbc.ysim.size();i++)
      {
          mbc.ymeas[i] = mbc.ysim[i]+noise;
          // std::cout<<"Measured output ["<<i<<"]: "<<mbc.ysim[i]*180/M_PI<<std::endl;
      }
    }
    else
    {
      // std::cout<<"Gain done, applied to system, get ready to engage"<<std::endl;
      if(mbc.testObserver == true)
      {
        // std::cout<<"Testing Observer Response, no feedback control effort applied"<<std::endl;
        input = densis.densisInput(ros::Time::now().toSec()-densis.m_time_start,balance_angle);
        noise = 0; //densis.generatePseudonoise(ros::Time::now().toSec()-densis.m_time_start,-0.1*M_PI/180,0.1*M_PI/180,0.0001,mbc.sys_d.Ts);
        mbc.stateSpaceSimulator(input, mbc.xsim, mbc.ysim);
        for(int i=0;i<mbc.ysim.size();i++)
        {
            mbc.ymeas[i] = mbc.ysim[i]+noise;
            // std::cout<<"Measured output ["<<i<<"]: "<<mbc.ysim[i]*180/M_PI<<std::endl;
        }
        mbc.wholeBodyStateObserver(input, mbc.wholeBodyStates, mbc.kalman.K, mbc.yest, mbc.ymeas);
      }
      else
      {
        // std::cout<<"Testing Controller Closed Loop Response, control effort applied!"<<std::endl;
        noise = 0; //densis.generatePseudonoise(ros::Time::now().toSec()-densis.m_time_start,-0.1*M_PI/180,0.1*M_PI/180,0.0001,mbc.sys_d.Ts);
        mbc.stateSpaceSimulator(input, mbc.xsim, mbc.ysim);
        for(int i=0;i<mbc.ysim.size();i++)
        {
            mbc.ymeas[i] = mbc.ysim[i]+noise;
            // std::cout<<"Measured output ["<<i<<"]: "<<mbc.ysim[i]*180/M_PI<<std::endl;
        }
        mbc.wholeBodyStateObserver(input, mbc.wholeBodyStates,mbc.kalman.K, mbc.yest, mbc.ymeas);
        input = mbc.outputFeedback(mbc.pitchRef*M_PI/180,mbc.wholeBodyStates,mbc.dlqr.K,mbc.dlqr.Kr);
      }
    }

    //Publish messages
    densisMsgs.LAnklePitch_refPosFromPos = input;
    densisMsgs.RAnklePitch_refPosFromPos = input;

    densisMsgs.COMDSPrpy_pitch_Est = mbc.yest[0];
    densisMsgs.gyro_pitch_Est = mbc.yest[1];

    densisMsgs.COMDSPrpy_pitch_Meas = mbc.ymeas[0];
    densisMsgs.gyro_pitch_Meas = mbc.ymeas[1];
    // densisMsgs.gyro_pitch_Fil = gyro_y_;
    densis_pub_.publish(densisMsgs);

    ros::spinOnce();

    loop_rate.sleep();
    auto t2 = boost::chrono::high_resolution_clock::now();
    auto elapsed_time = boost::chrono::duration_cast<boost::chrono::milliseconds>(t2-t1).count();
    std::cout << "Elapsed Node Time : " << elapsed_time << " miliseconds" << std::endl;

  }

  return 0;
}
