#include "op3_kinematics_dynamics/servoObserver.h"


namespace robotis_op
{
servoObserver::servoObserver()
{
   config_path_ = ros::package::getPath("op3_kinematics_dynamics")+"/config/physicalParam.yaml";
    reset = 0;
    loadConfig();
    Eigen::VectorXd x_next(2), x_prev(2), y_ss(1), u_ss(1);
    x_next << 0, 0;
    x_prev << 0, 0;
    y_ss << 0;
    u_ss << 0;
    for(int i=0; i < x_next.rows(); i++)
    {
        x_temp.push_back(0.0);
    }
    r_ank_pitch.name = "r_ank_pitch";
    l_ank_pitch.name = "l_ank_pitch";
    r_ank_roll.name = "r_ank_roll";
    l_ank_roll.name = "l_ank_roll";
}
servoObserver::~servoObserver()
{
}

void servoObserver::loadConfig()
{
    YAML::Node ph;

    try
    {
        ph = YAML::LoadFile(config_path_.c_str());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return;
    }
    m  = ph["m"].as<double>();
    dT = ph["dT"].as<double>();
    B  = ph["B"].as<double>();
    F  = ph["F"].as<double>();
    Is = ph["Is"].as<double>();

    YAML::Node pD = ph["DSP"];
    phDSP.L = pD["L"].as<double>();
    phDSP.I_xx = pD["I_xx"].as<double>();
    phDSP.I_yy = pD["I_yy"].as<double>();

    YAML::Node pL = ph["LSSP"];
    phLSSP.L = pL["L"].as<double>();
    phLSSP.I_xx = pL["I_xx"].as<double>();
    phLSSP.I_yy = pL["I_yy"].as<double>();

    YAML::Node pR = ph["RSSP"];
    phRSSP.L = pR["L"].as<double>();
    phRSSP.I_xx = pR["I_xx"].as<double>();
    phRSSP.I_yy = pR["I_yy"].as<double>();

    YAML::Node mx64Node = ph["MX64"];
    MX64.Kp = mx64Node["Kp"].as<double>()/8;
    MX64.Kt = pow(10,-3)*mx64Node["Kt"].as<double>();
    MX64.N =  mx64Node["N"].as<double>();
    MX64.R = mx64Node["R"].as<double>();
    MX64.eta = mx64Node["eta"].as<double>();
    MX64.nominalVoltage = mx64Node["nominalVoltage"].as<double>();
    MX64.omegaNoLoad = rpmToRad*mx64Node["omegaNoLoad"].as<double>(); //10600 #* rpmToRad;
    MX64.iNoLoad = pow(10,-3)*mx64Node["iNoLoad"].as<double>(); //9.2 #* 10^-3;
    MX64.Jm = pow(10,-3)*pow(pow(10,-2),2)*mx64Node["Jm"].as<double>(); //0.868 #* 10^-3 * (10^-2)^2;
    MX64.stallTorqueMotor = pow(10,-3)*mx64Node["stallTorqueMotor"].as<double>();
    MX64.stallTorqueServo = mx64Node["stallTorqueServo"].as<double>();

    YAML::Node trqGain = ph["servoTorqueController"];
    Kposition = trqGain["Kposition"].as<double>();
    Kvelocity =  trqGain["Kvelocity"].as<double>();
    KrTrq = trqGain["KrTrq"].as<double>();
}

void servoObserver::resetValue()
{
    Eigen::VectorXd x_next(2), x_prev(2), y_ss(1), u_ss(1);
    x_next << 0, 0;
    x_prev << 0, 0;
    y_ss << 0;
    u_ss << 0;
    for(int i=0; i < x_next.rows(); i++)
    {
        x_temp.push_back(0.0);
    }
}

void servoObserver::servoStateObserver(double u, std::vector<double> &x, double & y)
{   
    // loadConfig();
    MatrixXd A(2,2), B(2,1), C(1,2);
    MatrixXd Ad(2,2), Bd(2,1);
    Eigen::VectorXd x_next(2), x_prev(2), y_ss(1), u_ss(1);

    MX64.Bm = MX64.Kt * MX64.iNoLoad / MX64.omegaNoLoad;
    // MX64.eta = MX64.stallTorqueServo / (MX64.stallTorqueMotor * MX64.N);
    MX64.KLoadTorque = MX64.Kt * MX64.N * MX64.eta / MX64.R;
    MX64.speedToEMF = MX64.N * MX64.Kt;
    MX64.JmGz = MX64.Jm * pow(MX64.N,2) * MX64.eta + phDSP.I_yy;
    MX64.BmGz = MX64.Bm * pow(MX64.N,2) * MX64.eta;

    MX64.a_21 = -MX64.KLoadTorque*MX64.Kp*pwmToVoltage*angleToPWM/MX64.JmGz;
    MX64.a_22 = -(MX64.KLoadTorque*MX64.speedToEMF+MX64.BmGz)/MX64.JmGz;
    MX64.b_2 = MX64.KLoadTorque*MX64.Kp*pwmToVoltage*angleToPWM/MX64.JmGz; 
    A << 0, 1,
         MX64.a_21, MX64.a_22;
    B << 0,
         MX64.b_2;
    C << 1, 0;
    Ad = (A*dT).MatrixBase::exp();
    Bd = A.inverse()*(Ad-MatrixXd::Identity(Ad.rows(), Ad.cols()))*B;
    u_ss(0) = u;
    for(int i=0; i<x_next.rows(); i++)
    {
        x_prev(i) = x_temp[i];
        x[i] = x_temp[i];
    }
    y_ss = C*x_prev;
    y = y_ss(0);
    x_next = Ad*x_prev + Bd*u_ss;
    for(int i=0; i<x_next.rows(); i++)
    {
        x_temp[i] = x_next(i);
    }
    std::cout<<"Input: "<<u<<"\tAngle: "<<x_prev(0)<<"\tSpeed:"<<x_prev(1)<<std::endl;
    if(reset == 1)
    {
        x_next << 0, 0;
        x_prev << 0, 0;
        y_ss << 0;
        u_ss << 0;
        for(int i=0; i < x_next.rows(); i++)
        {
            x_temp.push_back(0.0);
        }
    }
    // std::cout<<"KLoadTorque: "<<MX64.KLoadTorque<<"\tKp: "<<MX64.Kp<<"\tJmGz: "<<MX64.JmGz<<std::endl;
    // std::cout<<"pwmToVoltage: "<<pwmToVoltage<<"\tangleToPWM: "<<angleToPWM<<std::endl;
    // std::cout<<"A: "<<A<<std::endl;
    // std::cout<<"B: "<<B<<std::endl;
    // std::cout<<"dT: "<<dT<<std::endl;
    // std::cout<<"Ad: "<<Ad<<std::endl;
    // std::cout<<"Bd: "<<Bd<<std::endl;
//     Ad = expm(A*Ts)
// Bd = inv(A)*(Ad-eye(size(Ad)))*B
}

void servoObserver::torqueObserver(servo *s)
{
    // loadConfig();
    // MX64.eta = MX64.stallTorqueServo / (MX64.stallTorqueMotor * MX64.N);
    MX64.KLoadTorque = MX64.Kt * MX64.N * MX64.eta / MX64.R;
    MX64.speedToEMF = MX64.N * MX64.Kt;
    std::cout<<"Observing "<<s->name<<std::endl;
    s->Verr = MX64.Kp*pwmToVoltage*angleToPWM*(s->refPosFromPos - (s->positionNow-s->positionIK));
    if(s->Verr > MX64.nominalVoltage) //voltage saturation
        s->Verr = MX64.nominalVoltage;
    if(s->Verr < -MX64.nominalVoltage)
        s->Verr = -MX64.nominalVoltage;
    s->torqueEst  = MX64.KLoadTorque*(s->Verr - MX64.speedToEMF*s->velocityNow);
    std::cout<<"Present Position: "<<s->positionNow<<std::endl;
    std::cout<<"Present Velocity: "<<s->velocityNow<<std::endl;
    std::cout<<"Estimated Torque: "<<s->torqueEst<<std::endl;
    std::cout<<"Position from Torque: "<<s->refPosFromTrq<<std::endl;
}

void servoObserver::convertTrq2Pos(servo *s)
{
    // loadConfig();
    MX64.KLoadTorque = MX64.Kt * MX64.N * MX64.eta / MX64.R;
    MX64.speedToEMF = MX64.N * MX64.Kt;
    s->Verr = s->torqueIn/MX64.KLoadTorque + MX64.speedToEMF*s->velocityNow;
    // if(s->Verr > MX64.nominalVoltage)
    //     s->Verr = MX64.nominalVoltage;
    // if(s->Verr < -MX64.nominalVoltage)
    //     s->Verr = -MX64.nominalVoltage;
    std::cout<<"Input voltage: "<<s->Verr<<"\tVolt"<<std::endl;
    std::cout<<"PositionNow(DEG): "<<s->positionNow*180/M_PI<<"\tPositionIK(DEG): "<<s->positionIK*180/M_PI<<std::endl;
    s->refPosFromTrq = (s->Verr/(MX64.Kp*pwmToVoltage*angleToPWM) + (s->positionNow - s->positionIK));
    std::cout<<"RefPosFromTrq(DEG): "<<s->refPosFromTrq*180/M_PI<<std::endl;
    
    // s->refPosFromTrq = ((1/MX64.KLoadTorque)*s->refPosFromTrq+MX64.speedToEMF*s->velocityNow)/(MX64.Kp*pwmToVoltage + (s->positionNow - s->positionIK)); 
    // s->Verr = MX64.Kp*pwmToVoltage*angleToPWM*(s->refPosFromTrq-(s->positionNow - s->positionIK));
    // if(s->Verr > MX64.nominalVoltage)
    //     s->Verr = MX64.nominalVoltage;
    // if(s->Verr < -MX64.nominalVoltage)
    //     s->Verr = -MX64.nominalVoltage;
    // std::cout<<"Input voltage: "<<s->Verr<<"\tVolt"<<std::endl;
    // std::cout<<"PositionNow(DEG): "<<s->positionNow*180/M_PI<<"\tPositionIK(DEG): "<<s->positionIK*180/M_PI<<std::endl;
    // s->refPosFromTrq = s->Verr/(MX64.Kp*pwmToVoltage*angleToPWM) + (s->positionNow - s->positionIK);
    // std::cout<<"RefPosFromTrq(DEG): "<<s->refPosFromTrq*180/M_PI<<std::endl;
}

void servoObserver::servoTorqueFeedback(servo *s)
{
    // loadConfig();
    // MX64.KLoadTorque = MX64.Kt * MX64.N * MX64.eta / MX64.R;
    // MX64.speedToEMF = MX64.N * MX64.Kt;
    // Output Feedback
    MatrixXf K(1,2);
    VectorXf Kr(1);
    VectorXf x(2);
    VectorXf u(1);
    K << Kposition, Kvelocity;
    Kr(0) = KrTrq;
    // s->Verr = s->torqueIn/MX64.KLoadTorque + MX64.speedToEMF*s->velocityNow;
    // if(s->Verr > MX64.nominalVoltage)
    // {
    //     s->Verr = MX64.nominalVoltage;
    //     x << (s->refPosFromTrq - s->Verr/(MX64.Kp*pwmToVoltage*angleToPWM)),
    //          s->velocityNow;
    // }
    // else if(s->Verr < -MX64.nominalVoltage)
    // {
    //     s->Verr = -MX64.nominalVoltage;
    //     x << (s->refPosFromTrq - s->Verr/(MX64.Kp*pwmToVoltage*angleToPWM)),
    //          s->velocityNow;
    // }
    // else
    // {
    //     x << (s->positionNow - s->positionIK),
    //      s->velocityNow;
    // }
    //s->refPosFromTrq  = (s->positionNow-s->positionIK)+s->Verr/(MX64.Kp*pwmToVoltage*angleToPWM); 
    x << (s->positionNow - s->positionIK),
         s->velocityNow;
    u = -K*x + Kr*s->torqueIn;
    s->refPosFromTrq = u(0);

    //Output Error
}

void servoObserver::resetServoValues()
{
    l_ank_pitch.torqueIn = 0; r_ank_pitch.torqueIn = 0;
    l_ank_pitch.velocityNow = 0; r_ank_pitch.velocityNow = 0;
    l_ank_pitch.refPosFromPos = 0; r_ank_pitch.refPosFromPos = 0;
    l_ank_pitch.refPosFromTrq = 0; r_ank_pitch.refPosFromTrq = 0;
    l_ank_roll.torqueIn = 0; r_ank_roll.torqueIn = 0;
    l_ank_roll.velocityNow = 0; r_ank_roll.velocityNow = 0;
    l_ank_roll.refPosFromPos = 0; r_ank_roll.refPosFromPos = 0;
    l_ank_roll.refPosFromTrq = 0; r_ank_roll.refPosFromTrq = 0;
}

}