#include "model_based_control/modelBasedControl.h"

namespace robotis_op
{
modelBasedControl::modelBasedControl(std::string constr_modelParamPath_)
// modelBasedControl::modelBasedControl()
{
    reset = 0;
    modelParamPath_ = constr_modelParamPath_;
    loadModelParam(constr_modelParamPath_);
    for(int i=0; i<sys_d.A.size(); i++)
    {
         x_temp.push_back(0.0);
         xsim_temp.push_back(0.0);
         xhat_temp.push_back(0.0);
         wholeBodyStates.push_back(0.0);
         xhat.push_back(0.0);
         xsim.push_back(0.0);
    }
    for(int j=0; j<sys_d.C.size(); j++)
    {
        ysim.push_back(0.0);
        ymeas.push_back(0.0);
        yest.push_back(0.0);
        y_temp.push_back(0.0);
    }
    COMrefX = 0; COMrefY = 0;
    pitchRef = 0; rollRef = 0;
    kalman.K = kalmanGain(&sys_d,&kalman);
    dlqr.K = dlqrGain(&sys_d,&dlqr);
    // dlqr.Kr = feedforwardGain(&sys_d, &dlqr);
    dlqr.Kr = feedforwardGain(&sys_d, dlqr.K);
}
modelBasedControl::~modelBasedControl()
{
}

void modelBasedControl::loadModelParam(std::string modelParamPath_)
{
    YAML::Node mbc;
    std::string mbc_yaml_path_ = modelParamPath_ + "/config/quintic_walk/modelBasedControl.yaml";
    try
    {
        mbc = YAML::LoadFile(mbc_yaml_path_.c_str());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load Model Based Control Param yaml file.");
        return;
    }

    ROS_INFO("Success to load Model Based Control Param yaml file.");

    YAML::Node ss = mbc["ss"];
    sys_c.Ts = ss["Ts"].as<double>();
    sys_d.Ts = ss["Ts"].as<double>();

    YAML::Node obsv = mbc["observer"];
    gyroThreshold = obsv["gyroThreshold"].as<double>();

    YAML::Node ctrl = mbc["controller"];
    tuneGain = ctrl["tuneGain"].as<bool>();
    testObserver = ctrl["testObserver"].as<bool>();
    COMrefX = ctrl["COMrefX"].as<double>();
    COMrefY = ctrl["COMrefY"].as<double>();
    pitchRef = ctrl["pitchRef"].as<double>();
    rollRef = ctrl["rollRef"].as<double>();
    GainIntegral = ctrl["GainIntegral"].as<double>();
    batas = ctrl["Batas"].as<double>();

    // ROS_WARN("INI BENERRRRRRRRRR");

    // read CSV supaya bisa baca array langsung
    sys_c.A = csv.getMatrix(modelParamPath_ + "/config/stateSpace/A.csv");
    sys_c.B = csv.getMatrix(modelParamPath_ + "/config/stateSpace/B.csv");
    sys_c.C = csv.getMatrix(modelParamPath_ + "/config/stateSpace/C.csv");
    sys_c.D = csv.getMatrix(modelParamPath_ + "/config/stateSpace/D.csv");
    sys_d.A = sys_c.A;
    sys_d.B = sys_c.B;
    sys_d.C = sys_c.C;
    sys_d.D = sys_c.D;
    dlqr.Q = csv.getMatrix(modelParamPath_ + "/config/dlqr/Q.csv");
    dlqr.R = csv.getMatrix(modelParamPath_ + "/config/dlqr/R.csv");
    kalman.Q = csv.getMatrix(modelParamPath_ + "/config/kalman/Q.csv");
    kalman.R = csv.getMatrix(modelParamPath_ + "/config/kalman/R.csv");
    c2d(&sys_c);
    // dlqr.K = dlqrGain(&sys_d,&dlqr);
    // kalman.K = kalmanGain(&sys_d,&kalman);
    // std::cout<<"Finished loading config "<<modelParamPath_<<std::endl;
}


void modelBasedControl::resetValue()
{

}

void modelBasedControl::c2d(stateSpace *sys_c)
{
    MatrixXd A(sys_c->A.size(),sys_c->A[0].size()), B(sys_c->B.size(),sys_c->B[0].size());
    MatrixXd Ad(sys_c->A.size(),sys_c->A[0].size()), Bd(sys_c->B.size(),sys_c->B[0].size());
    // std::cout<<"Size A: "<<sys_c->A.size()<<","<<sys_c->A[0].size()<<std::endl;
    // std::cout<<"Size B: "<<sys_c->B.size()<<","<<sys_c->B[0].size()<<std::endl;
    for(int i=0; i<sys_c->A.size(); i++)
    {
        for(int j=0; j<sys_c->A[i].size(); j++)
        A(i,j) = sys_c->A[i][j];
    }
    for(int i=0; i<sys_c->B.size(); i++)
    {
        for(int j=0; j<sys_c->B[i].size(); j++)
        B(i,j) = sys_c->B[i][j];
    }
    Ad = (A*sys_c->Ts).MatrixBase::exp();
    Bd = A.inverse()*(Ad-MatrixXd::Identity(Ad.rows(), Ad.cols()))*B;
    for(int i=0; i<sys_c->A.size(); i++)
    {
        for(int j=0; j<sys_c->A[i].size(); j++)
        sys_d.A[i][j] = Ad(i,j);
    }
    for(int i=0; i<sys_c->B.size(); i++)
    {
        for(int j=0; j<sys_c->B[i].size(); j++)
        sys_d.B[i][j] = Bd(i,j);
    }
    // std::cout<<"Ts: "<<sys_d.Ts<<std::endl;
    // std::cout<<"Ad: "<<Ad<<std::endl;
    // std::cout<<"Bd: "<<Bd<<std::endl;
}

std::vector<std::vector<double> > modelBasedControl::dlqrGain(stateSpace *sys_d, gain *dlqr)
{
    loadModelParam(modelParamPath_);
    MatrixXd Ad(sys_d->A.size(),sys_d->A[0].size()), Bd(sys_d->B.size(),sys_d->B[0].size());
    MatrixXd Q(dlqr->Q.size(),dlqr->Q[0].size()), R(dlqr->R.size(),dlqr->R[0].size());
    MatrixXd K(sys_d->B[0].size(),sys_d->A.size()), P(dlqr->Q.size(),dlqr->Q[0].size()), Pt(dlqr->Q.size(),152*dlqr->Q[0].size());
    std::vector<std::vector<double> > K_DP (sys_d->B[0].size(),std::vector<double> (sys_d->A.size()));
    for(int i=0; i<sys_d->A.size(); i++)
    {
        for(int j=0; j<sys_d->A[i].size(); j++)
        Ad(i,j) = sys_d->A[i][j];
    }
    for(int i=0; i<sys_d->B.size(); i++)
    {
        for(int j=0; j<sys_d->B[i].size(); j++)
        Bd(i,j) = sys_d->B[i][j];
    }
    for(int i=0; i<dlqr->Q.size(); i++)
    {
        for(int j=0; j<dlqr->Q[i].size(); j++)
        Q(i,j) = dlqr->Q[i][j];
    }
    for(int i=0; i<dlqr->R.size(); i++)
    {
        for(int j=0; j<dlqr->R[i].size(); j++)
        R(i,j) = dlqr->R[i][j];
    }

    // Dynamic Programming
    P = MatrixXd::Zero(P.rows(),P.cols());
    Pt = MatrixXd::Zero(Pt.rows(),Pt.cols());
    for (int i=152; i>0; i--)
    {
        P = Q + Ad.transpose()*P*Ad-Ad.transpose()*P*Bd*(R+Bd.transpose()*P*Bd).inverse()*Bd.transpose()*P*Ad;
        // Pt << P, Pt;
        Pt.block(0,(i-1)*P.cols(),P.rows(),P.cols()) = P;
    }
    // std::cout<<"P: "<<P<<std::endl;
    // std::cout<<"Pt: "<<Pt<<std::endl;
    P = Pt.block(0,0,P.rows(),P.cols());
    K = (R+Bd.transpose()*P*Bd).inverse()*Bd.transpose()*P*Ad;
    for(int i=0; i<K.rows(); i++)
    {
        for(int j=0; j<K.cols(); j++)
        K_DP[i][j] = K(i,j);
    }
    // std::cout<<"DLQR Gain Kdlqr: "<<K<<std::endl;
    return K_DP;
}

std::vector<std::vector<double> > modelBasedControl::feedforwardGain(stateSpace *sys_d, std::vector<std::vector<double>> K)//gain *dlqr)
{
    MatrixXd Ad(sys_d->A.size(),sys_d->A[0].size()), Bd(sys_d->B.size(),sys_d->B[0].size()), Cd(1,sys_d->C[0].size()), Dd(1,sys_d->D[0].size());
    MatrixXd Kr_ss(sys_d->B[0].size(),1), K_ss(sys_d->B[0].size(), sys_d->A.size());
    std::vector<std::vector<double> > Kr (sys_d->B[0].size(),std::vector<double> (1));
    for(int i=0; i<sys_d->A.size(); i++)
    {
        for(int j=0; j<sys_d->A[i].size(); j++)
        Ad(i,j) = sys_d->A[i][j];
    }
    for(int i=0; i<sys_d->B.size(); i++)
    {
        for(int j=0; j<sys_d->B[i].size(); j++)
        Bd(i,j) = sys_d->B[i][j];
    }
    for(int j=0; j<sys_d->C[0].size(); j++)
        Cd(0,j) = sys_d->C[0][j];
    for(int j=0; j<sys_d->D[0].size(); j++)
        Dd(0,j) = sys_d->D[0][j];
    for(int i=0; i<sys_d->B[0].size(); i++)
    {
        for(int j=0; j<sys_d->A.size(); j++)
            // K_ss(i,j) = dlqr->K[i][j];
            K_ss(i,j) = K[i][j];
    }
    // Kr_ss = (Cd*(MatrixXd::Identity(sys_d->A.size(),sys_d->A[0].size()) - (Ad-Bd*K_ss)).inverse() ).inverse()*Bd;
    Kr_ss = ((Cd-Dd*K_ss)*(MatrixXd::Identity(sys_d->A.size(),sys_d->A[0].size()) - (Ad-Bd*K_ss)).inverse()*Bd+Dd).inverse();
    // std::cout<<"Kr_ss: "<<Kr_ss<<std::endl;
    for(int i=0; i<Kr_ss.rows(); i++)
    {
        for(int j=0; j<Kr_ss.cols(); j++)
        {
            Kr[i][j] = Kr_ss(i,j);
        }
    }
    //     Acl = [Ad-Bd*K];
    // Ccl = [Cd-Dd*K];
    // Kr  =  pinv(Ccl*inv(eye(size(Acl))-Acl)*Bd+Dd)
    // std::cout<<"Feedforward gain Kr: "<<Kr_ss<<std::endl;
    return Kr;
}

std::vector<std::vector<double> > modelBasedControl::kalmanGain(stateSpace *sys_d, gain *kalman)
{
    loadModelParam(modelParamPath_);
    MatrixXd At(sys_d->A[0].size(),sys_d->A.size()), Ct(sys_d->C[0].size(),sys_d->C.size());
    MatrixXd Q(kalman->Q.size(),kalman->Q[0].size()), R(kalman->R.size(),kalman->R[0].size());
    MatrixXd K(sys_d->A.size(),sys_d->C.size()), P(kalman->Q.size(),kalman->Q[0].size()), Pt(kalman->Q.size(),152*kalman->Q[0].size());
    std::vector<std::vector<double> > K_DP (sys_d->A.size(),std::vector<double> (sys_d->C.size()));
    for(int i=0; i<sys_d->A[0].size(); i++)
    {
        for(int j=0; j<sys_d->A.size(); j++)
        At(i,j) = sys_d->A[j][i];
    }
    // std::cout<<"At: "<<At<<std::endl;
    for(int i=0; i<sys_d->C[0].size(); i++)
    {
        for(int j=0; j<sys_d->C.size(); j++)
        Ct(i,j) = sys_d->C[j][i];
        // std::cout<<sys_d->C[j][i]<<" "<<std::endl;
    }
    // std::cout<<"Ct: "<<Ct<<std::endl;
    for(int i=0; i<kalman->Q.size(); i++)
    {
        for(int j=0; j<kalman->Q[i].size(); j++)
        Q(i,j) = kalman->Q[i][j];
    }
    for(int i=0; i<kalman->R.size(); i++)
    {
        for(int j=0; j<kalman->R[i].size(); j++)
        R(i,j) = kalman->R[i][j];
    }

    // Dynamic Programming
    P = MatrixXd::Zero(P.rows(),P.cols());
    Pt = MatrixXd::Zero(Pt.rows(),Pt.cols());
    for (int i=152; i>0; i--)
    {
        P = Q + At.transpose()*P*At-At.transpose()*P*Ct*(R+Ct.transpose()*P*Ct).inverse()*Ct.transpose()*P*At;
        // Pt << P, Pt;
        Pt.block(0,(i-1)*P.cols(),P.rows(),P.cols()) = P;
    }
    // std::cout<<"P: "<<P<<std::endl;
    // std::cout<<"Pt: "<<Pt<<std::endl;
    P = Pt.block(0,0,P.rows(),P.cols());
    K = ((R+Ct.transpose()*P*Ct).inverse()*Ct.transpose()*P*At).transpose();
    for(int i=0; i<K.rows(); i++)
    {
        for(int j=0; j<K.cols(); j++)
        K_DP[i][j] = K(i,j);
    }
    // std::cout<<"Kalman gain: "<<K<<std::endl;
    return K_DP;
}

void modelBasedControl::stateSpaceSimulator(double u, std::vector<double> &x, std::vector<double> &y)
{
    loadModelParam(modelParamPath_);
    MatrixXd Ad(sys_d.A.size(),sys_d.A[0].size()), Bd(sys_d.B.size(),sys_d.B[0].size()), Cd(sys_d.C.size(),sys_d.C[0].size()), Dd(sys_d.D.size(),sys_d.D[0].size());
    Eigen::VectorXd x_next(sys_d.A.size()), x_prev(sys_d.A.size()), ysim(sys_d.C.size()), u_ss(sys_d.B[0].size());
    for(int i=0; i<sys_d.A.size(); i++)
    {
        for(int j=0; j<sys_d.A[i].size(); j++)
        Ad(i,j) = sys_d.A[i][j];
    }
    for(int i=0; i<sys_d.B.size(); i++)
    {
        for(int j=0; j<sys_d.B[i].size(); j++)
        Bd(i,j) = sys_d.B[i][j];
    }
    for(int i=0; i<sys_d.C.size(); i++)
    {
        for(int j=0; j<sys_d.C[i].size(); j++)
        Cd(i,j) = sys_d.C[i][j];
    }
    for(int i=0; i<sys_d.D.size(); i++)
    {
        for(int j=0; j<sys_d.D[i].size(); j++)
        Dd(i,j) = sys_d.D[i][j];
    }
    for(int i=0; i<x_next.rows(); i++)
    {
        x_prev(i) = xsim_temp[i];
        x[i] = x_prev(i);
    }
    u_ss(0) = u;
    ysim = Cd*x_prev + Dd*u_ss;
    x_next = Ad*x_prev + Bd*u_ss;
    for(int i=0; i<sys_d.C.size(); i++)
    {
        y[i] = ysim(i);
    }
    for(int i=0; i<x_next.rows(); i++)
    {
        xsim_temp[i] = x_next(i);
    }
    // std::cout<<"Simulated Output (degrees): Angle: "<<ysim(0)*180/M_PI<<"\tGyro: "<<ysim(1)*180/M_PI<<std::endl;
    // std::cout<<"Input: "<<u*180/M_PI<<"\tAngle: "<<x_prev(0)*180/M_PI<<"\tGyro: "<<x_prev(1)*180/M_PI<<std::endl;
    // std::cout<<"Ts: "<<sys_d.Ts<<std::endl;
    // std::cout<<"Ad: "<<Ad<<std::endl;
    // std::cout<<"Bd: "<<Bd<<std::endl;
}

// void modelBasedControl::wholeBodyStateObserver(double u, std::vector<double> &xhat, std::vector<double> &yh, std::vector<double> y)
void modelBasedControl::wholeBodyStateObserver(double u, std::vector<double> &xhat, std::vector<std::vector<double>> kalmanK, std::vector<double> &yh, std::vector<double> y)
{
    loadModelParam(modelParamPath_);
    // kalman.K = kalmanGain(&sys_d,&kalman);
    MatrixXd Ad(sys_d.A.size(),sys_d.A[0].size()), Bd(sys_d.B.size(),sys_d.B[0].size()), Cd(sys_d.C.size(),sys_d.C[0].size()), Dd(sys_d.D.size(),sys_d.D[0].size());
    MatrixXd L(sys_d.A.size(),sys_d.C.size());
    Eigen::VectorXd x_next(sys_d.A.size()), x_prev(sys_d.A.size()), xhat_next(sys_d.A.size()), xhat_prev(sys_d.A.size()), yhat(sys_d.C.size()), ymeas(sys_d.C.size()), u_ss(sys_d.B[0].size());
    u_ss(0) = u;
    for(int i=0; i<sys_d.A.size(); i++)
    {
        for(int j=0; j<sys_d.A[i].size(); j++)
        Ad(i,j) = sys_d.A[i][j];
    }
    for(int i=0; i<sys_d.B.size(); i++)
    {
        for(int j=0; j<sys_d.B[i].size(); j++)
        Bd(i,j) = sys_d.B[i][j];
    }
    for(int i=0; i<sys_d.C.size(); i++)
    {
        for(int j=0; j<sys_d.C[i].size(); j++)
        Cd(i,j) = sys_d.C[i][j];
    }
    for(int i=0; i<sys_d.D.size(); i++)
    {
        for(int j=0; j<sys_d.D[i].size(); j++)
        Dd(i,j) = sys_d.D[i][j];
    }
    for(int i=0; i<sys_d.A.size(); i++)
    {
        for(int j=0; j<sys_d.C.size(); j++)
        L(i,j) = kalmanK[i][j];
    }

    for(int i=0; i<x_next.rows(); i++)
    {
        ymeas(i) = y[i];
        x_prev(i) = x_temp[i];
        xhat_prev(i) = xhat_temp[i];
        // x[i] = x_temp[i];
        // xhat[i] = xhat_temp[i];
    }
    yhat = Cd*xhat_prev + Dd*u_ss;
    // yhat(0) = y[0];
    xhat_next = Ad*xhat_prev + Bd*u_ss + L*(ymeas - yhat);
    // // Gyro Filter 
    // std::cout<<"GyroThreshold: "<<gyroThreshold<<std::endl;
    // // std::cout<<"Diff Gyro "<<(y[1]-y_temp[1])*180/M_PI<<" deg/s"<<std::endl;
    // if(fabs(xhat_next(1)) <= gyroThreshold*M_PI/180)
    // {
    //     ROS_WARN("GYRO SATURATED FILTER");
    //     xhat_next(1) = 0;
    // }
    // Matlab:
    // yhat = C*xhat + D*u;
    // x = A*x+B*u;
    // xhat = A*xhat+B*u+L*(y(k+1,:)'-yhat);

    for(int i=0; i<x_next.rows(); i++)
    {
        x_temp[i] = x_next(i);
        xhat_temp[i] = xhat_next(i);
        xhat[i] = xhat_next(i);
    }
    for(int i=0; i<yhat.rows(); i++)
    {
        y_temp[i] = ymeas(i);
        yh[i] = yhat(i);
    }

    // if (k >=1 && abs(y(k+1,2)-y(k,2)) > gyroTol && abs(y(k,2))>20*pi/180)
    //         gyroPitchHat = [gyroPitchHat, xhat(2)];
    //     else
    //         gyroPitchHat = [gyroPitchHat, x(2)];
    //     end
    // std::cout<<"Measured Output (degrees): Angle: "<<ymeas(0)*180/M_PI<<"\tGyro: "<<ymeas(1)*180/M_PI<<std::endl;
    // std::cout<<"Input: "<<u*180/M_PI<<"\tAngleEst: "<<yhat(0)*180/M_PI<<"\tGyroEst: "<<yhat(1)*180/M_PI<<std::endl;
    // if(reset == 1)
    // {
    //     x_next << 0, 0;
    //     x_prev << 0, 0;
    //     y_ss << 0;
    //     u_ss << 0;
    //     for(int i=0; i < x_next.rows(); i++)
    //     {
    //         x_temp.push_back(0.0);
    //     }
    // }
}

double modelBasedControl::outputFeedback(double ref, std::vector<double> &x, std::vector<std::vector<double>> dlqrK, std::vector<std::vector<double>> dlqrKr)
// double modelBasedControl::outputFeedback(double ref, std::vector<double> &x, double *controlEffort)
{
    Eigen::VectorXd x_ss(sys_d.A.size()), u_ss(sys_d.B[0].size()), r_ss(sys_d.C.size());
    Eigen::MatrixXd K(sys_d.B[0].size(),sys_d.A.size()), Kr(sys_d.B[0].size(),sys_d.C.size());
    double u;
    for(int i=0; i<sys_d.A.size(); i++)
        x_ss(i) = x[i];
    r_ss(0) = ref;
    for(int i=0; i<dlqrK.size(); i++)
    {
        for(int j=0; j<dlqrK[0].size(); j++)
        {
            K(i,j) = dlqrK[i][j];
        }
    }
    for(int i=0; i<dlqrKr.size(); i++)
    {
        for(int j=0; j<dlqrKr[0].size(); j++)
        {
            Kr(i,j) = dlqrKr[i][j];
        }
    }
    u_ss = -K*x_ss + Kr*r_ss;
    u = u_ss(0);

    // std::cout<<"Control effort: "<<u_ss*180/M_PI<<" degrees"<<std::endl;
    return u;
}

}
