#include "model_based_control/densis.h"


namespace robotis_op
{
densis::densis(std::string densisParamPath_)
// densis::densis()
{
//    kickManip_path_ = ros::package::getPath("kicking") + "/config/kickManip.yaml";
//    densis_path_ = ros::package::getPath("op3_kinematics_dynamics") + "/config/densisParam.yaml";
   densis_path_ = densisParamPath_ + "/config/densisParam.yaml";
   physicalParam_path_ = ros::package::getPath("op3_kinematics_dynamics") + "/config/physicalParam.yaml";
   reset = 0;
//    loadPhysicalParam();
   loadDensisParam();
   // Observer Initialize
    for(int i=0; i < 2; i++)
    {
        ymeas.push_back(0.0);
    }
    m_time_start = ros::Time::now().toSec();
}
densis::~densis()
{
}

void densis::loadPhysicalParam()
{
    YAML::Node ph;

    try
    {
        ph = YAML::LoadFile(physicalParam_path_.c_str());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load Physical Param yaml file.");
        return;
    }
    m  = ph["m"].as<double>();
    dT = ph["dT"].as<double>();
    B  = ph["B"].as<double>();
    F  = ph["F"].as<double>();
    Is = ph["Is"].as<double>();
}

void densis::loadDensisParam()
{
    YAML::Node dn;

    try
    {
        dn = YAML::LoadFile(densis_path_.c_str());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load Densis Param yaml file.");
        return;
    }
    densisMode = dn["densisMode"].as<bool>();
    YAML::Node densis = dn["densis"];
    inputType = densis["inputType"].as<int>();
    inputAmp = densis["inputAmp"].as<double>();
    freq = densis["freq"].as<double>();

    YAML::Node psn = densis["pseudonoise"];
    minPSN =  psn["min"].as<double>();
    maxPSN =  psn["max"].as<double>();
    resolutionPSN = psn["resolution"].as<double>();
    holdTimePSN = psn["holdTime"].as<double>();

    YAML::Node pr = densis["pulsedRich"];
    minPulsedRich = pr["min"].as<double>();
    maxPulsedRich = pr["max"].as<double>();
    onTime = pr["onTime"].as<double>();
    offTime = pr["offTime"].as<double>();

    YAML::Node rpr = densis["randomPulsedRich"];
    minRandomPulsedRich = rpr["min"].as<double>();
    maxRandomPulsedRich = rpr["max"].as<double>();

    YAML::Node siswi = densis["sinesweep"];
    minSine =  siswi["min"].as<double>();
    maxSine =  siswi["max"].as<double>();
    fmin = siswi["fmin"].as<double>();
    fmax = siswi["fmax"].as<double>();
    sweepPeriod = siswi["period"].as<double>();
}

void densis::resetValue()
{

}


double densis::generatePseudonoise(double t, double min, double max, double resolution, double holdTime)
{
    double random;
    random = min/resolution + (rand() % (int)((max - min + 1)/resolution));
    if(holdValue)
    {
        signalPSN = random * resolution;
        // std::cout<<"ChangeSignal: "<<signalPSN<<std::endl;
    }
    holdValue = false;
    if( ((t-prevSeconds) >= holdTime) && holdValue == false )
    {
        prevSeconds = t;
        holdValue = true;
    }
    // std::cout<<"HoldValue: "<<holdValue<<std::endl;
    // std::cout<<"OutSignal: "<<signalPSN<<std::endl;
    return signalPSN;
}

double densis::sineSweep(double t, double minAmp, double maxAmp, double fmin, double fmax, double sweepPeriod)
{
    double signal;
    if(sweepState == false)
    {
        tSweepStart = t;
        sweepState = true;
    }
    if((t-tSweepStart) < sweepPeriod && sweepState == true)
    {
        fsweep = fmin + (fmax-fmin)*(t-tSweepStart)/sweepPeriod;
        // fsweep = fmin + fabs((fmax-fmin)*sin(2*M_PI*(t-tSweepStart)/sweepPeriod));
        signal = (minAmp+maxAmp)/2 + ((maxAmp-minAmp)/2)*sin(2*M_PI*fsweep*(t-tSweepStart));
    }
    else
    {
        sweepState = false;
        // fsweep = 0;
        // tSweepStart = t;
    }
    if(signal > maxAmp) signal = maxAmp;
        if(signal < minAmp) signal = minAmp;
    // std::cout<<"Frequency Sweep: "<<fsweep<<"Hz \tDeltaT: "<<t-tSweepStart<<std::endl;
    // std::cout<<"Sine Sweep Magnitude: "<<signal<<std::endl;
    return signal;
}

double densis::pseudonoise2PRBS(double pseudonoise)
{
    if(pseudonoise >= (maxPSN-minPSN)/2+minPSN)
    {
        PRBS = maxPSN;
    }
    else
    {
        PRBS = minPSN;
    }
    return PRBS;
}

double densis::pulsedRich(double t, double min, double max, double pseudonoise, double onTime, double offTime)
{
    if((pulseState == true) && (t - prevSquareSeconds >= offTime))
    {
        prevSquareSeconds = t;
        pulseState = false;
        signalPulse = max;
    }
    else if ((pulseState == false) && (t - prevSquareSeconds >= onTime))
    {
        pulseState = true;
        prevSquareSeconds = t;
        signalPulse = min;
    }

    if(signalPulse == min)
    {
        signalPulsedRich = min;
    }
    else
    {
        signalPulsedRich = signalPulse+pseudonoise;
    }
    // std::cout<<"t: "<<t<<"\tt-prev: "<<t-prevSquareSeconds<<std::endl;
    // std::cout<<"pulseState: "<<pulseState<<std::endl;
    // std::cout<<"OutSignalPulsedRich: "<<signalPulsedRich<<std::endl;
    return signalPulsedRich;
}

double densis::randomPulsedRich(double t, double min, double max, double pseudonoise, double onTime, double offTime)
{
    double random;
    random = min + (rand() % (int)((max - min + 1)));
    if((pulseState == true) && (t - prevSquareSeconds >= offTime))
    {
        prevSquareSeconds = t;
        pulseState = false;
        signalPulse = random;
    }
    else if ((pulseState == false) && (t - prevSquareSeconds >= onTime))
    {
        pulseState = true;
        prevSquareSeconds = t;
        signalPulse = min;
    }

    if(signalPulse == min)
    {
        signalPulsedRich = min;
    }
    else
    {
        signalPulsedRich = signalPulse+pseudonoise;
    }
    // std::cout<<"t: "<<t<<"\tt-prev: "<<t-prevSquareSeconds<<std::endl;
    // std::cout<<"pulseState: "<<pulseState<<std::endl;
    // std::cout<<"OutSignalPulsedRich: "<<signalPulsedRich<<std::endl;
    return signalPulsedRich;
}

double densis::densisInput(double m_delta_time)
{
    loadDensisParam();
    time_densis = m_delta_time;
    pseudonoise = generatePseudonoise(time_densis, minPSN,maxPSN,resolutionPSN ,holdTimePSN);
    if(inputType == 0)  //constant angle
    {
        inputSignal = inputAmp*PI/180;
        // std::cout<<"1.Step Input: "<<inputSignal<<std::endl;
    }
    else if(inputType == 1) //sinusoidal angle
    {
        inputSignal = (inputAmp*PI/180)*sin(2*PI*freq*time_densis);
        // std::cout<<"2.Sinusoidal Input: "<<inputSignal<<std::endl;
    }
    else if(inputType == 2) //pseudonoise
    {
        inputSignal = pseudonoise*PI/180;//generatePseudonoise(time_densis, minPSN,maxPSN,resolutionPSN ,holdTimePSN)*PI/180;
        // std::cout<<"3.Pseudonoise: "<<inputSignal<<std::endl;
    }
    else if(inputType == 3) //PRBS
    {
        inputSignal = pseudonoise2PRBS(pseudonoise)*PI/180;
        // std::cout<<"3.PRBS: "<<inputSignal<<std::endl;
    }
    else if(inputType == 4) // 4: pulsed rich input
    {
        inputSignal = pulsedRich(time_densis, minPulsedRich, maxPulsedRich,
                      pseudonoise, onTime, offTime)*PI/180;
        // std::cout<<"4.PulsedRich: "<<inputSignal<<std::endl;
    }
    else if(inputType == 5) // 5: random pulsed rich input
    {
        inputSignal = randomPulsedRich(time_densis, minRandomPulsedRich, maxRandomPulsedRich,
                      pseudonoise, onTime, offTime)*PI/180;
        // std::cout<<"5.RandomPulsedRich: "<<inputSignal<<std::endl;
    }
    else if(inputType == 6) //sine sweep
    {
        inputSignal = sineSweep(time_densis, minSine, maxSine, fmin, fmax, sweepPeriod)*PI/180;//generatePseudonoise(time_densis, minPSN,maxPSN,resolutionPSN ,holdTimePSN)*PI/180;
        // std::cout<<"6.SineSweep: "<<inputSignal<<std::endl;
    }
    else
    {
        inputSignal = 0;
        // std::cout<<"0.No Input"<<std::endl;
    }
    // std::cout<<"Angle to Angle Input (deg)"<<inputSignal*180/PI<<std::endl;
    return inputSignal;
}

}
