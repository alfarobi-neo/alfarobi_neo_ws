#include <quintic_walk/quinticwalk.h>

namespace robotis_op

{
double QuinticWalk::getPhase() const
{
    return _phase;
}

double QuinticWalk::getTrajsTime() const
{
    double t;
    if (_phase < 0.5) {
        t = _phase/walking_param_.freq;
    } else {
        t = (_phase - 0.5)/walking_param_.freq;
    }

    return t;
}

//const VectorLabel& QuinticWalk::getParameters() const
//{
//    return _params;
//}

const Eigen::Vector3d& QuinticWalk::getOrders() const
{
    return _orders;
}

Footstep QuinticWalk::getFootstep(){
    return _footstep;
}

double QuinticWalk::getWeightBalance(){
    double balance_left_right;
    if(isDoubleSupport()){
        Eigen::Vector3d trunkPos;
        Eigen::Vector3d trunkAxis;
        Eigen::Vector3d footPos;
        Eigen::Vector3d footAxis;
        TrajectoriesTrunkFootPos(getTrajsTime(), _trajs, trunkPos, trunkAxis, footPos, footAxis);
        // simply consider the position of the trunk in relation to the feet
        // this is not completly correct since trunk!=COM
        // we consider the trunk to be always between the feet, therefore we can directly use the vector lengths
        // we clamp at 0 and 1, for robustnes
        double balance = std::min(1.0, std::max(0.0,trunkPos.norm()/footPos.norm()));
        if(isLeftSupport()){
            balance_left_right = 1- balance;
        }else{
            balance_left_right = balance;
        }
    }else{
        if(isLeftSupport()){
            balance_left_right = 1.0;
        }else{
            balance_left_right = 0.0;
        }
    }
}


bool QuinticWalk::isEnabled() const
{
    return _isTrajsOscillating;
}

bool QuinticWalk::isLeftSupport(){
    return _footstep.isLeftSupport();
}

bool QuinticWalk::isDoubleSupport(){
    // returns true if the value of the "is_double_support" spline is currently higher than 0.5
    // the spline should only have values of 0 or 1
    return _trajs.get("is_double_support").pos(getTrajsTime()) >= 0.5 ? true : false;
}

//void QuinticWalk::setParameters(const VectorLabel& params)
//{
//    _params = params;
//    _footstep.setFootDistance(walking_param_.footDistance"));
//    _footstep.reset(true);
//}

void QuinticWalk::forceRebuildTrajectories()
{
    //Reset the trunk saved state
    resetTrunkLastState();
    //Rebuild the trajectories
    buildTrajectories();
}

void QuinticWalk::setOrders(
        const Eigen::Vector3d& orders,
        bool isEnabled,
        bool beginWithLeftSupport)
{
    //Set the walk orders
    _orders = orders;
    //Set the walk enable
    bool lastEnable = _isEnabled;
    _isEnabled = isEnabled;
    //Reset the support footstep and phase
    //to specific support foot on enable
    if (isEnabled && !lastEnable) {
        //Reset the footsteps
        if (beginWithLeftSupport) {
            _phase = 0.0 + 1e-6;
            _footstep.reset(true);
        } else {
            _phase = 0.5 + 1e-6;
            _footstep.reset(false);
        }
        //Reset the trunk saved state
        //as the support foot as been updated
        resetTrunkLastState();
        //Rebuild the trajectories
        buildTrajectories();
        //Save last enabled value
        _wasEnabled = _isEnabled;
    }
}

const Trajectories& QuinticWalk::getTrajectories() const
{
    return _trajs;
}

void QuinticWalk::update(double dt)
{
    //Check for negative time step
    if (dt <= 0.0) {
        std::cerr << "QuinticWalk exception negative dt phase="
                  << _phase << " dt="
                  << dt << std::endl;
        //throw std::logic_error(
        //    "QuinticWalk negative dt: "
        //    + std::to_string(dt));
    }
    //Check for too long dt
    if (dt > 0.25/walking_param_.freq) {
        std::cerr << "QuinticWalk error too long dt phase="
                  << _phase << " dt="
                  << dt << std::endl;
        return;
    }

    //Update the phase
    double lastPhase = _phase;
    _phase += dt*walking_param_.freq;
    if (_phase >= 1.0) {
        _phase -= 1.0;
        //Bound to zero in case
        //of floating point error
        if (_phase < 0.0) {
            _phase = 0.0;
        }
    }

    //Detect the phase of support foot swap
    //and computed the next half cycle
    if (
            (lastPhase < 0.5 && _phase >= 0.5) ||
            (lastPhase > 0.5 && _phase <= 0.5)
            ) {
        //Evaluate current trunk state
        //(position, velocity, acceleration)
        //in next support foot frame
        double halfPeriod = 1.0/(2.0*walking_param_.freq);
        Eigen::Vector2d trunkPos(
                    _trajs.get("trunk_pos_x").pos(halfPeriod),
                    _trajs.get("trunk_pos_y").pos(halfPeriod));
        Eigen::Vector2d trunkVel(
                    _trajs.get("trunk_pos_x").vel(halfPeriod),
                    _trajs.get("trunk_pos_y").vel(halfPeriod));
        Eigen::Vector2d trunkAcc(
                    _trajs.get("trunk_pos_x").acc(halfPeriod),
                    _trajs.get("trunk_pos_y").acc(halfPeriod));
        //Convert in next support foot frame
        trunkPos.x() -= _footstep.getNext().x();
        trunkPos.y() -= _footstep.getNext().y();
        trunkPos = Eigen::Rotation2Dd(
                    -_footstep.getNext().z()).toRotationMatrix()
                *trunkPos;
        trunkVel = Eigen::Rotation2Dd(
                    -_footstep.getNext().z()).toRotationMatrix()
                *trunkVel;
        trunkAcc = Eigen::Rotation2Dd(
                    -_footstep.getNext().z()).toRotationMatrix()
                *trunkAcc;
        //Save state
        _trunkPosAtLast.x() = trunkPos.x();
        _trunkPosAtLast.y() = trunkPos.y();
        _trunkVelAtLast.x() = trunkVel.x();
        _trunkVelAtLast.y() = trunkVel.y();
        _trunkAccAtLast.x() = trunkAcc.x();
        _trunkAccAtLast.y() = trunkAcc.y();
        //No transformation for height
        _trunkPosAtLast.z() = _trajs.get("trunk_pos_z").pos(halfPeriod);
        _trunkVelAtLast.z() = _trajs.get("trunk_pos_z").vel(halfPeriod);
        _trunkAccAtLast.z() = _trajs.get("trunk_pos_z").acc(halfPeriod);
        //Evaluate and save trunk orientation
        //in next support foot frame
        Eigen::Vector3d trunkAxis(
                    _trajs.get("trunk_axis_x").pos(halfPeriod),
                    _trajs.get("trunk_axis_y").pos(halfPeriod),
                    _trajs.get("trunk_axis_z").pos(halfPeriod));
        //Convert in intrinsic euler angle
        Eigen::Matrix3d trunkMat = AxisToMatrix(trunkAxis);
        Eigen::Vector3d trunkEuler = MatrixToEulerIntrinsic(trunkMat);
        //Transform to next support foot
        trunkEuler.z() -= _footstep.getNext().z();
        //Reconvert to axis and save it
        trunkMat = EulerIntrinsicToMatrix(trunkEuler);
        trunkAxis = MatrixToAxis(trunkMat);
        _trunkAxisPosAtLast = trunkAxis;
        //Evaluate trunk orientation velocity
        //and acceleration without frame
        //transformation
        _trunkAxisVelAtLast.x() = _trajs.get("trunk_axis_x").vel(halfPeriod);
        _trunkAxisVelAtLast.y() = _trajs.get("trunk_axis_y").vel(halfPeriod);
        _trunkAxisVelAtLast.z() = _trajs.get("trunk_axis_z").vel(halfPeriod);
        _trunkAxisAccAtLast.x() = _trajs.get("trunk_axis_x").acc(halfPeriod);
        _trunkAxisAccAtLast.y() = _trajs.get("trunk_axis_y").acc(halfPeriod);
        _trunkAxisAccAtLast.z() = _trajs.get("trunk_axis_z").acc(halfPeriod);
        //Update footstep
        if (_isEnabled) {
            _footstep.stepFromOrders(_orders);
        } else {
            _footstep.stepFromOrders(Eigen::Vector3d(0.0, 0.0 , 0.0));
        }
        //Build trajectories
        buildTrajectories();
        //Save last enabled value
        _wasEnabled = _isEnabled;
    }

    //Check support foot state
    if (
            (_phase < 0.5 && !_footstep.isLeftSupport()) ||
            (_phase >= 0.5 && _footstep.isLeftSupport())
            ) {
        std::cerr << "QuinticWalk exception invalid state phase="
                  << _phase << " support="
                  << _footstep.isLeftSupport() << " dt="
                  << dt << std::endl;
        //throw std::logic_error(
        //    "QuinticWalk invalid support state");
    }
}

void QuinticWalk::computeCartesianPosition(Eigen::Vector3d& trunkPos, Eigen::Vector3d& trunkAxis, Eigen::Vector3d& footPos,
                                           Eigen::Vector3d& footAxis, bool& isLeftsupportFoot)
{
    //Compute trajectories time
    double time = getTrajsTime();

    computeCartesianPositionAtTime(trunkPos, trunkAxis, footPos, footAxis, isLeftsupportFoot, time);

}


void QuinticWalk::computeCartesianPositionAtTime(Eigen::Vector3d& trunkPos, Eigen::Vector3d& trunkAxis, Eigen::Vector3d& footPos,
                                                 Eigen::Vector3d& footAxis, bool& isLeftsupportFoot, double time)
{
    //Evaluate target cartesian
    //state from trajectories
    bool isDoubleSupport;
    TrajectoriesTrunkFootPos(time, _trajs, trunkPos, trunkAxis, footPos, footAxis);
    TrajectoriesSupportFootState(time, _trajs, isDoubleSupport, isLeftsupportFoot);
}

void QuinticWalk::buildTrajectories()
{
    //Reset the trajectories
    _trajs = TrajectoriesInit();

    //Set up the trajectories
    //for the half cycle
    double halfPeriod = 1.0/(2.0*walking_param_.freq);
    double period = 2.0*halfPeriod;

    //Time length of double and single
    //support phase during the half cycle
    double doubleSupportLength = walking_param_.doubleSupportRatio*halfPeriod;
    double singleSupportLength = halfPeriod-doubleSupportLength;

    //Sign of support foot with
    //respect to lateral
    double supportSign = (_footstep.isLeftSupport() ? 1.0 : -1.0);

    //Walk disable special case
    if (!_isEnabled) {
        //Set double support phase
        double isDoubleSupport = (_wasEnabled ? 0.0 : 1.0);
        _trajs.get("is_double_support").addPoint(
                    0.0, isDoubleSupport);
        _trajs.get("is_double_support").addPoint(
                    halfPeriod, isDoubleSupport);
        //Set support foot
        _trajs.get("is_left_support_foot").addPoint(
                    0.0, _footstep.isLeftSupport());
        _trajs.get("is_left_support_foot").addPoint(
                    halfPeriod, _footstep.isLeftSupport());
        //Flying foot position
        _trajs.get("foot_pos_x").addPoint(
                    0.0, _footstep.getLast().x());
        _trajs.get("foot_pos_x").addPoint(
                    singleSupportLength*walking_param_.footOvershootPhase,
                    0.0 + (0.0-_footstep.getLast().x())
                    *walking_param_.footOvershootRatio);
        _trajs.get("foot_pos_x").addPoint(
                    singleSupportLength, 0.0);
        _trajs.get("foot_pos_x").addPoint(
                    halfPeriod, 0.0);

        _trajs.get("foot_pos_y").addPoint(
                    0.0, _footstep.getLast().y());
        _trajs.get("foot_pos_y").addPoint(
                    singleSupportLength*walking_param_.footOvershootPhase,
                    -supportSign*walking_param_.footDistance
                    + (-supportSign*walking_param_.footDistance-_footstep.getLast().y())
                    *walking_param_.footOvershootRatio);
        _trajs.get("foot_pos_y").addPoint(
                    singleSupportLength, -supportSign*walking_param_.footDistance);
        _trajs.get("foot_pos_y").addPoint(
                    halfPeriod, -supportSign*walking_param_.footDistance);
        //If the walk has just been disabled,
        //make one single step to neutral pose
        if (_wasEnabled) {
            _trajs.get("foot_pos_z").addPoint(
                        0.0, 0.0);
            _trajs.get("foot_pos_z").addPoint(
                        singleSupportLength*walking_param_.footApexPhase,
                        walking_param_.footRise);
            _trajs.get("foot_pos_z").addPoint(
                        singleSupportLength, walking_param_.footPutDownZOffset);
            _trajs.get("foot_pos_z").addPoint(
                        singleSupportLength + walking_param_.footPutDownPhase * doubleSupportLength , 0.0);
            _trajs.get("foot_pos_z").addPoint(
                        halfPeriod, 0.0);
            _isTrajsOscillating = true;
        } else {
            _trajs.get("foot_pos_z").addPoint(
                        0.0, 0.0);
            _trajs.get("foot_pos_z").addPoint(
                        halfPeriod, 0.0);
            _isTrajsOscillating = false;
        }
        //Flying foot orientation
        _trajs.get("foot_axis_x").addPoint(
                    0.0, 0.0);
        _trajs.get("foot_axis_x").addPoint(
                    halfPeriod, 0.0);

        _trajs.get("foot_axis_y").addPoint(
                    0.0, 0.0);
        _trajs.get("foot_axis_y").addPoint(
                    halfPeriod, 0.0);

        _trajs.get("foot_axis_z").addPoint(
                    0.0, _footstep.getLast().z());
        _trajs.get("foot_axis_z").addPoint(
                    singleSupportLength, 0.0);
        _trajs.get("foot_axis_z").addPoint(
                    halfPeriod, 0.0);

        //Trunk position
        _trajs.get("trunk_pos_x").addPoint(
                    0.0,
                    _trunkPosAtLast.x(),
                    _trunkVelAtLast.x(),
                    _trunkAccAtLast.x());
        _trajs.get("trunk_pos_x").addPoint(
                    halfPeriod,
                    walking_param_.trunkXOffset);

        _trajs.get("trunk_pos_y").addPoint(
                    0.0,
                    _trunkPosAtLast.y(),
                    _trunkVelAtLast.y(),
                    _trunkAccAtLast.y());
        _trajs.get("trunk_pos_y").addPoint(
                    halfPeriod,
                    -supportSign*0.5*walking_param_.footDistance
                    + walking_param_.trunkYOffset);

        _trajs.get("trunk_pos_z").addPoint(
                    0.0,
                    _trunkPosAtLast.z(),
                    _trunkVelAtLast.z(),
                    _trunkAccAtLast.z());
        _trajs.get("trunk_pos_z").addPoint(
                    halfPeriod, walking_param_.trunkHeight);
        //Trunk orientation
        _trajs.get("trunk_axis_x").addPoint(
                    0.0,
                    _trunkAxisPosAtLast.x(),
                    _trunkAxisVelAtLast.x(),
                    _trunkAxisAccAtLast.x());
        _trajs.get("trunk_axis_x").addPoint(
                    halfPeriod, 0.0);
        _trajs.get("trunk_axis_y").addPoint(
                    0.0,
                    _trunkAxisPosAtLast.y(),
                    _trunkAxisVelAtLast.y(),
                    _trunkAxisAccAtLast.y());
        _trajs.get("trunk_axis_y").addPoint(
                    halfPeriod, walking_param_.trunkPitch);
        _trajs.get("trunk_axis_z").addPoint(
                    0.0,
                    _trunkAxisPosAtLast.z(),
                    _trunkAxisVelAtLast.z(),
                    _trunkAxisAccAtLast.z());
        _trajs.get("trunk_axis_z").addPoint(
                    halfPeriod, 0.0);
        return;
    }

    //Only move the trunk on the first
    //half cycle after a walk enable
    if (_isEnabled && !_wasEnabled) {
        doubleSupportLength = halfPeriod;
        singleSupportLength = 0.0;
    }
    _isTrajsOscillating = true;

    //Set double support phase
    _trajs.get("is_double_support").addPoint(
                0.0, 0.0);
    _trajs.get("is_double_support").addPoint(
                singleSupportLength, 0.0);
    _trajs.get("is_double_support").addPoint(
                singleSupportLength, 1.0);
    _trajs.get("is_double_support").addPoint(
                halfPeriod, 1.0);
    //Set support foot
    _trajs.get("is_left_support_foot").addPoint(
                0.0, _footstep.isLeftSupport());
    _trajs.get("is_left_support_foot").addPoint(
                halfPeriod, _footstep.isLeftSupport());

    //Flying foot position
    _trajs.get("foot_pos_x").addPoint(
                0.0, _footstep.getLast().x());
    _trajs.get("foot_pos_x").addPoint(
                singleSupportLength*walking_param_.footOvershootPhase,
                _footstep.getNext().x()
                + (_footstep.getNext().x()-_footstep.getLast().x())
                *walking_param_.footOvershootRatio);
    _trajs.get("foot_pos_x").addPoint(
                singleSupportLength, _footstep.getNext().x());
    _trajs.get("foot_pos_x").addPoint(
                halfPeriod, _footstep.getNext().x());

    _trajs.get("foot_pos_y").addPoint(
                0.0, _footstep.getLast().y());
    _trajs.get("foot_pos_y").addPoint(
                singleSupportLength*walking_param_.footOvershootPhase,
                _footstep.getNext().y()
                + (_footstep.getNext().y()-_footstep.getLast().y())
                *walking_param_.footOvershootRatio);
    _trajs.get("foot_pos_y").addPoint(
                singleSupportLength, _footstep.getNext().y());
    _trajs.get("foot_pos_y").addPoint(
                halfPeriod, _footstep.getNext().y());

    _trajs.get("foot_pos_z").addPoint(
                0.0, 0.0);
    float apex_time = singleSupportLength*walking_param_.footApexPhase;
    _trajs.get("foot_pos_z").addPoint(
                apex_time,
                walking_param_.footRise);
    _trajs.get("foot_pos_z").addPoint(
                singleSupportLength, walking_param_.footPutDownZOffset);
    _trajs.get("foot_pos_z").addPoint(
                singleSupportLength + walking_param_.footPutDownPhase * doubleSupportLength , 0.0);
    _trajs.get("foot_pos_z").addPoint(
                halfPeriod, 0);

    //_trajs.get("foot_pos_z").addPoint(
    //    halfPeriod, 0.0);

    //Flying foot orientation
    _trajs.get("foot_axis_x").addPoint(
                0.0, 0.0);
    _trajs.get("foot_axis_x").addPoint(
                halfPeriod, 0.0);

    _trajs.get("foot_axis_y").addPoint(
                0.0, 0.0);
    _trajs.get("foot_axis_y").addPoint(
                halfPeriod, 0.0);

    _trajs.get("foot_axis_z").addPoint(
                0.0, _footstep.getLast().z());
    _trajs.get("foot_axis_z").addPoint(
                singleSupportLength, _footstep.getNext().z());
    _trajs.get("foot_axis_z").addPoint(
                halfPeriod, _footstep.getNext().z());

    //The trunk trajectory is defined for a
    //complete cycle to handle trunk phase shift
    //Trunk phase shift.
    double timeShift = -walking_param_.trunkPhase*halfPeriod;

    //Half pause length of trunk swing
    //lateral oscillation
    double pauseLength = 0.5*walking_param_.trunkPause*halfPeriod;

    //Trunk support foot and next
    //support foot external
    //oscillating position
    Eigen::Vector2d trunkPointSupport(
                walking_param_.trunkXOffset
                + walking_param_.trunkXOffsetPCoefForward*_footstep.getNext().x()
                + walking_param_.trunkXOffsetPCoefTurn*fabs(_footstep.getNext().z()),
                walking_param_.trunkYOffset);
    Eigen::Vector2d trunkPointNext(
                _footstep.getNext().x() + walking_param_.trunkXOffset
                + walking_param_.trunkXOffsetPCoefForward*_footstep.getNext().x()
                + walking_param_.trunkXOffsetPCoefTurn*fabs(_footstep.getNext().z()),
                _footstep.getNext().y() + walking_param_.trunkYOffset);
    //Trunk middle neutral (no swing) position
    Eigen::Vector2d trunkPointMiddle =
            0.5*trunkPointSupport + 0.5*trunkPointNext;
    //Trunk vector from middle to support apex
    Eigen::Vector2d trunkVect =
            trunkPointSupport-trunkPointMiddle;
    //Apply swing amplitude ratio
    trunkVect.y() *= walking_param_.trunkSwing;
    //Trunk support and next apex position
    Eigen::Vector2d trunkApexSupport =
            trunkPointMiddle + trunkVect;
    Eigen::Vector2d trunkApexNext =
            trunkPointMiddle - trunkVect;
    //Trunk forward velocity
    double trunkVelSupport =
            (_footstep.getNext().x()-_footstep.getLast().x())/period;
    double trunkVelNext =
            _footstep.getNext().x()/halfPeriod;

    //Trunk position
    _trajs.get("trunk_pos_x").addPoint(
                0.0,
                _trunkPosAtLast.x(),
                _trunkVelAtLast.x(),
                _trunkAccAtLast.x());
    _trajs.get("trunk_pos_x").addPoint(
                halfPeriod+timeShift,
                trunkApexSupport.x(),
                trunkVelSupport);
    _trajs.get("trunk_pos_x").addPoint(
                period+timeShift,
                trunkApexNext.x(),
                trunkVelNext);

    _trajs.get("trunk_pos_y").addPoint(
                0.0,
                _trunkPosAtLast.y(),
                _trunkVelAtLast.y(),
                _trunkAccAtLast.y());
    if(walking_param_.trunkYOnlyInDoubleSupport){
        _trajs.get("trunk_pos_y").addPoint(
                    singleSupportLength+timeShift,
                    trunkApexSupport.y());
        _trajs.get("trunk_pos_y").addPoint(
                    halfPeriod+timeShift,
                    trunkApexSupport.y());
        _trajs.get("trunk_pos_y").addPoint(
                    halfPeriod +timeShift + singleSupportLength,
                    trunkApexNext.y());
        _trajs.get("trunk_pos_y").addPoint(
                    period+timeShift,
                    trunkApexNext.y());
    }else{
        _trajs.get("trunk_pos_y").addPoint(
                    halfPeriod+timeShift-pauseLength,
                    trunkApexSupport.y());
        _trajs.get("trunk_pos_y").addPoint(
                    halfPeriod+timeShift+pauseLength,
                    trunkApexSupport.y());
        _trajs.get("trunk_pos_y").addPoint(
                    period+timeShift-pauseLength,
                    trunkApexNext.y());
        _trajs.get("trunk_pos_y").addPoint(
                    period+timeShift+pauseLength,
                    trunkApexNext.y());
    }

    _trajs.get("trunk_pos_z").addPoint(
                0.0,
                _trunkPosAtLast.z(),
                _trunkVelAtLast.z(),
                _trunkAccAtLast.z());
    _trajs.get("trunk_pos_z").addPoint(
                halfPeriod+timeShift,
                walking_param_.trunkHeight);
    _trajs.get("trunk_pos_z").addPoint(
                period+timeShift,
                walking_param_.trunkHeight);

    //Define trunk yaw target
    //orientation position and velocity
    //in euler angle and convertion
    //to axis vector
    Eigen::Vector3d eulerAtSuport(
                0.0,
                walking_param_.trunkPitch
                + walking_param_.trunkPitchPCoefForward*_footstep.getNext().x()
                + walking_param_.trunkPitchPCoefTurn*fabs(_footstep.getNext().z()),
                0.5*_footstep.getLast().z()+0.5*_footstep.getNext().z());
    Eigen::Vector3d eulerAtNext(
                0.0,
                walking_param_.trunkPitch
                + walking_param_.trunkPitchPCoefForward*_footstep.getNext().x()
                + walking_param_.trunkPitchPCoefTurn*fabs(_footstep.getNext().z()),
                _footstep.getNext().z());
    Eigen::Matrix3d matAtSupport = EulerIntrinsicToMatrix(eulerAtSuport);
    Eigen::Matrix3d matAtNext = EulerIntrinsicToMatrix(eulerAtNext);
    Eigen::Vector3d axisAtSupport = MatrixToAxis(matAtSupport);
    Eigen::Vector3d axisAtNext = MatrixToAxis(matAtNext);
    Eigen::Vector3d axisVel(
                0.0, 0.0,
                AngleDistance(
                    _footstep.getLast().z(),
                    _footstep.getNext().z())/period);

    //Trunk orientation
    _trajs.get("trunk_axis_x").addPoint(
                0.0,
                _trunkAxisPosAtLast.x(),
                _trunkAxisVelAtLast.x(),
                _trunkAxisAccAtLast.x());
    _trajs.get("trunk_axis_x").addPoint(
                halfPeriod+timeShift,
                axisAtSupport.x(),
                axisVel.x());
    _trajs.get("trunk_axis_x").addPoint(
                period+timeShift,
                axisAtNext.x(),
                axisVel.x());
    _trajs.get("trunk_axis_y").addPoint(
                0.0,
                _trunkAxisPosAtLast.y(),
                _trunkAxisVelAtLast.y(),
                _trunkAxisAccAtLast.y());
    _trajs.get("trunk_axis_y").addPoint(
                halfPeriod+timeShift,
                axisAtSupport.y(),
                axisVel.y());
    _trajs.get("trunk_axis_y").addPoint(
                period+timeShift,
                axisAtNext.y(),
                axisVel.y());
    _trajs.get("trunk_axis_z").addPoint(
                0.0,
                _trunkAxisPosAtLast.z(),
                _trunkAxisVelAtLast.z(),
                _trunkAxisAccAtLast.z());
    _trajs.get("trunk_axis_z").addPoint(
                halfPeriod+timeShift,
                axisAtSupport.z(),
                axisVel.z());
    _trajs.get("trunk_axis_z").addPoint(
                period+timeShift,
                axisAtNext.z(),
                axisVel.z());
}

void QuinticWalk::resetTrunkLastState()
{
    if (_footstep.isLeftSupport()) {
        _trunkPosAtLast <<
                           walking_param_.trunkXOffset,
                -walking_param_.footDistance/2.0 + walking_param_.trunkYOffset,
                walking_param_.trunkHeight;
    } else {
        _trunkPosAtLast <<
                           walking_param_.trunkXOffset,
                walking_param_.footDistance/2.0 + walking_param_.trunkYOffset,
                walking_param_.trunkHeight;
    }
    _trunkVelAtLast.setZero();
    _trunkAccAtLast.setZero();
    _trunkAxisPosAtLast << 0.0, walking_param_.trunkPitch, 0.0;
    _trunkAxisVelAtLast.setZero();
    _trunkAxisAccAtLast.setZero();
}
}
