#include <OpenSoT/constraints/velocity/Acceleration_limits.h>

using namespace OpenSoT::constraints::velocity;

AccelerationLimits::AccelerationLimits(const XBot::ModelInterface& robot,
                                        const Eigen::VectorXd& qDDotLimit,
                                        const double dT,
                                        const double boundScaling) :
    Constraint("acceleration_limits", robot.getNv()),
    _robot(robot),
    _qDDotLimit(qDDotLimit),
    _dT(dT),
    _boundScaling(boundScaling) {

    if(qDDotLimit.size() != _x_size)
        throw std::runtime_error("qDDotLimit.size() != _x_size()");

    _lowerBound.setZero(_x_size);
    _upperBound.setZero(_x_size);

    this->setAccelerationLimits(qDDotLimit);

    this->update();
}

void JointLimits::update()
{
    _v = _robot.getJointVelocity();

/************************ COMPUTING BOUNDS ****************************/
    assert(_qDDotLimit.size() == _x_size);
    for(unsigned int i = 0; i < _qDDotLimit.size(); ++i)
    {
        // _lowerBound[i] = (-1.0*std::fabs(_qDDotLimit[i])*_dT*_dT + _v*_dT)*_boundScaling; // assume optimized velocity is in m/timestep
        _lowerBound[i] = (-1.0*std::fabs(_qDDotLimit[i])*_dT + _v*)*_boundScaling; // assume optimized velocity is in m/s
        // _upperBound[i] = (1.0*std::fabs(_qDDotLimit[i])*_dT*_dT + _v*_dT)*_boundScaling;
        _upperBound[i] = (1.0*std::fabs(_qDDotLimit[i])*_dT + _v)*_boundScaling;

        // avoid infeasibility
        _upperBound[i] = _upperBound[i].cwiseMax(0.0);
        _lowerBound[i] = _lowerBound[i].cwiseMin(0.0);
    }
/**********************************************************************/

}

void JointLimits::setBoundScaling(const double boundScaling)
{
    _boundScaling = boundScaling;
}

double OpenSoT::constraints::velocity::AccelerationLimits::getDT()
{
    return _dT;
}

Eigen::VectorXd OpenSoT::constraints::velocity::AccelerationLimits::getAccelerationLimits()
{
    return _upperBound/_dT;
}

void OpenSoT::constraints::velocity::AccelerationLimits::setAccelerationLimits(const Eigen::VectorXd& qDotLimit)
{
    this->generateBounds(qDotLimit);
}

void JointLimits::setBoundScaling(const double boundScaling)
{
    _boundScaling = boundScaling;
}

