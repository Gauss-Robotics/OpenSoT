#include <OpenSoT/constraints/velocity/AccelerationLimits.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <sstream>

using namespace OpenSoT::constraints::velocity;

AccelerationLimits::AccelerationLimits(const XBot::ModelInterface& robot,
                                        const Eigen::VectorXd& qDDotLimit,
                                        const double dT,
                                        const double boundScaling):
    Constraint("acceleration_limits", robot.getNv()),
    _robot(robot),
    _qDDotLimit(qDDotLimit),
    _dT(dT),
    _boundScaling(boundScaling) {

    if(qDDotLimit.size() != _x_size)
        throw std::runtime_error("qDDotLimit.size() != _x_size()");

    _lowerBound.setZero(_x_size);
    _upperBound.setZero(_x_size);

    this->update();
}


void AccelerationLimits::update()
{
    // the joint velocities are obtained from the robot model
    // these could be either the measured values, or commanded values
    _v = _robot.getJointVelocity();

    assert(_qDDotLimit.size() == _x_size);
    for(unsigned int i = 0; i < _qDDotLimit.size(); ++i)
    {
        // Note: assume optimized joint velocity is in rad/second
        // this interpertation is different from the rest of OpenSoT which assumes rad/timestep
        _lowerBound[i] = (-1.0*std::fabs(_qDDotLimit[i])*_dT + _v[i]);
        _upperBound[i] = (1.0*std::fabs(_qDDotLimit[i])*_dT + _v[i]);

    }
}

double AccelerationLimits::getDT()
{
    return _dT;
}

Eigen::VectorXd AccelerationLimits::getAccelerationLimits()
{
    return _upperBound/_dT;
}

void AccelerationLimits::setAccelerationLimits(const Eigen::VectorXd& qDotLimit)
{
    this->_qDDotLimit = qDotLimit;
    this->update();
}

void AccelerationLimits::setBoundScaling(const double boundScaling)
{
    this->_boundScaling = boundScaling;
}

