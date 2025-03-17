#include <OpenSoT/constraints/velocity/JerkLimits.h>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

using namespace OpenSoT::constraints::velocity;

JerkLimits::JerkLimits(const XBot::ModelInterface &robot, const Eigen::VectorXd &qDDDotLimit, const double dT,
                       const double boundScaling)
    : Constraint("jerk_limits", robot.getNv()), _robot(robot), _qDDDotLimit(qDDDotLimit), _dT(dT),
      _boundScaling(boundScaling)
{

    if (qDDDotLimit.size() != _x_size)
        throw std::runtime_error("qDDDotLimit.size() != _x_size()");

    _lowerBound.setZero(_x_size);
    _upperBound.setZero(_x_size);

    this->update();
}

void JerkLimits::update()
{
    // the joint velocity and acceleration are obtained from the robot model
    // these could be either the measured values, or commanded values
    _v = _robot.getJointVelocity();
    _a = _robot.getJointAcceleration();
    assert(_qDDDotLimit.size() == _x_size);
    for (unsigned int i = 0; i < _qDDDotLimit.size(); ++i)
    {
        // Note: assume optimized joint velocity is in rad/second
        // this interpertation is different from the rest of OpenSoT which assumes rad/timestep
        _lowerBound[i] = (-1.0 * std::fabs(_qDDDotLimit[i]) * _dT * _dT + _a[i] * _dT + _v[i]);
        _upperBound[i] = (1.0 * std::fabs(_qDDDotLimit[i]) * _dT * _dT + _a[i] * _dT + _v[i]);
    }
}

double JerkLimits::getDT()
{
    return _dT;
}

Eigen::VectorXd JerkLimits::getJerkLimits()
{
    return _upperBound / (_dT * _dT);
}

void JerkLimits::setJerkLimits(const Eigen::VectorXd &qDotLimit)
{
    this->_qDDDotLimit = qDotLimit;
    this->update();
}

void JerkLimits::setBoundScaling(const double boundScaling)
{
    this->_boundScaling = boundScaling;
}
