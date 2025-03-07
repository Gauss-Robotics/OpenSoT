#include <OpenSoT/constraints/velocity/AccelerationLimits.h>

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

    this->setAccelerationLimits(qDDotLimit);

    this->update();
}

void AccelerationLimits::update()
{
    _v = _robot.getJointVelocity();

    /************************ COMPUTING BOUNDS ****************************/
    assert(_qDDotLimit.size() == _x_size);
    for(unsigned int i = 0; i < _qDDotLimit.size(); ++i)
    {
        // _lowerBound[i] = (-1.0*std::fabs(_qDDotLimit[i])*_dT*_dT + _v[i]*_dT)*_boundScaling; // assume optimized velocity is in rad/s
        _lowerBound[i] = (-1.0*std::fabs(_qDDotLimit[i])*_dT + _v[i])*_boundScaling; // assume optimized velocity is in rad/timestep
        // _upperBound[i] = (1.0*std::fabs(_qDDotLimit[i])*_dT*_dT + _v[i]*_dT)*_boundScaling;
        _upperBound[i] = (1.0*std::fabs(_qDDotLimit[i])*_dT + _v[i])*_boundScaling;

    }
    // avoid infeasibility
    _upperBound = _upperBound.cwiseMax(Eigen::VectorXd::Zero(_x_size));
    _lowerBound = _lowerBound.cwiseMin(Eigen::VectorXd::Zero(_x_size));
    /**********************************************************************/

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

