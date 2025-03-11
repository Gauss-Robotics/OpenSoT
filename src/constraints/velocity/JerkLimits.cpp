#include <OpenSoT/constraints/velocity/JerkLimits.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <sstream>

using namespace OpenSoT::constraints::velocity;

JerkLimits::JerkLimits(const XBot::ModelInterface& robot,
                                        const Eigen::VectorXd& qDDDotLimit,
                                        const double dT,
                                        const double boundScaling):
    Constraint("jerk_limits", robot.getNv()),
    _robot(robot),
    _qDDDotLimit(qDDDotLimit),
    _dT(dT),
    _boundScaling(boundScaling) {

    if(qDDDotLimit.size() != _x_size)
        throw std::runtime_error("qDDDotLimit.size() != _x_size()");

    _lowerBound.setZero(_x_size);
    _upperBound.setZero(_x_size);

    // // Create the CSV file
    // const std::string dir = "/root/ros2_ws/src/out_OpenSoT/logs";
    // _outFile.open(dir + "/jerk_limits.csv");
    // if (!_outFile.is_open()) {
    //     throw std::runtime_error("Failed to create file: " + dir + "/jerk_limits.csv");
    // }

    // // Write the header
    // _outFile << "v_0";
    // for (unsigned int i = 1; i < _x_size; ++i) {
    //     _outFile << ",v_" << i;
    // }
    // for (unsigned int i = 0; i < _x_size; ++i) {
    //     _outFile << ",a_" << i;
    // }
    // for (unsigned int i = 0; i < _x_size; ++i) {
    //     _outFile << ",lowerBound_" << i;
    // }
    // for (unsigned int i = 0; i < _x_size; ++i) {
    //     _outFile << ",upperBound_" << i;
    // }
    // _outFile << "\n";

    this->update();
}

// JerkLimits::~JerkLimits() {
//     if (_outFile.is_open()) {
//         _outFile.close();
//     }
// }

void JerkLimits::update()
{
    _v = _robot.getJointVelocity();
    _a = _robot.getJointAcceleration();
    /************************ COMPUTING BOUNDS ****************************/
    assert(_qDDDotLimit.size() == _x_size);
    for(unsigned int i = 0; i < _qDDDotLimit.size(); ++i)
    {
        _lowerBound[i] = (-1.0*std::fabs(_qDDDotLimit[i])*_dT*_dT + _a[i]*_dT + _v[i]); // assume optimized velocity is in rad/second
        _upperBound[i] = (1.0*std::fabs(_qDDDotLimit[i])*_dT*_dT + _a[i]*_dT + _v[i]);

    }
    /**********************************************************************/
    // for(unsigned int i = 0; i < _qDDDotLimit.size(); ++i)
    // {
    //     _outFile << _v[i];
    //     if (i < _qDDDotLimit.size() - 1) {
    //         _outFile << ",";
    //     }
    // }
    // for(unsigned int i = 0; i < _qDDDotLimit.size(); ++i)
    // {
    //     _outFile << "," << _lowerBound[i];
    // }
    // for(unsigned int i = 0; i < _qDDDotLimit.size(); ++i)
    // {
    //     _outFile << "," << _a[i];
    // }
    // for(unsigned int i = 0; i < _qDDDotLimit.size(); ++i)
    // {
    //     _outFile << "," << _upperBound[i];
    // }
    // _outFile << "\n";
}

double JerkLimits::getDT()
{
    return _dT;
}

Eigen::VectorXd JerkLimits::getJerkLimits()
{
    return _upperBound/(_dT*_dT);
}

void JerkLimits::setJerkLimits(const Eigen::VectorXd& qDotLimit)
{
    this->_qDDDotLimit = qDotLimit;
    this->update();
}

void JerkLimits::setBoundScaling(const double boundScaling)
{
    this->_boundScaling = boundScaling;
}

