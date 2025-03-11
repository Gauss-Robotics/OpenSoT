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

    // // Create the CSV file
    // const std::string dir = "/root/ros2_ws/src/out_OpenSoT/logs";
    // _outFile.open(dir + "/acceleration_limits.csv");
    // if (!_outFile.is_open()) {
    //     throw std::runtime_error("Failed to create file: " + dir + "/acceleration_limits.csv");
    // }

    // // Write the header
    // _outFile << "v_0";
    // for (unsigned int i = 1; i < _x_size; ++i) {
    //     _outFile << ",v_" << i;
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

// AccelerationLimits::~AccelerationLimits() {
//     if (_outFile.is_open()) {
//         _outFile.close();
//     }
// }

void AccelerationLimits::update()
{
    _v = _robot.getJointVelocity();
    /************************ COMPUTING BOUNDS ****************************/
    assert(_qDDotLimit.size() == _x_size);
    for(unsigned int i = 0; i < _qDDotLimit.size(); ++i)
    {
        _lowerBound[i] = (-1.0*std::fabs(_qDDotLimit[i])*_dT + _v[i]); // assume optimized velocity is in rad/second
        _upperBound[i] = (1.0*std::fabs(_qDDotLimit[i])*_dT + _v[i]);

    }
    /**********************************************************************/
    // for(unsigned int i = 0; i < _qDDotLimit.size(); ++i)
    // {
    //     _outFile << _v[i];
    //     if (i < _qDDotLimit.size() - 1) {
    //         _outFile << ",";
    //     }
    // }
    // for(unsigned int i = 0; i < _qDDotLimit.size(); ++i)
    // {
    //     _outFile << "," << _lowerBound[i];
    // }
    // for(unsigned int i = 0; i < _qDDotLimit.size(); ++i)
    // {
    //     _outFile << "," << _upperBound[i];
    // }
    // _outFile << "\n";
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

