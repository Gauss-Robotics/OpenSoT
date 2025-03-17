#ifndef __BOUNDS_VELOCITY_JERKLIMITS_H__
#define __BOUNDS_VELOCITY_JERKLIMITS_H__

#include <Eigen/Dense>
#include <OpenSoT/Constraint.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT
{
namespace constraints
{
namespace velocity
{
/**
 * @brief The JerkLimits class implements a bound on joint jerks
 */
class JerkLimits : public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
{
  public:
    typedef std::shared_ptr<JerkLimits> Ptr;

  private:
    const XBot::ModelInterface &_robot;
    double _boundScaling;
    Eigen::VectorXd _qDDDotLimit;
    double _dT;
    Eigen::VectorXd _v; // joint velocities
    Eigen::VectorXd _a; // joint accelerations

  public:
    /**
     * @brief JerkLimits constructor
     * @param qDDotLimit the joint jerk limit. It is always a positive number [rad/s^2]
     * @param dT the time constant at which we are performing velocity control [s]
     * @param x_size the size of the unknowns that we want to bound (it CANNOT be a subset)
     */
    JerkLimits(const XBot::ModelInterface &robot, const Eigen::VectorXd &qDDotLimit, const double dT,
               const double boundScaling = 1.0);

    /**
     * @brief getJerkLimits returns the current jerk limits.
     * @return the joint jerk limits. It is always a positive double [rad/s^2]
     */
    Eigen::VectorXd getJerkLimits();

    /**
     * @brief setJerkLimits
     * @param qDDotLimit the joint jerk limits. It needs be a positive number [rad/s^2]
     */
    void setJerkLimits(const Eigen::VectorXd &qDDotLimit);

    /**
     * @brief getDT returns the (constant) sample time we assume on the system.
     * @return the system sample time in [s]
     */
    double getDT();
    void setBoundScaling(const double boundScaling);
    void update();
};
} // namespace velocity
} // namespace constraints
} // namespace OpenSoT

#endif
