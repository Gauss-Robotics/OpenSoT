/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __OPENSOT_ACCELERATION_TASK_CARTESIAN_H__
#define __OPENSOT_ACCELERATION_TASK_CARTESIAN_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/common/utils.h>
#include <OpenSoT/tasks/acceleration/GainType.h>
#include <kdl/frames.hpp>

namespace OpenSoT { namespace tasks { namespace acceleration {
    
    class Cartesian : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
        
    public:
        
        typedef std::shared_ptr<Cartesian> Ptr;

        Cartesian(const std::string task_id,
                  const XBot::ModelInterface& robot,
                  const std::string& distal_link,
                  const std::string& base_link
                 );
        
        Cartesian(const std::string task_id,
                  const XBot::ModelInterface& robot, 
                  const std::string& distal_link,
                  const std::string& base_link,
                  const AffineHelper& qddot
                 );

        void setGainType(GainType type);
        GainType getGainType() const;
        
        const std::string& getBaseLink() const;
        const std::string& getDistalLink() const;
        
        /**
         * @brief setReference sets a new reference for the Cartesian task.
         * The task error IS NOT recomputed immediately, you need to call the _update(x) function
         * Notice how the setReference(...) needs to be called before each _update(x) of the Cartesian task,
         * since THE _update() RESETS THE FEED-FORWARD VELOCITY and ACCELERATION TERMS for safety reasons.
         * @param pose_ref the \f$R^{4x4}\f$ homogeneous transform matrix describing the desired pose
         * for the distal_link in the base_link frame of reference.
         * @param vel_ref is a \f$R^{6}\f$ twist describing the desired trajectory velocity
         * @param acc_ref is a \f$R^{6}\f$ twist describing the desired trajectory acceleration, and it represents
         * a feed-forward term in the cartesian task computation.
         */
        [[deprecated]]
        void setPositionReference(const Eigen::Vector3d& pos_ref);

        void setReference(const Eigen::Affine3d& ref);
        void setReference(const KDL::Frame& ref);

        void setReference(const Eigen::Affine3d& pose_ref,
                          const Eigen::Vector6d& vel_ref);
        void setReference(const KDL::Frame& pose_ref,
                          const KDL::Twist& vel_ref);

        void setReference(const Eigen::Affine3d& pose_ref,
                          const Eigen::Vector6d& vel_ref,
                          const Eigen::Vector6d& acc_ref);
        void setReference(const KDL::Frame& pose_ref,
                          const KDL::Twist& vel_ref,
                          const KDL::Twist& acc_ref);

        /**
         * @brief setVirtualForce this version permits to set a virtual force which is transformed into an acceleration
         * using:
         *
         *      xddot = (JB^1J')F
         *
         * where F is the virtual force, J is the task Jacobian and B is the inertia matrix.
         * NOTICE:
         * if the SubTask is used, due to the coupling given by the inertia matrix, the provided reference F
         * have to contain 0 in the non controlled directions.
         * For example:
         *
         *  T%{0,2,4} : F = [fx, 0, fz, 0, fp, 0]'
         *
         * @param virtual_force_ref
         */
        void setVirtualForce(const Eigen::Vector6d& virtual_force_ref);


        void getReference(Eigen::Affine3d& ref) const;
        void getReference(KDL::Frame& ref) const;

        void getReference(Eigen::Affine3d& desiredPose,
                          Eigen::Vector6d& desiredTwist) const;
        void getReference(KDL::Frame& desiredPose,
                          KDL::Twist& desiredTwist) const;

        void getReference(Eigen::Affine3d& desiredPose,
                          Eigen::Vector6d& desiredTwist,
                          Eigen::Vector6d& desiredAcceleration) const;
        void getReference(KDL::Frame& desiredPose,
                          KDL::Twist& desiredTwist,
                          KDL::Twist& desiredAcceleration) const;

        /**
         * @brief getCachedVelocityReference can be used to get Velocity reference after update(), it will reset
         * next update()
         * @return internal velcity reference
         */
        const Eigen::Vector6d& getCachedVelocityReference() const;

        /**
         * @brief getCachedAccelerationReference can be used to get Velocity reference after update(), it will reset
         * next update()
         * @return internal acceleration reference
         */
        const Eigen::Vector6d& getCachedAccelerationReference() const;

        /**
         * @brief getCachedVirtualForceReference can be used to get virtual force reference after update(), it will reset
         * next update()
         * @return internal virtual force reference
         */
        const Eigen::Vector6d& getCachedVirtualForceReference() const;

        const Eigen::Affine3d& getActualPose() const;
        void getActualPose(Eigen::Affine3d& actual) const;
        void getActualPose(KDL::Frame& actual);

        const Eigen::Vector6d& getActualTwist() const;
        void getActualTwist(Eigen::Vector6d& actual) const;
        void getActualTwist(KDL::Twist& actual);
        
        bool reset();

        void setLambda(double lambda1, double lambda2);
        virtual void setLambda(double lambda);

        /**
         * @brief getLambda return both position and velocity convergence gains
         * @param lambda position gain
         * @param lambda2 velocity gain
         */
        void getLambda(double & lambda, double & lambda2);
        using Task::getLambda;

        /**
         * @brief getLambda2 return velocity convergence gain
         * @return lambda2 gain
         */
        const double getLambda2() const;

        void setOrientationGain(double orientation_gain);
        const double getOrientationErrorGain() const;

        const bool baseLinkIsWorld() const;

        /**
         * @brief Changes the distal link of the task. It also resets the reference according to the current robot pose.
         * @param distal_link the new distal link
         * @return false if the distal link does not exists
         */
        bool setDistalLink(const std::string& distal_link);

        /**
         * @brief setBaseLink change the base link of the task
         * @param base_link the new base link
         * @return false if the base link does not exists
         */
        bool setBaseLink(const std::string& base_link);

        /**
         * @brief setKp set position gain
         * @param Kp a SPD matrix
         */
        void setKp(const Eigen::Matrix6d& Kp);

        /**
         * @brief setKd set velocity gain
         * @param Kd a SPD matrix
         */
        void setKd(const Eigen::Matrix6d& Kd);

        /**
         * @brief setGains set both position and velocity gains
         * @param Kp a SPD matrix
         * @param Kd a SPD matrix
         */
        void setGains(const Eigen::Matrix6d& Kp, const Eigen::Matrix6d& Kd);

        /**
         * @brief getKp
         * @return position gain
         */
        const Eigen::Matrix6d& getKp() const;

        /**
         * @brief getKd
         * @return  velocity gain
         */
        const Eigen::Matrix6d& getKd() const;

        /**
         * @brief getGains return both position and velocity gains
         * @param Kp
         * @param Kd
         */
        void getGains(Eigen::Matrix6d& Kp, Eigen::Matrix6d& Kd);

        static bool isCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

        static OpenSoT::tasks::acceleration::Cartesian::Ptr asCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);
        
        /**
         * @brief getError returns the 6d cartesian error (position and orientation) between actual and reference pose
         * @return a \f$R^{6}\f$ vector describing cartesian error between actual and reference pose
         */
        const Eigen::Vector6d& getError() const;
        const Eigen::Vector6d& getVelocityError() const;

    private:

        GainType _gain_type;

        Eigen::Vector6d _velocity_error;


        static const std::string world_name;
        
        std::string _base_link, _distal_link;
        const XBot::ModelInterface& _robot;
        AffineHelper _qddot;
        AffineHelper _cartesian_task;
        
        Eigen::MatrixXd _J;
        Eigen::Vector6d _jdotqdot;
        
        Eigen::Affine3d _pose_ref, _pose_current;
        Eigen::Vector6d _pose_error, _vel_ref, _vel_current, _acc_ref, _vel_ref_cached, _acc_ref_cached;
        
        Eigen::Vector3d _orientation_error;

        Eigen::Matrix6d _Kp, _Kd;
        
        double _orientation_gain;

        double _lambda2;

        /**
         * @brief _base_T_distal is used to change new distal link!
         */
        Eigen::Affine3d _base_T_distal;

        Eigen::Affine3d _tmpMatrix, _tmpMatrix2;

        void compute_cartesian_inertia_inverse();
        void resetReference();


        //
        Eigen::Vector6d _virtual_force_ref, _virtual_force_ref_cached;
        /**
         * @brief _Mi inverse of Cartesian Inertia matrix
         */
        Eigen::Matrix6d _Mi;

        Eigen::MatrixXd _tmpMatrixXd;
        Eigen::MatrixXd _Bi;

        virtual void _update();
        virtual void _log(XBot::MatLogger2::Ptr logger);

    };
    
} } }






#endif
