/*
 * Copyright (C) 2014 Walkman
 * Author: Cheng Fang
 * email:  cheng.fang@iit.it
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

#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>

// local version of vectorKDLToEigen since oldest versions are bogous.
// To use instead of:
// #include <eigen_conversions/eigen_kdl.h>
// tf::vectorKDLToEigen
void vectorKDLToEigen(const KDL::Vector &k, Eigen::Matrix<double, 3, 1> &e)
{
  for(int i = 0; i < 3; ++i)
    e[i] = k[i];
}

using namespace OpenSoT::constraints::velocity;

using namespace Eigen;

const double SMALL_NUM = pow(10.0, -5);

SelfCollisionAvoidance::SelfCollisionAvoidance(const Eigen::VectorXd& x,
                                               iDynUtils &robot,
                                               double detection_threshold,
                                               double linkPair_threshold,
                                               const double boundScaling):
    Constraint("self_collision_avoidance", x.size()),
    _detection_threshold(detection_threshold),
    _linkPair_threshold(linkPair_threshold),
    computeLinksDistance(robot),
    robot_col(robot),
    _x_cache(x),
    _boundScaling(boundScaling) {

    std::string base_name = robot.getBaseLink();
    base_index = robot_col.iDyn3_model.getLinkIndex(base_name);

    if(base_index == -1)
        std::cout << "Failed to get base_index" << std::endl;

    update(x);

}

double SelfCollisionAvoidance::getLinkPairThreshold()
{
    return _linkPair_threshold;
}

double SelfCollisionAvoidance::getDetectionThreshold()
{
    return _detection_threshold;
}


void SelfCollisionAvoidance::setLinkPairThreshold(const double linkPair_threshold)
{
    _linkPair_threshold = std::fabs(linkPair_threshold);
    //this->update();
}

void SelfCollisionAvoidance::setDetectionThreshold(const double detection_threshold)
{
    _detection_threshold = std::fabs(detection_threshold);
    //this->update();
}


void SelfCollisionAvoidance::update(const Eigen::VectorXd &x)
{
    // we update _Aineq and _bupperBound only if x has changed
    if(!(x == _x_cache)) {
        _x_cache = x;
        calculate_Aineq_bUpperB (_Aineq, _bUpperBound );
    }
//    std::cout << "_Aineq" << _Aineq.toString() << std::endl << std::endl;
    //    std::cout << "_bUpperBound" << _bUpperBound.toString() << std::endl << std::endl;
}

bool OpenSoT::constraints::velocity::SelfCollisionAvoidance::setCollisionWhiteList(std::list<LinkPairDistance::LinksPair> whiteList)
{
    bool ok = computeLinksDistance.setCollisionWhiteList(whiteList);
    this->calculate_Aineq_bUpperB(_Aineq, _bUpperBound);
    return ok;
}

bool OpenSoT::constraints::velocity::SelfCollisionAvoidance::setCollisionBlackList(std::list<LinkPairDistance::LinksPair> blackList)
{
    bool ok = computeLinksDistance.setCollisionBlackList(blackList);
    this->calculate_Aineq_bUpperB(_Aineq, _bUpperBound);
    return ok;
}

Eigen::MatrixXd SelfCollisionAvoidance::skewSymmetricOperator (const Eigen::Vector3d & r_cp)
{
    MatrixXd J_transform(3,6);
    Matrix3d left_part, right_part;
    left_part = MatrixXd::Identity(3,3);
    right_part << 0, r_cp(2), -r_cp(1),
                   -r_cp(2), 0, r_cp(0),
                   r_cp(1), -r_cp(0), 0;
    J_transform.block(0,0,3,3) = left_part;
    J_transform.block(0,3,3,3) = right_part;

    return J_transform;
}



void SelfCollisionAvoidance::calculate_Aineq_bUpperB (Eigen::MatrixXd & Aineq_fc,
                                                      Eigen::VectorXd & bUpperB_fc )
{

//    robot_col.updateiDyn3Model(x, false);

    std::list<LinkPairDistance> interested_LinkPairs;
    std::list<LinkPairDistance>::iterator j;
    interested_LinkPairs = computeLinksDistance.getLinkDistances(_detection_threshold);

    /*//////////////////////////////////////////////////////////*/

    MatrixXd Aineq_fc_Eigen(interested_LinkPairs.size(), robot_col.iDyn3_model.getNrOfDOFs());
    VectorXd bUpperB_fc_Eigen(interested_LinkPairs.size());

    double Dm_LinkPair;
    KDL::Frame Link1_T_CP,Link2_T_CP;
    std::string Link1_name, Link2_name;

    int Link1_index, Link2_index;

    KDL::Frame Waist_T_Link1, Waist_T_Link2, Waist_T_Link1_CP, Waist_T_Link2_CP;
    KDL::Vector Link1_origin_kdl, Link2_origin_kdl, Link1_CP_kdl, Link2_CP_kdl;
    Eigen::Matrix<double, 3, 1> Link1_origin, Link2_origin, Link1_CP, Link2_CP;

    Vector3d closepoint_dir;

    MatrixXd Link1_CP_Jaco, Link2_CP_Jaco;

    MatrixXd Waist_frame_world_Eigen = robot_col.getPosition(base_index, true);
    Matrix3d Waist_frame_world_Eigen_Ro = Waist_frame_world_Eigen.block(0,0,3,3);
    MatrixXd temp_trans_matrix(6,6);
    temp_trans_matrix.block(0,0,3,3) = Waist_frame_world_Eigen_Ro;
    temp_trans_matrix.block(3,3,3,3) = Waist_frame_world_Eigen_Ro;
    temp_trans_matrix.block(0,3,3,3) = MatrixXd::Zero(3,3);
    temp_trans_matrix.block(3,0,3,3) = MatrixXd::Zero(3,3);

    int linkPairIndex = 0;
    for (j = interested_LinkPairs.begin(); j != interested_LinkPairs.end(); ++j)
    {

        LinkPairDistance& linkPair(*j);

        Dm_LinkPair = linkPair.getDistance();
        Link1_T_CP = linkPair.getLink_T_closestPoint().first;
        Link2_T_CP = linkPair.getLink_T_closestPoint().second;
        Link1_name = linkPair.getLinkNames().first;
        Link2_name = linkPair.getLinkNames().second;

        Link1_index = robot_col.iDyn3_model.getLinkIndex(Link1_name);
        if(Link1_index == -1)
            std::cout << "Failed to get " << Link1_name << std::endl;
        Link2_index = robot_col.iDyn3_model.getLinkIndex(Link2_name);
        if(Link2_index == -1)
            std::cout << "Failed to get " << Link2_name << std::endl;

        Waist_T_Link1 = robot_col.iDyn3_model.getPositionKDL( base_index , Link1_index );
        Waist_T_Link2 = robot_col.iDyn3_model.getPositionKDL( base_index , Link2_index );
        Waist_T_Link1_CP = Waist_T_Link1 * Link1_T_CP;
        Waist_T_Link2_CP = Waist_T_Link2 * Link2_T_CP;
        Link1_origin_kdl = Waist_T_Link1.p;
        Link2_origin_kdl = Waist_T_Link2.p;
        Link1_CP_kdl = Waist_T_Link1_CP.p;
        Link2_CP_kdl = Waist_T_Link2_CP.p;

        vectorKDLToEigen(Link1_origin_kdl, Link1_origin);
        vectorKDLToEigen(Link2_origin_kdl, Link2_origin);
        vectorKDLToEigen(Link1_CP_kdl, Link1_CP);
        vectorKDLToEigen(Link2_CP_kdl, Link2_CP);


        closepoint_dir = Link2_CP - Link1_CP;
        closepoint_dir = closepoint_dir / Dm_LinkPair;


        robot_col.getRelativeJacobian( Link1_index, base_index, Link1_CP_Jaco, true);
        Link1_CP_Jaco = temp_trans_matrix * Link1_CP_Jaco;
        Link1_CP_Jaco = skewSymmetricOperator(Link1_CP - Link1_origin) * Link1_CP_Jaco;


        robot_col.getRelativeJacobian( Link2_index, base_index, Link2_CP_Jaco, true);
        Link2_CP_Jaco = temp_trans_matrix * Link2_CP_Jaco;
        Link2_CP_Jaco = skewSymmetricOperator(Link2_CP - Link2_origin) * Link2_CP_Jaco;


        Aineq_fc_Eigen.row(linkPairIndex) = closepoint_dir.transpose() * ( Link1_CP_Jaco - Link2_CP_Jaco );
        bUpperB_fc_Eigen(linkPairIndex) = (Dm_LinkPair - _linkPair_threshold) * _boundScaling;

        ++linkPairIndex;

    }

    Aineq_fc = Aineq_fc_Eigen;
    bUpperB_fc = bUpperB_fc_Eigen;

}

void SelfCollisionAvoidance::setBoundScaling(const double boundScaling)
{
    _boundScaling = boundScaling;
}

