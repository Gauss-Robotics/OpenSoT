#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cmath>
#include <OpenSoT/tasks/Aggregated.h>
#include <idynutils/cartesian_utils.h>

#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define toRad(X) (X * M_PI/180.0)


namespace{


class testSelfCollisionAvoidanceConstraint : public ::testing::Test{
 protected:

  testSelfCollisionAvoidanceConstraint():
      robot("coman",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
      q(robot.iDyn3_model.getNrOfDOFs(), 0.0),
      sc_constraint(new OpenSoT::constraints::velocity::SelfCollisionAvoidance(q, robot, 0.005))
  {}

  virtual ~testSelfCollisionAvoidanceConstraint() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

  iDynUtils robot;
  yarp::sig::Vector q;
  OpenSoT::constraints::velocity::SelfCollisionAvoidance::Ptr sc_constraint;
};


  TEST_F(testSelfCollisionAvoidanceConstraint, testConversions) {

    yarp::sig::Matrix testMatrix(6,6);
    for(unsigned int i = 0; i < testMatrix.rows(); ++i)
        for(unsigned int j = 0; j < testMatrix.cols(); ++j)
            testMatrix(i,j) = i*j+1;

    Eigen::MatrixXd testEigenMatrix = sc_constraint->from_yarp_to_Eigen_matrix(testMatrix);
    yarp::sig::Matrix resultMatrix = sc_constraint->from_Eigen_to_Yarp_matrix(testEigenMatrix);

    for(unsigned int i = 0; i < testMatrix.rows(); ++i)
        for(unsigned int j = 0; j < testMatrix.cols(); ++j)
            EXPECT_DOUBLE_EQ(testMatrix(i,j), resultMatrix(i,j));

  }

  yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils) {
      yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
      yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
      leg[0] = -25.0 * M_PI/180.0;
      leg[3] =  50.0 * M_PI/180.0;
      leg[5] = -25.0 * M_PI/180.0;
      idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
      idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
      yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
      arm[0] = 20.0 * M_PI/180.0;
      arm[1] = 10.0 * M_PI/180.0;
      arm[3] = -80.0 * M_PI/180.0;
      idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
      arm[1] = -arm[1];
      idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
      return q;
  }

  TEST_F(testSelfCollisionAvoidanceConstraint, testCartesianTask){

    this->robot.iDyn3_model.setFloatingBaseLink(this->robot.left_leg.index);
    this->q = getGoodInitialPosition(this->robot);
    this->robot.updateiDyn3Model(this->q, true);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", this->q, this->robot,"LWrMot3", "Waist"));
    task_left_arm->setOrientationErrorGain(0.1);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist", this->q, this->robot,"RWrMot3", "Waist"));
    task_right_arm->setOrientationErrorGain(0.1);

    yarp::sig::Matrix T_init_l_arm(4,4);
    T_init_l_arm = task_left_arm->getReference();

    yarp::sig::Matrix T_init_r_arm(4,4);
    T_init_r_arm = task_right_arm->getReference();

    yarp::sig::Matrix T_reference_l_arm(4,4);
    T_reference_l_arm = task_left_arm->getReference();
    T_reference_l_arm(1,3) = 0.0;
    task_left_arm->setReference(T_reference_l_arm);

    yarp::sig::Matrix T_reference_r_arm(4,4);
    T_reference_r_arm = task_right_arm->getReference();
    T_reference_r_arm(1,3) = 0.0;
    task_right_arm->setReference(T_reference_r_arm);

    std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
    cartesianTasks.push_back(task_left_arm);
    cartesianTasks.push_back(task_right_arm);
    OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr taskCartesianAggregated = OpenSoT::tasks::Aggregated::TaskPtr(
       new OpenSoT::tasks::Aggregated(cartesianTasks,this->q.size()));

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(new OpenSoT::tasks::velocity::Postural(this->q));


    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    int t = 100;
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
        new OpenSoT::constraints::velocity::JointLimits(this->q,
                                                        this->robot.iDyn3_model.getJointBoundMax(),
                                                        this->robot.iDyn3_model.getJointBoundMin()));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

    OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

    OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr sot = OpenSoT::solvers::QPOases_sot::Ptr(
        new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds));

    yarp::sig::Vector dq(this->q.size(), 0.0);
    for(unsigned int i = 0; i < 50*t; ++i)
    {
        this->robot.updateiDyn3Model(this->q, true);

        taskCartesianAggregated->update(this->q);
        postural_task->update(this->q);
        bounds->update(this->q);

        if(!sot->solve(dq)){
            std::cout<<"error"<<std::endl;
            dq = 0.0;}
        this->q += dq;

    }

    std::cout<<"Initial Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_l_arm);
        std::cout<<std::endl;
    std::cout<<"Reference Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_l_arm);
        std::cout<<std::endl;
    std::cout<<"Actual Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_left_arm->getActualPose());

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Initial Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_r_arm);
        std::cout<<std::endl;
    std::cout<<"Reference Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_r_arm);
        std::cout<<std::endl;
    std::cout<<"Actual Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_right_arm->getActualPose());

    std::cout<<std::endl;

    for(unsigned int i = 0; i < 4; ++i)
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_NEAR(task_left_arm->getActualPose()(i,j), T_reference_l_arm(i,j), 1E-4);

    for(unsigned int i = 0; i < 4; ++i)
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_NEAR(task_right_arm->getActualPose()(i,j), T_reference_r_arm(i,j), 1E-4);

    int link_index_Lhand, link_index_Lwrist, link_index_Rhand, link_index_Rwrist;
    Eigen::Vector3d Lhand, Lwrist, Rhand, Rwrist;
//    Lhand = sc_constraint->Transform_name_to_point("LWrMot3", this->robot.iDyn3_model.getLinkIndex("Waist"), link_index_Lhand);
//    Lwrist = sc_constraint->Transform_name_to_point("LWrMot2", this->robot.iDyn3_model.getLinkIndex("Waist"), link_index_Lwrist);
//    Rhand = sc_constraint->Transform_name_to_point("RWrMot3", this->robot.iDyn3_model.getLinkIndex("Waist"), link_index_Rhand);
//    Rwrist = sc_constraint->Transform_name_to_point("RWrMot2", this->robot.iDyn3_model.getLinkIndex("Waist"), link_index_Rwrist);


    Eigen::Vector3d CP1, CP2;
    double min_distance;
//    min_distance = this->sc_constraint->dist3D_Segment_to_Segment(Lwrist, Lhand, Rwrist, Rhand, CP1, CP2);
    EXPECT_NEAR(0.0, min_distance, 1E-4);

    //Now we add the constraint

    this->q = getGoodInitialPosition(this->robot);
    this->robot.updateiDyn3Model(this->q, true);

    task_left_arm.reset();
    task_left_arm.reset(new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", this->q, this->robot,"LWrMot3", "Waist"));
    task_left_arm->setOrientationErrorGain(0.3);
    task_left_arm->setReference(T_reference_l_arm);

    task_right_arm.reset();
    task_right_arm.reset(new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist", this->q, this->robot,"RWrMot3", "Waist"));
    task_right_arm->setOrientationErrorGain(0.3);
    task_right_arm->setReference(T_reference_r_arm);

    cartesianTasks.clear();
    cartesianTasks.push_back(task_left_arm);
    cartesianTasks.push_back(task_right_arm);

    taskCartesianAggregated.reset();
    taskCartesianAggregated.reset(new OpenSoT::tasks::Aggregated(cartesianTasks,this->q.size()));
    taskCartesianAggregated->getConstraints().push_back(this->sc_constraint);

    postural_task.reset();
    postural_task.reset(new OpenSoT::tasks::velocity::Postural(this->q));
    postural_task->getConstraints().push_back(this->sc_constraint);

    stack_of_tasks.clear();
    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    joint_limits.reset();
    joint_limits.reset(new OpenSoT::constraints::velocity::JointLimits(this->q,
                                                        this->robot.iDyn3_model.getJointBoundMax(),
                                                        this->robot.iDyn3_model.getJointBoundMin()));

    joint_velocity_limits.reset();
    joint_velocity_limits.reset(new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

    bounds.reset();
    bounds.reset(new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

    sot.reset();
    sot.reset(new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds));

    dq.zero();
    for(unsigned int i = 0; i < 100*t; ++i)
    {
        this->robot.updateiDyn3Model(this->q, true);

        taskCartesianAggregated->update(this->q);
        postural_task->update(this->q);
        bounds->update(this->q);

        if(!sot->solve(dq)){
            std::cout<<"error"<<std::endl;
            dq = 0.0;}
        this->q += dq;
    }

    std::cout<<"Initial Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_l_arm);
        std::cout<<std::endl;
    std::cout<<"Reference Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_l_arm);
        std::cout<<std::endl;
    std::cout<<"Actual Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_left_arm->getActualPose());

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Initial Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_r_arm);
        std::cout<<std::endl;
    std::cout<<"Reference Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_r_arm);
        std::cout<<std::endl;
    std::cout<<"Actual Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_right_arm->getActualPose());

    std::cout<<std::endl;

//    Lhand = sc_constraint->Transform_name_to_point("LWrMot3", this->robot.iDyn3_model.getLinkIndex("Waist"), link_index_Lhand);
//    Lwrist = sc_constraint->Transform_name_to_point("LWrMot2", this->robot.iDyn3_model.getLinkIndex("Waist"), link_index_Lwrist);
//    Rhand = sc_constraint->Transform_name_to_point("RWrMot3", this->robot.iDyn3_model.getLinkIndex("Waist"), link_index_Rhand);
//    Rwrist = sc_constraint->Transform_name_to_point("RWrMot2", this->robot.iDyn3_model.getLinkIndex("Waist"), link_index_Rwrist);
    Eigen::Vector3d CP1_, CP2_;
    double min_distance_;
//    min_distance_ = this->sc_constraint->dist3D_Segment_to_Segment(Lwrist, Lhand, Rwrist, Rhand, CP1_, CP2_);
    EXPECT_NEAR(0.105, min_distance_, 1e-3);
    EXPECT_TRUE(min_distance < min_distance_);
    std::cout<<"min_distance - min_distance_ = "<<min_distance<<"-"<<min_distance_<<"="<<min_distance-min_distance_<<std::endl;

  }



}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
