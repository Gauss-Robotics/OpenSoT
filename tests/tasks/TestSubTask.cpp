#include <idynutils/tests_utils.h>
#include <idynutils/idynutils.h>
#include <gtest/gtest.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>

using namespace yarp::math;
using namespace OpenSoT::tasks;

#define DOFS 10

namespace {

class TestSubTaskMap: public ::testing::Test
{
protected:

    TestSubTaskMap()
    {

    }

    virtual ~TestSubTaskMap() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

class TestSubTask: public ::testing::Test
{
protected:

    OpenSoT::tasks::velocity::Postural::Ptr _postural;
    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_limits;

    TestSubTask()

    {
        Eigen::VectorXd tmp(DOFS);
        tmp.setZero(DOFS);
        _postural.reset(new OpenSoT::tasks::velocity::Postural(tmp));
        Eigen::VectorXd tmp1 = tmp;
        tmp1<<tmp1.setOnes(DOFS)*M_PI_2;
        _joint_limits.reset(new OpenSoT::constraints::velocity::JointLimits(tmp,tmp1,-tmp1));

    }

    virtual ~TestSubTask() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(TestSubTaskMap, testSubMapConstructor)
{
    std::list<unsigned int> indices;
    // first chunk of size 3
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    // second chunk of size 2
    indices.push_back(7);
    indices.push_back(8);

    // third chunk of size 2 (11 will get added later)
    indices.push_back(10);

    // fourth chunk of size 1
    indices.push_back(13);

    // fifth chunk of size 3
    indices.push_back(17);
    indices.push_back(18);
    indices.push_back(19);

    // goes into third chunk and increases it to size 2
    indices.push_back(11);

    OpenSoT::Indices subTaskMap(indices);
    ASSERT_EQ(subTaskMap.getChunks().size(), 5);

    OpenSoT::Indices::ChunkList::const_iterator i = subTaskMap.getChunks().begin();
    EXPECT_EQ(i->size(), 3);
    i++;

    ASSERT_TRUE(i != subTaskMap.getChunks().end());
    EXPECT_EQ(i->size(), 2);
    i++;

    ASSERT_TRUE(i != subTaskMap.getChunks().end());
    EXPECT_EQ(i->size(), 2);
    i++;

    ASSERT_TRUE(i != subTaskMap.getChunks().end());
    EXPECT_EQ(i->size(), 1);
    i++;

    ASSERT_TRUE(i != subTaskMap.getChunks().end());
    EXPECT_EQ(i->size(), 3);

    ASSERT_EQ(subTaskMap.asList().size(), 11);
    ASSERT_EQ(subTaskMap.asVector().size(), 11);
    ASSERT_EQ(subTaskMap.size(), 11);
}

TEST_F(TestSubTaskMap, testSubMapToString)
{
    std::list<unsigned int> indices;
    // first chunk of size 3
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    // second chunk of size 2
    indices.push_back(7);
    indices.push_back(8);

    // third chunk of size 2 (11 will get added later)
    indices.push_back(10);

    // fourth chunk of size 1
    indices.push_back(13);

    // fifth chunk of size 3
    indices.push_back(17);
    indices.push_back(18);
    indices.push_back(19);

    // goes into third chunk and increases it to size 2
    indices.push_back(11);

    OpenSoT::Indices subTaskMap(indices);
    EXPECT_EQ(std::string(subTaskMap), "1-3+7-8+10-11+13+17-19");
}

TEST_F(TestSubTaskMap, testSubMapOperatorPlus)
{
    std::list<unsigned int> indices;
    std::list<unsigned int> indices2;
    std::list<unsigned int> indices3;

    std::list<unsigned int> indices1_plus_3;


    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    indices2.push_back(7);
    indices2.push_back(8);

    indices3.push_back(3);
    indices3.push_back(4);
    indices3.push_back(5);

    indices1_plus_3.insert(indices1_plus_3.end(), indices.begin(), indices.end());
    indices1_plus_3.insert(indices1_plus_3.end(), indices3.begin(), indices3.end());
    indices1_plus_3.sort(); indices1_plus_3.unique();

    OpenSoT::Indices subTaskMap(indices.begin(),indices.end());
    OpenSoT::Indices subTaskMap2(indices2);
    OpenSoT::Indices subTaskMap3(indices3);

    EXPECT_EQ(std::string(subTaskMap+subTaskMap2),"1-3+7-8");
    EXPECT_EQ(std::list<unsigned int>(subTaskMap+subTaskMap3),indices1_plus_3);
    EXPECT_EQ(std::string(subTaskMap+subTaskMap3),"1-5");
    EXPECT_EQ((subTaskMap+subTaskMap3).getChunks().size(),1);
    EXPECT_EQ(std::string(subTaskMap2+subTaskMap3),"3-5+7-8");
    EXPECT_EQ(std::string(subTaskMap2+subTaskMap3+9),"3-5+7-9");
    EXPECT_EQ(std::string(subTaskMap2+10+subTaskMap3),"3-5+7-8+10");
    EXPECT_EQ(std::string(subTaskMap2+10+11+12+subTaskMap3),"3-5+7-8+10-12");
}

TEST_F(TestSubTaskMap, testSubMapOperatorMinus)
{
    std::list<unsigned int> indices;
    std::list<unsigned int> indices2;
    std::list<unsigned int> indices3;

    std::list<unsigned int> indices1_minus_3;


    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    indices3.push_back(3);

    indices1_minus_3.push_back(1);
    indices1_minus_3.push_back(2);

    OpenSoT::Indices subTaskMap(indices.begin(),indices.end());
    OpenSoT::Indices subTaskMap3(indices3);

    EXPECT_EQ(std::string(subTaskMap-subTaskMap3),"1-2");
    EXPECT_EQ(std::list<unsigned int>(subTaskMap-subTaskMap3),indices1_minus_3);
    EXPECT_EQ(std::string(subTaskMap-1-2),"3");
}

TEST_F(TestSubTaskMap, testSubTaskOperatorToList)
{
    std::list<unsigned int> indices;
    // first chunk of size 3
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    // second chunk of size 2
    indices.push_back(7);
    indices.push_back(8);

    // third chunk of size 2 (11 will get added later)
    indices.push_back(10);

    // fourth chunk of size 1
    indices.push_back(13);

    // fifth chunk of size 3
    indices.push_back(17);
    indices.push_back(18);
    indices.push_back(19);

    // goes into third chunk and increases it to size 2
    indices.push_back(11);

    OpenSoT::Indices subTaskMap(indices);
    indices.sort(); indices.unique();
    EXPECT_EQ(std::list<unsigned int>(subTaskMap), indices);
}

TEST_F(TestSubTaskMap, testRange)
{
    std::list<unsigned int> indices;
    // first chunk of size 3
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);

    OpenSoT::Indices subTaskMap(indices);
    EXPECT_EQ("1-3", std::string(OpenSoT::Indices::range(1,3)));
    EXPECT_EQ(subTaskMap, OpenSoT::Indices::range(1,3));
}



TEST_F(TestSubTask, testgetA)
{
    using namespace OpenSoT;

    SubTask::Ptr subTask(new SubTask(_postural, Indices::range(0,2)));
    ASSERT_EQ(subTask->getA().rows(), 3);
    ASSERT_EQ(subTask->getA().cols(), _postural->getXSize());
    yarp::sig::Matrix A(3, _postural->getXSize());
    A = cartesian_utils::fromEigentoYarp(_postural->getA()).submatrix(0,2,0,_postural->getXSize()-1);
    EXPECT_TRUE(tests_utils::matrixAreEqual(
                    cartesian_utils::fromEigentoYarp(subTask->getA()),A));

    subTask= SubTask::Ptr(new SubTask(_postural, Indices::range(0,2) +
                                                 Indices::range(5,6)));
    ASSERT_EQ(subTask->getA().rows(), 5);
    ASSERT_EQ(subTask->getA().cols(), _postural->getXSize());
    A.resize(5, _postural->getXSize());
    A = cartesian_utils::fromEigentoYarp(_postural->getA()).submatrix(0,2,0,_postural->getXSize()-1);
    A = yarp::math::pile(A,
                         cartesian_utils::fromEigentoYarp(_postural->getA()).submatrix(5,6,0,_postural->getXSize()-1));
    EXPECT_TRUE(tests_utils::matrixAreEqual(
                    cartesian_utils::fromEigentoYarp(subTask->getA()),A));
}

TEST_F(TestSubTask, testGetHessianAType)
{
    using namespace OpenSoT;

    SubTask::Ptr subTask(new SubTask(_postural, Indices::range(1,3)));
    ASSERT_EQ(subTask->getHessianAtype(), HST_POSDEF);
}

TEST_F(TestSubTask, testgetb)
{
    using namespace OpenSoT;

    SubTask::Ptr subTask(new SubTask(_postural, Indices::range(0,2)));
    ASSERT_EQ(subTask->getb().size(), 3);
    yarp::sig::Vector b(3), bLambda(3);
    b = cartesian_utils::fromEigentoYarp(_postural->getb()).subVector(0,2);
    EXPECT_TRUE(tests_utils::vectorAreEqual(
                    cartesian_utils::fromEigentoYarp(subTask->getb()),b));

    subTask= SubTask::Ptr(new SubTask(_postural, Indices::range(0,2) +
                                                 Indices::range(5,6)));
    ASSERT_EQ(subTask->getb().size(), 5);
    b.resize(5);
    b = cartesian_utils::fromEigentoYarp(_postural->getb()).subVector(0,2);
    b = yarp::math::cat(b,
                        cartesian_utils::fromEigentoYarp(_postural->getb()).subVector(5,6));
    EXPECT_TRUE(tests_utils::vectorAreEqual(
                    cartesian_utils::fromEigentoYarp(subTask->getb()),b));

    b = cartesian_utils::fromEigentoYarp(subTask->getb());
    subTask->setLambda(0.1);
    subTask->update(_postural->getActualPositions());
    bLambda = cartesian_utils::fromEigentoYarp(subTask->getb());

    EXPECT_TRUE(tests_utils::vectorAreEqual(bLambda, b*0.1));
}

TEST_F(TestSubTask, testgetWeight)
{
    using namespace OpenSoT;

    yarp::sig::Vector W_diag(10);
    for(unsigned int i = 0; i < 10; ++i) W_diag(i) = i;
    yarp::sig::Matrix W(10,10); W.diagonal(W_diag);
    _postural->setWeight(cartesian_utils::toEigen(W));


    SubTask::Ptr subTask(new SubTask(_postural, Indices::range(0,2)));
    ASSERT_EQ(subTask->getWeight().rows(), 3);
    ASSERT_EQ(subTask->getWeight().cols(), 3);

    W = cartesian_utils::fromEigentoYarp(_postural->getWeight()).submatrix(0,2,0,2);
    EXPECT_TRUE(tests_utils::matrixAreEqual(
                    cartesian_utils::fromEigentoYarp(subTask->getWeight()),W));

    subTask= SubTask::Ptr(new SubTask(_postural, Indices::range(0,2) +
                                                 Indices::range(5,6)));
    ASSERT_EQ(subTask->getWeight().rows(), 5);
    ASSERT_EQ(subTask->getWeight().cols(), 5);
    W.resize(5, 5);
    unsigned int c_indices[] = {0, 1, 2, 5, 6};
    std::vector<unsigned int> indices(c_indices,c_indices+sizeof(c_indices)/sizeof(indices[0]));
    for(unsigned int r = 0; r < indices.size(); ++r)
        for(unsigned int c = 0; c < indices.size(); ++c)
            W(r,c) = _postural->getWeight()(indices[r], indices[c]);
    EXPECT_TRUE(tests_utils::matrixAreEqual(
                    cartesian_utils::fromEigentoYarp(subTask->getWeight()),W));
}

TEST_F(TestSubTask, testsetWeight)
{
    using namespace OpenSoT;

    yarp::sig::Vector W_diag(3);
    yarp::sig::Matrix fullW = cartesian_utils::fromEigentoYarp(_postural->getWeight());
    for(unsigned int i = 0; i < 3; ++i) { W_diag(i) = i+1; fullW(i,i) = i+1; }
    yarp::sig::Matrix W(3,3); W.diagonal(W_diag);

    SubTask::Ptr subTask(new SubTask(_postural, Indices::range(0,2)));
    subTask->setWeight(cartesian_utils::toEigen(W));

    ASSERT_EQ(subTask->getWeight().rows(), 3);
    ASSERT_EQ(subTask->getWeight().cols(), 3);

    EXPECT_TRUE(tests_utils::matrixAreEqual(
                    cartesian_utils::fromEigentoYarp(_postural->getWeight()),fullW))  <<
        "_postural->getWeight() is:\n" << _postural->getWeight() <<
        "\nshould be:\n" << fullW.toString();
    EXPECT_TRUE(tests_utils::matrixAreEqual(
                    cartesian_utils::fromEigentoYarp(subTask->getWeight()),W));

    W.resize(5,5); W_diag.resize(5);
    for(unsigned int i = 0; i < 3; ++i) { W_diag(i) = i+1; fullW(i,i) = i+1; }
    for(unsigned int i = 3; i < 5 ; ++i) { W_diag(i) = i+1; fullW(i+2,i+2) = i+1; }
    W.diagonal(W_diag);

    subTask = SubTask::Ptr(new SubTask(_postural, Indices::range(0,2) +
                                                  Indices::range(5,6)));
    subTask->setWeight(cartesian_utils::toEigen(W));
    ASSERT_EQ(cartesian_utils::fromEigentoYarp(subTask->getWeight()).rows(), 5);
    ASSERT_EQ(cartesian_utils::fromEigentoYarp(subTask->getWeight()).cols(), 5);

    EXPECT_TRUE(tests_utils::matrixAreEqual(
                    cartesian_utils::fromEigentoYarp(_postural->getWeight()),fullW)) << "\n"    << _postural->getWeight()
                                                                           << "\nto\n" << fullW.toString() << "\n";
    EXPECT_TRUE(tests_utils::matrixAreEqual(
                    cartesian_utils::fromEigentoYarp(subTask->getWeight()),W));
}


TEST_F(TestSubTask, testGetConstraints)
{
    using namespace OpenSoT;

    _postural->getConstraints().push_back(_joint_limits);

    SubTask::Ptr subTask(new SubTask(_postural, Indices::range(1,3)));

    yarp::sig::Vector lowerBounds =
            cartesian_utils::fromEigentoYarp(subTask->getConstraints().front()->getLowerBound());
    yarp::sig::Vector b = cartesian_utils::fromEigentoYarp(subTask->getb());

    subTask->update(cartesian_utils::toEigen(yarp::sig::Vector(DOFS, 1.0)));

    EXPECT_FALSE( b == cartesian_utils::fromEigentoYarp(subTask->getb()));
    EXPECT_FALSE( lowerBounds ==
                  cartesian_utils::fromEigentoYarp(subTask->getConstraints().front()->getLowerBound()));

    lowerBounds = cartesian_utils::fromEigentoYarp(subTask->getConstraints().front()->getLowerBound());
    b = cartesian_utils::fromEigentoYarp(subTask->getb());

    subTask->update(cartesian_utils::toEigen(yarp::sig::Vector(DOFS, 0.5)));

    EXPECT_FALSE( b == cartesian_utils::fromEigentoYarp(subTask->getb()));
    EXPECT_FALSE( lowerBounds ==
                  cartesian_utils::fromEigentoYarp(subTask->getConstraints().front()->getLowerBound()));
}

TEST_F(TestSubTask, testUpdate)
{
    using namespace OpenSoT;

    _postural->getConstraints().push_back(_joint_limits);
    SubTask::Ptr subTask(new SubTask(_postural, Indices::range(1,3)));


    ASSERT_EQ(_postural->getConstraints(), subTask->getConstraints());
}

TEST_F(TestSubTask, testGetTaskSize)
{
    using namespace OpenSoT;

    _postural->getConstraints().push_back(_joint_limits);
    SubTask::Ptr subTask(new SubTask(_postural, Indices::range(1,3)));
    ASSERT_EQ(subTask->getTaskSize(), 3);

    subTask = SubTask::Ptr(new SubTask(_postural, Indices::range(1,3) +
                                                  Indices::range(6,7)));
    ASSERT_EQ(subTask->getTaskSize(), 5);
}



}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
