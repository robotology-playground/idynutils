#include <gtest/gtest.h>
#include <drc_shared/idynutils.h>
#include <drc_shared/cartesian_utils.h>
#include <drc_shared/tests_utils.h>

namespace {

class testIDynUtils: public ::testing::Test, public iDynUtils
{
protected:
    testIDynUtils()
    {

    }

    virtual ~testIDynUtils() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

TEST_F(testIDynUtils, testFromRobotToIDynThree)
{
    yarp::sig::Vector q(this->coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector q_left_leg(this->left_leg.getNrOfDOFs(), 0.0);

    for(unsigned int i = 0; i < q_left_leg.size(); ++i)
        q_left_leg[i] = M_PI_2;

    this->fromRobotToIDyn(q_left_leg, q, this->left_leg);

    for(unsigned int i = 0; i < q.size(); ++i)
    {
        bool is_in_kinematic_chain = false;
        for(unsigned int j = 0; j < this->left_leg.getNrOfDOFs(); ++j)
        {
            if(i == this->left_leg.joint_numbers[j])
                is_in_kinematic_chain = true;
        }

        if(is_in_kinematic_chain)
            EXPECT_DOUBLE_EQ(q[i], M_PI_2);
        else
            EXPECT_DOUBLE_EQ(q[i], 0.0);
    }
}

TEST_F(testIDynUtils, testUpdateIdyn3Model)
{
    yarp::sig::Vector q(this->coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector dq(q);
    yarp::sig::Vector ddq(q);

    this->updateiDyn3Model(q, dq, ddq);

    /// In this case I expect that the world is the Identity
    yarp::sig::Matrix I(4,4); I.eye();
    for(unsigned int i = 0; i < I.rows(); ++i)
    {
        for(unsigned int j = 0; j < I.cols(); ++j)
            EXPECT_DOUBLE_EQ(I(i,j), this->worldT(i,j));
    }

    std::cout<<"World "<<std::endl; cartesian_utils::printHomogeneousTransform(this->worldT);std::cout<<std::endl;
    yarp::sig::Matrix base_link_T_l_sole = this->coman_iDyn3.getPosition(this->coman_iDyn3.getLinkIndex("l_sole"));
    std::cout<<"base_link_T_l_sole "<<std::endl; cartesian_utils::printHomogeneousTransform(base_link_T_l_sole);std::cout<<std::endl;

    KDL::Frame base_link_T_l_ankle = this->coman_iDyn3.getPositionKDL(this->coman_iDyn3.getLinkIndex("l_ankle"));

    /// In this case I update the world pose considering l_sole as my inertial frame
    this->updateiDyn3Model(q, dq, ddq, true);

    std::cout<<"World "<<std::endl; cartesian_utils::printHomogeneousTransform(this->worldT);std::cout<<std::endl;
    yarp::sig::Matrix world_T_l_sole = this->coman_iDyn3.getPosition(this->coman_iDyn3.getLinkIndex("l_sole"));
    std::cout<<"world_T_l_sole "<<std::endl; cartesian_utils::printHomogeneousTransform(world_T_l_sole);std::cout<<std::endl;
    std::cout<<"anchor_T_world "<<std::endl; cartesian_utils::printKDLFrame(this->anchor_T_world);std::cout<<std::endl;


    //I expect the world under the base_link so worldT.x = 0.0 and worldT.y = 0.0 (will change in future!)
    EXPECT_DOUBLE_EQ(worldT(0,3), 0.0);
    EXPECT_DOUBLE_EQ(worldT(1,3), 0.0);
    //I expect the base_link at "-" the same height of the l_sole ref. frame
    EXPECT_DOUBLE_EQ(worldT(2,3), -base_link_T_l_sole(2,3));
    //I expect the base_link orientation the opposite of the l_sole ref_frame, in this particular case btw is the I
    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_DOUBLE_EQ(worldT(i,j), I(i,j));
    }

    yarp::sig::Matrix world_link_T_l_sole = this->coman_iDyn3.getPosition(this->coman_iDyn3.getLinkIndex("l_sole"));
    //Now poses are computed wrt the new set world
    EXPECT_DOUBLE_EQ(world_link_T_l_sole(0,3), 0.0);
    EXPECT_DOUBLE_EQ(world_link_T_l_sole(1,3), base_link_T_l_sole(1,3));
    EXPECT_DOUBLE_EQ(world_link_T_l_sole(2,3), 0.0);

    /// Here I rotate the left foot of 45° on the pitch
    double pitch = M_PI_2/2.0;
    q[this->left_leg.joint_numbers.at(this->left_leg.joint_numbers.size()-1)] = pitch;
    this->updateiDyn3Model(q, dq, ddq, true);

    //I expect the base_link rotated of -45° wrt world
    KDL::Rotation rotY;
    rotY.DoRotY(pitch);
    KDL::Frame T(rotY, KDL::Vector(0.0, 0.0, 0.0));

    KDL::Frame l_ankle_T_l_sole = this->coman_iDyn3.getPositionKDL(this->coman_iDyn3.getLinkIndex("l_ankle"), this->coman_iDyn3.getLinkIndex("l_sole"));
    KDL::Frame rotated_l_ankle_T_base_link = T.Inverse() * base_link_T_l_ankle.Inverse();
    std::cout<<"rotated_l_ankle_T_base_link "<<std::endl; cartesian_utils::printKDLFrame(rotated_l_ankle_T_base_link);std::cout<<std::endl;
    KDL::Frame l_sole_T_base_link = l_ankle_T_l_sole.Inverse() * rotated_l_ankle_T_base_link;
    std::cout<<"l_sole_T_base_link "<<std::endl; cartesian_utils::printKDLFrame(l_sole_T_base_link);std::cout<<std::endl;

    //I expect the world under the base_link so worldT.x = 0.0 and worldT.y = 0.0 (will change in future!)
    std::cout<<"World "<<std::endl; cartesian_utils::printHomogeneousTransform(this->worldT);std::cout<<std::endl;
    EXPECT_DOUBLE_EQ(worldT(0,3), l_sole_T_base_link(0,3));
    EXPECT_DOUBLE_EQ(worldT(1,3), 0.0);
    //I expect the base_link at "-" the same height of the l_sole ref. frame
    EXPECT_DOUBLE_EQ(worldT(2,3), l_sole_T_base_link(2,3));
    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_DOUBLE_EQ(this->worldT(i,j), T.Inverse()(i,j));
    }
}



TEST_F(testIDynUtils, testGerenicRotationUpdateIdyn3Model)
{
    yarp::sig::Vector q(this->coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector dq(q);
    yarp::sig::Vector ddq(q);

    this->updateiDyn3Model(q, dq, ddq, true);

    KDL::Frame old_world;
    cartesian_utils::fromYARPMatrixtoKDLFrame(this->worldT, old_world);
    KDL::Frame world_T_l_sole = this->anchor_T_world.Inverse();
    std::cout<<"world_T_l_sole "<<std::endl; cartesian_utils::printKDLFrame(world_T_l_sole); std::cout<<std::endl;

    for(unsigned int i = 0; i < this->left_leg.getNrOfDOFs(); ++i)
        q[this->left_leg.joint_numbers[i]] = tests_utils::getRandomAngle();
    this->updateiDyn3Model(q, dq, ddq, true);

    EXPECT_TRUE(world_T_l_sole == this->anchor_T_world.Inverse());

    KDL::Frame base_link_T_l_sole = this->coman_iDyn3.getPositionKDL(0, this->coman_iDyn3.getLinkIndex("l_sole"));
    std::cout<<"base_link_T_l_sole"<<std::endl; cartesian_utils::printKDLFrame(base_link_T_l_sole); std::cout<<std::endl;
    KDL::Frame new_world = world_T_l_sole * base_link_T_l_sole.Inverse();

    KDL::Frame actual_world;
    cartesian_utils::fromYARPMatrixtoKDLFrame(this->worldT, actual_world);

    EXPECT_TRUE(actual_world == new_world);
}

TEST_F(testIDynUtils, testGravityVector)
{
    yarp::sig::Vector g(this->g);

    yarp::sig::Vector q(this->coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector dq(q);
    yarp::sig::Vector ddq(q);

    this->updateiDyn3Model(q, dq, ddq, true);

    EXPECT_DOUBLE_EQ(g[0], this->g[0]);
    EXPECT_DOUBLE_EQ(g[1], this->g[1]);
    EXPECT_DOUBLE_EQ(g[2], this->g[2]);

    /// Here I rotate the left foot of 45° on the pitch
    double pitch = M_PI_2/2.0;
    q[this->left_leg.joint_numbers.at(this->left_leg.joint_numbers.size()-1)] = pitch;
    this->updateiDyn3Model(q, dq, ddq, true);

    KDL::Rotation rotY;
    rotY.DoRotY(pitch);
    KDL::Frame T(rotY, KDL::Vector(0.0, 0.0, 0.0));

    KDL::Vector rotated_g = T.M * KDL::Vector(g[0], g[1], g[2]);

    EXPECT_DOUBLE_EQ(rotated_g[0], this->g[0]);
    EXPECT_DOUBLE_EQ(rotated_g[1], this->g[1]);
    EXPECT_DOUBLE_EQ(rotated_g[2], this->g[2]);
}

TEST_F(testIDynUtils, testGenerarRotationGravityVector)
{
    yarp::sig::Vector g(this->g);

    yarp::sig::Vector q(this->coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector dq(q);
    yarp::sig::Vector ddq(q);

    for(unsigned int i = 0; i < this->left_leg.getNrOfDOFs(); ++i)
        q[this->left_leg.joint_numbers[i]] = tests_utils::getRandomAngle();
    this->updateiDyn3Model(q, dq, ddq, true);

    KDL::Frame worldT_KDL;
    cartesian_utils::fromYARPMatrixtoKDLFrame(worldT, worldT_KDL);
    KDL::Vector rotated_g = worldT_KDL.M.Inverse() * KDL::Vector(g[0], g[1], g[2]);

    EXPECT_TRUE(rotated_g == KDL::Vector(this->g[0], this->g[1], this->g[2]));
}

TEST_F(testIDynUtils, testTauGravityID)
{
    yarp::sig::Vector q(this->coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector dq(q);
    yarp::sig::Vector ddq(q);

    this->updateiDyn3Model(q, dq, ddq, true);
    this->updateiDyn3Model(q, dq, ddq, true);

    yarp::sig::Vector tau_g = this->coman_iDyn3.getTorques();
    yarp::sig::Matrix world = this->worldT;

    yarp::sig::Vector q_left_leg(6, 0.0);
    q_left_leg[0] = -25.0;
    q_left_leg[1] = 0.0;
    q_left_leg[2] = 0.0;
    q_left_leg[3] = 50.0;
    q_left_leg[4] = 0.0;
    q_left_leg[5] = -25.0;

    yarp::sig::Vector q_right_leg(6, 0.0);
    q_right_leg[0] = -25.0;
    q_right_leg[1] = 0.0;
    q_right_leg[2] = 0.0;
    q_right_leg[3] = 50.0;
    q_right_leg[4] = 0.0;
    q_right_leg[5] = -25.0;

    this->fromRobotToIDyn(q_left_leg, q, this->left_leg);
    this->fromRobotToIDyn(q_right_leg, q, this->right_leg);

    this->updateiDyn3Model(q, dq, ddq, true);
    this->updateiDyn3Model(q, dq, ddq, true);
    yarp::sig::Matrix world2 = this->worldT;

    yarp::sig::Vector tau_g2 = this->coman_iDyn3.getTorques();

    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(world(i,j), world2(i,j), 1E-16);
    }

    for(unsigned int i = 0; i < 7; ++i){
        EXPECT_NEAR(tau_g[this->left_arm.joint_numbers[i]], tau_g2[this->left_arm.joint_numbers[i]], 1E-12)<<"left_arm @ joint "<<i;
        EXPECT_NEAR(tau_g[this->right_arm.joint_numbers[i]], tau_g2[this->right_arm.joint_numbers[i]], 1E-12)<<"right_arm @ joint "<<i;
    }
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(tau_g[this->torso.joint_numbers[i]], tau_g2[this->torso.joint_numbers[i]], 1E-12 )<<"torso @ joint "<<i;
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
