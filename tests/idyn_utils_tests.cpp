#include <gtest/gtest.h>
#include <idynutils/idynutils.h>
#include <idynutils/cartesian_utils.h>
#include <idynutils/tests_utils.h>
#include <yarp/math/Math.h>
#include <kdl/frames_io.hpp>



using namespace yarp::math;

namespace {

enum switchingTest {
    SWITCH_ANCHOR = 1,
    SWITCH_FLOATING_BASE = 2,
    SWITCH_BOTH = 3
};

typedef std::pair<bool,switchingTest> switchingType;

class testIDynUtils: public ::testing::Test, public iDynUtils
{
protected:
    testIDynUtils():
        q(iDyn3_model.getNrOfDOFs(),0.0)
    {

    }

    virtual ~testIDynUtils() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    void setGoodInitialPosition() {

        yarp::sig::Vector leg(left_leg.getNrOfDOFs(), 0.0);
        leg[0] = -25.0 * M_PI/180.0;
        leg[3] =  50.0 * M_PI/180.0;
        leg[5] = -25.0 * M_PI/180.0;
        fromRobotToIDyn(leg, q, left_leg);
        fromRobotToIDyn(leg, q, right_leg);
        yarp::sig::Vector arm(left_arm.getNrOfDOFs(), 0.0);
        arm[0] = 20.0 * M_PI/180.0;
        arm[1] = 10.0 * M_PI/180.0;
        arm[3] = -80.0 * M_PI/180.0;
        fromRobotToIDyn(arm, q, left_arm);
        arm[1] = -arm[1];
        fromRobotToIDyn(arm, q, right_arm);

        updateiDyn3Model(q,true);
    }

    yarp::sig::Vector q;
};

class testIDynUtilsWithAndWithoutUpdate : public testIDynUtils,
                public ::testing::WithParamInterface<switchingType> {
};

TEST_F(testIDynUtils, testFromRobotToIDynThree)
{
    yarp::sig::Vector q(this->iDyn3_model.getNrOfDOFs(), 0.0);
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
    yarp::sig::Vector q(this->iDyn3_model.getNrOfDOFs(), 0.0);
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
    yarp::sig::Matrix base_link_T_l_sole = this->iDyn3_model.getPosition(this->iDyn3_model.getLinkIndex("l_sole"));
    std::cout<<"base_link_T_l_sole "<<std::endl; cartesian_utils::printHomogeneousTransform(base_link_T_l_sole);std::cout<<std::endl;

    KDL::Frame base_link_T_l_ankle = this->iDyn3_model.getPositionKDL(this->iDyn3_model.getLinkIndex("l_ankle"));

    /// In this case I update the world pose considering l_sole as my inertial frame
    this->updateiDyn3Model(q, dq, ddq, true);

    std::cout<<"World "<<std::endl; cartesian_utils::printHomogeneousTransform(this->worldT);std::cout<<std::endl;
    yarp::sig::Matrix world_T_l_sole = this->iDyn3_model.getPosition(this->iDyn3_model.getLinkIndex("l_sole"));
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

    yarp::sig::Matrix world_link_T_l_sole = this->iDyn3_model.getPosition(this->iDyn3_model.getLinkIndex("l_sole"));
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

    KDL::Frame l_ankle_T_l_sole = this->iDyn3_model.getPositionKDL(this->iDyn3_model.getLinkIndex("l_ankle"), this->iDyn3_model.getLinkIndex("l_sole"));
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
    yarp::sig::Vector q(this->iDyn3_model.getNrOfDOFs(), 0.0);
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

    KDL::Frame base_link_T_l_sole = this->iDyn3_model.getPositionKDL(0, this->iDyn3_model.getLinkIndex("l_sole"));
    std::cout<<"base_link_T_l_sole"<<std::endl; cartesian_utils::printKDLFrame(base_link_T_l_sole); std::cout<<std::endl;
    KDL::Frame new_world = world_T_l_sole * base_link_T_l_sole.Inverse();

    KDL::Frame actual_world;
    cartesian_utils::fromYARPMatrixtoKDLFrame(this->worldT, actual_world);

    EXPECT_TRUE(actual_world == new_world);
}

TEST_F(testIDynUtils, testGravityVector)
{
    yarp::sig::Vector g(this->g);

    yarp::sig::Vector q(this->iDyn3_model.getNrOfDOFs(), 0.0);
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

    yarp::sig::Vector q(this->iDyn3_model.getNrOfDOFs(), 0.0);
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
    yarp::sig::Vector q(this->iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector dq(q);
    yarp::sig::Vector ddq(q);

    this->updateiDyn3Model(q, dq, ddq, true);

    yarp::sig::Vector tau_g = this->iDyn3_model.getTorques();
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
    yarp::sig::Vector tau_g2 = this->iDyn3_model.getTorques();
    yarp::sig::Matrix world2 = this->worldT;

    q.zero();
    this->updateiDyn3Model(q, dq, ddq, true);
    yarp::sig::Vector tau_g3 = this->iDyn3_model.getTorques();
    yarp::sig::Matrix world3 = this->worldT;

    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(world(i,j), world2(i,j), 1E-16);
    }

    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(world(i,j), world3(i,j), 1E-16);
    }

    for(unsigned int i = 0; i < 7; ++i){
        EXPECT_NEAR(tau_g[this->left_arm.joint_numbers[i]], tau_g2[this->left_arm.joint_numbers[i]], 1E-12)<<"left_arm @ joint "<<i;
        EXPECT_NEAR(tau_g[this->right_arm.joint_numbers[i]], tau_g2[this->right_arm.joint_numbers[i]], 1E-12)<<"right_arm @ joint "<<i;
    }
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(tau_g[this->torso.joint_numbers[i]], tau_g2[this->torso.joint_numbers[i]], 1E-12 )<<"torso @ joint "<<i;

    for(unsigned int i = 0; i < 7; ++i){
        EXPECT_NEAR(tau_g[this->left_arm.joint_numbers[i]], tau_g3[this->left_arm.joint_numbers[i]], 1E-12)<<"left_arm @ joint "<<i;
        EXPECT_NEAR(tau_g[this->right_arm.joint_numbers[i]], tau_g3[this->right_arm.joint_numbers[i]], 1E-12)<<"right_arm @ joint "<<i;
    }
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(tau_g[this->torso.joint_numbers[i]], tau_g3[this->torso.joint_numbers[i]], 1E-12 )<<"torso @ joint "<<i;
}

TEST_F(testIDynUtils, testIDyn3Model)
{
    EXPECT_TRUE(this->iDyn3Model())<<"Failed to load the model, are you sure that you have generated the model files? Try to "<<
                                     "generate file model before going to Enrico or Alessio complaining."<<std::endl;
}

TEST_F(testIDynUtils, testSetJointNames)
{
    EXPECT_TRUE(this->setJointNames());
}

TEST_F(testIDynUtils, testSetChainIndex)
{
    EXPECT_TRUE(this->setChainIndex(this->left_arm.end_effector_name, this->left_arm));
    std::string fake_name = "sossio";
    EXPECT_FALSE(this->setChainIndex(fake_name, this->left_arm));
}

TEST_F(testIDynUtils, testWorld)
{
    iDynUtils idynutils1;
    iDynUtils idynutils2;
    idynutils2.iDyn3_model.setFloatingBaseLink(idynutils2.left_leg.index);

    yarp::sig::Vector q(idynutils1.iDyn3_model.getNrOfDOFs(), 0.0);
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = tests_utils::getRandomAngle();

    idynutils1.updateiDyn3Model(q, true);
    idynutils2.updateiDyn3Model(q, true);

    yarp::sig::Matrix w_T_bl = idynutils1.iDyn3_model.getWorldBasePose();
    EXPECT_EQ(idynutils1.iDyn3_model.getLinkIndex("Waist"), 0);
    yarp::sig::Matrix bl_T_lf = idynutils1.iDyn3_model.getPosition(0, idynutils1.left_leg.index);
    yarp::sig::Matrix w_T_lf = w_T_bl * bl_T_lf;

    yarp::sig::Matrix w_T_bl2 = idynutils2.iDyn3_model.getWorldBasePose();
    EXPECT_EQ(idynutils2.iDyn3_model.getLinkIndex("Waist"), 0);

    std::cout<<"w_T_lf: "<<std::endl;cartesian_utils::printHomogeneousTransform(w_T_lf);
    std::cout<<"w_T_bl2: "<<std::endl;cartesian_utils::printHomogeneousTransform(w_T_bl2);

    for(unsigned int i = 0; i < 4; ++i)
    {
        for(unsigned int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(w_T_lf(i,j), w_T_bl2(i,j), 1E-15);
        }
    }

    yarp::sig::Matrix w_T_rh = idynutils1.iDyn3_model.getPosition(idynutils1.right_arm.index);
    yarp::sig::Matrix w_T_rh2 = idynutils2.iDyn3_model.getPosition(idynutils2.right_arm.index);

    for(unsigned int i = 0; i < 4; ++i)
    {
        for(unsigned int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(w_T_rh(i,j), w_T_rh2(i,j), 1E-15);
        }
    }

    q.zero();
    idynutils1.updateiDyn3Model(q,true);
    idynutils2.updateiDyn3Model(q,true);
    yarp::sig::Vector w_T_CoM = idynutils1.iDyn3_model.getCOM();
    yarp::sig::Vector w_T_CoM2 = idynutils2.iDyn3_model.getCOM();
    yarp::sig::Matrix w_T_lh = idynutils1.iDyn3_model.getPosition(idynutils1.left_arm.index);
    yarp::sig::Matrix w_T_lh2 = idynutils2.iDyn3_model.getPosition(idynutils2.left_arm.index);

    for(unsigned int i = 0; i < 4; ++i)
    {
        for(unsigned int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(w_T_lh(i,j), w_T_lh2(i,j), 1E-15);
        }
    }

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(w_T_CoM(i), w_T_CoM2(i), 1E-15);

}

TEST_F(testIDynUtils, testAnchorSwitch)
{
    KDL::Frame w_T_b0 = iDyn3_model.getWorldBasePoseKDL();
    std::cout<<"Initial World to base_link Transform:"<<std::endl;
    cartesian_utils::printKDLFrame(w_T_b0);

    setGoodInitialPosition();

    KDL::Frame w_T_b1 = iDyn3_model.getWorldBasePoseKDL();
    std::cout<<"World to base_link Transform before switching:"<<std::endl;
    cartesian_utils::printKDLFrame(w_T_b1);

    std::string new_anchor = "r_sole";
    std::cout<<"Setting new anchor in "<<new_anchor<<std::endl;
    switchAnchor(new_anchor);

    KDL::Frame w_T_b2 = iDyn3_model.getWorldBasePoseKDL();
    std::cout<<"World to base_link Transform after switching:"<<std::endl;
    cartesian_utils::printKDLFrame(w_T_b2);

    EXPECT_TRUE(w_T_b1 == w_T_b2);

    q.zero();

    updateiDyn3Model(q,true);

    KDL::Frame w_T_b22 = iDyn3_model.getWorldBasePoseKDL();
    std::cout<<"World to base_link Transform before switching:"<<std::endl;
    cartesian_utils::printKDLFrame(w_T_b22);

    std::string old_anchor = "l_sole";
    std::cout<<"Setting old anchor in "<<old_anchor<<std::endl;
    switchAnchor(new_anchor);


    KDL::Frame w_T_b12 = iDyn3_model.getWorldBasePoseKDL();
    std::cout<<"World to base_link Transform after switching:"<<std::endl;
    cartesian_utils::printKDLFrame(w_T_b12);

    EXPECT_TRUE(w_T_b12 == w_T_b22);

    //EXPECT_TRUE(w_T_b12 == w_T_b0);

}

TEST_P(testIDynUtilsWithAndWithoutUpdate, testAnchorSwitchWGetPosition)
{
    bool updateIDynAfterSwitch = GetParam().first;
    switchingTest whatToSwitch = GetParam().second;

    setGoodInitialPosition();

    KDL::Frame w_T_l_wrist_l_sole = iDyn3_model.getPositionKDL(left_arm.index);
    std::cout << "World to l_wrist Transform in q=good, anchor=l_sole:" << std::endl
              << w_T_l_wrist_l_sole << std::endl;

    std::string new_anchor = "r_sole";
    switch(whatToSwitch) {
        case SWITCH_ANCHOR:
        {
            std::cout   << "Setting new anchor in " << new_anchor << std::endl;
            switchAnchor(new_anchor);
            break;
        }
        case SWITCH_FLOATING_BASE:
        {
            std::cout   << "Setting new floating base in " << new_anchor << std::endl;
            setFloatingBaseLink(new_anchor);
            break;
        }
        case SWITCH_BOTH:
        {
            std::cout   << "Setting new floating base and anchor in " << new_anchor << std::endl;
            switchAnchorAndFloatingBase(new_anchor);
            break;
        }
    }
    if(updateIDynAfterSwitch) updateiDyn3Model(q,true);


    KDL::Frame w_T_l_wrist_r_sole = iDyn3_model.getPositionKDL(left_arm.index);
    std::cout   << "World to l_wrist Transform in q=good, anchor=r_sole:"  << std::endl
                << w_T_l_wrist_r_sole << std::endl;

    EXPECT_TRUE(w_T_l_wrist_l_sole == w_T_l_wrist_r_sole);

    q.zero();

    updateiDyn3Model(q,true);

    w_T_l_wrist_r_sole = iDyn3_model.getPositionKDL(left_arm.index);
    std::cout   << "World to l_wrist Transform in q=0, anchor=r_sole:" << std::endl
                << w_T_l_wrist_r_sole << std::endl;

    std::string old_anchor = "l_sole";
    switch(whatToSwitch) {
        case SWITCH_ANCHOR:
        {
            std::cout   << "Setting new anchor in " << old_anchor << std::endl;
            switchAnchor(old_anchor);
            break;
        }
        case SWITCH_FLOATING_BASE:
        {
            std::cout   << "Setting new floating base in " << old_anchor << std::endl;
            setFloatingBaseLink(old_anchor);
            break;
        }
        case SWITCH_BOTH:
        {
            std::cout   << "Setting new floating base and anchor in " << old_anchor << std::endl;
            switchAnchorAndFloatingBase(old_anchor);
            break;
        }
    }
    if(updateIDynAfterSwitch) updateiDyn3Model(q,true);

    w_T_l_wrist_l_sole = iDyn3_model.getPositionKDL(left_arm.index);
    std::cout<<"World to l_wrist Transform in q=0, anchor=l_sole:"<<std::endl;
    cartesian_utils::printKDLFrame(w_T_l_wrist_l_sole);

    EXPECT_TRUE(w_T_l_wrist_l_sole == w_T_l_wrist_r_sole);

}


TEST_P(testIDynUtilsWithAndWithoutUpdate, testAnchorSwitchWGetCoM)
{
    bool updateIDynAfterSwitch = GetParam().first;
    switchingTest whatToSwitch = GetParam().second;

    setGoodInitialPosition();

    KDL::Vector w_T_CoM_l_sole = iDyn3_model.getCOMKDL();
    std::cout   << "World to CoM Transform in q=good, anchor=l_sole:" << std::endl
                << w_T_CoM_l_sole << std::endl;
    //cartesian_utils::printKDLFrame(w_T_CoM_l_sole);

    std::string new_anchor = "r_sole";
    switch(whatToSwitch) {
        case SWITCH_ANCHOR:
        {
            std::cout   << "Setting new anchor in " << new_anchor << std::endl;
            switchAnchor(new_anchor);
            break;
        }
        case SWITCH_FLOATING_BASE:
        {
            std::cout   << "Setting new floating base in " << new_anchor << std::endl;
            setFloatingBaseLink(new_anchor);
            break;
        }
        case SWITCH_BOTH:
        {
            std::cout   << "Setting new floating base and anchor in " << new_anchor << std::endl;
            switchAnchorAndFloatingBase(new_anchor);
            break;
        }
    }
    if(updateIDynAfterSwitch) updateiDyn3Model(q,true);

    KDL::Vector w_T_CoM_r_sole = iDyn3_model.getCOMKDL();
    std::cout   << "World to CoM Transform in q=good, anchor=r_sole:" << std::endl
                << w_T_CoM_r_sole << std::endl;
    //cartesian_utils::printKDLFrame(w_T_l_wrist_r_sole);

    EXPECT_TRUE(w_T_CoM_l_sole == w_T_CoM_r_sole);

    q.zero();

    updateiDyn3Model(q,true);

    w_T_CoM_r_sole = iDyn3_model.getCOMKDL();
    std::cout   << "World to CoM Transform in q=0, anchor=r_sole:" << std::endl
                << w_T_CoM_r_sole << std::endl;
    //cartesian_utils::printKDLFrame(w_T_CoM_r_sole);

    std::string old_anchor = "l_sole";
    switch(whatToSwitch) {
        case SWITCH_ANCHOR:
        {
            std::cout   << "Setting new anchor in " << old_anchor << std::endl;
            switchAnchor(old_anchor);
            break;
        }
        case SWITCH_FLOATING_BASE:
        {
            std::cout   << "Setting new floating base in " << old_anchor << std::endl;
            setFloatingBaseLink(old_anchor);
            break;
        }
        case SWITCH_BOTH:
        {
            std::cout   << "Setting new anchor in " << old_anchor << std::endl;
            switchAnchor(old_anchor);
            std::cout   << "Setting new floating base in " << old_anchor << std::endl;
            setFloatingBaseLink(old_anchor);
            break;
        }
    }
    if(updateIDynAfterSwitch) updateiDyn3Model(q,true);

    w_T_CoM_l_sole = iDyn3_model.getCOMKDL();
    std::cout   << "World to CoM Transform in q=0, anchor=l_sole:" << std::endl
                << w_T_CoM_l_sole << std::endl;
    //cartesian_utils::printKDL(w_T_CoM_l_sole);

    EXPECT_TRUE(w_T_CoM_l_sole == w_T_CoM_r_sole);

}

INSTANTIATE_TEST_CASE_P(SwitchAnchor,
                        testIDynUtilsWithAndWithoutUpdate,
                        ::testing::Values(std::make_pair(true,SWITCH_ANCHOR),
                                          std::make_pair(false,SWITCH_ANCHOR),
                                          std::make_pair(true,SWITCH_FLOATING_BASE),
                                          std::make_pair(false,SWITCH_FLOATING_BASE),
                                          std::make_pair(true,SWITCH_BOTH),
                                          std::make_pair(false,SWITCH_BOTH)));
} //namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
