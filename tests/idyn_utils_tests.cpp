#include <gtest/gtest.h>
#include <idynutils/idynutils.h>
#include <idynutils/cartesian_utils.h>
#include <idynutils/tests_utils.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/os/Time.h>
#include <kdl/frames_io.hpp>

#include <iostream>
#include <cstdlib>

using namespace yarp::math;

namespace {

enum switchingTest {
    SWITCH_ANCHOR = 1,
    SWITCH_FLOATING_BASE = 2,
    SWITCH_BOTH = 3
};

enum walkingTestStartingFoot {
    START_WITH_LEFT = 1,
    START_WITH_RIGHT = 2
};

typedef std::pair<bool,switchingTest> switchingType;
typedef std::pair<bool,walkingTestStartingFoot> walkingType;

class testFoo: public ::testing::Test
{
protected:
    testFoo(){}
    virtual ~testFoo() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

class testIDynUtils: public ::testing::Test, public iDynUtils
{
protected:
    testIDynUtils():
        iDynUtils("coman",
                  std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                  std::string(IDYNUTILS_TESTS_ROBOTS_DIR) + "coman/coman.srdf"),
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

class testIDynUtilsWithAndWithoutUpdateAndDifferentSwitchTypes : public testIDynUtils,
                public ::testing::WithParamInterface<switchingType> {
};

class testIDynUtilsWithAndWithoutUpdate : public testIDynUtils,
                public ::testing::WithParamInterface<bool> {
};

class testIDynUtilsWithAndWithoutUpdateAndWithFootSwitching : public testIDynUtils,
        public ::testing::WithParamInterface<walkingType> {

};

TEST_F(testFoo, testInitialization)
{
    assert((IDYNUTILS_TESTS_ROBOTS_DIR != NULL) &&
        "Error: IDYNUTILS_TESTS_ROBOTS_DIR is not defined. Please check tests/CMakeLists.txt");
    std::string urdf_file = std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf";
    std::string srdf_file = std::string(IDYNUTILS_TESTS_ROBOTS_DIR) + "bigman/bigman.srdf";

    iDynUtils idynutils("bigman", urdf_file, srdf_file);

    //Test ALL active joint list
    for(unsigned int i = 0; i < idynutils.moveit_robot_model->getActiveJointModels().size(); ++i)
        EXPECT_TRUE(idynutils.getJointNames()[i] ==
                    idynutils.moveit_robot_model->getActiveJointModels()[i]->getName());

    for(unsigned int i = 0; i < idynutils.getJointNames().size(); ++i)
        ASSERT_EQ(  idynutils.iDyn3_model.getDOFIndex(idynutils.getJointNames()[i]),
                    i);

    std::vector<std::string> expected_kinematic_chains;
    expected_kinematic_chains.push_back("left_arm");
    expected_kinematic_chains.push_back("right_arm");
    expected_kinematic_chains.push_back("left_leg");
    expected_kinematic_chains.push_back("right_leg");
    expected_kinematic_chains.push_back("torso");

    EXPECT_TRUE(idynutils.left_arm.chain_name == expected_kinematic_chains[0]);
    EXPECT_TRUE(idynutils.right_arm.chain_name == expected_kinematic_chains[1]);
    EXPECT_TRUE(idynutils.left_leg.chain_name == expected_kinematic_chains[2]);
    EXPECT_TRUE(idynutils.right_leg.chain_name == expected_kinematic_chains[3]);
    EXPECT_TRUE(idynutils.torso.chain_name == expected_kinematic_chains[4]);

    std::vector<std::string> expected_joints_left_arm;
    expected_joints_left_arm.push_back("LShSag");
    expected_joints_left_arm.push_back("LShLat");
    expected_joints_left_arm.push_back("LShYaw");
    expected_joints_left_arm.push_back("LElbj");
    expected_joints_left_arm.push_back("LForearmPlate");
    expected_joints_left_arm.push_back("LWrj1");
    expected_joints_left_arm.push_back("LWrj2");
    std::vector<std::string> expected_fixed_joints_left_arm;
    expected_fixed_joints_left_arm.push_back("l_wrist_joint");
//    expected_fixed_joints_left_arm.push_back("l_arm_ft_joint");
    std::vector<std::string> expected_joints_right_arm;
    expected_joints_right_arm.push_back("RShSag");
    expected_joints_right_arm.push_back("RShLat");
    expected_joints_right_arm.push_back("RShYaw");
    expected_joints_right_arm.push_back("RElbj");
    expected_joints_right_arm.push_back("RForearmPlate");
    expected_joints_right_arm.push_back("RWrj1");
    expected_joints_right_arm.push_back("RWrj2");
    std::vector<std::string> expected_fixed_joints_right_arm;
    expected_fixed_joints_right_arm.push_back("r_wrist_joint");
//    expected_fixed_joints_right_arm.push_back("r_arm_ft_joint");
    std::vector<std::string> expected_joints_torso;
    expected_joints_torso.push_back("WaistLat");
    expected_joints_torso.push_back("WaistSag");
    expected_joints_torso.push_back("WaistYaw");
    std::vector<std::string> expected_fixed_joints_torso;
//    expected_fixed_joints_torso.push_back("torso_joint");
    std::vector<std::string> expected_joints_left_leg;
    expected_joints_left_leg.push_back("LHipLat");
    expected_joints_left_leg.push_back("LHipYaw");
    expected_joints_left_leg.push_back("LHipSag");
    expected_joints_left_leg.push_back("LKneeSag");
    expected_joints_left_leg.push_back("LAnkSag");
    expected_joints_left_leg.push_back("LAnkLat");
    std::vector<std::string> expected_fixed_joints_left_leg;
    expected_fixed_joints_left_leg.push_back("l_sole_joint");
//    expected_fixed_joints_left_leg.push_back("l_leg_ft_joint");
    std::vector<std::string> expected_joints_right_leg;
    expected_joints_right_leg.push_back("RHipLat");
    expected_joints_right_leg.push_back("RHipYaw");
    expected_joints_right_leg.push_back("RHipSag");
    expected_joints_right_leg.push_back("RKneeSag");
    expected_joints_right_leg.push_back("RAnkSag");
    expected_joints_right_leg.push_back("RAnkLat");
    std::vector<std::string> expected_fixed_joints_right_leg;
    expected_fixed_joints_right_leg.push_back("r_sole_joint");
//    expected_fixed_joints_right_leg.push_back("r_leg_ft_joint");

    for(unsigned int i = 0; i < expected_joints_left_arm.size(); ++i){
        EXPECT_TRUE(expected_joints_left_arm[i] == idynutils.left_arm.joint_names[i]);
        EXPECT_TRUE(expected_joints_right_arm[i] == idynutils.right_arm.joint_names[i]);}
    for(unsigned int i = 0; i < expected_fixed_joints_left_arm.size(); ++i){
        EXPECT_TRUE(expected_fixed_joints_left_arm[i] == idynutils.left_arm.fixed_joint_names[i]);
        EXPECT_TRUE(expected_fixed_joints_right_arm[i] == idynutils.right_arm.fixed_joint_names[i]);}
    for(unsigned int i = 0; i < expected_joints_torso.size(); ++i)
        EXPECT_TRUE(expected_joints_torso[i] == idynutils.torso.joint_names[i]);
    for(unsigned int i = 0; i < expected_fixed_joints_torso.size(); ++i)
        EXPECT_TRUE(expected_fixed_joints_torso[i] == idynutils.torso.fixed_joint_names[i]);
    for(unsigned int i = 0; i < expected_joints_left_leg.size(); ++i){
        EXPECT_TRUE(expected_joints_left_leg[i] == idynutils.left_leg.joint_names[i]);
        EXPECT_TRUE(expected_joints_right_leg[i] == idynutils.right_leg.joint_names[i]);}
    for(unsigned int i = 0; i < expected_fixed_joints_left_leg.size(); ++i){
        EXPECT_TRUE(expected_fixed_joints_left_leg[i] == idynutils.left_leg.fixed_joint_names[i]);
        EXPECT_TRUE(expected_fixed_joints_right_leg[i] == idynutils.right_leg.fixed_joint_names[i]);}

    for(unsigned int i = 0; i < idynutils.getJointNames().size(); ++i){
        bool found = false;

        std::vector<std::string>::iterator it = std::find(idynutils.left_arm.joint_names.begin(),
                                                          idynutils.left_arm.joint_names.end(),
                                                          idynutils.getJointNames()[i]);
        if (it != idynutils.left_arm.joint_names.end())
            found = true;

        if(!found)
        {
            it = std::find(idynutils.right_arm.joint_names.begin(), idynutils.right_arm.joint_names.end(),
                           idynutils.getJointNames()[i]);
            if (it != idynutils.right_arm.joint_names.end())
                found = true;
        }

        if(!found)
        {
            it = std::find(idynutils.right_leg.joint_names.begin(), idynutils.right_leg.joint_names.end(),
                           idynutils.getJointNames()[i]);
            if (it != idynutils.right_leg.joint_names.end())
                found = true;
        }

        if(!found)
        {
            it = std::find(idynutils.left_leg.joint_names.begin(), idynutils.left_leg.joint_names.end(),
                           idynutils.getJointNames()[i]);
            if (it != idynutils.left_leg.joint_names.end())
                found = true;
        }

        if(!found)
        {
            it = std::find(idynutils.torso.joint_names.begin(), idynutils.torso.joint_names.end(),
                           idynutils.getJointNames()[i]);
            if (it != idynutils.torso.joint_names.end())
                found = true;
        }

        if(!found)
        {
            it = std::find(idynutils.head.joint_names.begin(), idynutils.head.joint_names.end(),
                           idynutils.getJointNames()[i]);
            if (it != idynutils.head.joint_names.end())
                found = true;
        }

        EXPECT_TRUE(found) << "Joint " << idynutils.getJointNames()[i] << " not found";
    }

        std::vector<std::string> fixed_joints;
        fixed_joints.push_back("base_joint");
        fixed_joints.push_back("l_ankle_joint");
        fixed_joints.push_back("l_sole_joint");
        fixed_joints.push_back("l_foot_lower_left_joint");
        fixed_joints.push_back("l_foot_lower_right_joint");
        fixed_joints.push_back("l_foot_upper_left_joint");
        fixed_joints.push_back("l_foot_upper_right_joint");
        fixed_joints.push_back("l_toe_joint");
        fixed_joints.push_back("LRaisingSupport_joint");
        fixed_joints.push_back("l_leg_ft_joint");
        fixed_joints.push_back("r_ankle_joint");
        fixed_joints.push_back("r_sole_joint");
        fixed_joints.push_back("r_foot_lower_left_joint");
        fixed_joints.push_back("r_foot_lower_right_joint");
        fixed_joints.push_back("r_foot_upper_left_joint");
        fixed_joints.push_back("r_foot_upper_right_joint");
        fixed_joints.push_back("r_toe_joint");
        fixed_joints.push_back("RRaisingSupport_joint");
        fixed_joints.push_back("r_leg_ft_joint");
        fixed_joints.push_back("l_arm_ft_joint");
        fixed_joints.push_back("l_handj");
        fixed_joints.push_back("l_handj2");
        fixed_joints.push_back("l_handj3");
        fixed_joints.push_back("l_hand_lower_left_joint");
        fixed_joints.push_back("l_hand_lower_right_joint");
        fixed_joints.push_back("l_hand_upper_left_joint");
        fixed_joints.push_back("l_hand_upper_right_joint");
        fixed_joints.push_back("l_forearm_backward_contact_joint");
        fixed_joints.push_back("l_forearm_central_contact_joint");
        fixed_joints.push_back("l_forearm_forward_contact_joint");
        fixed_joints.push_back("l_wrist_joint");
        fixed_joints.push_back("r_arm_ft_joint");
        fixed_joints.push_back("r_handj");
        fixed_joints.push_back("r_handj2");
        fixed_joints.push_back("r_handj3");
        fixed_joints.push_back("r_hand_lower_left_joint");
        fixed_joints.push_back("r_hand_lower_right_joint");
        fixed_joints.push_back("r_hand_upper_left_joint");
        fixed_joints.push_back("r_hand_upper_right_joint");
        fixed_joints.push_back("r_forearm_backward_contact_joint");
        fixed_joints.push_back("r_forearm_central_contact_joint");
        fixed_joints.push_back("r_forearm_forward_contact_joint");
        fixed_joints.push_back("r_wrist_joint");
        fixed_joints.push_back("neck_joint");
        fixed_joints.push_back("center_bottom_led_frame_joint");
        fixed_joints.push_back("center_top_led_frame_joint");
        fixed_joints.push_back("head_imu_joint");
        fixed_joints.push_back("hokuyo_joint");
        fixed_joints.push_back("head_hokuyo_joint");
        fixed_joints.push_back("left_camera_frame_joint");
        fixed_joints.push_back("left_camera_optical_frame_joint");
        fixed_joints.push_back("left_led_frame_joint");
        fixed_joints.push_back("right_camera_frame_joint");
        fixed_joints.push_back("right_camera_optical_frame_joint");
        fixed_joints.push_back("right_led_frame_joint");
        fixed_joints.push_back("torso_joint");
        fixed_joints.push_back("gaze_joint");
        fixed_joints.push_back("imu_joint");
        fixed_joints.push_back("imu_joint2");
        fixed_joints.push_back("backpack_joint");
        fixed_joints.push_back("torso_protection_joint");

        for(unsigned int i = 0; i < idynutils.getFixedJointNames().size(); ++i){
            std::vector<std::string>::iterator it = find(fixed_joints.begin(), fixed_joints.end(),
                    idynutils.getFixedJointNames()[i]);
            EXPECT_TRUE(it != fixed_joints.end())<<idynutils.getFixedJointNames()[i]<<" not found at index "<<i<<std::endl;}


      std::vector<std::string> ft_names = idynutils.getForceTorqueFrameNames();
      std::vector<std::string> ground_truth_ft_names;
      ground_truth_ft_names.push_back("l_leg_ft");
      ground_truth_ft_names.push_back("r_leg_ft");
      ground_truth_ft_names.push_back("l_arm_ft");
      ground_truth_ft_names.push_back("r_arm_ft");

      EXPECT_EQ(ft_names.size(), ground_truth_ft_names.size());
      for(unsigned int i = 0; i < ft_names.size(); ++i)
      {
          bool a = false;
          if(std::find(ground_truth_ft_names.begin(),
                       ground_truth_ft_names.end(), ft_names[i]) !=
                       ground_truth_ft_names.end())
              a = true;
          EXPECT_TRUE(a);
      }

      std::vector<std::string> imu_names = idynutils.getIMUFrameNames();
      std::vector<std::string> ground_truth_imu_names;
      ground_truth_imu_names.push_back("imu_link");
      ground_truth_imu_names.push_back("imu_link2");
      ground_truth_imu_names.push_back("multisense/head_imu_link");

      EXPECT_EQ(imu_names.size(), ground_truth_imu_names.size());
      for(unsigned int i = 0; i < imu_names.size(); ++i)
      {
          bool a = false;
          if(std::find(ground_truth_imu_names.begin(),
                       ground_truth_imu_names.end(), imu_names[i]) !=
                       ground_truth_imu_names.end())
              a = true;
          EXPECT_TRUE(a);
      }
}

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

TEST_F(testIDynUtils, testUpdateIdyn3ModelFT)
{
    yarp::sig::Vector q(this->iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector dq(q);
    yarp::sig::Vector ddq(q);

    std::vector<std::string> ft_reference_frames;
    ft_reference_frames.push_back("r_arm_ft");
    ft_reference_frames.push_back("l_arm_ft");
    ft_reference_frames.push_back("r_leg_ft");
    ft_reference_frames.push_back("l_leg_ft");


    std::vector<yarp::sig::Vector> ft_values;
    yarp::sig::Vector ft_value(6,0.0);
    for(unsigned int i = 0; i < ft_value.size(); ++i)
        ft_value[i] = i;
    ft_values.push_back(ft_value);
    for(unsigned int i = 0; i < ft_value.size(); ++i)
        ft_value[i] = 2.0*i;
    ft_values.push_back(ft_value);
    for(unsigned int i = 0; i < ft_value.size(); ++i)
        ft_value[i] = -3.0*i;
    ft_values.push_back(ft_value);
    for(unsigned int i = 0; i < ft_value.size(); ++i)
        ft_value[i] = 4.0*i;
    ft_values.push_back(ft_value);

    std::vector<iDynUtils::ft_measure> ft_measurements;
    for(unsigned int i = 0; i < 4; ++i){
        iDynUtils::ft_measure ft_measurement;

        ft_measurement.first = ft_reference_frames[i];
        if(i == 2) //For the third measure we want to change the sign!
            ft_measurement.second = -1.0*ft_values[i];
        else
            ft_measurement.second = ft_values[i];

        ft_measurements.push_back(ft_measurement);
    }

    this->updateiDyn3Model(q, dq, ddq, ft_measurements);

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
    this->updateiDyn3Model(q, dq, ddq, ft_measurements, true);

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
    this->updateiDyn3Model(q, dq, ddq, ft_measurements, true);

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


    for(unsigned int i = 0; i < 4; ++i)
    {
        const moveit::core::LinkModel* ft_link = moveit_robot_model->getLinkModel(ft_reference_frames[i]);
        int ft_index = iDyn3_model.getFTSensorIndex(ft_link->getParentJointModel()->getName());

        yarp::sig::Vector ft(6, 0.0);
        EXPECT_TRUE(this->iDyn3_model.getSensorMeasurement(ft_index, ft));

        for(unsigned int j = 0; j < 6; ++j){
            if(i == 2)
                EXPECT_DOUBLE_EQ(ft_values[i][j], -1.0*ft[j]);
            else
                EXPECT_DOUBLE_EQ(ft_values[i][j], ft[j]);
        }
    }

    EXPECT_TRUE(this->getNrOfFTSensors() == ft_measurements.size());


}

TEST_F(testIDynUtils, testCheckSelfCollision)
{
    std::string urdf_file = std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"coman/coman.urdf";
    std::string srdf_file = std::string(IDYNUTILS_TESTS_ROBOTS_DIR) + "coman/coman.srdf";

    iDynUtils idynutils("coman", urdf_file, srdf_file);
    q = idynutils.iDyn3_model.getAng();
    q[idynutils.iDyn3_model.getDOFIndex("RShLat")] = 0.15;
    q[idynutils.iDyn3_model.getDOFIndex("LShLat")] = -0.15;
    idynutils.updateiDyn3Model(q, true);

    double begin = yarp::os::Time::now();
    EXPECT_TRUE(idynutils.checkSelfCollision());
    std::cout << "Single self-collision (during collision) detection took "
              << yarp::os::Time::now() - begin << std::endl;

    q = idynutils.iDyn3_model.getAng();
    q[idynutils.iDyn3_model.getDOFIndex("RShLat")] = -0.15;
    q[idynutils.iDyn3_model.getDOFIndex("LShLat")] = 0.15;
    begin = yarp::os::Time::now();
    EXPECT_FALSE(idynutils.checkSelfCollisionAt(q));
    std::cout << "Single self-collision (not in collision) detection took "
              << yarp::os::Time::now() - begin << std::endl;
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

// TODO check updating of iDyn, most important utility functions are "fast"
TEST_F(testIDynUtils, checkTimings)
{

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
    iDynUtils idynutils1("coman",
                         std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                         std::string(IDYNUTILS_TESTS_ROBOTS_DIR) + "coman/coman.srdf");
    iDynUtils idynutils2("coman",
                         std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                         std::string(IDYNUTILS_TESTS_ROBOTS_DIR) + "coman/coman.srdf");
    idynutils2.iDyn3_model.setFloatingBaseLink(idynutils2.left_leg.index);

    yarp::sig::Vector q(idynutils1.iDyn3_model.getNrOfDOFs(), 0.0);
    for(unsigned int j = 0; j < 100; ++j) {
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
                EXPECT_NEAR(w_T_lf(i,j), w_T_bl2(i,j), 1E-12);
            }
        }

        yarp::sig::Matrix w_T_rh = idynutils1.iDyn3_model.getPosition(idynutils1.right_arm.index);
        yarp::sig::Matrix w_T_rh2 = idynutils2.iDyn3_model.getPosition(idynutils2.right_arm.index);

        for(unsigned int i = 0; i < 4; ++i)
        {
            for(unsigned int j = 0; j < 4; ++j)
            {
                EXPECT_NEAR(w_T_rh(i,j), w_T_rh2(i,j), 1E-12);
            }
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
            EXPECT_NEAR(w_T_lh(i,j), w_T_lh2(i,j), 1E-12);
        }
    }

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(w_T_CoM(i), w_T_CoM2(i), 1E-12);

}

TEST_F(testIDynUtils, testSupportPolygon)
{
    std::list<std::string> links_in_contact;
    links_in_contact.push_back("l_foot_lower_left_link");
    links_in_contact.push_back("l_foot_lower_right_link");
    links_in_contact.push_back("l_foot_upper_left_link");
    links_in_contact.push_back("l_foot_upper_right_link");
    links_in_contact.push_back("r_foot_lower_left_link");
    links_in_contact.push_back("r_foot_lower_right_link");
    links_in_contact.push_back("r_foot_upper_left_link");
    links_in_contact.push_back("r_foot_upper_right_link");

    std::list<std::string> idyn_links_in_contact = this->getLinksInContact();

    std::list<std::string>::iterator it2 = idyn_links_in_contact.begin();
    for(std::list<std::string>::iterator it = links_in_contact.begin(); it != links_in_contact.end(); it++){
        EXPECT_TRUE(*it == *it2);
        it2++;}

    std::list<std::string> new_links_in_contact;
    new_links_in_contact.push_back("l_foot_lower_left_link");
    new_links_in_contact.push_back("l_foot_lower_right_link");
    new_links_in_contact.push_back("l_foot_upper_left_link");

    this->setLinksInContact(new_links_in_contact);

    std::list<std::string>::const_iterator it3 = this->getLinksInContact().begin();
    for(std::list<std::string>::iterator it = new_links_in_contact.begin(); it != new_links_in_contact.end(); it++){
        EXPECT_TRUE(*it == *it3);
        it3++;}
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

TEST_P(testIDynUtilsWithAndWithoutUpdateAndDifferentSwitchTypes, testAnchorSwitchWGetPosition)
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

TEST_P(testIDynUtilsWithAndWithoutUpdateAndDifferentSwitchTypes, testAnchorSwitchWGetCoM)
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

TEST_P(testIDynUtilsWithAndWithoutUpdate, testAnchorSwitchConsistency)
{
    bool updateIDynAfterSwitch = GetParam();

    iDynUtils normal_model("coman",
                           std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                           std::string(IDYNUTILS_TESTS_ROBOTS_DIR) + "coman/coman.srdf");
    iDynUtils com_model("coman",
                        std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                        std::string(IDYNUTILS_TESTS_ROBOTS_DIR) + "coman/coman.srdf");

    setGoodInitialPosition();

    yarp::sig::Vector pos = this->iDyn3_model.getAng();

    normal_model.updateiDyn3Model(pos,true);
    com_model.updateiDyn3Model(pos,true);
    com_model.setFloatingBaseLink("l_sole");
    if(updateIDynAfterSwitch)
        com_model.updateiDyn3Model(pos, true);

    std::cout << "[COM from com_model] " << com_model.iDyn3_model.getCOM().toString() << std::endl;
    std::cout << "[COM from model] " << normal_model.iDyn3_model.getCOM().toString() << std::endl;
    std::cout << "[left_foot from model] " << normal_model.iDyn3_model.getPosition(left_leg.end_effector_index).submatrix(0,2,3,3).getCol(0).toString() << std::endl;

    EXPECT_FALSE(normal_model.iDyn3_model.getPosition(left_leg.end_effector_index).submatrix(0,2,3,3).getCol(0) == yarp::sig::Vector(3,0.0));
    EXPECT_FALSE(com_model.iDyn3_model.getPosition(left_leg.end_effector_index).submatrix(0,2,3,3).getCol(0) == yarp::sig::Vector(3,0.0));

    EXPECT_TRUE((com_model.iDyn3_model.getCOMKDL()-normal_model.iDyn3_model.getCOMKDL()).Norm() < 1E-9);
    EXPECT_TRUE(norm2(com_model.iDyn3_model.getCOM()-normal_model.iDyn3_model.getCOM()) < 1E-9);

    for(unsigned int i = 0; i < 10; ++i) {
        pos[left_arm.joint_numbers[0]] = pos[left_arm.joint_numbers[0]] + .1;
        normal_model.updateiDyn3Model(pos,true);
        com_model.updateiDyn3Model(pos,true);
        EXPECT_TRUE((com_model.iDyn3_model.getCOMKDL()-normal_model.iDyn3_model.getCOMKDL()).Norm() < 1E-9);
        EXPECT_TRUE(norm2(com_model.iDyn3_model.getCOM()-normal_model.iDyn3_model.getCOM()) < 1E-9);
    }
}

TEST_P(testIDynUtilsWithAndWithoutUpdateAndWithFootSwitching, testWalking)
{
    bool updateIDynAfterSwitch = GetParam().first;
    walkingTestStartingFoot startingFootParam = GetParam().second;

    int startingFoot, followingFoot;
    std::string startingFootName, followingFootName;
    if(startingFootParam == START_WITH_LEFT) {
        startingFoot = left_leg.end_effector_index;
        startingFootName = left_leg.end_effector_name;
        followingFoot = right_leg.end_effector_index;
        followingFootName = right_leg.end_effector_name;
    } else if(startingFootParam == START_WITH_RIGHT) {
        startingFoot = right_leg.end_effector_index;
        startingFootName = right_leg.end_effector_name;
        followingFoot = left_leg.end_effector_index;
        followingFootName = left_leg.end_effector_name;
    } else ASSERT_TRUE(false);

    iDynUtils normal_model("coman",
                           std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                           std::string(IDYNUTILS_TESTS_ROBOTS_DIR) + "coman/coman.srdf");
    setGoodInitialPosition();

    yarp::sig::Vector q_whole = this->iDyn3_model.getAng();

    normal_model.updateiDyn3Model(q_whole,true);

    yarp::sig::Matrix x;
    yarp::sig::Matrix x_ref;
    yarp::sig::Vector positionError(3,0.0);
    yarp::sig::Vector orientationError(3,0.0);

    yarp::sig::Matrix J;
    yarp::sig::Vector b;

    int foot = -1; int anchor = -1;
    std::string footName, anchorName;
    unsigned int n_steps = 0;

    // try making 9 steps
    for(n_steps = 0; n_steps < 9; ++n_steps) {
        // we start with the foot specified by parameter
        if(n_steps % 2 == 0) {
            foot = startingFoot;
            footName = startingFootName;
            anchor = followingFoot;
            anchorName = followingFootName;
        }
        else {
            foot = followingFoot;
            footName = followingFootName;
            anchor = startingFoot;
            anchorName = startingFootName;
        }

        KDL::Frame footBeforeSwitch = normal_model.iDyn3_model.getPositionKDL(foot);
        KDL::Frame anchorBeforeSwitch = normal_model.iDyn3_model.getPositionKDL(anchor);

        std::cout << "Step "   << n_steps+1 << ": "    << footName   << " pose before switch"
                  << std::endl << footBeforeSwitch   << std::endl;
        std::cout << "Step "   << n_steps+1 << ": "    << anchorName << " pose before switch"
                  << std::endl << anchorBeforeSwitch << std::endl;

        normal_model.switchAnchorAndFloatingBase(anchorName);
        std::cout   << "---------------------------------" << std::endl
                    << "Switched anchor to " << anchorName << std::endl
                    << "---------------------------------" << std::endl;
        if(updateIDynAfterSwitch)
            normal_model.updateiDyn3Model(q_whole, true);

        KDL::Frame footAfterSwitch = normal_model.iDyn3_model.getPositionKDL(foot);
        KDL::Frame anchorAfterSwitch = normal_model.iDyn3_model.getPositionKDL(anchor);

        std::cout << "Step "   << n_steps+1 << ": "   << footName   << " pose after switch"
                  << std::endl << footAfterSwitch   << std::endl;
        std::cout << "Step "   << n_steps+1 << ": "   << anchorName << " pose after switch"
                  << std::endl << anchorAfterSwitch << std::endl;

        EXPECT_TRUE(footAfterSwitch == footBeforeSwitch);
        EXPECT_TRUE(anchorAfterSwitch == anchorBeforeSwitch);

        x = normal_model.iDyn3_model.getPosition(foot);
        x_ref = x;
        // stepping 10cm forward
        x_ref(0,3) = x_ref(0,3) + .01;

        std::cout << "--------------------------------------" << std::endl
                  << "Moving " << footName << " 10cm forward" << std::endl
                  << "--------------------------------------" << std::endl;

        int iterations = 0;

        do {
            ++iterations;
            normal_model.updateiDyn3Model(q_whole,true);

            x = normal_model.iDyn3_model.getPosition(foot);

            cartesian_utils::computeCartesianError(x, x_ref,
                                                   positionError, orientationError);
            b = cat(positionError, -1.0*orientationError);

            normal_model.iDyn3_model.getJacobian(foot, J);
            J.removeCols(0,6);
            q_whole += pinv(J,1E-7)*0.1*b;
            normal_model.updateiDyn3Model(q_whole,true);

            //std::cout << "e" << iterations << " = " << x_ref(0,3) - x(0,3) << std::endl;

            anchorAfterSwitch = normal_model.iDyn3_model.getPositionKDL(anchor);
            EXPECT_TRUE(anchorAfterSwitch == anchorBeforeSwitch);

        } while (norm(b) > 1e-10 && iterations < 1000);
        ASSERT_TRUE (iterations < 1000) << "IK did not converge after 1000 iterations. Stopping";
    }

    std::cout << "Step "   << n_steps+1 << ": "   << footName   << " pose after last step"
              << std::endl << normal_model.iDyn3_model.getPositionKDL(foot)   << std::endl;
    std::cout << "Step "   << n_steps+1 << ": "   << anchorName << " pose after last step"
              << std::endl << normal_model.iDyn3_model.getPositionKDL(anchor) << std::endl;
}

INSTANTIATE_TEST_CASE_P(SwitchAnchor,
                        testIDynUtilsWithAndWithoutUpdateAndDifferentSwitchTypes,
                        ::testing::Values(std::make_pair(true,SWITCH_ANCHOR),
                                          std::make_pair(false,SWITCH_ANCHOR),
                                          std::make_pair(true,SWITCH_FLOATING_BASE),
                                          std::make_pair(false,SWITCH_FLOATING_BASE),
                                          std::make_pair(true,SWITCH_BOTH),
                                          std::make_pair(false,SWITCH_BOTH)));

INSTANTIATE_TEST_CASE_P(CheckCoMConsistency,
                        testIDynUtilsWithAndWithoutUpdate,
                        ::testing::Values(true,false));

INSTANTIATE_TEST_CASE_P(CheckWorldConsistencyByWalking9Steps,
                        testIDynUtilsWithAndWithoutUpdateAndWithFootSwitching,
                        ::testing::Values(std::make_pair(true,START_WITH_LEFT),
                                          std::make_pair(false,START_WITH_LEFT),
                                          std::make_pair(true,START_WITH_RIGHT),
                                          std::make_pair(false,START_WITH_RIGHT)));


} //namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
