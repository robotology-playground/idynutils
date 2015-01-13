#include <gtest/gtest.h>
#include <idynutils/yarp_single_chain_interface.h>
#include <idynutils/comanutils.h>
#include <yarp/math/Math.h>
#include <kdl/frames_io.hpp>
#include <memory>


using namespace yarp::math;

namespace {
    // TODO create Yarp server, gazebo
    class testYSCI: public ::testing::Test
    {
    protected:
        std::shared_ptr<ComanUtils> coman;
        ::yarp::os::Network yarp_network;

        testYSCI() {
            coman = std::shared_ptr<ComanUtils>(new ComanUtils("TestRobotUtils"));
        }

        virtual ~testYSCI() {
            coman.reset();
        }

        virtual void SetUp() {
        }

        virtual void TearDown() {
        }
    };

    TEST_F(testYSCI, testSetReferenceSpeed)
    {
		double rad = 1.0; double sec = 1.0;
        ASSERT_TRUE(coman->left_hand.isAvailable) << "This test requires to be run "
                                                  << "with a \"Coman with hands\"" << std::endl;
        yarp::sig::Vector q_left_arm;
		double desiredSpeed = 0.1;
		double desiredDisplacement = 1*rad;
		double maximumHandAperture = .3*rad;
		double finalHandClosure = desiredDisplacement+maximumHandAperture;

        std::cout << "Setting left arm and left hand "
                  << "in the expected position" << std::endl;

        coman->left_hand.setPositionMode();
		coman->left_hand.setReferenceSpeed(.3);
        coman->left_hand.move(yarp::sig::Vector(1,maximumHandAperture));

        coman->left_arm.setPositionMode();
		coman->left_arm.setReferenceSpeed(.3);
        q_left_arm = coman->left_arm.sensePosition();
        q_left_arm[0] = 0.0;
        coman->left_arm.move(q_left_arm);

        yarp::os::Time::delay(7);
        //EXPECT_TRUE(coman->left_hand.moveDone());
        //EXPECT_TRUE(coman->left_arm.moveDone());

		coman->left_hand.setReferenceSpeed(desiredSpeed);
        coman->left_hand.move(yarp::sig::Vector(1,maximumHandAperture+desiredDisplacement));

        q_left_arm[0] = desiredDisplacement;
		coman->left_arm.setReferenceSpeed(desiredSpeed);
        coman->left_arm.move(q_left_arm);
        double t_start, t;

        yarp::os::Time::delay(.5);
        t_start = yarp::os::Time::now();
        bool hand_reached = false;
        bool arm_reached = false;
		double requiredTime = desiredDisplacement/desiredSpeed;
        do {
            t = yarp::os::Time::now() - t_start;

            if(!arm_reached && coman->left_arm.sensePosition()[0] > .98*desiredDisplacement) {
                arm_reached = true;
                ASSERT_TRUE( t <= 1.2*requiredTime &&
                             t >= .8*requiredTime) << "left_arm took "
                                            << t << " seconds to "
                                            << " complete movement. "
                                            << std::endl
                                            << "Should have finished"
                                            << " in " << requiredTime
                                            << "s";
            }

            if(!hand_reached && coman->left_hand.sensePosition()[0] > .98*finalHandClosure) {
                hand_reached = true;
                ASSERT_TRUE( t <= 1.2*requiredTime &&
                             t >= .8*requiredTime) << "left_hand took "
                                            << t << " seconds to "
                                            << " complete movement. "
                                            << std::endl
                                            << "Should have finished"
                                            << " in " << requiredTime
                                            << "s";
            }
        } while(t <= requiredTime*1.5);
        EXPECT_TRUE(coman->left_arm.sensePosition()[0] > .98*desiredDisplacement) << "left_arm still did not "
                                                               << "complete a movement after 4 seconds. "
                                                               << "Stuck at "
                                                               << coman->left_arm.sensePosition()[0];
        EXPECT_TRUE(coman->left_hand.sensePosition()[0] > .98*finalHandClosure) << "left_hand still did not "
                                                                << "complete a movement after 4 seconds. "
                                                                << "Stuck at "
                                                                << coman->left_hand.sensePosition()[0];

    }
} //namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
