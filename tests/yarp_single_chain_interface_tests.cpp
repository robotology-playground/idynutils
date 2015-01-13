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
        ASSERT_TRUE(coman->left_hand.isAvailable) << "This test requires to be run "
                                                  << "with a \"Coman with hands\"" << std::endl;
        yarp::sig::Vector q_left_arm;

        std::cout << "Setting left arm and left hand "
                  << "in the expected position" << std::endl;

        yarp::os::Time::delay(1);

        coman->left_hand.setPositionDirectMode();
        coman->left_hand.move(yarp::sig::Vector(1,0.0));

        coman->left_arm.setPositionDirectMode();
        q_left_arm = coman->left_arm.sensePosition();
        q_left_arm[0] = 0.0;
        coman->left_arm.move(q_left_arm);

        yarp::os::Time::delay(1.5);
        ASSERT_TRUE(coman->left_hand.moveDone());
        ASSERT_TRUE(coman->left_arm.moveDone());

        coman->left_hand.setPositionMode();
        coman->left_hand.setReferenceSpeed(.3);
        coman->left_hand.move(yarp::sig::Vector(1,1.0));

        q_left_arm[0] = 1;
        coman->left_arm.setPositionMode();
        coman->left_arm.setReferenceSpeed(.3);
        coman->left_arm.move(q_left_arm);
        double t_start, t;

        yarp::os::Time::delay(.1);
        t_start = yarp::os::Time::now();
        bool hand_reached = false;
        bool arm_reached = false;
        do {
            t = yarp::os::Time::now() - t_start;

            if(!arm_reached && coman->left_arm.sensePosition()[0] > 0.98) {
                arm_reached = true;
                ASSERT_TRUE( t <= 1.2*1.0/.3 &&
                             t >= .8*1.0/.3) << "left_arm took "
                                            << t << " seconds to "
                                            << " complete movement. "
                                            << std::endl
                                            << "Should have finished"
                                            << " in " << 1.0/.3
                                            << "s";
            }

            if(!hand_reached && coman->left_hand.sensePosition()[0] > 0.98) {
                hand_reached = true;
                ASSERT_TRUE( t <= 1.2*1.0/.3 &&
                             t >= .8*1.0/.3) << "left_hand took "
                                            << t << " seconds to "
                                            << " complete movement. "
                                            << std::endl
                                            << "Should have finished"
                                            << " in " << 1.0/.3
                                            << "s";
            }
        } while(t <= 4);
        EXPECT_TRUE(coman->left_arm.sensePosition()[0] > 0.98) << "left_arm still did not "
                                                               << "complete a movement after 4 seconds. "
                                                               << "Stuck at "
                                                               << coman->left_arm.sensePosition()[0];
        EXPECT_TRUE(coman->left_hand.sensePosition()[0] > 0.98) << "left_hand still did not "
                                                                << "complete a movement after 4 seconds. "
                                                                << "Stuck at "
                                                                << coman->left_hand.sensePosition()[0];

    }
} //namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
