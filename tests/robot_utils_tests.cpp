#include <gtest/gtest.h>
#include <idynutils/RobotUtils.h>
#include <idynutils/comanutils.h>
#include <yarp/math/Math.h>
#include <kdl/frames_io.hpp>



using namespace yarp::math;

namespace {
    // TODO create Yarp server, gazebo
    class testRobotUtils: public ::testing::Test
    {
    protected:
        boost::shared_ptr<ComanUtils> coman;
        ::yarp::os::Network yarp_network;

        testRobotUtils() {
            coman = boost::shared_ptr<ComanUtils>(new ComanUtils("TestRobotUtils"));
        }

        virtual ~testRobotUtils() {
            coman.reset();
        }

        virtual void SetUp() {
        }

        virtual void TearDown() {
        }
    };

    TEST_F(testRobotUtils, testGetKinematicChains)
    {
        RobotUtils::KinematicChains k_chains = coman->getKinematicChains();
        EXPECT_TRUE(k_chains.size() == 5);
        for(RobotUtils::KinematicChains::iterator it_chains = k_chains.begin();
            it_chains != k_chains.end();
            ++it_chains) {
            RobotUtils::KinematicChainPtr& chain = *it_chains;
            std::cout << "testGetKinematicChains: Iterating over chain "
                      << chain->chain_name << std::endl;
        }
    }

    // TODO check sensors are correctly loaded
    TEST_F(testRobotUtils, checkSensorsAreLoaded)
    {
        EXPECT_TRUE(coman->getftSensors().size() == 4);
        EXPECT_TRUE(coman->hasftSensors());
        EXPECT_TRUE(coman->hasIMU());
        EXPECT_TRUE((bool)coman->getIMU());
    }

    // TODO check sense and move are "fast"
    TEST_F(testRobotUtils, checkTimings)
    {
        double ftReadingTime, senseTime, moveTime, globalTime;
        const unsigned int iterations = 1000;

        double t = yarp::os::Time::now();
        for(unsigned int i = 0; i < iterations; ++i) {
            RobotUtils::ftReadings readings = coman->senseftSensors();
        }
        ftReadingTime = yarp::os::Time::now() - t;
        ASSERT_TRUE(ftReadingTime < 20e-6*iterations) <<
            "each ft reading should take less than 20microsec" <<
            "but "  << iterations << " iterations of senseftSensors take "
                    << ftReadingTime << " [s]" << std::endl;

        yarp::sig::Vector q, q_dot, tau;
        t = yarp::os::Time::now();
        for(unsigned int i = 0; i < iterations; ++i) {
            coman->sense(q, q_dot, tau);
        }
        senseTime = yarp::os::Time::now() - t;
        ASSERT_TRUE(senseTime < 20e-6*iterations) <<
            "each sense should take less than 20microsec" <<
            "but " << iterations << " iterations of sense take "
                   << senseTime << " [s]" << std::endl;

        coman->setPositionDirectMode();
        t = yarp::os::Time::now();
        for(unsigned int i = 0; i < iterations; ++i) {
            coman->move(q);
        }
        moveTime = yarp::os::Time::now() - t;
        ASSERT_TRUE(moveTime < 150e-6*iterations) <<
            "each move should take less than 50microsec" <<
            "but " << iterations << " iterations of move take "
                   << moveTime << " [s]" << std::endl;

        t = yarp::os::Time::now();
        for(unsigned int i = 0; i < iterations; ++i) {
            coman->sense(q, q_dot, tau);
            RobotUtils::ftReadings readings = coman->senseftSensors();
            coman->move(q);
        }
        globalTime = yarp::os::Time::now() - t;
        ASSERT_TRUE(globalTime < 200e-6*iterations) <<
            "each sense, move should take less than 200microsec" <<
            "but " << iterations << " iterations of sense,senseftSensors,move take "
                   << globalTime << " [s]" << std::endl;
        coman->setIdleMode();
    }


} //namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
