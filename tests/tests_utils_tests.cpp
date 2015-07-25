#include <gtest/gtest.h>
#include <idynutils/tests_utils.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/os/Time.h>
#include <idynutils/idynutils.h>


namespace{

class testsTestsUtils: public ::testing::Test, public tests_utils
{
protected:
    testsTestsUtils()
    {

    }

    virtual ~testsTestsUtils() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

TEST_F(testsTestsUtils, testGetRandomAngle)
{
    for(unsigned int i = 0; i < 1000; ++i) {
        double alpha = this->getRandomAngle();
        EXPECT_TRUE( (alpha >= -M_PI) && (alpha < M_PI));
    }
}

TEST_F(testsTestsUtils, testDifferenceGetRandomAngle)
{
    double alpha = this->getRandomAngle();
    double beta = this->getRandomAngle();

    EXPECT_FALSE(alpha == beta);
}

TEST_F(testsTestsUtils, testLargeInversion)
{
    yarp::sig::Matrix A(150, 150);
    this->getRandomAngle();
    for(unsigned int i = 0; i < A.rows(); ++i)
    {
        for(unsigned int j = 0; j < A.cols(); ++j)
            A(i,j) = drand48();
    }

    yarp::sig::Matrix invA(A.rows(), A.cols());

    double tic = yarp::os::Time::now();
    invA = yarp::math::luinv(A);
    double toc = yarp::os::Time::now();

    double t = toc - tic;
    std::cout<<"Time to invert A[150 x 150] is "<<t<<"[s]"<<std::endl;
}

TEST_F(testsTestsUtils, testGetRandomAngleBetweenMinMax)
{
    double min = -1.0 * fabs(getRandomAngle());
    double max = fabs(getRandomAngle());

    for(unsigned int i = 0; i < 1000; ++i) {
        double angle = getRandomAngle(min, max);
        EXPECT_TRUE(angle >= min && angle <= max);
    }
}

TEST_F(testsTestsUtils, testGetRandomAngles)
{
    yarp::sig::Vector min(3,0.0);
    yarp::sig::Vector max(3,0.0);
    for(unsigned int i = 0; i < 3; ++i) {
        min[i] = -1.0 * fabs(getRandomAngle());
        max[i] = fabs(getRandomAngle());
    }

    yarp::sig::Vector q = getRandomAngles(min, max, 3);

    for(unsigned int i = 0; i < 3; ++i) {
        EXPECT_TRUE(q[i] >= min[i] && q[i] <= max[i]);
    }
}

TEST_F(testsTestsUtils, testGetRandomLength)
{
    double min = -1.0 * fabs(drand48());
    double max = fabs(drand48());

    double length = getRandomLength(min, max);

    EXPECT_TRUE(length >= min && length <= max);
}

TEST_F(testsTestsUtils, testGetRandomVector)
{
    double min = -1.0 * fabs(drand48());
    double max = fabs(drand48());

    KDL::Vector v = getRandomVector(min, max);

    EXPECT_TRUE(v[0] >= min && v[0] <= max);
    EXPECT_TRUE(v[1] >= min && v[1] <= max);
    EXPECT_TRUE(v[2] >= min && v[2] <= max);
}

TEST_F(testsTestsUtils, testGetRandomRotation)
{
    double min = -1.0 * fabs(getRandomAngle());
    double max = fabs(getRandomAngle());

    for(unsigned int i = 0; i < 1000; ++i) {
        KDL::Rotation rot = getRandomRotation(min, max);

        double R,P,Y;
        rot.GetRPY(R,P,Y);
        EXPECT_TRUE(R >= min && R <= max);
        EXPECT_TRUE(P >= min && P <= max);
        EXPECT_TRUE(Y >= min && Y <= max);
    }
}

TEST_F(testsTestsUtils, testGetRandomFrame)
{
    double lengthMin = -1.0 * fabs(drand48());
    double lengthMax = fabs(drand48());
    double rotMin = -1.0 * fabs(getRandomAngle());
    double rotMax = fabs(getRandomAngle());

    for(unsigned int i = 0; i < 1000; ++i) {
        KDL::Frame f = getRandomFrame(lengthMin, lengthMax,
                                      rotMin,    rotMax);

        double R,P,Y;
        f.M.GetRPY(R,P,Y);
        EXPECT_TRUE(f.p[0] >= lengthMin && f.p[0] <= lengthMax);
        EXPECT_TRUE(f.p[1] >= lengthMin && f.p[1] <= lengthMax);
        EXPECT_TRUE(f.p[2] >= lengthMin && f.p[2] <= lengthMax);
        EXPECT_TRUE(R >= rotMin && R <= rotMax) <<
            "R is "      << R <<
            " which not between " << rotMin << " and " << rotMax;
        EXPECT_TRUE(P >= rotMin && P <= rotMax)  <<
            "P is "      << P <<
            " which not between " << rotMin << " and " << rotMax;
        EXPECT_TRUE(Y >= rotMin && Y <= rotMax) <<
            "Y is "      << Y <<
            " which not between " << rotMin << " and " << rotMax;
    }
}

TEST_F(testsTestsUtils, testIDYNUTILS_TESTS_ROBOTS_DIR)
{
    std::cout<<"IDYNUTILS_TESTS_ROBOTS_DIR = "+std::string(IDYNUTILS_TESTS_ROBOTS_DIR)<<std::endl;
    EXPECT_FALSE(std::string(IDYNUTILS_TESTS_ROBOTS_DIR) == "");
    iDynUtils robot("bigman",
          std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
          std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf");
}

TEST_F(testsTestsUtils, testInitializeIfNeeded)
{
    initializeIfNeeded();
    double a1 = drand48();
    int b1 = rand();
    initializeIfNeeded();
    double a2 = drand48();
    int b2 = rand();

    EXPECT_FALSE(a1 == a2);
    EXPECT_FALSE(b1 == b2);
}

TEST_F(testsTestsUtils, testStartStopYarpServer)
{
    yarp::os::Network yarp_network;

    if(!yarp_network.checkNetwork())
    {
        EXPECT_TRUE(tests_utils::startYarpServer());

        unsigned int number_of_trials = 1000;
        bool check = false;
        for(unsigned int i = 0; i < number_of_trials; ++i)
        {
            if(!yarp_network.checkNetwork())
                std::cout<<"Checking Yarp Network..."<<std::endl;
            else
            {
                check = true;
                break;
            }
        }
        EXPECT_TRUE(check);

        EXPECT_TRUE(tests_utils::stopYarpServer());
        for(unsigned int i = 0; i < number_of_trials; ++i)
        {
            if(yarp_network.checkNetwork())
                std::cout<<"Checking Yarp Network shut down..."<<std::endl;
            else
            {
                check = false;
                break;
            }
        }
        EXPECT_FALSE(check);
    }
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
