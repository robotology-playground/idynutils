#include <gtest/gtest.h>
#include <drc_shared/tests_utils.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/os/Time.h>

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
    double alpha = this->getRandomAngle();
    EXPECT_TRUE( (alpha >= -M_PI) && (alpha < M_PI));
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

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
