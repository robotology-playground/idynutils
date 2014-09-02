#include <gtest/gtest.h>
#include <drc_shared/tests_utils.h>

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

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
