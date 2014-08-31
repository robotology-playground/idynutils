#include <gtest/gtest.h>
#include <drc_shared/idynutils.h>

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


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
