#include <gtest/gtest.h>
#include <idynutils/convex_hull.h>

namespace {


class testConvexHull: public ::testing::Test
{

protected:

    testConvexHull()
    {

    }

    virtual ~testConvexHull() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

TEST_F(testConvexHull, testPointsOrdering)
{
    iDynUtils idynutils("coman");
    idynutils::convex_hull huller;
    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch;

    idynutils.getSupportPolygonPoints(points,"COM");
    ASSERT_TRUE(points.size() > 2);

    huller.getConvexHull(points,ch);

    for(std::vector<KDL::Vector>::iterator it = ch.begin();
        it!=ch.end();
        ++it) {

        std::vector<KDL::Vector> adjacent_points;
        std::vector<KDL::Vector> other_points;

        std::vector<KDL::Vector>::iterator explore_fwd = it;
        std::vector<KDL::Vector>::iterator explore_bwd = it;
        adjacent_points.push_back(*explore_fwd);
        ++explore_fwd;
        if(explore_fwd == ch.end()) adjacent_points.push_back(ch.front());
        else adjacent_points.push_back(*explore_fwd);

        while(explore_fwd != ch.end())
        {
            ++explore_fwd;
            other_points.push_back(*explore_fwd);
        }

        while(explore_bwd != ch.begin())
        {
            --explore_bwd;
            other_points.push_back(*explore_bwd);
        }

        ASSERT_EQ(ch.size(), adjacent_points.size()+other_points.size());

        KDL::Vector2 lineCoefficients; double lineOffset;

        double x1 = adjacent_points[0].x();
        double x2 = adjacent_points[1].x();
        double y1 = adjacent_points[0].y();
        double y2 = adjacent_points[1].y();

        a = y1 - y2;
        b = x2 - x1;
        c = -b*y1 -a*x1;

        lineCoefficients[0] = a;
        lineCoefficients[1] = b;
        lineOffset = c;

        bool otherPointsAreOnTheSameSide = false;
        // can be -1 or +1
        int side = -1;
        KDL::Vector last_points = other_points.back(); other_points.popopopo
        KDL::Vector2
    }
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
