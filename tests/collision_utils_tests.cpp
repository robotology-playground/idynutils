#include <gtest/gtest.h>
#include <idynutils/collision_utils.h>
#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cmath>
#include <fcl/distance.h>
#include <fcl/shape/geometric_shapes.h>
#include <eigen_conversions/eigen_kdl.h>


#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define toRad(X) (X * M_PI/180.0)
#define SMALL_NUM 1e-5

class TestCapsuleLinksDistance
{

private:
    ComputeLinksDistance& _computeDistance;

public:

    TestCapsuleLinksDistance(ComputeLinksDistance& computeDistance)
        :_computeDistance(computeDistance)
    {

    }

    std::map<std::string,boost::shared_ptr<fcl::CollisionGeometry> > getShapes()
    {
        return _computeDistance.shapes_;
    }

    std::map<std::string,boost::shared_ptr<fcl::CollisionObject> > getcollision_objects()
    {
        return _computeDistance.collision_objects_;
    }

    std::map<std::string,KDL::Frame> getlink_T_shape()
    {
        return _computeDistance.link_T_shape;
    }

};

namespace {


class testCollisionUtils : public ::testing::Test{
 protected:

  testCollisionUtils():
      robot("bigman",
            std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
            std::string(IDYNUTILS_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf"),
      q(robot.iDyn3_model.getNrOfDOFs(), 0.0),
      compute_distance(robot)
  {}

  virtual ~testCollisionUtils() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

  iDynUtils robot;
  yarp::sig::Vector q;
  ComputeLinksDistance compute_distance;

};

yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils) {
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 20.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
}

double dist3D_Segment_to_Segment (const Eigen::Vector3d & S1P0, const Eigen::Vector3d & S1P1, const Eigen::Vector3d & S2P0, const Eigen::Vector3d & S2P1, Eigen::Vector3d & CP1, Eigen::Vector3d & CP2)
{

    using namespace Eigen;

    Vector3d   u = S1P1 - S1P0;
    Vector3d   v = S2P1 - S2P0;
    Vector3d   w = S1P0 - S2P0;
    double    a = u.dot(u);         // always >= 0
    double    b = u.dot(v);
    double    c = v.dot(v);         // always >= 0
    double    d = u.dot(w);
    double    e = v.dot(w);
    double    D = a*c - b*b;        // always >= 0
    double    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    double    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) { // the lines are almost parallel
        sN = 0.0;         // force using point P0 on segment S1
        sD = 1.0;         // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                 // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (std::fabs(sN) < SMALL_NUM ? 0.0 : sN / sD);
    tc = (std::fabs(tN) < SMALL_NUM ? 0.0 : tN / tD);

    CP1 = S1P0 + sc * u;
    CP2 = S2P0 + tc * v;

//    std::cout << "CP1: " << std::endl << CP1 << std::endl;
//    std::cout << "CP2: " << std::endl << CP2 << std::endl;

    // get the difference of the two closest points
    Vector3d   dP = CP1 - CP2;  // =  S1(sc) - S2(tc)

    double Dm = dP.norm();   // return the closest distance

    // I leave the line here for observing the minimum distance between the inner line segments of the corresponding capsule pair
    //std::cout << "Dm: " << std::endl << Dm << std::endl;

    return Dm;
}


  TEST_F(testCollisionUtils, testCapsuleDistance) {

        q = getGoodInitialPosition(robot);
        robot.updateiDyn3Model(q, false);

        std::list<std::pair<std::string,std::string>> whiteList;
        whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RSoftHandLink"));
        compute_distance.setCollisionWhiteList(whiteList);

        std::list<LinkPairDistance> results = compute_distance.getLinkDistances();
        std::list<LinkPairDistance>::iterator it = results.begin();
        double actual_distance;
        actual_distance = it->getDistance();


        TestCapsuleLinksDistance testCapsuleLinksDistance(compute_distance);
        std::map<std::string,boost::shared_ptr<fcl::CollisionGeometry> > shapes_test;
        std::map<std::string,boost::shared_ptr<fcl::CollisionObject> > collision_objects_test;
        std::map<std::string,KDL::Frame> link_T_shape_test;

        shapes_test = testCapsuleLinksDistance.getShapes();
        collision_objects_test = testCapsuleLinksDistance.getcollision_objects();
        link_T_shape_test = testCapsuleLinksDistance.getlink_T_shape();

        boost::shared_ptr<fcl::CollisionObject> collision_geometry_l = collision_objects_test["LSoftHandLink"];
        boost::shared_ptr<fcl::CollisionObject> collision_geometry_r = collision_objects_test["RSoftHandLink"];

        KDL::Frame link_T_shape_l = link_T_shape_test["LSoftHandLink"];
        KDL::Frame link_T_shape_r = link_T_shape_test["RSoftHandLink"];

        std::string lefthand_name = "LSoftHandLink";
        int lefthand_index = robot.iDyn3_model.getLinkIndex(lefthand_name);
        if(lefthand_index == -1)
            std::cout << "Failed to get lefthand_index" << std::endl;

        std::string righthand_name = "RSoftHandLink";
        int righthand_index = robot.iDyn3_model.getLinkIndex(righthand_name);
        if(righthand_index == -1)
            std::cout << "Failed to get righthand_index" << std::endl;

        KDL::Frame lefthand_position = robot.iDyn3_model.getPositionKDL(lefthand_index);
        KDL::Frame righthand_position = robot.iDyn3_model.getPositionKDL(righthand_index);

        KDL::Frame lefthand_shape_transform = lefthand_position * link_T_shape_l;
        KDL::Frame righthand_shape_transform = righthand_position * link_T_shape_r;

        KDL::Vector lefthand_shape_origin = lefthand_shape_transform.p;
        KDL::Vector righthand_shape_origin = righthand_shape_transform.p;

        KDL::Vector lefthand_shape_Z = lefthand_shape_transform.M.UnitZ();
        KDL::Vector righthand_shape_Z = righthand_shape_transform.M.UnitZ();


        boost::shared_ptr<fcl::CollisionGeometry> leftH = shapes_test["LSoftHandLink"];
        boost::shared_ptr<fcl::CollisionGeometry> rightH = shapes_test["RSoftHandLink"];

        boost::shared_ptr<fcl::Capsule> leftH_capsule = boost::dynamic_pointer_cast<fcl::Capsule>(leftH);
        boost::shared_ptr<fcl::Capsule> rightH_capsule = boost::dynamic_pointer_cast<fcl::Capsule>(rightH);

        double origin_translation_lefthand = 0.5*leftH_capsule->lz;
        double origin_translation_righthand = 0.5*rightH_capsule->lz;

        KDL::Vector lefthand_capsule_A, lefthand_capsule_B, righthand_capsule_A, righthand_capsule_B;
        lefthand_capsule_A = lefthand_shape_origin + origin_translation_lefthand * lefthand_shape_Z;
        lefthand_capsule_B = lefthand_shape_origin - origin_translation_lefthand * lefthand_shape_Z;
        righthand_capsule_A = righthand_shape_origin + origin_translation_righthand * righthand_shape_Z;
        righthand_capsule_B = righthand_shape_origin - origin_translation_righthand * righthand_shape_Z;

        Eigen::Matrix<double, 3, 1> lefthand_capsule_A_eigen, lefthand_capsule_B_eigen, righthand_capsule_A_eigen, righthand_capsule_B_eigen;
        Eigen::Vector3d lefthand_CP, righthand_CP;
        double reference_distance;

        tf::vectorKDLToEigen(lefthand_capsule_A, lefthand_capsule_A_eigen);
        tf::vectorKDLToEigen(lefthand_capsule_B, lefthand_capsule_B_eigen);
        tf::vectorKDLToEigen(righthand_capsule_A, righthand_capsule_A_eigen);
        tf::vectorKDLToEigen(righthand_capsule_B, righthand_capsule_B_eigen);

        reference_distance = dist3D_Segment_to_Segment (lefthand_capsule_A_eigen, lefthand_capsule_B_eigen, righthand_capsule_A_eigen, righthand_capsule_B_eigen, lefthand_CP, righthand_CP);

        reference_distance = reference_distance - leftH_capsule->radius - rightH_capsule->radius;

        EXPECT_NEAR(actual_distance, reference_distance, 1E-4);


  }



}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

