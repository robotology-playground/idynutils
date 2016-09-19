/* File : pydynutils.i */
%module pydynutils
%include std_string.i
%include std_vector.i
%include std_map.i
%include std_pair.i
/* %include <boost_shared_ptr.i> */

%{
/* Note : always include headers following the inheritance order */
/* #include <boost/shared_ptr.hpp> */
#include "iCub/iDynTree/DynTree.h"
#include "idynutils/idynutils.h"
#include "idynutils/yarp_single_chain_interface.h"
#include "idynutils/ControlType.hpp"
#include "idynutils/RobotUtils.h"
#include <kdl/frames.hpp>
%}

/* Note : always include headers following the inheritance order */
%ignore RobotUtils::setImpedance(const yarp::sig::Vector& Kq, const yarp::sig::Vector& Dq);
%ignore RobotUtils::getImpedance(yarp::sig::Vector& Kq, yarp::sig::Vector& Dq);
%ignore RobotUtils::setImpedance(const ImpedanceMap& impedance_map);
%ignore RobotUtils::getImpedance(ImpedanceMap& impedance_map);
%ignore RobotUtils::sense(yarp::sig::Vector& q, yarp::sig::Vector& qdot, yarp::sig::Vector& tau);
%ignore iDynUtils::fromJointStateMsgToiDyn(const sensor_msgs::JointStateConstPtr& msg);
%ignore iDynUtils::updateiDyn3ModelFromJoinStateMsg(const sensor_msgs::JointStateConstPtr& msg);
%ignore iDynUtils::urdf_model; // A URDF Model TODO check
%ignore iDynUtils::robot_srdf; // A SRDF description TODO check
%ignore iDynUtils::moveit_robot_model; // A robot model
%ignore iDynUtils::moveit_planning_scene;
%ignore iDynUtils::moveit_collision_robot;
%ignore iDynUtils::checkCollisionWithWorld();
%ignore iDynUtils::checkCollisionWithWorldAt(const yarp::sig::Vector &q);
%ignore iDynUtils::resetOccupancyMap();
%ignore iDynUtils::updateOccupancyMap(const octomap_msgs::Octomap& octomapMsg);
%ignore iDynUtils::updateOccupancyMap(const octomap_msgs::OctomapWithPose& octomapMsgWithPose);
%ignore iDynUtils::updateOccupancyMap(const octomap_msgs::Octomap& octomapMsg, const yarp::sig::Vector& q);
%ignore iDynUtils::updateOccupancyMap(const octomap_msgs::OctomapWithPose& octomapMsgWithPose, const yarp::sig::Vector& q);
%ignore iDynUtils::checkSelfCollision();
%ignore iDynUtils::checkSelfCollisionAt(const yarp::sig::Vector &q,
                             std::list< std::pair<std::string,std::string> > * collisionPairs = NULL);
%ignore iDynUtils::checkCollision();
%ignore iDynUtils::hasOccupancyMap();
%ignore iDynUtils::getPlanningSceneMsg();
%ignore iDynUtils::getDisplayRobotStateMsgAt(const yarp::sig::Vector &q);
%ignore iDynUtils::getDisplayRobotStateMsg();

%ignore iDynUtils::updateRobotState();


%ignore iDynUtils::robot_kdl_tree; // A KDL Tree
%ignore iDynUtils::setJointNumbers(kinematic_chain& chain);
%ignore iDynUtils::setChainIndex(std::string endeffector_name,kinematic_chain& chain);
%ignore iDynUtils::setControlledKinematicChainsJointNumbers();
%ignore iDynUtils::setJointNames();
%ignore iDynUtils::findGroupChain(const std::vector<std::string>& chain_list, const std::vector<srdf::Model::Group>& groups,std::string chain_name, int& group_index);
%ignore iDynUtils::setChainJointNames(const srdf::Model::Group& group, kinematic_chain& k_chain);
%ignore iDynUtils::iDyn3Model();
%ignore iDynUtils::updateRobotState(const yarp::sig::Vector &q);

%include "iCub/iDynTree/DynTree.h"
%include "idynutils/idynutils.h"
%include "idynutils/ControlType.hpp"
%include "idynutils/yarp_single_chain_interface.h"
%include "idynutils/RobotUtils.h"

namespace std {
  %template(NamesVector) vector<string>;
};

%feature("autodoc",3);
