/*
 * Copyright (C) 2014 Walkman
 * Author: Mirko Ferrati, Enrico Mingo, Alessio Rocchi
 * email:  mirko.ferrati@gmail.com, enrico.mingo@iit.it, alessio.rocchi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef IDYNUTILS_H
#define IDYNUTILS_H

#include <urdf/model.h>
#include <iCub/iDynTree/DynTree.h>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/robot_model/robot_model.h>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>


struct kinematic_chain
{
public:
    kinematic_chain(std::string chain_name):chain_name(chain_name)
    {

    }
  std::string chain_name;
  std::string name;
  std::vector<std::string> joint_names;
  int index;
  std::vector<unsigned int> joint_numbers;
};

class iDynUtils
{
public:
    iDynUtils();
    kinematic_chain left_leg, left_arm,right_leg,right_arm,torso;
    iCub::iDynTree::DynTree coman_iDyn3; // iDyn3 Model

    void fromRobotToIDyn(const yarp::sig::Vector& q_chain,
                         yarp::sig::Vector& q_out,
                         kinematic_chain& chain);

    /**
     * @brief updateiDyn3Model updates the underlying robot model
     * @param q robot configuration
     * @param set_world_pose do we update the base link pose wrt the world frame?
     * @param support_foot what is the support foot link name in single stance mode?
     * @TODO in the future we should use the IMU + rgbdslam + FK
     */
    void updateiDyn3Model(const yarp::sig::Vector& q,
                          const bool set_world_pose = false,
                          const std::string& support_foot = "l_sole");

    /**
     * @brief updateiDyn3Model updates the underlying robot model
     * @param q robot configuration
     * @param dq robot joint velocities
     * @param set_world_pose do we update the base link pose wrt the world frame?
     * @param support_foot what is the support foot link name in single stance mode?
     * @TODO in the future we should use the IMU + rgbdslam + FK
     */
    void updateiDyn3Model(const yarp::sig::Vector& q,
                          const yarp::sig::Vector& dq,
                          const bool set_world_pose = false,
                          const std::string& support_foot = "l_sole");

    /**
     * @brief updateiDyn3Model updates the underlying robot model
     * @param q robot configuration
     * @param dq_ref robot joint velocities
     * @param ddq_ref robot joint accelerations
     * @param set_world_pose do we update the base link pose wrt the world frame?
     * @param support_foot what is the support foot link name in single stance mode?
     * @TODO in the future we should use the IMU + rgbdslam + FK
     */
    void updateiDyn3Model(const yarp::sig::Vector& q,
                          const yarp::sig::Vector& dq_ref,
                          const yarp::sig::Vector& ddq_ref,
                          const bool set_world_pose = false,
                          const std::string& support_foot = "l_sole");

    /**
     * @brief setWorldPose calls setWorldPose() after updating the iDyn3 model
     * @param q the robot configuration
     * @param dq_ref the robot velocities
     * @param ddq_ref the robot accelerations
     * @param support_foot
     */
    void setWorldPose(const yarp::sig::Vector& q,
                      const yarp::sig::Vector& dq_ref,
                      const yarp::sig::Vector& ddq_ref,
                      const std::string& support_foot = "l_sole");

    /**
     * @brief setWorldPose updates the transformation bTw from the world frame {W} to the base link {B},
     *                     which corresponds to the floating base configuration. This is done by taking a link,
     *                     and computing the z-distance (in world frame) between the link frame and the base link frame.
     *                     The x and y coordinates of the original frame remain unchanged.
     * @param support_foot the link name wrt which we compute the z-offset. By default, l_sole.
     *                     This should be, in general, the support foot.
     */
    void setWorldPose(const std::string& support_foot = "l_sole");

    yarp::sig::Matrix getSimpleChainJacobian(const kinematic_chain chain, bool world_frame=false);
    boost::shared_ptr<urdf::Model> coman_model; // A URDF Model
    robot_model::RobotModelPtr coman_robot_model; // A robot model

    std::vector<std::string> joint_names;

private:
    KDL::Tree coman_tree; // A KDL Tree

    void setJointNumbers(kinematic_chain& chain);
    void setChainIndex(std::string endeffector_name,kinematic_chain& chain);
    void setControlledKinematicChainsLinkIndex();
    void setControlledKinematicChainsJointNumbers();
    void setJointNames();
    void iDyn3Model();

    yarp::sig::Vector zeros;

    yarp::sig::Matrix worldT;
    boost::shared_ptr<srdf::Model> coman_srdf; // A SRDF description


};

#endif // IDYNUTILS_H
