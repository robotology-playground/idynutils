/*
 * Copyright (C) 2014 Walkman
 * Author: Mirko Ferrati, Enrico Mingo, Alessio Rocchi, Federico Moro
 * email:  mirko.ferrati@gmail.com, enrico.mingo@iit.it, alessio.rocchi@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
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

    /**
     * @brief getNrOfDOFs return # of dofs of the kinematic chain
     * @return # of dofs of the kinematic chain
     */
    unsigned int getNrOfDOFs(){return joint_numbers.size();}

   /**
   * @brief chain_name is the name of the kinematic chain
   *  e.g.
   *           kinematic_chain right_arm("right arm");
   */
  std::string chain_name;
  /**
   * @brief end_effector_name is the name of the end effector
   */
  std::string end_effector_name;
  std::vector<std::string> joint_names;
  /**
   * @brief end_effector_index index of the end effector
   */
  int end_effector_index; int &index = end_effector_index;

  std::vector<unsigned int> joint_numbers;
};

class iDynUtils
{
public:
    iDynUtils(std::string robot_name_="coman");
    kinematic_chain left_leg, left_arm,right_leg,right_arm,torso;
    iCub::iDynTree::DynTree iDyn3_model; // iDyn3 Model THIS HAS TO BE RENAMED!!!

    /**
     * @brief fromRobotToIDyn update q_chain values in q_out using joint numbers of chain.
     * @param q_chain vector of joint values in robot order
     * @param q_out whole body vector of joint values in model order
     * @param chain joints to update using q_chain in q_out
     */
    void fromRobotToIDyn(const yarp::sig::Vector& q_chain,
                         yarp::sig::Vector& q_out,
                         kinematic_chain& chain);

    /**
     * @brief fromIDynToRobot update q_chain values in q_out using joint numbers of chain.
     * @param q input whole body joint values vector in model order
     * @param q_chain_out vector of joint values for each chain in robot order
     * @param chain joints to update using q_chain in q_out
     */
    void fromIDynToRobot(const yarp::sig::Vector& q,
                         yarp::sig::Vector& q_chain_out,
                         kinematic_chain& chain);

    /**
     * @brief updateiDyn3Model updates the underlying robot model (uses both Kinematic and Dynamic RNEA)
     * @param q robot configuration
     * @param set_world_pose do we update the base link pose wrt the world frame?
     * @param support_foot what is the support foot link name in single stance mode?
     * @TODO in the future we should use the IMU + rgbdslam + FK
     */
    void updateiDyn3Model(const yarp::sig::Vector& q,
                          const bool set_world_pose = false,
                          const std::string& support_foot = "l_sole");

    /**
     * @brief updateiDyn3Model updates the underlying robot model (uses both Kinematic and Dynamic RNEA)
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
     * @brief updateiDyn3Model updates the underlying robot model (uses both Kinematic and Dynamic RNEA)
     * NOTE:
     *          if the IMU is NOT used we are assuming l_sole as inertial frame and a flat ground!
     *
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
     * @brief initWorldPose inits the inertial frame putting it on the plane idenfitied by
     *                      the anchor link, on the point located by the projection of the
     *                      base link position on that same plane
     * @param anchor        the anchor link
     */
    void initWorldPose(const std::string &anchor = "l_sole");

    /**
     * @brief updateWorldPose updates the world pose relative to the current robot state,
     *        using anchor and offset calculated by the initWorldPose function
     */
    void updateWorldPose();

    /**
     * @brief *DEPRECATED* setWorldPose calls setWorldPose() after updating the iDyn3 model
     * @param q the robot configuration
     * @param dq_ref the robot velocities
     * @param ddq_ref the robot accelerations
     * @param anchor the link name wrt which we compute the z-offset. By default, l_sole.
     *                     This should be, in general, the support foot.
     */
    KDL::Frame setWorldPose(const yarp::sig::Vector& q,
                            const yarp::sig::Vector& dq_ref,
                            const yarp::sig::Vector& ddq_ref,
                            const std::string& anchor = "l_sole");

    /**
     * @brief setWorldPose updates the transformation bTw from the world frame {W} to the base link {B},
     *                     which corresponds to the floating base configuration. This is done by taking a link,
     *                     and computing the z-distance (in world frame) between the link frame and the base link frame.
     *                     The x and y coordinates of the original frame remain unchanged.
     *                     Returns anchor_T_world, the offset betweent the anchor link and the
     *                     inertial frame
     * @param anchor the link name wrt which we compute the z-offset. By default, l_sole.
     *               This should be, in general, the support foot.
     */
    KDL::Frame setWorldPose(const std::string& anchor = "l_sole");

    /**
     * @brief setWorldPose updates the transformation bTw from the world frame {W} to the base link {B},
     *                     which corresponds to the floating base configuration. This is done by taking a link,
     *                     and computing the z-distance (in world frame) between the link frame and the base link frame.
     *                     The x and y coordinates of the original frame remain unchanged.
     * @param anchor_T_world the offset between the inertial frame and the anchor frame, expressed in the
     *                       anchor frame
     * @param anchor the link name wrt which we compute the z-offset. By default, l_sole.
     *               This should be, in general, the support foot.
     */
    void setWorldPose(const KDL::Frame& anchor_T_world, const std::string& anchor = "l_sole");

    yarp::sig::Matrix getSimpleChainJacobian(const kinematic_chain chain, bool world_frame=false);
    boost::shared_ptr<urdf::Model> urdf_model; // A URDF Model
    robot_model::RobotModelPtr moveit_robot_model; // A robot model

    const std::vector<std::string> &getJointNames() const;
    yarp::sig::Vector zeros;

    enum DefaultProjectorContacts {
        CONTACT_LEFT_FOOT  = 0x0001,
        CONTACT_RIGHT_FOOT = 0x0010,
        CONTACT_LEFT_HAND  = 0x0100,
        CONTACT_RIGHT_HAND = 0x1000
    };

    /**
     * @brief computeFloatingBaseProjector computes a projector for torques in floating base systems
     * @param contacts a list of contacts as defined in DefaultProjectorContacts
     * @return a R^{nx6+n} projector matrix that projects a R^{6+n} vector
     *         of generalized forces into joint torques
     */
    yarp::sig::Matrix computeFloatingBaseProjector(const int contacts);
    /**
     * @brief computeFloatingBaseProjector computes a projector for torques in floating base systems
     * @param JContacts a Jacobian of contact points expressed in the world frame coordinate system
     * @return a R^{nx6+n} projector matrix that projects a R^{6+n} vector
     *         of generalized forces into joint torques
     */
    yarp::sig::Matrix computeFloatingBaseProjector(const yarp::sig::Matrix& JContacts);

protected:
    std::vector<std::string> joint_names;
    KDL::Tree robot_kdl_tree; // A KDL Tree

    std::string anchor_name;    // last anchor used
    /**
     * @brief anchor_T_world CONSTANT Transformation between anchor and world frame
     */
    KDL::Frame anchor_T_world;  // offset between inertial frame and anchor link (e.g., l_sole)

    void setJointNumbers(kinematic_chain& chain);

    /**
     * @brief setChainIndex set an end effector to a kinematic chain
     * @param endeffector_name name of the ee
     * @param chain chain to set the ee
     * @return true if the name of the end effector is in the link list of iDyn3
     */
    bool setChainIndex(std::string endeffector_name,kinematic_chain& chain);

    /**
     * @brief setControlledKinematicChainsLinkIndex set end effectors name for all the controlled kinamtic chains
     * @return true if all the kinematic chains have an end effector
     */
    //bool setControlledKinematicChainsLinkIndex();


    void setControlledKinematicChainsJointNumbers();

    /**
     * @brief setJointNames get joint names from stored srdf
     * @return return true if all the needed kinematic chains are found
     */
    bool setJointNames();

    /**
     * @brief searches for a chain @chain_name into the @groups
     * @return true if a group exists
     * @param found_group the group with the name @chain_name
     */
    bool findGroupChain(const std::vector<std::string>& chain_list, const std::vector<srdf::Model::Group>& groups,std::string chain_name, int& group_index);
    
    /**
     * @brief sets the joint names for a single chain
     * 
     * @param group the group in which the chain is defined
     * @param k_chain the chain that will get the joint names
     * @return true if the group had a chain defined
     */
    bool setChainJointNames(const srdf::Model::Group& group, kinematic_chain& k_chain);
    
    
    /**
     * @brief iDyn3Model load robot urdf and srdf, setup iDynThree
     * @return return true if the model is loaded in iDynThree and urdf/srdf are correctly found
     */
    bool iDyn3Model();

    /**
     * @brief worldT Transformation between world and base_link
     */
    yarp::sig::Matrix worldT;
    boost::shared_ptr<srdf::Model> robot_srdf; // A SRDF description

    /**
     * @brief g gravity vector
     */
    yarp::sig::Vector g;

    std::string robot_name;
    std::string robot_urdf_folder;
    std::string robot_srdf_folder;
};

#endif // IDYNUTILS_H
