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
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>

/**
 * @brief The kinematic_chain struct defines usefull objects related to a kinematic chain
 */
struct kinematic_chain
{
public:
    /**
     * @brief kinematic_chain constructor
     * @param chain_name name associated to the kinematic chain
     */
    kinematic_chain(std::string chain_name) :
        chain_name(chain_name),
        index(end_effector_index)
    {

    }

    /**
     * @brief getNrOfDOFs return # of dofs of the kinematic chain
     * @return # of dofs of the kinematic chain
     */
    unsigned int getNrOfDOFs() const;

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
  /**
   * @brief joint_names vector contains the name of DOFs in the kinematic chain
   */
  std::vector<std::string> joint_names;

  /**
   * @brief fixed_joint_names contains the name of fixed joints in the kinematic chain
   */
  std::vector<std::string> fixed_joint_names;

  /**
   * @brief end_effector_index index of the end effector
   */
  int end_effector_index; int& index; //int &index = end_effector_index;

  /**
   * @brief joint_numbers a vector of joint IDs for this kinematic chain. All the joint IDs are unique for the whole body.
   */
  std::vector<unsigned int> joint_numbers;
};

class iDynUtils
{
public:

    typedef std::string ft_reference_frame;
    typedef yarp::sig::Vector ft_value;
    typedef std::pair<ft_reference_frame, ft_value> ft_measure;

    /**
     * @brief iDynUtils constructor that uses <robot_name>_folder as path for the urdf and srdf.
     * Some assumptions are done in the constructor:
     *
     *      anchor_name = "l_sole"
     *
     * and these links are in contact with the environment:
     *
     *      "l_foot_lower_left_link"
     *      "l_foot_lower_right_link"
     *      "l_foot_upper_left_link"
     *      "l_foot_upper_right_link"
     *      "r_foot_lower_left_link"
     *      "r_foot_lower_right_link"
     *      "r_foot_upper_left_link"
     *      "r_foot_upper_right_link"
     *
     * @param robot_name_ is the robot name used by the idynutils
     * @param urdf_path is the path to the urdf file
     *   e.g. /home/enrico/my_robot/my_robot_urdf/my_robot.urdf
     * @param srdf_path is the path to the srdf file
     *   e.g. /home/enrico/my_robot/my_robot_srdf/my_robot.srdf
     */
    iDynUtils(const std::string robot_name_,
              const std::string urdf_path,
              const std::string srdf_path);

    kinematic_chain left_leg, left_arm,right_leg,right_arm,torso,head;
    iCub::iDynTree::DynTree iDyn3_model;

    /**
     * @brief getNrOfFTSensors return # of FT sensors in the model
     * @return # of FT sensors in the model
     */
    unsigned int getNrOfFTSensors() const { return _ft_sensor_frames.size();}

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
     * @brief fromJointStateMsgToiDyn returns a vector of joint angles from a joint state msg
     * @param msg the joint state msg
     * @return a vector of joint positions
     */
    yarp::sig::Vector fromJointStateMsgToiDyn(const sensor_msgs::JointStateConstPtr& msg);

    /**
     * @brief updateiDyn3ModelFromJoinStateMsg updates the internal joint state using a ROS joint state msg
     * @param msg the joint state msg
     */
    void updateiDyn3ModelFromJoinStateMsg(const sensor_msgs::JointStateConstPtr& msg);

    /**
     * @brief updateiDyn3Model updates the underlying robot model (uses both Kinematic and Dynamic RNEA)
     * @param q robot configuration
     * @param set_world_pose do we update the base link pose wrt the world frame?
     * @TODO in the future we should use the IMU + rgbdslam + FK
     */
    void updateiDyn3Model(const yarp::sig::Vector& q,
                          const bool set_world_pose = false);
    /**
     * @brief updateiDyn3Model updates the underlying robot model (uses both Kinematic and Dynamic RNEA)
     * @param q robot configuration
     * @param force_torque_measurement update force_torque stored in the idyntree model
     * @param set_world_pose do we update the base link pose wrt the world frame?
     *
     * NB: The order for which the function updateForceTorqueMeasurement() is called is
     *     important! The standard updateiDyn3Model() will call inside the function
     *     dynamicRNEA() that will overwrite the internal vector of measurement. For
     *     this reason we first call the calssical updateiDyn3Model() and then we
     *     call updateForceTorqueMeasurement() taht will set the FT measurements.
     */
    void updateiDyn3Model(const yarp::sig::Vector &q,
                          const std::vector<ft_measure> &force_torque_measurement,
                          const bool set_world_pose = false);

    /**
     * @brief updateiDyn3Model updates the underlying robot model (uses both Kinematic and Dynamic RNEA)
     * @param q robot configuration
     * @param dq robot joint velocities
     * @param set_world_pose do we update the base link pose wrt the world frame?
     * @TODO in the future we should use the IMU + rgbdslam + FK
     */
    void updateiDyn3Model(const yarp::sig::Vector& q,
                          const yarp::sig::Vector& dq,
                          const bool set_world_pose = false);

    /**
     * @brief updateiDyn3Model updates the underlying robot model (uses both Kinematic and Dynamic RNEA)
     * @param q robot configuration
     * @param dq robot joint velocities
     * @param force_torque_measurement update force_torque stored in the idyntree model
     * @param set_world_pose do we update the base link pose wrt the world frame?
     *
     * NB: The order for which the function updateForceTorqueMeasurement() is called is
     *     important! The standard updateiDyn3Model() will call inside the function
     *     dynamicRNEA() that will overwrite the internal vector of measurement. For
     *     this reason we first call the calssical updateiDyn3Model() and then we
     *     call updateForceTorqueMeasurement() taht will set the FT measurements.
     */
    void updateiDyn3Model(const yarp::sig::Vector &q,
                          const yarp::sig::Vector& dq,
                          const std::vector<ft_measure> &force_torque_measurement,
                          const bool set_world_pose = false);

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
                          const bool set_world_pose = false);

    /**
     * @brief updateiDyn3Model updates the underlying robot model (uses both Kinematic and Dynamic RNEA)
     * @param q robot configuration
     * @param dq robot joint velocities
     * @param ddq_ref robot joint accelerations
     * @param force_torque_measurement update force_torque stored in the idyntree model
     * @param set_world_pose do we update the base link pose wrt the world frame?
     *
     * NB: The order for which the function updateForceTorqueMeasurement() is called is
     *     important! The standard updateiDyn3Model() will call inside the function
     *     dynamicRNEA() that will overwrite the internal vector of measurement. For
     *     this reason we first call the calssical updateiDyn3Model() and then we
     *     call updateForceTorqueMeasurement() taht will set the FT measurements.
     */
    void updateiDyn3Model(const yarp::sig::Vector &q,
                          const yarp::sig::Vector& dq,
                          const yarp::sig::Vector& ddq_ref,
                          const std::vector<ft_measure> &force_torque_measurement,
                          const bool set_world_pose = false);


    boost::shared_ptr<urdf::Model> urdf_model; // A URDF Model
    boost::shared_ptr<srdf::Model> robot_srdf; // A SRDF description
    robot_model::RobotModelPtr moveit_robot_model; // A robot model
    robot_state::RobotStatePtr moveit_robot_state;

    boost::shared_ptr<collision_detection::CollisionRobot> moveit_collision_robot;
    collision_detection::AllowedCollisionMatrixPtr allowed_collision_matrix;

    /**
     * @brief getJointNames return a vector with ALL the joint names
     * @return a vector with ALL the joint names
     */
    const std::vector<std::string> &getJointNames() const;

    const std::vector<std::string> &getFixedJointNames() const;

    yarp::sig::Vector zeros;

    enum DefaultProjectorContacts {
        CONTACT_LEFT_FOOT  = 0x0001,
        CONTACT_RIGHT_FOOT = 0x0010,
        CONTACT_LEFT_HAND  = 0x0100,
        CONTACT_RIGHT_HAND = 0x1000
    };
    
    /**
     * @brief switchAnchor switch the anchor frame according to the new given anchor name.
     * The anchor name needs to be a valid frame name (i.e., a link name)
     * @param new_anchor name
     * @return true if all went well, false otherwise
     */
    bool switchAnchor(const std::string& new_anchor);

    /**
     * @brief getAnchor returns the name of the anchor frame (the body frame that does not vary
     * w.r.t. the world frame while performing the current robot motion). The anchor name corresponds
     * to the link name which frame system is the anchor frame.
     * @return the anchor name
     */
    const std::string getAnchor() const;

    /**
     * Set the floating base link. Updates the transform from anchor to world accordingly:
     * anchor_T_fb is updated with the new world and the old anchor, so that
     * anchor_T_fb_new = anchor_T_fb_old * fb_old_T_fb_new
     * @param new_base the name of the link which will be the new floating base (i.e.,
     * the 6dof unactuated virtual joints will connect the world frame to this base link)
     * @return true if all went well, false otherwise
     */
    bool setFloatingBaseLink(const std::string new_base);

    /**
     * @brief switchAnchorAndFloatingBase a shortcut version of calling
     *                                    switchAnchor(new_anchor); setFloatingBaseLink(new_anchor);
     * @param new_anchor name
     * @return true if all went well, false otherwise
     */
    bool switchAnchorAndFloatingBase(const std::string new_anchor);

    /**
     * @brief getWorldPose returns the anchor frame name (the frame which does not move w.r.t. the world frame)
     * and the transform between that anchor frame and the world frame. If the world has never been initialized,
     * (i.e. when updateWorld has always been called with updateWorld = false) we return false.
     * @param anchor_T_world the frame representing the transformation between anchor and world frames
     * @param anchor the anchor frame (link) name
     * @return true if the world frame has been initialized
     */
    bool getWorldPose(KDL::Frame& anchor_T_world, std::string& anchor) const;


    /**
     * @brief getAnchor_T_World returns the offset between the inertial frame and the anchor frame, expressed
     * in the anchor frame of reference (transform between anchor and world)
     * @return the frame representing the transformation between anchor and world frames
     */
    const KDL::Frame getAnchor_T_World() const;

    /**
     * @brief setAnchor_T_World allows to set the transform between the anchor and world frames.
     * Notice you need to call the update() function before anchor_T_World has an effect on the getPosition
     * @param anchor_T_world the current transform between anchor and world
     */
    void setAnchor_T_World(const KDL::Frame& anchor_T_world);

    /**
     * @brief getRobotName
     * @return robot_name
     */
    const std::string getRobotName() const;

    /**
     * @brief getRobotURDFFolder returns the full file name of the URDF model for the current robot
     * @return the robot urdf path (folder name + filename with extension)
     */
    const std::string getRobotURDFPath() const;

    /**
     * @brief getRobotSRDFFolder returns the path of the SRDF model for the current robot
     * @return the robot srdf path (folder name + filename with extension)
     */
    const std::string getRobotSRDFPath() const;

   /**
    * @brief getSupportPolygonPoints given a vector of reference frames that we consider in contact wit the ground,
    * it return a list of points express in the specified frame referenceFrame.
    * By default it is the frame COM, oriented like the world frame and with origin on the CoM
    * Notice the polygon points are not projected on the support surface.
    * @param points a list of points in the same reference frame
    * @param referenceFrame the string defining the reference frame in which to express the
    * support polygon points. The possibilities are:
    * - "COM"
    * - "world"
    * - {linkName}
    * @return false if the vector of reference frames is empty. True otherwise.
    */
   bool getSupportPolygonPoints(std::list<KDL::Vector>& points,
                                const std::string referenceFrame = "COM");

   const std::list<std::string>& getLinksInContact();

   void setLinksInContact(const std::list<std::string>& list_links_in_contact);

   /**
    * @brief checkSelfCollision checks whether the robot is in self collision - uses most accurate collision detection info (i.e., no capsules)
    * @return true if the robot is in self collision
    * @TODO we should move this in collision_utils together with loadDisabledCollisionsFromSRDF and checkSelfCollisionAt
    */
   bool checkSelfCollision();

   /**
    * @brief checkSelfCollisionAt checks whether the robot is in self collision at q - uses most accurate collision detection info (i.e., no capsules)
    * @param q the robot joint configuration vector
    * @param collisionPairs a list of pairs of string with link pairs in contact
    * @return  true if the robot is in self collision
    * @TODO we should move this in collision_utils together with loadDisabledCollisionsFromSRDF and checkSelfCollision
    * @TODO maybe the At version could be static
    */
   bool checkSelfCollisionAt(const yarp::sig::Vector &q,
                             std::list< std::pair<std::string,std::string> > * collisionPairs = NULL);

   /**
    * @brief loadDisabledCollisionsFromSRDF disabled collisions between links as specified in the robot srdf.
    *        Notice this function will not reset the acm, rather just disable collisions that are flagged as
    *        "disabled" in the robot srdf. The default robot srdf will be used - i.e. the one associate with
    *        the most accurate collision detection info (i.e., no capsules)
    * @param acm the allowed collision matrix to modify according to the srdf info
    * @TODO we should move this in collision_utils together with checkSelfCollision and checkSelfCollisionAt
    */
   void loadDisabledCollisionsFromSRDF(collision_detection::AllowedCollisionMatrixPtr acm);

   /**
    * @brief loadDisabledCollisionsFromSRDF disabled collisions between links as specified in the robot srdf.
    *        Notice this function will not reset the acm, rather just disable collisions that are flagged as
    *        "disabled" in the robot srdf
    * @param srdf the srdf file to use to load the ACM
    * @param acm the allowed collision matrix to modify according to the srdf info
    * @TODO we should move this in collision_utils together with checkSelfCollision and checkSelfCollisionAt
    */
   void loadDisabledCollisionsFromSRDF(srdf::Model& srdf,
                                       collision_detection::AllowedCollisionMatrixPtr acm);

   /**
    * @brief getDisplayRobotStateMsgAt
    * @param q the joint positions to transform into a DisplayRobotState msg
    * @return a DisplayRobotState msg
    * @TODO should be static - or moved some place else
    */
   moveit_msgs::DisplayRobotState getDisplayRobotStateMsgAt(const yarp::sig::Vector &q);

   /**
    * @brief getDisplayRobotStateMsg gets the robot state msg relative to the internal model state
    * @return a DisplayRobotState msg
    */
   moveit_msgs::DisplayRobotState getDisplayRobotStateMsg();


   const std::vector<std::string> getForceTorqueFrameNames(){return _ft_sensor_frames;}

   const std::vector<std::string> getIMUFrameNames(){return _imu_sensor_frames;}
   
   /**
    * @brief getBaseLink returns the base link as defined in the SRDF.
    * Notice that it is in general different from the floating base link, even though upon instantiation
    * a new iDynUtils object will have a coincident floating base link and base link.
    * @return the robot base link as defined in the SRDF
    */
   std::string getBaseLink();

protected:
    /**
     * @brief joint_names this vector contains ALL the active joint names
     */
    std::vector<std::string> joint_names;

    /**
     * @brief fixed_joint_names this vector contains ALL the fixed joint names
     */
    std::vector<std::string> fixed_joint_names;

    /**
     * @brief links_in_contact list of links (reference frames) that are in contact with the environment
     */
    std::list<std::string> links_in_contact;

    KDL::Tree robot_kdl_tree; // A KDL Tree

    std::string anchor_name;    // last anchor used
    /**
     * @brief anchor_T_world CONSTANT Transformation between anchor and world frame
     */
    KDL::Frame anchor_T_world;  // offset between inertial frame and anchor link (e.g., l_sole)

    void setJointNumbers(kinematic_chain& chain);

    /**
     * @brief initWorldPose inits the inertial frame putting it on the plane idenfitied by
     *                      the anchor link, on the point located by the projection of the
     *                      base link position on that same plane.
     * It will set the robot to a "nice" initial configuration and call setWorldPose(anchor)
     * @param anchor        the anchor link
     */
    void initWorldPose();

    /**
     * @brief updateWorldPose updates the world pose relative to the current robot state,
     *        using anchor and offset calculated by the initWorldPose function.
     * It will call setWorldPose(anchor_T_w,anchor)
     */
    void updateWorldPose();

    /**
     * @brief setChainIndex set an end effector to a kinematic chain
     * @param endeffector_name name of the ee
     * @param chain chain to set the ee
     * @return true if the name of the end effector is in the link list of iDyn3
     */
    bool setChainIndex(std::string endeffector_name,kinematic_chain& chain);

    void setControlledKinematicChainsJointNumbers();

    /**
     * @brief setJointNames get joint names from stored srdf
     * @return return true if at least a kinematic chain is found
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


    /**
     * @brief updateRobotState updates the internal moveit model.
     * Collision checking functions (i.e. checkSelfCollition() and checkSelfCollisionAt())
     * and conversion functions (i.e. getRobotStateMsg() , @getRobotStateMsgAt())
     * automatically update the robot state, so that it is not necessary
     * to call this function manually.
     * @param q the joint angles of the robot state
     */
    void updateRobotState(const yarp::sig::Vector &q);

    /**
     * @brief updateRobotState updates the internal moveit model. Takes joint angles form the iDyn3 model.
     * Collision checking functions (i.e. checkSelfCollition() and checkSelfCollisionAt())
     * and conversion functions (i.e. getRobotStateMsg() , @getRobotStateMsgAt())
     * automatically update the robot state, so that it is not necessary
     * to call this function manually.
     */
    void updateRobotState();

    bool updateForceTorqueMeasurement(const ft_measure& force_torque_measurement);

    bool readForceTorqueSensorsNames();

    bool readIMUSensorsNames();

    /**
     * @brief worldT Transformation between world and base_link
     */
    yarp::sig::Matrix worldT;

    /**
     * @brief g gravity vector
     */
    yarp::sig::Vector g;

    /**
     * @brief base_link_name is the link to which the floating base is attached during robot loading
     * Notice that, while the floating base link can be changed, the base_link_name will remain constant
     */
    std::string base_link_name;

    std::string robot_name;
    std::string robot_urdf_folder;
    std::string robot_srdf_folder;

    bool world_is_inited;
    
    std::vector<std::string> _ft_sensor_frames;
    std::vector<std::string> _imu_sensor_frames;
};

#endif // IDYNUTILS_H
