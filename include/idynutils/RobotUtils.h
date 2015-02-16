/*
 * Copyright (C) 2014 Walkman
 * Author: Mirko Ferrati, Enrico Mingo, Alessio Rocchi
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

#ifndef ROBOTUTILS_H
#define ROBOTUTILS_H

#include <idynutils/yarp_single_chain_interface.h>
#include <idynutils/idynutils.h>
#include <idynutils/yarp_IMU_interface.h>
#include <idynutils/yarp_ft_interface.h>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>

/**
 * @brief The RobotUtils class eases whole body control for the coman robot.
 */
class RobotUtils
{
public:
    typedef std::pair<yarp::sig::Vector, yarp::sig::Vector> Impedance;
    typedef std::map<std::string,  Impedance> ImpedanceMap;
    typedef std::map<std::string,  yarp::sig::Vector> VelocityMap;
    typedef std::shared_ptr<yarp_IMU_interface> IMUPtr;
    typedef std::shared_ptr<yarp_ft_interface> ftPtr;
    typedef std::map<std::string, ftPtr> ftPtrMap;
    typedef std::map<std::string, yarp::sig::Vector> ftReadings;
    typedef kinematic_chain* KinematicChainPtr;
    typedef std::list<KinematicChainPtr> KinematicChains;

    /**
     * @brief RobotUtils creates interfaces for all kinematic chains.
     * At creation control mode is not changed. Use the methods
     * setTorqueMode, setPositionDirectMode, setPositionMode, setImpedanceControlMode
     * to switch control mode on all chains, or call the chain methods to
     * switch mode for each kinematic chain.
     * @param moduleName the name of the module which uses the facility
     * @param robotName the name of the robot
     * @param urdf_path is the path to the urdf file
     *   e.g. /home/enrico/my_robot/my_robot_urdf/my_robot.urdf
     * @param srdf_path is the path to the srdf file
     *   e.g. /home/enrico/my_robot/my_robot_srdf/my_robot.srdf
     */
    RobotUtils( const std::string moduleName,
                const std::string robotName,
	        const std::string urdf_path = "",
	        const std::string srdf_path = "" );

    walkman::yarp_single_chain_interface right_hand, left_hand;
    walkman::yarp_single_chain_interface right_arm, left_arm;
    walkman::yarp_single_chain_interface torso;
    walkman::yarp_single_chain_interface right_leg, left_leg;
    iDynUtils idynutils;

    std::vector<std::string> ft_reference_frames;

    /**
     * @brief hasHands check whether both hands are available
     * @return true if connection to both hands is successful
     */
    bool hasHands();

    /**
     * @brief hasftSensors checks whether the robot provides and exposes at least a ft (Force/Torque) sensor
     * @return true if at least a ft is present and connection to the ft sensor is successful
     */
    bool hasftSensors();

    /**
     * @brief getftSensors returns a map of available force torque sensors
     * @return a map <std::string kinematic chain name, ftPtr ft>
     */
    ftPtrMap getftSensors();

    /**
     * @brief hasIMU checks whether the robot provides and exposes an IMU
     * @return true if IMU is present and connection to the IMU is successful
     */
    bool hasIMU();

    /**
     * @brief getIMU returns a pointer to the IMU, if present
     * @return a pointer the IMU, if present
     */
    IMUPtr getIMU();

    /**
     * @brief sense returns position, velocities, torques sensed by the robot
     * @param q
     * @param qdot
     * @param tau
     */
    void sense(yarp::sig::Vector& q,
               yarp::sig::Vector& qdot,
               yarp::sig::Vector& tau);

    /**
     * @brief sensePosition returns the position of the robot's joints
     * @return
     */
    yarp::sig::Vector& sensePosition();

    /**
     * @brief senseVelocity returns the velocities of the robot's joints
     * @return
     */
    yarp::sig::Vector& senseVelocity();

    /**
     * @brief senseTorque returns the torques of the robot's joints
     * @return
     */
    yarp::sig::Vector& senseTorque();

    /**
     * @brief senseftSensors senses all available ft sensors
     * @return a map <std::string chain name, yarp::sig::Vector ft Reading>
     */
    ftReadings& senseftSensors();

    /**
     * @brief senseftSensor senses the ft sensor on specified chain
     * @param chain the yarp single chain interface corresponding to
     *        the chain where the ft sensor is located.
     * @param ftReading the reading back from the ft sensor
     * @return true if succeed
     */
    bool senseftSensor(const std::string &ft_frame,
                       yarp::sig::Vector& ftReading);

    /**
     * @brief sensePosition returns the position of the robot's joints
     * @param q_left_hand a vector where the left hand position will be stored
     * @param q_right_hand a vector where the right hand position will be stored
     * @return true if hands are available
     */
    bool senseHandsPosition(yarp::sig::Vector &q_left_hand,
                            yarp::sig::Vector &q_right_hand);


    /**
     * @brief move send potision commands to all the robot joints (except the hands). Works only when the robot is in joint poisition control mode.
     * @param q the desired joint position vector
     */
    void move(const yarp::sig::Vector &q);
    
    bool moveDone();

    /**
     * @brief move send potision commands to the robot hands.
     * @param q_left_hand the desired joint position vector for the left hand
     * @param q_right_hand the desired joint position vector for the right hand
     * @return true if hands are available
     */
    bool moveHands(const yarp::sig::Vector &q_left_hand,
                   const yarp::sig::Vector &q_right_hand);

//    /**
//     * @brief move send inputs to all robot joints. The type of input depends on the control mode.
//     * @param u the joint input.  It can imply a position command or a torque command depending on the joint control mode used.
//     */
//    void move(const std::map<std::string,yarp::sig::Vector&> &u);

//    /**
//     * @brief move send potision commands and torque offsets to all the robot joints. Works when the robot is in joint impedance or position control mode.
//     * @param q the desired joint position vector
//     * @param torqueOffset_map a map <chain_name, offset_torques> of torque offsets to send to the robot.
//     * Chains that accept a torque offset should be controlled in joint impedance mode.
//     */
//    void move(const yarp::sig::Vector &q,
//              const std::map<std::string,yarp::sig::Vector&> &torqueOffset_map);

//    /**
//     * @brief move send potision commands and torque offsets to all the robot joints. Works when the robot is in joint impedance or position control mode.
//     * @param q the desired joint position vector
//     * @param kq a map <chain_name, kq> of joint stiffness references to send to the robot.
//     * Chains that accept a torque offset should be controlled in joint impedance mode.
//     * @param torqueOffset_map a map <chain_name, offset_torques> of torque offsets to send to the robot.
//     * Chains that accept a torque offset should be controlled in joint impedance mode.
//     */
//    void move(const yarp::sig::Vector &q,
//              const std::map<std::string,yarp::sig::Vector&> &kq_map,
//              const std::map<std::string,yarp::sig::Vector&> &torqueOffset_map);

//    /**
//     * @brief move send potision commands and torque offsets to all the robot joints. Works when the robot is in joint impedance or position control mode.
//     * @param q the desired joint position vector
//     * @param kq a map <chain_name, kq> of joint stiffness references to send to the robot.
//     * Chains that accept a torque offset should be controlled in joint impedance mode.
//     * @param kd a map <chain_name, kd> of joint damping references to send to the robot.
//     * Chains that accept a torque offset should be controlled in joint impedance mode.
//     * @param torqueOffset_map a map <chain_name, offset_torques> of torque offsets to send to the robot.
//     * Chains that accept a torque offset should be controlled in joint impedance mode.
//     */
//    void move(const yarp::sig::Vector &q,
//              const std::map<std::string,yarp::sig::Vector&> &kq_map,
//              const std::map<std::string,yarp::sig::Vector&> &kd_map,
//              const std::map<std::string,yarp::sig::Vector&> &torqueOffset_map);

    /**
     * @brief setReferenceSpeeds sets the reference joint speeds used in position mode when moving, for all joints exluding hands
     * @param maximum_velocity the maximum velocity vector, a \f$R^{n_{robot\_joints}}\f$ in \f$[\frac{rad}{s}]\f$
     * @return true if all chains are in position mode and we are able to
     * succesfully set the reference velocity for position move
     * for all joints including hands
     */
    bool setReferenceSpeeds(const yarp::sig::Vector& maximum_velocity);

    /**
     * @brief setReferenceSpeeds sets the reference joint speeds used in position mode when moving
     * @param maximum_velocity_map a map<chain_name, chain_velocity_vector>
     * with chain_velocity vector a \f$R^{n_{chain\_joints}}\f$ in \f$[\frac{rad}{s}]\f$
     * @return true if all desired chains are in position mode and we are
     * able to set desired reference velocity to all desired chains
     */
    bool setReferenceSpeeds(const VelocityMap& maximum_velocity_map);

    /**
     * @brief setReferenceSpeed sets reference speed for position mode move, for all joints including hands
     * @param maximum_velocity the maximum velocity for all joints in \f$[\frac{rad}{s}]\f$
     * @return true if all chains are in position mode and we are able to
     * succesfully set the reference velocity for position move
     * for all joints including hands
     */
    bool setReferenceSpeed(const double& maximum_velocity);

    /**
     * @brief setImpedance sets stiffness for all joints except hands
     * @param Kq a \f$R^{n_\text{robot\_joints}}\f$ vector in \f$\frac{Nm}{\text{rad}}\f$,
     * the desired joint stiffness for all joints except hands
     * @param Dq \f$R^{n_\text{robot\_joints}}\f$ vector in \f$\frac{Nms}{\text{rad}}\f$,
     * the desired joint damping for all joints except hands
     * @return true if the whole robot is in impedance control mode and
     * we are able to set the desired impedance for all joints except hands
     */
    bool setImpedance(const yarp::sig::Vector& Kq, const yarp::sig::Vector& Dq);

    /**
     * @brief getImpedance returns stiffness for all joints except hands
     * @param Kq a \f$R^{n_\text{robot\_joints}}\f$ vector in \f$\frac{Nm}{\text{rad}}\f$,
     * the actual joint stiffness for all joints except hands
     * @param Dq \f$R^{n_\text{robot\_joints}}\f$ vector in \f$\frac{Nms}{\text{rad}}\f$,
     * the actual joint damping for all joints except hands
     * @return true if the whole robot is in impedance control mode and
     * we are able to get impedance for all joints except hands
     */
    bool getImpedance(yarp::sig::Vector& Kq, yarp::sig::Vector& Dq);

    /**
     * @brief setImpedance set stifness for chains defined in the impedance map
     * @param impedance_map a map<chain_name, pair<chain_stiffness_vector, chain_damping_vector>>,
     * with chain_stiffness_vector a \f$R^{n_\text{chain\_joints}}\f$ vector in \f$\frac{Nm}{\text{rad}}\f$
     * and chain_damping_vector a \f$R^{n_\text{chain\_joints}}\f$ vector in \f$\frac{Nms}{\text{rad}}\f$
     * @return true if all desired chains are in impedance mode and we are
     * able to set desired stiffness to all desired chains
     */
    bool setImpedance(const ImpedanceMap& impedance_map);

    /**
     * @brief getImpedance returns a map<chain_name, pair<chain_stiffness_vector, chain_damping_vector>>
     * @param impedance_map a map<chain_name, pair<chain_stiffness_vector, chain_damping_vector>>
     * with chain_stiffness_vector a \f$R^{n_\text{chain\_joints}}\f$ vector in \f$\frac{Nm}{\text{rad}}\f$
     * and chain_damping_vector a \f$R^{n_\text{chain\_joints}}\f$ vector in \f$\frac{Nms}{\text{rad}}\f$
     * @return true if at least a chain is in impedance control mode and we are able to
     * succesfully obtain impedance from it
     */
    bool getImpedance(ImpedanceMap& impedance_map);

    /**
     * @brief getNumberOfJoints gets the robot number of joints
     * @return the number of joints
     */
    const unsigned int& getNumberOfJoints() const;

    /**
     * @brief getJointNames returns a vector of joints, in model order
     * @return a vector of joints, in model order
     */
    const std::vector<std::string> &getJointNames() const;
    
    void fromIdynToRobot(const yarp::sig::Vector& _q,
                           yarp::sig::Vector& _right_arm,
                           yarp::sig::Vector& _left_arm,
                           yarp::sig::Vector& _torso,
                           yarp::sig::Vector& _right_leg,
                           yarp::sig::Vector& _left_leg);

    void fromRobotToIdyn(const yarp::sig::Vector &_right_arm,
                         const yarp::sig::Vector &_left_arm,
                         const yarp::sig::Vector &_torso,
                         const yarp::sig::Vector &_right_leg,
                         const yarp::sig::Vector &_left_leg,
                         yarp::sig::Vector& _q);
    
    /**
     * @brief setControlType sets the desired control type, if possible, for all kinematic chains
     * @param controlType the desired control type for all kinematic chains
     * @return true on success
     */
    bool setControlType(const walkman::ControlType& controlType);

    /**
     * @brief setPositionMode sets position mode for all kinematic chains
     * @return true if setPositionMode for all chains is successfull
     */
    bool setPositionMode();

    /**
     * @brief isInPositionMode checks the control mode for all kinematic chains is position
     * @return true if all chains are in position mode
     */
    bool isInPositionMode();

    /**
     * @brief setPositionDirectMode sets position direct mode for all kinematic chains
     * @return true if set PositionDirectMode() for all chains is successfull
     */
    bool setPositionDirectMode();

    /**
     * @brief isInPositionDirectMode checks the control mode for all kinematic chains is positionDirect
     * @return true if all chains are in positionDirect mode
     */
    bool isInPositionDirectMode();

    /**
     * @brief setTorqueMode sets torque mode on all chains except hands
     * @return true if setPositionDirectMode() on hands and setTorqueMode() on all other chains is succesfull
     */
    bool setTorqueMode();

    /**
     * @brief setIdleMode sets idle mode for all chains
     * @return true if setIdleMode() is succesfull on all chains
     */
    bool setIdleMode();

    /**
     * @brief setImpedanceMode sets impedance control mode for all chains except hands
     * @return true if setPositionDirectMode() on hands and setImpedanceMode() on all other chains is succesfull
     */
    bool setImpedanceMode();

    /**
     * @brief isImpedanceMode checks the control mode of the whole robot except hands is impedance
     * @return true if all robot chains except hands are in impedance mode
     */
    bool isInImpedanceMode();

    // TODO more methods should iterate over this list
    /**
     * @brief getKinematicChains returns a list of kinematic chains for the current robot
     * @return a list of kinematic chains for this robot
     */
    KinematicChains getKinematicChains();
private:
    unsigned int number_of_joints;
    /// @brief q_commanded_right_arm q sento to the right hand, in robot joint ordering
    yarp::sig::Vector q_commanded_right_hand;
    /// @brief q_commanded_left_arm q sent to the left hand, in robot joint ordering
    yarp::sig::Vector q_commanded_left_hand;
    /// @brief q_commanded_left_arm q sent to the left arm, in robot joint ordering
    yarp::sig::Vector q_commanded_left_arm;
    /// @brief q_commanded_right_arm q sento to the right arm, in robot joint ordering
    yarp::sig::Vector q_commanded_right_arm;
    /// @brief q_commanded_left_leg q sento to the left leg, in robot joint ordering
    yarp::sig::Vector q_commanded_left_leg;
    /// @brief q_commanded_right_leg q sento to the right leg, in robot joint ordering
    yarp::sig::Vector q_commanded_right_leg;
    /// @brief q_commanded_torso q sento to the torso, in robot joint ordering
    yarp::sig::Vector q_commanded_torso;

    yarp::sig::Vector q_sensed;

    yarp::sig::Vector q_sensed_left_hand;
    yarp::sig::Vector q_sensed_right_hand;
    yarp::sig::Vector q_sensed_left_arm;
    yarp::sig::Vector q_sensed_right_arm;
    yarp::sig::Vector q_sensed_left_leg;
    yarp::sig::Vector q_sensed_right_leg;
    yarp::sig::Vector q_sensed_torso;

    yarp::sig::Vector qdot_sensed;

    yarp::sig::Vector qdot_sensed_left_arm;
    yarp::sig::Vector qdot_sensed_right_arm;
    yarp::sig::Vector qdot_sensed_left_leg;
    yarp::sig::Vector qdot_sensed_right_leg;
    yarp::sig::Vector qdot_sensed_torso;

    yarp::sig::Vector tau_sensed;

    yarp::sig::Vector tau_sensed_left_arm;
    yarp::sig::Vector tau_sensed_right_arm;
    yarp::sig::Vector tau_sensed_left_leg;
    yarp::sig::Vector tau_sensed_right_leg;
    yarp::sig::Vector tau_sensed_torso;

    std::string _moduleName;

    IMUPtr IMU;

    ftPtrMap ftSensors;

    ftReadings ft_readings;

    walkman::yarp_single_chain_interface* const getChainByName(const std::string chain_name);

    bool bodyIsInPositionMode();

    bool bodyIsInPositionDirectMode();

    bool handsAreInPositionMode();

    bool handsAreInPositionDirectMode();

    /**
     * @brief loadForceTorqueSensors checks whether the current robot has force/torque sensors (from the SRDF,
     * then by checking on the robotInterface), then tries to allocate a number of sensors.
     * Since the interface of force torque sensors accepts a kinematic chain name..
     * @return true if succesfully parsed the SRDF, and the SRDF is consistent with the capabilities offered by the robot
     */
    bool loadForceTorqueSensors();

    /**
     * @brief loadIMUSensors checks whether the current robot has IMU sensors (from the SRDF,
     * then by checking on the robotInterface), then tries to allocate a number of sensors.
     * Since the interface of IMUs is thought ATM for robots with just one IMU, only the first KIMU will be loaded.
     * @return true if succesfully parsed the SRDF, and the SRDF is consistent with the capabilities offered by the robot
     */
    bool loadIMUSensors();
};

#endif // ROBOTUTILS_H
