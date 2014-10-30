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

#ifndef COMANUTILS_H
#define COMANUTILS_H

#include "drc_shared/yarp_single_chain_interface.h"
#include "drc_shared/idynutils.h"
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>

/**
 * @brief The ComanUtils class eases whole body control for the coman robot.
 */
class ComanUtils
{
public:
    /**
     * @brief ComanUtils
     * @param moduleName the name of the module which uses the facility
     * @param controlModeVocab the control mode for the whole robot. Defaults to position control
     */
    ComanUtils(const std::string moduleName,
               const int controlModeVocab = VOCAB_CM_POSITION);

    /**
     * @brief ComanUtils
     * @param moduleName the name of the module which uses the facility
     * @param controlModeVocabMap a map <chain_name, control_mode_vocab> that specifies control mode for each robot chain
     */
    ComanUtils(const std::string moduleName,
               std::map<std::string,const int> &controlModeVocabMap);

    walkman::drc::yarp_single_chain_interface right_hand, left_hand;
    walkman::drc::yarp_single_chain_interface right_arm, left_arm;
    walkman::drc::yarp_single_chain_interface torso;
    walkman::drc::yarp_single_chain_interface right_leg, left_leg;
    iDynUtils idynutils;

    /**
     * @brief hasHands check whether thboth hands are available
     * @return true if connection to both hands is successful
     */
    bool hasHands();

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
//    void move(std::map<std::string,yarp::sig::Vector&> &u);

//    /**
//     * @brief move send potision commands and torque offsets to all the robot joints. Works when the robot is in joint impedance or position control mode.
//     * @param q the desired joint position vector
//     * @param torqueOffset_map a map <chain_name, offset_torques> of torque offsets to send to the robot.
//     * Chains that accept a torque offset should be controlled in joint impedance mode.
//     */
//    void move(yarp::sig::Vector &q,
//              std::map<std::string,yarp::sig::Vector&> &torqueOffset_map);

//    /**
//     * @brief move send potision commands and torque offsets to all the robot joints. Works when the robot is in joint impedance or position control mode.
//     * @param q the desired joint position vector
//     * @param kq a map <chain_name, kq> of joint stiffness references to send to the robot.
//     * Chains that accept a torque offset should be controlled in joint impedance mode.
//     * @param torqueOffset_map a map <chain_name, offset_torques> of torque offsets to send to the robot.
//     * Chains that accept a torque offset should be controlled in joint impedance mode.
//     */
//    void move(yarp::sig::Vector &q,
//              std::map<std::string,yarp::sig::Vector&> &kq_map,
//              std::map<std::string,yarp::sig::Vector&> &torqueOffset_map);

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
//    void move(yarp::sig::Vector &q,
//              std::map<std::string,yarp::sig::Vector&> &kq_map,
//              std::map<std::string,yarp::sig::Vector&> &kd_map,
//              std::map<std::string,yarp::sig::Vector&> &torqueOffset_map);


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
};

#endif // COMANUTILS_H
