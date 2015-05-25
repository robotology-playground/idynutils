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
//     typedef std::pair<yarp::sig::Vector, yarp::sig::Vector> Impedance;
//     typedef std::map<std::string,  Impedance> ImpedanceMap;
//     typedef std::map<std::string,  yarp::sig::Vector> VelocityMap;
    typedef std::shared_ptr<yarp_IMU_interface> IMUPtr;
    typedef std::shared_ptr<yarp_ft_interface> ftPtr;
    typedef std::map<std::string, int> ftPtrMap;
    typedef std::map<std::string, yarp::sig::Vector> ftReadings;
//     typedef kinematic_chain* KinematicChainPtr;
//     typedef std::list<KinematicChainPtr> KinematicChains;

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

    walkman::yarp_single_chain_interface whole_robot;
    iDynUtils idynutils;

//     std::vector<std::string> ft_reference_frames;

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
     * @brief sensePosition returns the position of the robot's joints
     * @return
     */
    yarp::sig::Vector& sensePosition();

    /**
     * @brief sensePosition returns the position of the robot's joints
     * @return
     */
    void sensePosition(yarp::sig::Vector&);

    /**
     * @brief senseMotorPosition returns the position of the robot's joints
     * @return
     */
    yarp::sig::Vector& senseMotorPosition();

    /**
     * @brief senseMotorPosition returns the position of the robot's joints
     * @return
     */
    void senseMotorPosition(yarp::sig::Vector&);

    /**
     * @brief senseTorque returns the torques of the robot's joints
     * @return
     */
    yarp::sig::Vector& senseTorque();

    /**
     * @brief senseTorque returns the position of the robot's joints
     * @return
     */
    void senseTorque(yarp::sig::Vector&);

    /**
     * @brief sensePositionRefFeedback returns the last position ref feedback sent to the firmware
     * @return
     */
    yarp::sig::Vector& sensePositionRefFeedback();

    /**
     * @brief sensePositionRefFeedback returns the position of the robot's joints
     * @return
     */
    void sensePositionRefFeedback(yarp::sig::Vector&);

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
    void move29(const yarp::sig::Vector &_q);

    void moveHands(const yarp::sig::Vector &_q);

    void moveNeck(const yarp::sig::Vector &_q);

    /**
     * @brief getNumberOfActuatedJoints gets the hardware robot number of joints
     * @return the number of joints
     */
    const unsigned int getNumberOfActuatedJoints() const;

        /**
     * @brief getNumberOfKinematicJoints gets the idyntree number of joints
     * @return the number of joints
     */
    const unsigned int getNumberOfKinematicJoints() const;
    
    void fromIdynToRobot29(const yarp::sig::Vector& _q,
                           yarp::sig::Vector& _right_arm,
                           yarp::sig::Vector& _left_arm,
                           yarp::sig::Vector& _torso,
                           yarp::sig::Vector& _right_leg,
                           yarp::sig::Vector& _left_leg);

    void fromIdynToRobot31(const yarp::sig::Vector& _q,
                           yarp::sig::Vector& _right_arm,
                           yarp::sig::Vector& _left_arm,
                           yarp::sig::Vector& _torso,
                           yarp::sig::Vector& _right_leg,
                           yarp::sig::Vector& _left_leg,
                           yarp::sig::Vector& _head);

    void fromRobotToIdyn29(const yarp::sig::Vector &_right_arm,
                         const yarp::sig::Vector &_left_arm,
                         const yarp::sig::Vector &_torso,
                         const yarp::sig::Vector &_right_leg,
                         const yarp::sig::Vector &_left_leg,
                         yarp::sig::Vector& _q);

    void fromRobotToIdyn31(const yarp::sig::Vector &_right_arm,
                         const yarp::sig::Vector &_left_arm,
                         const yarp::sig::Vector &_torso,
                         const yarp::sig::Vector &_right_leg,
                         const yarp::sig::Vector &_left_leg,
                         const yarp::sig::Vector &_head,
                         yarp::sig::Vector& _q);

    /**
     * @brief setControlType sets the desired control type, if possible, for all kinematic chains
     * @param controlType the desired control type for all kinematic chains
     * @return true on success
     */
    bool setControlType(const walkman::ControlType& controlType);

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
     * @brief setIdleMode sets idle mode for all kinematic chains
     * @return true if set idle for all chains is successfull
     */
    bool setIdleMode();

    const int& left_hand_index; const int & right_hand_index;

    const std::vector<std::string>& getJointNames() const;

private:
    unsigned int number_of_joints;
    int left_hand_i,right_hand_i;
    int neck_y_index, neck_p_index;
    int j_29[29];
    yarp::sig::Vector q_sensed;
    yarp::sig::Vector tau_sensed;
    yarp::sig::Vector q_motor_sensed;
    yarp::sig::Vector q_ref_feedback_sensed;

    std::string _moduleName;

    IMUPtr IMU;

    ftPtr ft;

    ftPtrMap ftSensors;

    ftReadings ft_readings;

    bool bodyIsInPositionDirectMode();

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
