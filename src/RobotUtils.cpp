/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

#include <idynutils/RobotUtils.h>

using namespace iCub::iDynTree;
using namespace yarp::math;

RobotUtils::RobotUtils(const std::string moduleName, 
		       const std::string robotName,
		       const std::string urdf_path, 
		       const std::string srdf_path) :
//     right_hand(walkman::robot::right_hand, moduleName, robotName, true, walkman::controlTypes::none),
//     right_arm(walkman::robot::right_arm, moduleName, robotName, true, walkman::controlTypes::none),
    right_leg(walkman::robot::right_leg, moduleName, robotName, true, walkman::controlTypes::none),
//     left_hand(walkman::robot::left_hand, moduleName, robotName, true, walkman::controlTypes::none),
//     left_arm(walkman::robot::left_arm, moduleName, robotName, true, walkman::controlTypes::none),
    left_leg(walkman::robot::left_leg, moduleName, robotName, true, walkman::controlTypes::none),
//     torso(walkman::robot::torso, moduleName, robotName, true, walkman::controlTypes::none),
//     q_sensed_right_hand( 1 ),
//     q_sensed_left_hand( 1 ),
//     q_sensed_right_arm( right_arm.getNumberOfJoints() ),
//     q_sensed_left_arm( left_arm.getNumberOfJoints() ),
//     q_sensed_torso( torso.getNumberOfJoints() ),
    q_sensed_right_leg( right_leg.getNumberOfJoints() ),
    q_sensed_left_leg( left_leg.getNumberOfJoints() ),
//     q_commanded_right_hand( 1 ),
//     q_commanded_left_hand( 1 ),
//     q_commanded_right_arm( right_arm.getNumberOfJoints() ),
//     q_commanded_left_arm( left_arm.getNumberOfJoints() ),
//     q_commanded_torso( torso.getNumberOfJoints() ),
    q_commanded_right_leg( right_leg.getNumberOfJoints() ),
    q_commanded_left_leg( left_leg.getNumberOfJoints() ),
    idynutils( robotName, urdf_path, srdf_path ),
    _moduleName(moduleName)
{
    this->number_of_joints = idynutils.iDyn3_model.getNrOfDOFs();
    q_sensed.resize(this->number_of_joints,0.0);
    qdot_sensed.resize(this->number_of_joints,0.0);
    tau_sensed.resize(this->number_of_joints,0.0);

    loadIMUSensors();
    loadForceTorqueSensors();
}

// bool RobotUtils::hasHands()
// {
//     return left_hand.isAvailable && right_hand.isAvailable;
// }

bool RobotUtils::hasftSensors()
{
    return this->ftSensors.size() > 0;
}

RobotUtils::ftPtrMap RobotUtils::getftSensors()
{
    return this->ftSensors;
}

const unsigned int& RobotUtils::getNumberOfJoints() const
{
    return this->number_of_joints;
}

const std::vector<std::string> &RobotUtils::getJointNames() const
{
    return idynutils.getJointNames();
}

void RobotUtils::move(const yarp::sig::Vector &_q) {

    fromIdynToRobot(_q,
//                     q_commanded_right_arm,
//                     q_commanded_left_arm,
//                     q_commanded_torso,
                    q_commanded_right_leg,
                    q_commanded_left_leg);

//     torso.move(q_commanded_torso);
//     left_arm.move(q_commanded_left_arm);
//     right_arm.move(q_commanded_right_arm);
    left_leg.move(q_commanded_left_leg);
    right_leg.move(q_commanded_right_leg);
}

bool RobotUtils::moveDone()
{
    bool moveDone = 
// 			torso.moveDone() &&
// 		    left_arm.moveDone() &&
// 		    right_arm.moveDone() &&
		    left_leg.moveDone() &&
		    right_leg.moveDone();
    return moveDone;
}


// bool RobotUtils::moveHands(const yarp::sig::Vector &q_left_hand,
//                            const yarp::sig::Vector &q_right_hand)
// {
//     q_commanded_left_hand = q_left_hand;
//     q_commanded_right_hand = q_right_hand;
// 
//     if(left_hand.isAvailable)
//         left_hand.move(q_commanded_left_hand);
// 
//     if(right_hand.isAvailable)
//         right_hand.move(q_commanded_right_hand);
// 
//     return hasHands();
// }

bool RobotUtils::setReferenceSpeeds(const yarp::sig::Vector &maximum_velocity)
{
    assert(maximum_velocity.size() == this->getNumberOfJoints());

    if(!bodyIsInPositionMode()) {
        std::cout << "Trying to set reference speeds for the whole coman "
                  << "but the robot is not entirely in Position Mode";
        return false;
    }

    yarp::sig::Vector 
// 					  velocity_torso,
//                       velocity_right_arm,
//                       velocity_left_arm,
                      velocity_right_leg,
                      velocity_left_leg;
//     idynutils.fromIDynToRobot(maximum_velocity, velocity_torso, idynutils.torso);
//     idynutils.fromIDynToRobot(maximum_velocity, velocity_right_arm, idynutils.right_arm);
//     idynutils.fromIDynToRobot(maximum_velocity, velocity_left_arm, idynutils.left_arm);
    idynutils.fromIDynToRobot(maximum_velocity, velocity_right_leg, idynutils.right_leg);
    idynutils.fromIDynToRobot(maximum_velocity, velocity_left_leg, idynutils.left_leg);
    return  
// 			torso.setReferenceSpeeds(velocity_torso) &&
//             right_arm.setReferenceSpeeds(velocity_right_arm) &&
//             left_arm.setReferenceSpeeds(velocity_left_arm) &&
            right_leg.setReferenceSpeeds(velocity_right_leg) &&
            left_leg.setReferenceSpeeds(velocity_left_leg);
}

bool RobotUtils::setReferenceSpeeds(const RobotUtils::VelocityMap &maximum_velocity_map)
{

    bool success = true;
    int number_of_chains = 0;

    for(VelocityMap::const_iterator i = maximum_velocity_map.begin(); i != maximum_velocity_map.end(); ++i) {
        walkman::yarp_single_chain_interface * const chain = this->getChainByName(i->first);
        if(chain != NULL) {
            ++number_of_chains;
            success = success && chain->setReferenceSpeeds(i->second);
        }
    }

    if(number_of_chains == 0) success = false;

    return success;
}

bool RobotUtils::setReferenceSpeed(const double &maximum_velocity)
{
    return  
// 			(right_hand.isAvailable ? right_hand.setReferenceSpeed(maximum_velocity) : true) &&
//             (left_hand.isAvailable ? left_hand.setReferenceSpeed(maximum_velocity) : true) &&
//             torso.setReferenceSpeed(maximum_velocity) &&
//             right_arm.setReferenceSpeed(maximum_velocity) &&
//             left_arm.setReferenceSpeed(maximum_velocity) &&
            right_leg.setReferenceSpeed(maximum_velocity) &&
            left_leg.setReferenceSpeed(maximum_velocity);
}

bool RobotUtils::setImpedance(const yarp::sig::Vector &Kq, const yarp::sig::Vector &Dq)
{
    assert(Kq.size() == this->getNumberOfJoints());
    assert(Dq.size() == this->getNumberOfJoints());

    if(!isInImpedanceMode()) {
        std::cout << "Trying to set impedance for the whole coman "
                  << "but the robot is not entirely in Position Mode";
        return false;
    }
    yarp::sig::Vector 
// 					  Kq_torso, Dq_torso,
//                       Kq_right_arm, Dq_right_arm,
//                       Kq_left_arm, Dq_left_arm,
                      Kq_right_leg, Dq_right_leg,
                      Kq_left_leg, Dq_left_leg;
//     idynutils.fromIDynToRobot(Kq, Kq_torso, idynutils.torso);
//     idynutils.fromIDynToRobot(Dq, Dq_torso, idynutils.torso);
//     idynutils.fromIDynToRobot(Kq, Kq_right_arm, idynutils.right_arm);
//     idynutils.fromIDynToRobot(Dq, Dq_right_arm, idynutils.right_arm);
//     idynutils.fromIDynToRobot(Kq, Kq_left_arm, idynutils.left_arm);
//     idynutils.fromIDynToRobot(Dq, Dq_left_arm, idynutils.left_arm);
    idynutils.fromIDynToRobot(Kq, Kq_right_leg, idynutils.right_leg);
    idynutils.fromIDynToRobot(Dq, Dq_right_leg, idynutils.right_leg);
    idynutils.fromIDynToRobot(Kq, Kq_left_leg, idynutils.left_leg);
    idynutils.fromIDynToRobot(Dq, Dq_left_leg, idynutils.left_leg);

    return      
// 				torso.setImpedance(Kq_torso, Dq_torso) &&
//                 right_arm.setImpedance(Kq_right_arm, Dq_right_arm) &&
//                 left_arm.setImpedance(Kq_left_arm, Dq_left_arm) &&
                right_leg.setImpedance(Kq_right_leg, Dq_right_leg) &&
                left_leg.setImpedance(Kq_left_leg, Dq_left_leg);
}

bool RobotUtils::setImpedance(const std::map<std::string, std::pair<yarp::sig::Vector, yarp::sig::Vector> >& impedance_map)
{
    bool success = true;
    int number_of_chains = 0;

    for(ImpedanceMap::const_iterator i = impedance_map.begin(); i != impedance_map.end(); ++i) {
        walkman::yarp_single_chain_interface* chain = this->getChainByName(i->first);
        if(chain != NULL) {
            if(chain->isInImpedanceMode()) {
                ++number_of_chains;
                success = success && chain->setImpedance(i->second.first,
                                                         i->second.second);
            } else success = false;
        }
    }

    if(number_of_chains == 0) success = false;

    return success;
}

bool RobotUtils::getImpedance(std::map<std::string, std::pair<yarp::sig::Vector, yarp::sig::Vector> >& impedance_map)
{
    bool atLeastAChainInImpedanceMode = false;
    impedance_map.clear();

//     if(torso.isInImpedanceMode()) {
//         yarp::sig::Vector Kq, Dq;
//         torso.getImpedance(Kq,Dq);
//         impedance_map[torso.getChainName()] = Impedance(Kq,Dq);
//         atLeastAChainInImpedanceMode = true;
//     }
// 
//     if(right_arm.isInImpedanceMode()) {
//         yarp::sig::Vector Kq, Dq;
//         right_arm.getImpedance(Kq,Dq);
//         impedance_map[right_arm.getChainName()] = Impedance(Kq,Dq);
//         atLeastAChainInImpedanceMode = true;
//     }
// 
//     if(left_arm.isInImpedanceMode()) {
//         yarp::sig::Vector Kq, Dq;
//         left_arm.getImpedance(Kq,Dq);
//         impedance_map[left_arm.getChainName()] = Impedance(Kq,Dq);
//         atLeastAChainInImpedanceMode = true;
//     }

    if(right_leg.isInImpedanceMode()) {
        yarp::sig::Vector Kq, Dq;
        right_leg.getImpedance(Kq,Dq);
        impedance_map[right_leg.getChainName()] = Impedance(Kq,Dq);
        atLeastAChainInImpedanceMode = true;
    }

    if(left_leg.isInImpedanceMode()) {
        yarp::sig::Vector Kq, Dq;
        left_leg.getImpedance(Kq,Dq);
        impedance_map[left_leg.getChainName()] = Impedance(Kq,Dq);
        atLeastAChainInImpedanceMode = true;
    }

//     if(right_hand.isAvailable && right_hand.isInImpedanceMode()) {
//         yarp::sig::Vector Kq, Dq;
//         right_hand.getImpedance(Kq,Dq);
//         impedance_map[right_hand.getChainName()] = Impedance(Kq,Dq);
//         atLeastAChainInImpedanceMode = true;
//     }
// 
//     if(left_hand.isAvailable && left_hand.isInImpedanceMode()) {
//         yarp::sig::Vector Kq, Dq;
//         left_hand.getImpedance(Kq,Dq);
//         impedance_map[left_hand.getChainName()] = Impedance(Kq,Dq);
//         atLeastAChainInImpedanceMode = true;
//     }

    return atLeastAChainInImpedanceMode;
}

void RobotUtils::sense(yarp::sig::Vector &q,
                       yarp::sig::Vector &qdot,
                       yarp::sig::Vector &tau)
{
    q = sensePosition();
    qdot = senseVelocity();
    tau = senseTorque();
}

yarp::sig::Vector &RobotUtils::sensePosition()
{
//     right_arm.sensePosition(q_sensed_right_arm);
//     left_arm.sensePosition(q_sensed_left_arm);
//     torso.sensePosition(q_sensed_torso);
    right_leg.sensePosition(q_sensed_right_leg);
    left_leg.sensePosition(q_sensed_left_leg);

    fromRobotToIdyn(
// 					q_sensed_right_arm,
//                     q_sensed_left_arm,
//                     q_sensed_torso,
                    q_sensed_right_leg,
                    q_sensed_left_leg,
                    q_sensed);

    return q_sensed;
}

yarp::sig::Vector &RobotUtils::senseVelocity()
{
//     right_arm.senseVelocity(qdot_sensed_right_arm);
//     left_arm.senseVelocity(qdot_sensed_left_arm);
//     torso.senseVelocity(qdot_sensed_torso);
    right_leg.senseVelocity(qdot_sensed_right_leg);
    left_leg.senseVelocity(qdot_sensed_left_leg);

    fromRobotToIdyn(
// 					qdot_sensed_right_arm,
//                     qdot_sensed_left_arm,
//                     qdot_sensed_torso,
                    qdot_sensed_right_leg,
                    qdot_sensed_left_leg,
                    qdot_sensed);

    return qdot_sensed;
}

yarp::sig::Vector &RobotUtils::senseTorque()
{
//     right_arm.senseTorque(tau_sensed_right_arm);
//     left_arm.senseTorque(tau_sensed_left_arm);
//     torso.senseTorque(tau_sensed_torso);
    right_leg.senseTorque(tau_sensed_right_leg);
    left_leg.senseTorque(tau_sensed_left_leg);

    fromRobotToIdyn(
// 					tau_sensed_right_arm,
//                     tau_sensed_left_arm,
//                     tau_sensed_torso,
                    tau_sensed_right_leg,
                    tau_sensed_left_leg,
                    tau_sensed);

    return tau_sensed;
}

RobotUtils::ftReadings& RobotUtils::senseftSensors()
{
    ft_readings.clear();
    for( ftPtrMap::iterator i = ftSensors.begin(); i != ftSensors.end(); ++i)
    {
        ft_readings[i->first] = i->second->sense();
    }
    return ft_readings;
}

bool RobotUtils::senseftSensor(const std::string &ft_frame,
                               yarp::sig::Vector &ftReading)
{
    if(ftSensors[ft_frame]) {
        ftPtr ft(ftSensors[ft_frame]);
        return ft->sense(ftReading);
    }
    return false;
}

// bool RobotUtils::senseHandsPosition(yarp::sig::Vector &q_left_hand,
//                                     yarp::sig::Vector &q_right_hand)
// {
//     if(left_hand.isAvailable) {
//         left_hand.sensePosition(q_sensed_left_hand);
//         q_left_hand = q_sensed_left_hand;
//     }
// 
//     if(right_hand.isAvailable) {
//         right_hand.sensePosition(q_sensed_right_hand);
//         q_right_hand = q_sensed_right_hand;
//     }
// 
//     return hasHands();
// }


void RobotUtils::fromIdynToRobot(const yarp::sig::Vector &_q,
//                                  yarp::sig::Vector &_right_arm,
//                                  yarp::sig::Vector &_left_arm,
//                                  yarp::sig::Vector &_torso,
                                 yarp::sig::Vector &_right_leg,
                                 yarp::sig::Vector &_left_leg)
{
//     idynutils.fromIDynToRobot(_q, _right_arm, idynutils.right_arm);
//     idynutils.fromIDynToRobot(_q, _left_arm, idynutils.left_arm);
//     idynutils.fromIDynToRobot(_q, _torso, idynutils.torso);
    idynutils.fromIDynToRobot(_q, _right_leg, idynutils.right_leg);
    idynutils.fromIDynToRobot(_q, _left_leg, idynutils.left_leg);
}

void RobotUtils::fromRobotToIdyn(
// 								 const yarp::sig::Vector &_right_arm,
//                                  const yarp::sig::Vector &_left_arm,
//                                  const yarp::sig::Vector &_torso,
                                 const yarp::sig::Vector &_right_leg,
                                 const yarp::sig::Vector &_left_leg,
                                 yarp::sig::Vector &_q)
{
//     idynutils.fromRobotToIDyn(_right_arm, _q, idynutils.right_arm);
//     idynutils.fromRobotToIDyn(_left_arm, _q, idynutils.left_arm);
//     idynutils.fromRobotToIDyn(_torso, _q, idynutils.torso);
    idynutils.fromRobotToIDyn(_right_leg, _q, idynutils.right_leg);
    idynutils.fromRobotToIDyn(_left_leg, _q, idynutils.left_leg);
}

bool RobotUtils::setControlType(const walkman::ControlType& controlType)
{
    std::cout << "Setting control type : " << controlType.toString() << std::endl;
    return  
// 			(right_hand.isAvailable ? right_hand.setControlType(controlType) : true) &&
//             (left_hand.isAvailable ? left_hand.setControlType(controlType) : true) &&
//             torso.setControlType(controlType) &&
//             right_arm.setControlType(controlType) &&
//             left_arm.setControlType(controlType) &&
            right_leg.setControlType(controlType) &&
            left_leg.setControlType(controlType);
}

bool RobotUtils::setPositionMode()
{
    return setControlType(walkman::controlTypes::position);
}

bool RobotUtils::setPositionDirectMode()
{
    return setControlType(walkman::controlTypes::positionDirect);
}

bool RobotUtils::setTorqueMode()
{
    return setControlType(walkman::controlTypes::torque);
}

bool RobotUtils::setIdleMode()
{
    return setControlType(walkman::controlTypes::idle);
}

bool RobotUtils::setImpedanceMode()
{
    return setControlType(walkman::controlTypes::impedance);
}


bool RobotUtils::isInPositionMode()
{
    return bodyIsInPositionMode(); // &&
           //(!hasHands() || handsAreInPositionMode());
}

bool RobotUtils::isInImpedanceMode()
{
    return  
// 			torso.isInImpedanceMode() &&
//             right_arm.isInImpedanceMode() &&
//             left_arm.isInImpedanceMode() &&
            right_leg.isInImpedanceMode() &&
            left_leg.isInImpedanceMode();
}

walkman::yarp_single_chain_interface* const RobotUtils::getChainByName(const std::string chain_name) {
//     if(chain_name == walkman::robot::left_arm) return &left_arm;
//     if(chain_name == walkman::robot::right_arm) return &right_arm;
    if(chain_name == walkman::robot::left_leg) return &left_leg;
    if(chain_name == walkman::robot::right_leg) return &right_leg;
//     if(chain_name == walkman::robot::torso) return &torso;
//     if(chain_name == walkman::robot::right_hand) return &right_hand;
//     if(chain_name == walkman::robot::left_hand) return &left_hand;
    return NULL;
}

bool RobotUtils::bodyIsInPositionMode()
{
    return  
// 			torso.isInPositionMode() &&
//             right_arm.isInPositionMode() &&
//             left_arm.isInPositionMode() &&
            right_leg.isInPositionMode() &&
            left_leg.isInPositionMode();
}

bool RobotUtils::handsAreInPositionMode()
{
    return true; // (right_hand.isAvailable && right_hand.isInPositionMode()) &&
				 // (left_hand.isAvailable && left_hand.isInPositionMode());
}


bool RobotUtils::hasIMU()
{
    return (bool)this->IMU;
}


RobotUtils::IMUPtr RobotUtils::getIMU()
{
    return this->IMU;
}


bool RobotUtils::loadForceTorqueSensors()
{
    std::vector<srdf::Model::Group> robot_groups = idynutils.robot_srdf->getGroups();
    for(auto group: robot_groups)
    {
        if (group.name_ == walkman::robot::force_torque_sensors)
        {
            if(group.joints_.size() > 0) {
                for(auto joint_name : group.joints_)
                {
                    std::cout << "ft sensors found on joint " << joint_name;

                    std::string reference_frame = idynutils.moveit_robot_model->getJointModel(joint_name)->
                            getChildLinkModel()->getName();

                    std::cout << " on frame " << reference_frame << ". Loading ft ..." << std::endl; std::cout.flush();

                    try {
                        std::shared_ptr<yarp_ft_interface> ft( new yarp_ft_interface(reference_frame,
                                                        _moduleName,
                                                        idynutils.getRobotName(), reference_frame) );

                        ftSensors[reference_frame] = ft;
                        ft_reference_frames.push_back(reference_frame);

                        std::cout << "ft on " << reference_frame << " loaded" << std::endl;
                    } catch(...) {
                        std::cerr << "Error loading " << reference_frame << " ft " << std::endl;
                        return false;}
                }
                return true;
            }
        }
    }
    std::cout << "Robot does not have any ft sensor" << std::endl;
    return false;
}


bool RobotUtils::loadIMUSensors()
{
//     std::vector<srdf::Model::Group> robot_groups = idynutils.robot_srdf->getGroups();
//     for(auto group: robot_groups)
//     {
//         if (group.name_ == walkman::robot::imu_sensors)
//         {
//             if(group.joints_.size() > 0) {
//                 try {
//                     IMU = IMUPtr(new yarp_IMU_interface(_moduleName));
//                     std::cout << "IMU loaded" << std::endl;
//                     return true;
//                 } catch(...) {
//                     std::cerr << "Error loading IMU" << std::endl;
//                     return false;
//                 }
//             }
//         }
//     }
    std::cout << "Robot does not have an IMU" << std::endl;
    return false;
}

RobotUtils::KinematicChains RobotUtils::getKinematicChains()
{
    KinematicChains k_chains;
//     k_chains.push_back(&idynutils.torso);
//     k_chains.push_back(&idynutils.right_arm);
//     k_chains.push_back(&idynutils.left_arm);
    k_chains.push_back(&idynutils.right_leg);
    k_chains.push_back(&idynutils.left_leg);
    return k_chains;
}
