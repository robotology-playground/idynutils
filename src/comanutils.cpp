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

#include <idynutils/comanutils.h>
#include <idynutils/idynutils.h>

using namespace iCub::iDynTree;
using namespace yarp::math;

ComanUtils::ComanUtils(const std::string moduleName):
right_hand(walkman::robot::right_hand, moduleName, "coman", true, WALKMAN_CM_NONE),
    right_arm(walkman::robot::right_arm, moduleName, "coman", true, WALKMAN_CM_NONE),
    right_leg(walkman::robot::right_leg, moduleName, "coman", true, WALKMAN_CM_NONE),
    left_hand(walkman::robot::left_hand, moduleName, "coman", true, WALKMAN_CM_NONE),
    left_arm(walkman::robot::left_arm, moduleName, "coman", true, WALKMAN_CM_NONE),
    left_leg(walkman::robot::left_leg, moduleName, "coman", true, WALKMAN_CM_NONE),
    torso(walkman::robot::torso, moduleName, "coman", true, WALKMAN_CM_NONE),
    q_sensed_right_hand( 1 ),
    q_sensed_left_hand( 1 ),
    q_sensed_right_arm( right_arm.getNumberOfJoints() ),
    q_sensed_left_arm( left_arm.getNumberOfJoints() ),
    q_sensed_torso( torso.getNumberOfJoints() ),
    q_sensed_right_leg( right_leg.getNumberOfJoints() ),
    q_sensed_left_leg( left_leg.getNumberOfJoints() ),
    q_commanded_right_hand( 1 ),
    q_commanded_left_hand( 1 ),
    q_commanded_right_arm( right_arm.getNumberOfJoints() ),
    q_commanded_left_arm( left_arm.getNumberOfJoints() ),
    q_commanded_torso( torso.getNumberOfJoints() ),
    q_commanded_right_leg( right_leg.getNumberOfJoints() ),
    q_commanded_left_leg( left_leg.getNumberOfJoints() )
{
    this->number_of_joints = idynutils.iDyn3_model.getNrOfDOFs();
    q_sensed.resize(this->number_of_joints,0.0);
    qdot_sensed.resize(this->number_of_joints,0.0);
    tau_sensed.resize(this->number_of_joints,0.0);
}

bool ComanUtils::hasHands()
{
    return left_hand.isAvailable && right_hand.isAvailable;
}

const unsigned int& ComanUtils::getNumberOfJoints() const
{
    return this->number_of_joints;
}

const std::vector<std::string> &ComanUtils::getJointNames() const
{
    return idynutils.getJointNames();
}

void ComanUtils::move(const yarp::sig::Vector &_q) {

    fromIdynToRobot(_q,
                    q_commanded_right_arm,
                    q_commanded_left_arm,
                    q_commanded_torso,
                    q_commanded_right_leg,
                    q_commanded_left_leg);

    torso.move(q_commanded_torso);
    left_arm.move(q_commanded_left_arm);
    right_arm.move(q_commanded_right_arm);
    left_leg.move(q_commanded_left_leg);
    right_leg.move(q_commanded_right_leg);
}

bool ComanUtils::moveHands(const yarp::sig::Vector &q_left_hand,
                           const yarp::sig::Vector &q_right_hand)
{
    q_commanded_left_hand = q_left_hand;
    q_commanded_right_hand = q_right_hand;

    if(left_hand.isAvailable)
        left_hand.move(q_commanded_left_hand);

    if(right_hand.isAvailable)
        right_hand.move(q_commanded_right_hand);

    return hasHands();
}

bool ComanUtils::setReferenceSpeeds(const yarp::sig::Vector &maximum_velocity)
{
    assert(maximum_velocity.size() == this->getNumberOfJoints());

    if(!bodyIsInPositionMode()) {
        std::cout << "Trying to set reference speeds for the whole coman "
                  << "but the robot is not entirely in Position Mode";
        return false;
    }

    yarp::sig::Vector velocity_torso,
                      velocity_right_arm,
                      velocity_left_arm,
                      velocity_right_leg,
                      velocity_left_leg;
    idynutils.fromIDynToRobot(maximum_velocity, velocity_torso, idynutils.torso);
    idynutils.fromIDynToRobot(maximum_velocity, velocity_right_arm, idynutils.right_arm);
    idynutils.fromIDynToRobot(maximum_velocity, velocity_left_arm, idynutils.left_arm);
    idynutils.fromIDynToRobot(maximum_velocity, velocity_right_leg, idynutils.right_leg);
    idynutils.fromIDynToRobot(maximum_velocity, velocity_left_leg, idynutils.left_leg);
    return  torso.setReferenceSpeeds(velocity_torso) &&
            right_arm.setReferenceSpeeds(velocity_right_arm) &&
            left_arm.setReferenceSpeeds(velocity_left_arm) &&
            right_leg.setReferenceSpeeds(velocity_right_leg) &&
            left_leg.setReferenceSpeeds(velocity_left_leg);
}

bool ComanUtils::setReferenceSpeeds(const ComanUtils::VelocityMap &maximum_velocity_map)
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

bool ComanUtils::setReferenceSpeed(const double &maximum_velocity)
{
    return  (right_hand.isAvailable ? right_hand.setReferenceSpeed(maximum_velocity) : true) &&
            (left_hand.isAvailable ? left_hand.setReferenceSpeed(maximum_velocity) : true) &&
            torso.setReferenceSpeed(maximum_velocity) &&
            right_arm.setReferenceSpeed(maximum_velocity) &&
            left_arm.setReferenceSpeed(maximum_velocity) &&
            right_leg.setReferenceSpeed(maximum_velocity) &&
            left_leg.setReferenceSpeed(maximum_velocity);
}

bool ComanUtils::setImpedance(const yarp::sig::Vector &Kq, const yarp::sig::Vector &Dq)
{
    assert(Kq.size() == this->getNumberOfJoints());
    assert(Dq.size() == this->getNumberOfJoints());

    if(!isInImpedanceMode()) {
        std::cout << "Trying to set impedance for the whole coman "
                  << "but the robot is not entirely in Position Mode";
        return false;
    }
    yarp::sig::Vector Kq_torso, Dq_torso,
                      Kq_right_arm, Dq_right_arm,
                      Kq_left_arm, Dq_left_arm,
                      Kq_right_leg, Dq_right_leg,
                      Kq_left_leg, Dq_left_leg;
    idynutils.fromIDynToRobot(Kq, Kq_torso, idynutils.torso);
    idynutils.fromIDynToRobot(Dq, Dq_torso, idynutils.torso);
    idynutils.fromIDynToRobot(Kq, Kq_right_arm, idynutils.right_arm);
    idynutils.fromIDynToRobot(Dq, Dq_right_arm, idynutils.right_arm);
    idynutils.fromIDynToRobot(Kq, Kq_left_arm, idynutils.left_arm);
    idynutils.fromIDynToRobot(Dq, Dq_left_arm, idynutils.left_arm);
    idynutils.fromIDynToRobot(Kq, Kq_right_leg, idynutils.right_leg);
    idynutils.fromIDynToRobot(Dq, Dq_right_leg, idynutils.right_leg);
    idynutils.fromIDynToRobot(Kq, Kq_left_leg, idynutils.left_leg);
    idynutils.fromIDynToRobot(Dq, Dq_left_leg, idynutils.left_leg);

    return      torso.setImpedance(Kq_torso, Dq_torso) &&
                right_arm.setImpedance(Kq_right_arm, Dq_right_arm) &&
                left_arm.setImpedance(Kq_left_arm, Dq_left_arm) &&
                right_leg.setImpedance(Kq_right_leg, Dq_right_leg) &&
                left_leg.setImpedance(Kq_left_leg, Dq_left_leg);
}

bool ComanUtils::setImpedance(const std::map<std::string, std::pair<yarp::sig::Vector, yarp::sig::Vector> >& impedance_map)
{
    bool success = true;
    int number_of_chains = 0;

    for(ImpedanceMap::const_iterator i = impedance_map.begin(); i != impedance_map.end(); ++i) {
        walkman::yarp_single_chain_interface* chain = this->getChainByName(i->first);
        if(chain != NULL) {
            if(chain->getControlMode() == VOCAB_CM_IMPEDANCE_POS) {
                ++number_of_chains;
                success = success && chain->setImpedance(i->second.first,
                                                         i->second.second);
            } else success = false;
        }
    }

    if(number_of_chains == 0) success = false;

    return success;
}

bool ComanUtils::getImpedance(std::map<std::string, std::pair<yarp::sig::Vector, yarp::sig::Vector> >& impedance_map)
{
    bool atLeastAChainInImpedanceMode = false;
    impedance_map.clear();

    if(torso.getControlMode() == VOCAB_CM_IMPEDANCE_POS) {
        yarp::sig::Vector Kq, Dq;
        torso.getImpedance(Kq,Dq);
        impedance_map[torso.getChainName()] = Impedance(Kq,Dq);
        atLeastAChainInImpedanceMode = true;
    }

    if(right_arm.getControlMode() == VOCAB_CM_IMPEDANCE_POS) {
        yarp::sig::Vector Kq, Dq;
        right_arm.getImpedance(Kq,Dq);
        impedance_map[right_arm.getChainName()] = Impedance(Kq,Dq);
        atLeastAChainInImpedanceMode = true;
    }

    if(left_arm.getControlMode() == VOCAB_CM_IMPEDANCE_POS) {
        yarp::sig::Vector Kq, Dq;
        left_arm.getImpedance(Kq,Dq);
        impedance_map[left_arm.getChainName()] = Impedance(Kq,Dq);
        atLeastAChainInImpedanceMode = true;
    }

    if(right_leg.getControlMode() == VOCAB_CM_IMPEDANCE_POS) {
        yarp::sig::Vector Kq, Dq;
        right_leg.getImpedance(Kq,Dq);
        impedance_map[right_leg.getChainName()] = Impedance(Kq,Dq);
        atLeastAChainInImpedanceMode = true;
    }

    if(left_leg.getControlMode() == VOCAB_CM_IMPEDANCE_POS) {
        yarp::sig::Vector Kq, Dq;
        left_leg.getImpedance(Kq,Dq);
        impedance_map[left_leg.getChainName()] = Impedance(Kq,Dq);
        atLeastAChainInImpedanceMode = true;
    }

    if(right_hand.isAvailable && right_hand.getControlMode() == VOCAB_CM_IMPEDANCE_POS) {
        yarp::sig::Vector Kq, Dq;
        right_hand.getImpedance(Kq,Dq);
        impedance_map[right_hand.getChainName()] = Impedance(Kq,Dq);
        atLeastAChainInImpedanceMode = true;
    }

    if(left_hand.isAvailable && left_hand.getControlMode() == VOCAB_CM_IMPEDANCE_POS) {
        yarp::sig::Vector Kq, Dq;
        left_hand.getImpedance(Kq,Dq);
        impedance_map[left_hand.getChainName()] = Impedance(Kq,Dq);
        atLeastAChainInImpedanceMode = true;
    }

    return atLeastAChainInImpedanceMode;
}

void ComanUtils::sense(yarp::sig::Vector &q,
                       yarp::sig::Vector &qdot,
                       yarp::sig::Vector &tau)
{
    q = sensePosition();
    qdot = senseVelocity();
    tau = senseTorque();
}

yarp::sig::Vector &ComanUtils::sensePosition()
{
    right_arm.sensePosition(q_sensed_right_arm);
    left_arm.sensePosition(q_sensed_left_arm);
    torso.sensePosition(q_sensed_torso);
    right_leg.sensePosition(q_sensed_right_leg);
    left_leg.sensePosition(q_sensed_left_leg);

    fromRobotToIdyn(q_sensed_right_arm,
                    q_sensed_left_arm,
                    q_sensed_torso,
                    q_sensed_right_leg,
                    q_sensed_left_leg,
                    q_sensed);

    return q_sensed;
}

yarp::sig::Vector &ComanUtils::senseVelocity()
{
    right_arm.senseVelocity(qdot_sensed_right_arm);
    left_arm.senseVelocity(qdot_sensed_left_arm);
    torso.senseVelocity(qdot_sensed_torso);
    right_leg.senseVelocity(qdot_sensed_right_leg);
    left_leg.senseVelocity(qdot_sensed_left_leg);

    fromRobotToIdyn(qdot_sensed_right_arm,
                    qdot_sensed_left_arm,
                    qdot_sensed_torso,
                    qdot_sensed_right_leg,
                    qdot_sensed_left_leg,
                    qdot_sensed);

    return qdot_sensed;
}

yarp::sig::Vector &ComanUtils::senseTorque()
{
    right_arm.senseTorque(tau_sensed_right_arm);
    left_arm.senseTorque(tau_sensed_left_arm);
    torso.senseTorque(tau_sensed_torso);
    right_leg.senseTorque(tau_sensed_right_leg);
    left_leg.senseTorque(tau_sensed_left_leg);

    fromRobotToIdyn(tau_sensed_right_arm,
                    tau_sensed_left_arm,
                    tau_sensed_torso,
                    tau_sensed_right_leg,
                    tau_sensed_left_leg,
                    tau_sensed);

    return tau_sensed;
}

bool ComanUtils::senseHandsPosition(yarp::sig::Vector &q_left_hand,
                                    yarp::sig::Vector &q_right_hand)
{
    if(left_hand.isAvailable) {
        left_hand.sensePosition(q_sensed_left_hand);
        q_left_hand = q_sensed_left_hand;
    }

    if(right_hand.isAvailable) {
        right_hand.sensePosition(q_sensed_right_hand);
        q_right_hand = q_sensed_right_hand;
    }

    return hasHands();
}


void ComanUtils::fromIdynToRobot(const yarp::sig::Vector &_q,
                                 yarp::sig::Vector &_right_arm,
                                 yarp::sig::Vector &_left_arm,
                                 yarp::sig::Vector &_torso,
                                 yarp::sig::Vector &_right_leg,
                                 yarp::sig::Vector &_left_leg)
{
    idynutils.fromIDynToRobot(_q, _right_arm, idynutils.right_arm);
    idynutils.fromIDynToRobot(_q, _left_arm, idynutils.left_arm);
    idynutils.fromIDynToRobot(_q, _torso, idynutils.torso);
    idynutils.fromIDynToRobot(_q, _right_leg, idynutils.right_leg);
    idynutils.fromIDynToRobot(_q, _left_leg, idynutils.left_leg);
}

void ComanUtils::fromRobotToIdyn(const yarp::sig::Vector &_right_arm,
                                 const yarp::sig::Vector &_left_arm,
                                 const yarp::sig::Vector &_torso,
                                 const yarp::sig::Vector &_right_leg,
                                 const yarp::sig::Vector &_left_leg,
                                 yarp::sig::Vector &_q)
{
    idynutils.fromRobotToIDyn(_right_arm, _q, idynutils.right_arm);
    idynutils.fromRobotToIDyn(_left_arm, _q, idynutils.left_arm);
    idynutils.fromRobotToIDyn(_torso, _q, idynutils.torso);
    idynutils.fromRobotToIDyn(_right_leg, _q, idynutils.right_leg);
    idynutils.fromRobotToIDyn(_left_leg, _q, idynutils.left_leg);
}

bool ComanUtils::setPositionMode()
{
    return  (right_hand.isAvailable ? right_hand.setPositionMode() : true) &&
            (left_hand.isAvailable ? left_hand.setPositionMode() : true) &&
            torso.setPositionMode() &&
            right_arm.setPositionMode() &&
            left_arm.setPositionMode() &&
            right_leg.setPositionMode() &&
            left_leg.setPositionMode();
}

bool ComanUtils::isInPositionMode()
{
    return bodyIsInPositionMode() &&
           (!hasHands() || handsAreInPositionMode());
}

bool ComanUtils::setPositionDirectMode()
{
    return  (right_hand.isAvailable ? right_hand.setPositionDirectMode() : true) &&
            (left_hand.isAvailable ? left_hand.setPositionDirectMode() : true) &&
            torso.setPositionDirectMode() &&
            right_arm.setPositionDirectMode() &&
            left_arm.setPositionDirectMode() &&
            right_leg.setPositionDirectMode() &&
            left_leg.setPositionDirectMode();
}

bool ComanUtils::setTorqueMode()
{
    return  (right_hand.isAvailable ? right_hand.setPositionDirectMode() : true) &&
            (left_hand.isAvailable ? left_hand.setPositionDirectMode() : true) &&
            torso.setTorqueMode() &&
            right_arm.setTorqueMode() &&
            left_arm.setTorqueMode() &&
            right_leg.setTorqueMode() &&
            left_leg.setTorqueMode();
}

bool ComanUtils::setIdleMode()
{
    return  (right_hand.isAvailable ? right_hand.setIdleMode() : true) &&
            (left_hand.isAvailable ? left_hand.setIdleMode() : true) &&
            torso.setIdleMode() &&
            right_arm.setIdleMode() &&
            left_arm.setIdleMode() &&
            right_leg.setIdleMode() &&
            left_leg.setIdleMode();
}

bool ComanUtils::setImpedanceMode()
{
//    return  (right_hand.isAvailable ? right_hand.setImpedanceMode() : true) &&
//            (left_hand.isAvailable ? left_hand.setImpedanceMode() : true) &&
    return  (right_hand.isAvailable ? right_hand.setPositionDirectMode() : true) &&
            (left_hand.isAvailable ? left_hand.setPositionDirectMode() : true) &&
            torso.setImpedanceMode() &&
            right_arm.setImpedanceMode() &&
            left_arm.setImpedanceMode() &&
            right_leg.setImpedanceMode() &&
            left_leg.setImpedanceMode();
}

bool ComanUtils::isInImpedanceMode()
{
    return  torso.getControlMode() == VOCAB_CM_IMPEDANCE_POS &&
            right_arm.getControlMode() == VOCAB_CM_IMPEDANCE_POS &&
            left_arm.getControlMode() == VOCAB_CM_IMPEDANCE_POS &&
            right_leg.getControlMode() == VOCAB_CM_IMPEDANCE_POS &&
            left_leg.getControlMode() == VOCAB_CM_IMPEDANCE_POS;
}

walkman::yarp_single_chain_interface* const ComanUtils::getChainByName(const std::string chain_name) {
    if(chain_name == walkman::robot::left_arm) return &left_arm;
    if(chain_name == walkman::robot::right_arm) return &right_arm;
    if(chain_name == walkman::robot::left_leg) return &left_leg;
    if(chain_name == walkman::robot::right_leg) return &right_leg;
    if(chain_name == walkman::robot::torso) return &torso;
    if(chain_name == walkman::robot::right_hand) return &right_hand;
    if(chain_name == walkman::robot::left_hand) return &left_hand;
    return NULL;
}

bool ComanUtils::bodyIsInPositionMode()
{
    return  torso.getControlMode() == VOCAB_CM_POSITION &&
            right_arm.getControlMode() == VOCAB_CM_POSITION &&
            left_arm.getControlMode() == VOCAB_CM_POSITION &&
            right_leg.getControlMode() == VOCAB_CM_POSITION &&
            left_leg.getControlMode() == VOCAB_CM_POSITION;
}

bool ComanUtils::handsAreInPositionMode()
{
    return  (right_hand.isAvailable && right_hand.getControlMode() == VOCAB_CM_POSITION) &&
            (left_hand.isAvailable && left_hand.getControlMode() == VOCAB_CM_POSITION);
}
