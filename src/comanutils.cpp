#include "drc_shared/comanutils.h"
#include "drc_shared/idynutils.h"

using namespace iCub::iDynTree;
using namespace yarp::math;

ComanUtils::ComanUtils(const std::string moduleName,
                       const int controlModeVocab):
    right_hand(walkman::robot::right_hand, moduleName, "coman", true, controlModeVocab),
    right_arm(walkman::robot::right_arm, moduleName, "coman", true, controlModeVocab),
    right_leg(walkman::robot::right_leg, moduleName, "coman", true, controlModeVocab),
    left_hand(walkman::robot::left_hand, moduleName, "coman", true, controlModeVocab),
    left_arm(walkman::robot::left_arm, moduleName, "coman", true, controlModeVocab),
    left_leg(walkman::robot::left_leg, moduleName, "coman", true, controlModeVocab),
    torso(walkman::robot::torso, moduleName, "coman", true, controlModeVocab),
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
    this->number_of_joints = idynutils.coman_iDyn3.getNrOfDOFs();
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
        right_hand.move(q_sensed_right_hand);
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


