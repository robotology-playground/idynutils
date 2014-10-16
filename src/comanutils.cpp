#include "drc_shared/comanutils.h"
#include "drc_shared/idynutils.h"

using namespace iCub::iDynTree;
using namespace yarp::math;

ComanUtils::ComanUtils(const std::string moduleName,
                       const int controlModeVocab):
    right_arm("right_arm", moduleName, "coman", controlModeVocab, true),
    right_leg("right_leg", moduleName, "coman", controlModeVocab, true),
    left_arm("left_arm", moduleName, "coman", controlModeVocab, true),
    left_leg("left_leg", moduleName, "coman", controlModeVocab, true),
    torso("torso", moduleName, "coman", controlModeVocab, true)
{
    this->number_of_joints = idynutils.coman_iDyn3.getNrOfDOFs();
    q_sensed.resize(this->number_of_joints,0.0);
    qdot_sensed.resize(this->number_of_joints,0.0);
    tau_sensed.resize(this->number_of_joints,0.0);
}

const unsigned int& ComanUtils::getNumberOfJoints() const
{
    return this->number_of_joints;
}

const std::vector<std::string> &ComanUtils::getJointNames() const
{
    return idynutils.getJointNames();
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

void ComanUtils::fromRobotToIdyn(yarp::sig::Vector &_right_arm,
                                 yarp::sig::Vector &_left_arm,
                                 yarp::sig::Vector &_torso,
                                 yarp::sig::Vector &_right_leg,
                                 yarp::sig::Vector &_left_leg,
                                 yarp::sig::Vector &_q)
{
    idynutils.fromRobotToIDyn(_right_arm, _q, idynutils.right_arm);
    idynutils.fromRobotToIDyn(_left_arm, _q, idynutils.left_arm);
    idynutils.fromRobotToIDyn(_torso, _q, idynutils.torso);
    idynutils.fromRobotToIDyn(_right_leg, _q, idynutils.right_leg);
    idynutils.fromRobotToIDyn(_left_leg, _q, idynutils.left_leg);
}


