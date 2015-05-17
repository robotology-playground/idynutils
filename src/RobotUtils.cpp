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

#define FT_SIZE 6

using namespace iCub::iDynTree;
using namespace yarp::math;

RobotUtils::RobotUtils(const std::string moduleName, 
		       const std::string robotName,
		       const std::string urdf_path, 
		       const std::string srdf_path) :
    whole_robot(walkman::robot::whole_robot, moduleName, robotName, true, walkman::controlTypes::none),
    idynutils( robotName, urdf_path, srdf_path ),
    _moduleName(moduleName)
{
    this->number_of_joints = idynutils.iDyn3_model.getNrOfDOFs();
    q_sensed.resize(this->number_of_joints,0.0);
    tau_sensed.resize(this->number_of_joints,0.0);
    q_motor_sensed.resize(this->number_of_joints,0.0);
    q_ref_feedback_sensed.resize(this->number_of_joints,0.0);

    loadIMUSensors();
    loadForceTorqueSensors();
}

bool RobotUtils::hasHands()
{
    return true;
}

bool RobotUtils::hasftSensors()
{
    return this->ftSensors.size() > 0;
}

const unsigned int& RobotUtils::getNumberOfJoints() const
{
    return this->number_of_joints;
}

const std::vector<std::string> &RobotUtils::getJointNames() const
{
    return idynutils.getJointNames();
}

void RobotUtils::move(const yarp::sig::Vector &_q)
{
    whole_robot.move(_q);
}

yarp::sig::Vector &RobotUtils::sensePosition()
{
    whole_robot.sensePosition(q_sensed);
    return q_sensed;
}

void RobotUtils::sensePosition(yarp::sig::Vector& q_sensed)
{
    whole_robot.senseTorque(q_sensed);
}


yarp::sig::Vector &RobotUtils::senseTorque()
{
    whole_robot.senseTorque(tau_sensed);
    return tau_sensed;
}

void RobotUtils::senseTorque(yarp::sig::Vector& tau_sensed)
{
    whole_robot.senseTorque(tau_sensed);
}

yarp::sig::Vector& RobotUtils::senseMotorPosition()
{
    whole_robot.senseMotorPosition(q_motor_sensed);
    return q_motor_sensed;
}

void RobotUtils::senseMotorPosition(yarp::sig::Vector& q_motor_sensed)
{
    whole_robot.senseMotorPosition(q_motor_sensed);

}

yarp::sig::Vector& RobotUtils::sensePositionRefFeedback()
{
    whole_robot.sensePositionRefFeedback(q_ref_feedback_sensed);
    return q_ref_feedback_sensed;
}

void RobotUtils::sensePositionRefFeedback(yarp::sig::Vector& q_ref_feedback_sensed)
{
    whole_robot.sensePositionRefFeedback(q_ref_feedback_sensed);
}

RobotUtils::ftReadings& RobotUtils::senseftSensors()
{
    ft_readings.clear();
    for( ftPtrMap::iterator i = ftSensors.begin(); i != ftSensors.end(); ++i)
    {
        senseftSensor(i->first, ft_readings[i->first]);
    }
    return ft_readings;
}

bool RobotUtils::senseftSensor(const std::string &ft_frame,
                               yarp::sig::Vector &ftReading)
{
    if(ftSensors.count(ft_frame)) {
        yarp::sig::Vector ft_global;
        bool ret = ft->sense(ft_global);
        ftReading = ft_global.subVector(ftSensors[ft_frame], ftSensors[ft_frame] + (FT_SIZE - 1));
        return ret;
    }
    return false;
}

bool RobotUtils::senseHandsPosition(yarp::sig::Vector &q_left_hand,
                                    yarp::sig::Vector &q_right_hand)
{
    whole_robot.sensePosition(q_sensed);
    q_left_hand = q_sensed[whole_robot.getNumberOfJoints() - 1];
    q_right_hand = q_sensed[whole_robot.getNumberOfJoints() - 2];
    return true;
}

void RobotUtils::fromIdynToRobot(const yarp::sig::Vector &_q,
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

void RobotUtils::fromIdynToRobot(const yarp::sig::Vector &_q,
                                 yarp::sig::Vector &_right_arm,
                                 yarp::sig::Vector &_left_arm,
                                 yarp::sig::Vector &_torso,
                                 yarp::sig::Vector &_right_leg,
                                 yarp::sig::Vector &_left_leg,
                                 yarp::sig::Vector &_head)
{
    idynutils.fromIDynToRobot(_q, _right_arm, idynutils.right_arm);
    idynutils.fromIDynToRobot(_q, _left_arm, idynutils.left_arm);
    idynutils.fromIDynToRobot(_q, _torso, idynutils.torso);
    idynutils.fromIDynToRobot(_q, _right_leg, idynutils.right_leg);
    idynutils.fromIDynToRobot(_q, _left_leg, idynutils.left_leg);
    if(idynutils.head.joint_numbers.size() > 0)
        idynutils.fromIDynToRobot(_q, _head, idynutils.head);
}

void RobotUtils::fromRobotToIdyn(const yarp::sig::Vector &_right_arm,
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

void RobotUtils::fromRobotToIdyn(const yarp::sig::Vector &_right_arm,
                                 const yarp::sig::Vector &_left_arm,
                                 const yarp::sig::Vector &_torso,
                                 const yarp::sig::Vector &_right_leg,
                                 const yarp::sig::Vector &_left_leg,
                                 const yarp::sig::Vector &_head,
                                 yarp::sig::Vector &_q)
{
    idynutils.fromRobotToIDyn(_right_arm, _q, idynutils.right_arm);
    idynutils.fromRobotToIDyn(_left_arm, _q, idynutils.left_arm);
    idynutils.fromRobotToIDyn(_torso, _q, idynutils.torso);
    idynutils.fromRobotToIDyn(_right_leg, _q, idynutils.right_leg);
    idynutils.fromRobotToIDyn(_left_leg, _q, idynutils.left_leg);
    if(idynutils.head.joint_numbers.size() > 0)
        idynutils.fromRobotToIDyn(_head, _q, idynutils.head);
}

bool RobotUtils::setControlType(const walkman::ControlType& controlType)
{
    std::cout << "Setting control type : " << controlType.toString() << std::endl;
    return  whole_robot.setControlType(controlType);
}

bool RobotUtils::setPositionDirectMode()
{
    return setControlType(walkman::controlTypes::positionDirect);
}

bool RobotUtils::setIdleMode()
{
    return setControlType(walkman::controlTypes::idle);
}

bool RobotUtils::isInPositionDirectMode()
{
    return bodyIsInPositionDirectMode();
}

bool RobotUtils::bodyIsInPositionDirectMode()
{
    return  whole_robot.isInPositionDirectMode();
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
    // create
    ft = std::shared_ptr<yarp_ft_interface>( new yarp_ft_interface("whole_robot_ft",
                                _moduleName,
                                idynutils.getRobotName()));
    std::vector<srdf::Model::Group> robot_groups = idynutils.robot_srdf->getGroups();
    int offset = 0;
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

                        ftSensors[reference_frame] = FT_SIZE * offset;
//                         ft_reference_frames.push_back(reference_frame);
//                         std::cout << "ft on " << reference_frame << " loaded" << std::endl;
                    } catch(...) {
                        std::cerr << "Error loading " << reference_frame << " ft " << std::endl;
                        return false;}
                }
                return true;
            }
        }
        offset++;
    }
    std::cout << "Robot does not have any ft sensor" << std::endl;
    return false;
}

///TODO: CHECK LINK IN GROUP imu_sensors in SRDF!!!
bool RobotUtils::loadIMUSensors()
{
    std::vector<srdf::Model::Group> robot_groups = idynutils.robot_srdf->getGroups();
    for(auto group: robot_groups)
    {
        if (group.name_ == walkman::robot::imu_sensors)
        {
            if(group.joints_.size() > 0) {
                try {
                    IMU = IMUPtr(new yarp_IMU_interface(_moduleName, idynutils.getRobotName(),true));
                    std::cout << "IMU loaded" << std::endl;
                    return true;
                } catch(...) {
                    std::cerr << "Error loading IMU" << std::endl;
                    return false;
                }
            }
        }
    }
    std::cout << "Robot does not have an IMU" << std::endl;
    return false;
}