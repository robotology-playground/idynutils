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
    _moduleName(moduleName),left_hand_index(left_hand_i),right_hand_index(right_hand_i),neck_p_index(neck_p_i),neck_y_index(neck_y_i)
{
    this->number_of_joints = idynutils.iDyn3_model.getNrOfDOFs();
    q_sensed.resize(this->number_of_joints,0.0);
    tau_sensed.resize(this->number_of_joints,0.0);
    q_motor_sensed.resize(this->number_of_joints,0.0);
    q_ref_feedback_sensed.resize(this->number_of_joints,0.0);
    left_hand_i = whole_robot.getNumberOfJoints() - 1;
    right_hand_i = whole_robot.getNumberOfJoints() - 2;
    loadIMUSensors();
    neck_p_i = idynutils.head.joint_numbers[1];
    neck_y_i = idynutils.head.joint_numbers[0];
    //get neck index, initialize neck_y_index neck_p_index and j_29
    int j=0;
    for (int i=0;i<31;i++)
    {
      if (i!=neck_p_index && i!=neck_y_index)
      {
	j_29[j]=i;
	j++;
      }
    }
    
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

const unsigned int RobotUtils::getNumberOfKinematicJoints() const
{
    return this->number_of_joints;
}

const unsigned int RobotUtils::getNumberOfActuatedJoints() const
{
    return this->whole_robot.getNumberOfJoints();
}

const std::vector<std::string> &RobotUtils::getJointNames() const
{
    return idynutils.getJointNames();
}

void RobotUtils::move29(const yarp::sig::Vector &_q)
{
    //q is 31
    double _qtemp[29];
    for (int i=0;i<29;i++)
        _qtemp[i]=_q[j_29[i]];
    whole_robot.move(_qtemp,j_29,29);
}

void RobotUtils::moveHands(const yarp::sig::Vector &_q)
{
    int temp[2];
    temp[0]=left_hand_index;
    temp[1]=right_hand_index;
    double _qtemp[2];
    _qtemp[0]=_q[left_hand_index];
    _qtemp[1]=_q[right_hand_index];
    whole_robot.move(_qtemp,temp,2);
}

void RobotUtils::moveHands(double q_left, double q_right)
{
    int temp[2];
    temp[0]=left_hand_index;
    temp[1]=right_hand_index;
    double _qtemp[2];
    _qtemp[0]= q_left;
    _qtemp[1]= q_right;
    whole_robot.move(_qtemp,temp,2);
}

void RobotUtils::moveNeck(const yarp::sig::Vector &_q)
{
    int temp[2];
    temp[0]=neck_y_index;
    temp[1]=neck_p_index;
    double _qtemp[2];
    _qtemp[0]=_q[neck_y_index];
    _qtemp[1]=_q[neck_p_index];
    whole_robot.move(_qtemp,temp,2);
}

void RobotUtils::moveNeck(double q_yaw, double q_pitch)
{
    int temp[2];
    temp[0]=neck_y_index;
    temp[1]=neck_p_index;
    double _qtemp[2];
    _qtemp[0] = q_yaw;
    _qtemp[1] = q_pitch;
    whole_robot.move(_qtemp,temp,2);
}


yarp::sig::Vector &RobotUtils::sensePosition()
{
    whole_robot.sensePosition(q_sensed);
    return q_sensed;
}

void RobotUtils::sensePosition(yarp::sig::Vector& q_sensed)
{
    whole_robot.sensePosition(q_sensed);
}

void RobotUtils::sensePositionTimed(yarp::sig::Vector& jointPosition, yarp::sig::Vector &dataTiming)
{

	whole_robot.sensePositionTimed(q_sensed, dataTiming);                                                     
	
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
        ft->getLastTimeStamp();
        return ret;
    }
    return false;
}
bool RobotUtils::senseftSensorTimed(const std::string &ft_frame,
                               yarp::sig::Vector &ftReading, 
                               double &timeStamp
                                   )
{
    if(ftSensors.count(ft_frame)) {
        yarp::sig::Vector ft_global;
        bool ret = ft->sense(ft_global);
        ftReading = ft_global.subVector(ftSensors[ft_frame], ftSensors[ft_frame] + (FT_SIZE - 1));
        timeStamp = ft->getLastTimeStamp();
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

void RobotUtils::fromIdynToRobot29(const yarp::sig::Vector &_q,
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

void RobotUtils::fromIdynToRobot31(const yarp::sig::Vector &_q,
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

void RobotUtils::fromRobotToIdyn29(const yarp::sig::Vector &_right_arm,
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

void RobotUtils::fromRobotToIdyn31(const yarp::sig::Vector &_right_arm,
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
    std::cout << ">------------------------ Loading FT sensor ------------------ !!!!!!!"<< std::endl;
    // create
    ft = std::shared_ptr<yarp_ft_interface>( new yarp_ft_interface("whole_robot_ft",
                                _moduleName,
                                idynutils.getRobotName()));
    //std::vector<srdf::Model::Group> robot_groups = idynutils.robot_srdf->getGroups();
//     <!--legR legL armR armL -->
    std::vector<std::string> ft_joints = {"r_leg_ft","l_leg_ft","r_arm_ft","l_arm_ft"};
    int offset = 0;
                for(auto reference_frame : ft_joints)
                {
                  std::cout << " on frame " << reference_frame << ". Loading ft ..." << std::endl; std::cout.flush();

                    try {

                        ftSensors[reference_frame] = FT_SIZE * offset;
//                         ft_reference_frames.push_back(reference_frame);
//                         std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAA ft on " << reference_frame << " loaded on offset"<<ftSensors[reference_frame] << std::endl;
                        offset++;
                    } catch(...) {
                        std::cerr << "Error loading " << reference_frame << " ft " << std::endl;
                        return false;}
                }
                return true;
  
}

///TODO: CHECK LINK IN GROUP imu_sensors in SRDF!!!
bool RobotUtils::loadIMUSensors()
{
//     std::vector<srdf::Model::Group> robot_groups = idynutils.robot_srdf->getGroups();
//     for(auto group: robot_groups)
//     {
//         if (group.name_ == walkman::robot::imu_sensors)
//         {
//             if(group.joints_.size() > 0) {
//                 try {
//                     IMU = IMUPtr(new yarp_IMU_interface(_moduleName, idynutils.getRobotName(),true));
//                     std::cout << "IMU loaded" << std::endl;
//                     return true;
//                 } catch(...) {
//                     std::cerr << "Error loading IMU" << std::endl;
//                     return false;
//                 }
//             }
//         }
//     }
//     std::cout << "Robot does not have an IMU" << std::endl;
    IMU = IMUPtr(new yarp_IMU_interface(_moduleName, idynutils.getRobotName(),true));
    return true;
}