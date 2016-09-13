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

#include "idynutils/yarp_single_chain_interface.h"
#include <algorithm>
#include <assert.h>

using namespace walkman;
using namespace yarp::dev;

yarp_single_chain_interface::yarp_single_chain_interface(std::string kinematic_chain,
                                                         std::string module_prefix_with_no_slash,
                                                         std::string robot_name,
                                                         bool useSI,
                                                         const ControlType& controlType):
    module_prefix(robot_name + "/" + module_prefix_with_no_slash),
    kinematic_chain(kinematic_chain),
    isAvailable(internal_isAvailable),
    _useSI(useSI),
    _robot_name(robot_name),
    joints_number(0),
    q_buffer(1,0.0),
    qdot_buffer(1,0.0),
    tau_buffer(1,0.0),
    q_motor_buffer(1,0.0),
    // init list for control interfaces
    encodersMotor(NULL), controlLimits(NULL), controlMode(NULL),
    interactionMode(NULL), pidControl(NULL), positionControl(NULL),
    positionDirect(NULL), impedancePositionControl(NULL), torqueControl(NULL),\
    velocityControl(NULL)
{
    internal_isAvailable=false;
    if (module_prefix_with_no_slash.find_first_of("/")!=std::string::npos)
    {
        std::cout<<"ERROR: do not insert / into module prefix"<<std::endl;
        return;
    }
    if(createPolyDriver(kinematic_chain.c_str(), _robot_name.c_str(), polyDriver))
    {
        bool temp=true;
        temp=temp&&polyDriver.view(encodersMotor);
        temp=temp&&polyDriver.view(controlMode);
        temp=temp&&polyDriver.view(interactionMode);
        temp=temp&&polyDriver.view(motorEncoders);
        temp=temp&&polyDriver.view(positionControl);
        temp=temp&&polyDriver.view(positionDirect);
        temp=temp&&polyDriver.view(impedancePositionControl);
        temp=temp&&polyDriver.view(torqueControl);
        temp=temp&&polyDriver.view(velocityControl);
        internal_isAvailable = temp;

        // optional interfaces
        if(polyDriver.view(pidControl))
            std::cout << "Loaded PID control interface for "
                      << _robot_name << "/"
                      << kinematic_chain << std::endl;
        if(polyDriver.view(controlLimits))
            std::cout << "Loaded control limits interface for "
                      << _robot_name << "/"
                      << kinematic_chain << std::endl;
    }
    if (!internal_isAvailable)
    {
        return;
    }
    
    encodersMotor->getAxes(&(this->joints_number));
    q_buffer.resize(joints_number);
    qdot_buffer.resize(joints_number);
    tau_buffer.resize(joints_number);
    q_motor_buffer.resize(joints_number);
    q_ref_feedback_buffer.resize(joints_number);
    
    if(!setControlType(controlType))
        std::cout << "PROBLEM initializing " << kinematic_chain << " with " << controlType << std::endl;
}


bool yarp_single_chain_interface::setReferenceSpeeds( const yarp::sig::Vector& maximum_velocity )
{
    yarp::sig::Vector maximum_velocity_deg;

    assert(maximum_velocity.size() == joints_number);
    if(_controlType != walkman::controlTypes::position) {
        std::cout << "Tryng to set Reference Speed for chain " << this->getChainName()
                  << " which is not in Position mode" << std::endl;
        return false;
    }

    if(_useSI) maximum_velocity_deg = convertMotorCommandFromSI(maximum_velocity);
    else maximum_velocity_deg = maximum_velocity;

    bool set_success = true;
    for( int i = 0; i < joints_number && set_success; i++ ) {
        set_success = set_success && positionControl->setRefSpeed( i, maximum_velocity_deg[i] );
    }
    return set_success;

//    // set the speed references
//    return positionControl->setRefSpeeds( maximum_velocity_deg.data() );
}

bool yarp_single_chain_interface::setReferenceSpeed( const double& maximum_velocity )
{
    // set the speed references
    yarp::sig::Vector maximum_velocities = yarp::sig::Vector( joints_number,
                                                              maximum_velocity );

    return this->setReferenceSpeeds(maximum_velocities);
}

bool walkman::yarp_single_chain_interface::setImpedance(const yarp::sig::Vector &Kq, const yarp::sig::Vector &Dq)
{
    // get joints number
    int impedanceSize = std::min(Kq.size(),
                                Dq.size());

    assert(impedanceSize == joints_number);

    if(_controlType != walkman::controlTypes::impedance) {
        std::cout << "Tryng to set Impedance for chain " << this->getChainName()
                  << "which is not in Impedance mode" << std::endl;
        return false;
    }

    bool set_success = true;
    for(unsigned int i = 0; i < joints_number; ++i) {
        double Kqi;
        double Dqi;
        if(_useSI) {
            Kqi = convertImpedanceFromSI(Kq[i]);
            Dqi = convertImpedanceFromSI(Dq[i]);
        } else {
            Kqi = Kq[i];
            Dqi = Dq[i];
        }

            set_success = set_success && impedancePositionControl->setImpedance(i, Kqi, Dqi);
    }
    return set_success;
}

bool walkman::yarp_single_chain_interface::getImpedance(yarp::sig::Vector &Kq, yarp::sig::Vector &Dq)
{
    if(Kq.size() < joints_number)
        Kq.resize(joints_number);
    if(Dq.size() > joints_number)
        Dq.resize(joints_number);

    bool set_success = true;
    for(unsigned int i = 0; i < joints_number; ++i) {
        set_success = set_success && impedancePositionControl->getImpedance(i, &Kq[i], &Dq[i]);
    }

    if(_useSI) {
        convertEncoderToSI(Kq);
        convertEncoderToSI(Dq);
    }

    return set_success && (_controlType == walkman::controlTypes::impedance);
}

bool walkman::yarp_single_chain_interface::getControlTypes(walkman::yarp_single_chain_interface::ControlTypes &controlTypes)
{
    std::vector<int> controlModes;
    std::vector<yarp::dev::InteractionModeEnum> interactionModes;
    if( this->getControlModes(controlModes) &&
        this->getInteractionModes(interactionModes)) {
        controlTypes.resize(joints_number);
        for(unsigned int i = 0; i < joints_number; ++i) {
            controlTypes[i] = walkman::ControlType::fromYarp(controlModes[i],interactionModes[i]);
        }
        return true;
    } else return false;
}

bool walkman::yarp_single_chain_interface::setControlTypes(const walkman::yarp_single_chain_interface::ControlTypes &controlTypes)
{
    assert(controlTypes.size() == joints_number);
    std::vector<int> controlModes(joints_number,0);
    std::vector<yarp::dev::InteractionModeEnum> interactionModes(joints_number,
                                                                 (yarp::dev::InteractionModeEnum)0);
    for(unsigned int i = 0; i < joints_number; ++i) {
        controlModes[i] = controlTypes[i].toYarp().first;
        interactionModes[i] = controlTypes[i].toYarp().second;
    }

    return  controlMode->setControlModes(controlModes.data()) &&
            interactionMode->setInteractionModes(interactionModes.data());
}

void walkman::yarp_single_chain_interface::vectorsFromControlTypes(const walkman::yarp_single_chain_interface::ControlTypes &controlTypes,
                                                                   std::vector<int> &controlModes,
                                                                   std::vector<yarp::dev::InteractionModeEnum> &interactionModes)
{
    assert(controlTypes.size() == joints_number);

    controlModes.reserve(joints_number);
    controlModes.assign(joints_number, 0);
    interactionModes.reserve(joints_number);
    interactionModes.assign(joints_number, (yarp::dev::InteractionModeEnum)0);

    for(unsigned int i = 0; i < joints_number; ++i) {
        controlModes[i] = controlTypes[i].toYarp().first;
        interactionModes[i] = controlTypes[i].toYarp().second;
    }

    return;
}

walkman::yarp_single_chain_interface::ControlTypes walkman::yarp_single_chain_interface::controlTypesFromVectors(const std::vector<int> &controlModes,
                                                                                                                 const std::vector<yarp::dev::InteractionModeEnum> &interactionModes)
{
    assert(controlModes.size() == interactionModes.size());
    unsigned int controlTypesSize = controlModes.size();
    ControlTypes controlTypes;
    for(unsigned int i = 0; i < controlTypesSize; ++i) {
        controlTypes.push_back(walkman::ControlType::fromYarp(controlModes[i],interactionModes[i]));
    }
    return controlTypes;
}

bool walkman::yarp_single_chain_interface::getControlModes(std::vector<int> &controlModes)
{
    controlModes.reserve(joints_number);
    controlModes.assign(joints_number, 0);
    return controlMode->getControlModes(controlModes.data());
}

std::vector<int> walkman::yarp_single_chain_interface::getControlModes()
{
    std::vector<int> controlModes;
    this->getControlModes(controlModes);
    return controlModes;
}

bool walkman::yarp_single_chain_interface::getInteractionModes(std::vector<InteractionModeEnum> &interactionModes)
{
    interactionModes.reserve(joints_number);
    interactionModes.assign(joints_number, (yarp::dev::InteractionModeEnum)0);
    return interactionMode->getInteractionModes(interactionModes.data());
}

std::vector<yarp::dev::InteractionModeEnum> walkman::yarp_single_chain_interface::getInteractionModes()
{
    std::vector<yarp::dev::InteractionModeEnum> interactionModes;
    getInteractionModes(interactionModes);
    return interactionModes;
}


bool yarp_single_chain_interface::setIdleMode()
{
    return setControlType(walkman::controlTypes::idle);
}

bool walkman::yarp_single_chain_interface::isInIdleMode()
{
    return (getControlType() == walkman::controlTypes::idle);
}

bool yarp_single_chain_interface::setTorqueMode()
{
    return setControlType(walkman::controlTypes::torque);
}

bool walkman::yarp_single_chain_interface::isInTorqueMode()
{
    return (getControlType() == walkman::controlTypes::torque);
}

bool yarp_single_chain_interface::setPositionMode()
{
    return setControlType(walkman::controlTypes::position);
}

bool walkman::yarp_single_chain_interface::isInPositionMode()
{
    return (getControlType() == walkman::controlTypes::position);
}

bool yarp_single_chain_interface::setImpedanceMode()
{
    return setControlType(walkman::controlTypes::impedance);
}

bool walkman::yarp_single_chain_interface::isInImpedanceMode()
{
    return (getControlType() == walkman::controlTypes::impedance);
}

bool yarp_single_chain_interface::setVelocityMode()
{
    return setControlType(walkman::controlTypes::velocity);
}

bool walkman::yarp_single_chain_interface::isInVelocityMode()
{
    return (getControlType() == walkman::controlTypes::velocity);
}

bool walkman::yarp_single_chain_interface::useSI() const
{
    return _useSI;
}

bool yarp_single_chain_interface::setPositionDirectMode()
{
    return setControlType(walkman::controlTypes::positionDirect);
}

bool walkman::yarp_single_chain_interface::isInPositionDirectMode()
{
    return (getControlType() == walkman::controlTypes::positionDirect);
}

yarp::sig::Vector yarp_single_chain_interface::senseTorque() {
    torqueControl->getTorques(tau_buffer.data());
    return tau_buffer;
}

void yarp_single_chain_interface::senseTorque(yarp::sig::Vector& tau_sensed) {
    if(tau_sensed.size() != this->joints_number)
        tau_sensed.resize(this->joints_number);
    torqueControl->getTorques(tau_sensed.data());
}

yarp::sig::Vector yarp_single_chain_interface::senseVelocity() {
    encodersMotor->getEncoderSpeeds(qdot_buffer.data());
    if(_useSI) convertEncoderToSI(qdot_buffer);
    return qdot_buffer;
}

void yarp_single_chain_interface::senseVelocity(yarp::sig::Vector& velocity_sensed) {
    if(velocity_sensed.size() != this->joints_number)
        velocity_sensed.resize(this->joints_number);
    encodersMotor->getEncoderSpeeds(velocity_sensed.data());
    if(_useSI) convertEncoderToSI(velocity_sensed);
}

yarp::sig::Vector yarp_single_chain_interface::sensePosition() {
    return sense();
}

void yarp_single_chain_interface::sensePosition(yarp::sig::Vector& q_sensed) {
    sense(q_sensed);
}

yarp::sig::Vector yarp_single_chain_interface::senseMotorPosition() {
    motorEncoders->getMotorEncoders(q_motor_buffer.data());
    if(_useSI) convertEncoderToSI(q_motor_buffer);
    return q_motor_buffer;
}

void yarp_single_chain_interface::senseMotorPosition(yarp::sig::Vector& q_sensed) {
    if(q_sensed.size() != this->joints_number)
        q_sensed.resize(this->joints_number);
     motorEncoders->getMotorEncoders(q_sensed.data());
    if(_useSI) convertEncoderToSI(q_sensed);
}

yarp::sig::Vector yarp_single_chain_interface::sense() {
    encodersMotor->getEncoders(q_buffer.data());
    if(_useSI) convertEncoderToSI(q_buffer);
    return q_buffer;
}

void yarp_single_chain_interface::sensePositionRefFeedback(yarp::sig::Vector& q_position_ref_feedback) {
    if(q_position_ref_feedback.size() != this->joints_number)
        q_position_ref_feedback.resize(this->joints_number);
    positionDirect->getRefPositions(q_position_ref_feedback.data());
    if(_useSI) convertEncoderToSI(q_position_ref_feedback);
}

yarp::sig::Vector yarp_single_chain_interface::sensePositionRefFeedback() {
    positionDirect->getRefPositions(q_ref_feedback_buffer.data());
    if(_useSI) convertEncoderToSI(q_ref_feedback_buffer);
    return q_ref_feedback_buffer;
}

void yarp_single_chain_interface::sense(yarp::sig::Vector& q_sensed) {
    if(q_sensed.size() != this->joints_number)
        q_sensed.resize(this->joints_number);
    encodersMotor->getEncoders(q_sensed.data());
    if(_useSI) convertEncoderToSI(q_sensed);
}

void yarp_single_chain_interface::move(const yarp::sig::Vector& u_d)
{
    yarp::sig::Vector u_sent(u_d);

    switch (_controlType.toYarp().first)
    {
        case VOCAB_CM_POSITION_DIRECT:
        case VOCAB_CM_IMPEDANCE_POS:
            if(_useSI) convertMotorCommandFromSI(u_sent);
            if(!positionDirect->setPositions(u_sent.data()))
                std::cout<<"Cannot move "<< kinematic_chain <<" using Direct Position Ctrl"<<std::endl;
            break;
        case VOCAB_CM_POSITION:
            if(_useSI) convertMotorCommandFromSI(u_sent);
            if(!positionControl->positionMove(u_sent.data()))
                std::cout<<"Cannot move "<< kinematic_chain <<" using Position Ctrl"<<std::endl;
            break;
        case VOCAB_CM_TORQUE:
            if(!torqueControl->setRefTorques(u_sent.data()))
                std::cout<<"Cannot move "<< kinematic_chain <<" using Torque Ctrl"<<std::endl;
            break;
        case VOCAB_CM_VELOCITY:
            if(!velocityControl->velocityMove(u_sent.data()))
                std::cout<<"Cannot move "<< kinematic_chain <<" using Velocity Ctrl"<<std::endl;
            break;
        /*case VOCAB_CM_MIXED:
            break;
        */
        case VOCAB_CM_IDLE:        
        default:
                std::cout<<"Cannot move "<< kinematic_chain <<" using Idle Ctrl"<<std::endl;
            break;
    }
}

bool walkman::yarp_single_chain_interface::moveDone()
{
    bool moveDone;
    switch (_controlType.toYarp().first)
    {
        case VOCAB_CM_POSITION_DIRECT:
        case VOCAB_CM_IMPEDANCE_POS:
	    
            break;
        case VOCAB_CM_POSITION:
	    positionControl->checkMotionDone(&moveDone);
            break;
        case VOCAB_CM_TORQUE:

            break;
        case VOCAB_CM_IDLE:
        default:

            break;
    }
    return moveDone;
}


const int& yarp_single_chain_interface::getNumberOfJoints() const
{
    return this->joints_number;
}

const std::string& yarp_single_chain_interface::getChainName() const
{
    return kinematic_chain;
}

bool walkman::yarp_single_chain_interface::setControlType(const ControlType &controlType)
{    
    if(controlType.toYarp().first == VOCAB_CM_UNKNOWN)
        return getControlType(_controlType);
    else {
        bool check = true;

        std::cout<<this->getChainName()<<":"<<std::endl;

        for(unsigned int i = 0; check && i < joints_number; ++i) {
            if(controlType.toYarp().second != VOCAB_IM_UNKNOWN){
                yarp::dev::InteractionModeEnum actual_interaction_mode;
                interactionMode->getInteractionMode(i, &actual_interaction_mode);
                if( actual_interaction_mode != controlType.toYarp().second ) {
                    check = check && interactionMode->setInteractionMode(i, controlType.toYarp().second);
                    std::cout<<"    Changing Interaction Mode for joint "<<i<<std::endl;
                }
            }

            int actual_control_mode;
            controlMode->getControlMode(i, &actual_control_mode);
            if( actual_control_mode != controlType.toYarp().first ) {
                check = check && controlMode->setControlMode(i, controlType.toYarp().first);
                std::cout<<"    Changing Control Mode for joint "<<i<<std::endl;
            }

            if(!check) {
                std::cout << "  ERROR setting " << kinematic_chain << " to " << controlType <<
                             ". Kinematic chain in inconsistent state at joint " << i;
            }
        }

        if(!check)  return false;
        
        yarp::os::Time::delay(0.01); // set and get too fast
        ControlType currentControlType;
        if(!this->getControlType(currentControlType)) {
            std::cout << "  ERROR asking the current control Type for verification. Something went wrong" << std::endl;
            return false;
        }

        _controlType = currentControlType;

        if(controlType != currentControlType) {
            std::cout << "  ERROR: we were able to set the desired control type, but upon check the robot"
                      << " returns the control type was not updated." << std::endl;
            return false;
        }
        std::cout<<"    Changed Control Type for "<<this->getChainName()<<": "<<_controlType.toString()<<std::endl;
        std::cout<<std::endl;

        return true;
    }
}

walkman::ControlType walkman::yarp_single_chain_interface::getControlType() throw()
{
    if(getControlType(_controlType)) return _controlType;
    throw("Unable to correctly read control type");
}

bool walkman::yarp_single_chain_interface::getControlType(ControlType &controlType)
{
    unsigned int i = 0;

    int ctrlMode0, ctrlMode;
    bool ableToGetCtrlMode = true;
    bool ctrlModeIsConsistent = true;
    ableToGetCtrlMode = controlMode->getControlMode(0, &ctrlMode0);

    for(i = 1; ableToGetCtrlMode && ctrlModeIsConsistent && i < joints_number; ++i) {
        ableToGetCtrlMode = ableToGetCtrlMode && controlMode->getControlMode(i, &ctrlMode);
        ctrlModeIsConsistent = (ctrlMode0 == ctrlMode);
    }

    if(!ableToGetCtrlMode) {
        std::cout << "ERROR asking the current control Type for verification. Something went "
                  << "wrong while asking joint " << i << " for its control mode.";
        return false;
    }

//     if(!ctrlModeIsConsistent) {
//         std::cout << "ERROR: joint" << i << " is in different control mode than joint 0 for this chain.";
//         return false;
//     }


    yarp::dev::InteractionModeEnum intMode0, intMode;
    bool ableToGetIntMode = true;
    bool intModeIsConsistent = true;
    ableToGetIntMode = interactionMode->getInteractionMode(0, &intMode0);

    for(i = 1; ableToGetIntMode && intModeIsConsistent && i < joints_number; ++i) {
        ableToGetIntMode = ableToGetIntMode && interactionMode->getInteractionMode(i, &intMode);
        intModeIsConsistent = (intMode0 == intMode);
    }

    if(!ableToGetIntMode) {
        std::cout << "ERROR asking the current control Type for verification. Something went "
                  << "wrong while asking joint " << i << " for its interaction mode.";
        return false;
    }

//     if(!intModeIsConsistent) {
//         std::cout << "ERROR: joint" << i << " is in different interaction mode than joint 0 for this chain.";
//         return false;
//     }

    controlType = ControlType::fromYarp(ctrlMode0, intMode0);
    return true;
}

bool walkman::yarp_single_chain_interface::getJointLimits(yarp::sig::Vector &lowerLimits, yarp::sig::Vector &upperLimits)
{
    if(controlLimits != NULL)
    {
        lowerLimits.resize(this->joints_number);
        upperLimits.resize(this->joints_number);
        bool res = true;

        for(unsigned int i = 0; i < this->joints_number; ++i)
        {
            bool limitsQueryOk = controlLimits->getLimits(i, &lowerLimits[i], &upperLimits[i]);

            res = res && limitsQueryOk;
            if(!limitsQueryOk)
                std::cout << this->_robot_name << "/"
                          << this->kinematic_chain << " : "
                          << "Error while querying joint limits for axis " << i << std::endl;
        }

        if(res && _useSI)
        {
            convertEncoderToSI(lowerLimits);
            convertEncoderToSI(upperLimits);
        }

        return res;
    } else return false;
}

bool yarp_single_chain_interface::createPolyDriver(const std::string& kinematic_chain, const std::string &robot_name, yarp::dev::PolyDriver& polyDriver)
{
    yarp::os::Property options;
    options.put("robot", robot_name);
    options.put("device", "remote_controlboard");

    yarp::os::ConstString s;
    s = "/"+module_prefix+"/" + kinematic_chain;

    options.put("local", s.c_str());

    yarp::os::ConstString ss;
    ss = "/" + robot_name + "/" + kinematic_chain;
    options.put("remote", ss.c_str());

    polyDriver.open(options);
    if (!polyDriver.isValid()) {
        std::cout<<"Device "<<kinematic_chain<<" not available."<<std::endl;
        return false;
    }
    else {
        std::cout<<"Device "<<kinematic_chain<<" available."<<std::endl;
        return true;
    }
}


yarp_single_chain_interface::~yarp_single_chain_interface()
{
    if (polyDriver.isValid())
        polyDriver.close();
}

inline void yarp_single_chain_interface::convertEncoderToSI(yarp::sig::Vector &vector)
{
    for(unsigned int i = 0; i < vector.size(); ++i) {
        vector[i] *= M_PI / 180.0;
    }
}

inline void yarp_single_chain_interface::convertImpedanceFromSI(yarp::sig::Vector &vector)
{
    for(unsigned int i = 0; i < vector.size(); ++i) {
        vector[i] *= M_PI / 180.0;
    }
}

inline double yarp_single_chain_interface::convertImpedanceFromSI(const double& in) const
{
    return in * M_PI / 180.0;
}

inline void yarp_single_chain_interface::convertMotorCommandFromSI(yarp::sig::Vector &vector)
{
    for(unsigned int i = 0; i < vector.size(); ++i) {
        vector[i] *= 180.0 / M_PI;
    }
}

inline yarp::sig::Vector yarp_single_chain_interface::convertMotorCommandFromSI(const yarp::sig::Vector &vector_in)
{
    yarp::sig::Vector vector_out(vector_in);
    for(unsigned int i = 0; i < vector_out.size(); ++i) {
        vector_out[i] *= 180.0 / M_PI;
    }
    return vector_out;
}

inline double yarp_single_chain_interface::convertMotorCommandFromSI(const double& in) const
{
    return in * 180.0 / M_PI;
}
