#include "drc_shared/yarp_single_chain_interface.h"

using namespace walkman::drc;

yarp_single_chain_interface::yarp_single_chain_interface(std::string kinematic_chain,
                                                         std::string module_prefix_with_no_slash,
                                                         bool useSI,
                                                         const int controlModeVocab):
    module_prefix(module_prefix_with_no_slash),
    kinematic_chain(kinematic_chain),
    isAvailable(internal_isAvailable),
    _useSI(useSI)
{
    internal_isAvailable=false;
    if (module_prefix_with_no_slash.find_first_of("/")!=std::string::npos)
    {
        std::cout<<"ERROR: do not insert / into module prefix"<<std::endl;
        return;
    }
    if(createPolyDriver(kinematic_chain.c_str(), polyDriver))
    {
        bool temp=true;
        temp=temp&&polyDriver.view(encodersMotor);
        temp=temp&&polyDriver.view(positionDirect);
        temp=temp&&polyDriver.view(controlMode);
        temp=temp&&polyDriver.view(positionControl);
        temp=temp&&polyDriver.view(impedancePositionControl);
        temp=temp&&polyDriver.view(torqueControl);
	temp=temp&&polyDriver.view(velocityControl);
        internal_isAvailable = temp;
    }
    if (!internal_isAvailable)
    {
        //TODO
        return;
    }
    
    encodersMotor->getAxes(&(this->joint_numbers));
    q_buffer.resize(joint_numbers);
    qdot_buffer.resize(joint_numbers);
    tau_buffer.resize(joint_numbers);

    switch(controlModeVocab) {
        case VOCAB_CM_TORQUE:
        for(unsigned int i = 0; i < joint_numbers; ++i)
        {
            controlMode->setTorqueMode(i);
        }
        break;
        case VOCAB_CM_IMPEDANCE_POS:
        for(unsigned int i = 0; i < joint_numbers; ++i)
        {
            controlMode->setImpedancePositionMode(i);
        }
        break;
        case VOCAB_CM_VELOCITY:
        for(unsigned int i = 0; i < joint_numbers; ++i)
        {
            controlMode->setVelocityMode(i);
        }
        break;
    case VOCAB_CM_POSITION:
    default:
        for(unsigned int i = 0; i < joint_numbers; ++i)
        {
            controlMode->setPositionMode(i);
        }
        break;
    }
    
}

yarp::sig::Vector yarp_single_chain_interface::senseTorque() {
    torqueControl->getTorques(tau_buffer.data());
    return tau_buffer;
}

void yarp_single_chain_interface::senseTorque(yarp::sig::Vector& tau_sensed) {
    if(tau_sensed.size() != this->joint_numbers)
        tau_sensed.resize(this->joint_numbers);
    torqueControl->getTorques(tau_sensed.data());
}

yarp::sig::Vector yarp_single_chain_interface::senseVelocity() {
    encodersMotor->getEncoderSpeeds(qdot_buffer.data());
    if(_useSI) convertEncoderToSI(qdot_buffer);
    return qdot_buffer;
}

void yarp_single_chain_interface::senseVelocity(yarp::sig::Vector& velocity_sensed) {
    if(velocity_sensed.size() != this->joint_numbers)
        velocity_sensed.resize(this->joint_numbers);
    encodersMotor->getEncoderSpeeds(velocity_sensed.data());
    if(_useSI) convertEncoderToSI(velocity_sensed);
}

yarp::sig::Vector yarp_single_chain_interface::sensePosition() {
    return sense();
}

void yarp_single_chain_interface::sensePosition(yarp::sig::Vector& q_sensed) {
    sense(q_sensed);
}

yarp::sig::Vector yarp_single_chain_interface::sense() {
    encodersMotor->getEncoders(q_buffer.data());
    if(_useSI) convertEncoderToSI(q_buffer);
    return q_buffer;
}

void yarp_single_chain_interface::sense(yarp::sig::Vector& q_sensed) {
    if(q_sensed.size() != this->joint_numbers)
        q_sensed.resize(this->joint_numbers);
    encodersMotor->getEncoders(q_sensed.data());
    if(_useSI) convertEncoderToSI(q_sensed);
}

void yarp_single_chain_interface::move(const yarp::sig::Vector& q_d) {
    yarp::sig::Vector q_sent(q_d);
    if(_useSI) convertMotorCommandToSI(q_sent);
    if(!positionDirect->setPositions(q_sent.data()))
        std::cout<<"Cannot move "<< kinematic_chain <<" using Direct Position Ctrl"<<std::cout;

}

const int& yarp_single_chain_interface::getNumberOfJoints()
{
    return this->joint_numbers;
}

const std::string& yarp_single_chain_interface::getChainName(){
    return kinematic_chain;
}

bool yarp_single_chain_interface::createPolyDriver(const std::string& kinematic_chain, yarp::dev::PolyDriver& polyDriver)
{
    yarp::os::Property options;
    options.put("robot", "coman");
    options.put("device", "remote_controlboard");

    yarp::os::ConstString s;
    s = "/"+module_prefix+"/" + kinematic_chain;

    options.put("local", s.c_str());

    yarp::os::ConstString ss;
    ss = "/coman/" + kinematic_chain;
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

inline void yarp_single_chain_interface::convertMotorCommandToSI(yarp::sig::Vector &vector)
{
    for(unsigned int i = 0; i < vector.size(); ++i) {
        vector[i] *= 180.0 / M_PI;
    }
}

inline yarp::sig::Vector yarp_single_chain_interface::convertMotorCommandToSI(const yarp::sig::Vector &vector_in)
{
    yarp::sig::Vector vector_out(vector_in);
    for(unsigned int i = 0; i < vector_out.size(); ++i) {
        vector_out[i] *= 180.0 / M_PI;
    }
    return vector_out;
}

// #endif




/*
 
 
 yarp_interface::yarp_interface()
 {
     
     #if (FT_PORT==TRUE)
     FT_left_leg_port.open("/coman/left_leg/analog:o/forceTorque");
     FT_right_leg_port.open("/coman/right_leg/analog:o/forceTorque");
     #else
     yarp::os::Property FT_left_prop;
     FT_left_prop.put("device", "analogsensorclient");
     FT_left_prop.put("robotName", "coman");
     FT_left_prop.put("remote", "/coman/left_leg/analog:o/forceTorque");
     FT_left_prop.put("local", "/FT_client_left_leg:o");
     FT_left_prop.put("rate", 1);
     
     polyDriver_left_leg_FT.open(FT_left_prop);
     if(!polyDriver_left_leg_FT.isValid())
         printf("error opening analogSensor");
     
     polyDriver_left_leg_FT.view(this->FT_left_leg);
     
     
     yarp::os::Property FT_right_prop;
     FT_right_prop.put("device", "analogsensorclient");
     FT_right_prop.put("robotName", "coman");
     FT_right_prop.put("remote", "/coman/right_leg/analog:o/forceTorque");
     FT_right_prop.put("local", "/FT_client_right_leg:o");
     FT_right_prop.put("rate", 1);
     
     polyDriver_right_leg_FT.open(FT_right_prop);
     if(!polyDriver_right_leg_FT.isValid())
         printf("error opening analogServer");
     
     polyDriver_right_leg_FT.view(this->FT_right_leg);
     #endif
     
     encodersMotor_left_leg->getAxes(&left_leg_dofs);
     encodersMotor_right_leg->getAxes(&right_leg_dofs);
     
     #if (FT_PORT==TRUE)
     ft_left_leg_channels = 6; //Here we consider classic FT sensors
     ft_right_leg_channels = 6;
     #else
     ft_left_leg_channels = FT_left_leg->getChannels();
     ft_right_leg_channels = FT_right_leg->getChannels();
     #endif
     
     
     
     #if(FT_PORT)
     FT_left_arm_port.open("/coman/left_arm/analog:o/forceTorque");
     FT_right_arm_port.open("/coman/right_arm/analog:o/forceTorque");
     #else
     yarp::os::Property FT_left_prop;
     FT_left_prop.put("device", "analogsensorclient");
     FT_left_prop.put("robotName", "coman");
     FT_left_prop.put("remote", "/coman/left_arm/analog:o/forceTorque");
     FT_left_prop.put("local", "/FT_client_left_arm:o");
     FT_left_prop.put("rate", 1);
     
     polyDriver_left_arm_FT.open(FT_left_prop);
     if(!polyDriver_left_arm_FT.isValid())
         printf("error opening analogSensor");
     
     polyDriver_left_arm_FT.view(this->FT_left_arm);
     
     
     yarp::os::Property FT_right_prop;
     FT_right_prop.put("device", "analogsensorclient");
     FT_right_prop.put("robotName", "coman");
     FT_right_prop.put("remote", "/coman/right_arm/analog:o/forceTorque");
     FT_right_prop.put("local", "/FT_client_right_arm:o");
     FT_right_prop.put("rate", 1);
     
     polyDriver_right_arm_FT.open(FT_right_prop);
     if(!polyDriver_right_arm_FT.isValid())
         printf("error opening analogServer");
     
     polyDriver_right_arm_FT.view(this->FT_right_arm);
     
     #endif
     
     
     
 }
 
 
 
 
 void yarp_interface::sense(yarp::sig::Vector &left_leg_q,
 yarp::sig::Vector &right_leg_q,
 yarp::sig::Vector &left_foot_FT,
 yarp::sig::Vector &right_foot_FT) {
     
     
     #if(FT_PORT)
     yarp::os::Bottle* ft_leg_bot = NULL;
     ft_leg_bot = FT_left_leg_port.read(false);
     if(!(ft_leg_bot == NULL) && !ft_leg_bot->isNull())
     {
         for(unsigned int i = 0; i < ft_leg_bot->size(); ++i)
             left_foot_FT[i] = ft_leg_bot->get(i).asDouble();
     }
     ft_leg_bot = FT_right_leg_port.read(false);
     if(!(ft_leg_bot == NULL) && !ft_leg_bot->isNull())
     {
         for(unsigned int i = 0; i < ft_leg_bot->size(); ++i)
             right_foot_FT[i] = ft_leg_bot->get(i).asDouble();
     }
     #else
     FT_left_leg->read(left_foot_FT);
     FT_right_leg->read(right_foot_FT);
     #endif
     }
     
     const robot_state_input& yarp_interface::sense()
     {
                 
         #ifdef FT_ENABLED
         #if(FT_PORT)
         yarp::os::Bottle* ft_arm_bot = FT_left_arm_port.read(false);
         if(!(ft_arm_bot == NULL) && !ft_arm_bot->isNull())
         {
             for(unsigned int i = 0; i < ft_arm_bot->size(); ++i)
                 input.tau_left[i] = ft_arm_bot->get(i).asDouble();
         }
         ft_arm_bot = FT_right_arm_port.read(false);
         if(!(ft_arm_bot == NULL) && !ft_arm_bot->isNull())
         {
             for(unsigned int i = 0; i < ft_arm_bot->size(); ++i)
                 input.tau_right[i] = ft_arm_bot->get(i).asDouble();
         }
         #else
         FT_left_arm->read(input.tau_left);
         FT_right_arm->read(input.tau_right);
         #endif
         #endif
         return input;
     } 
 
 
 */


