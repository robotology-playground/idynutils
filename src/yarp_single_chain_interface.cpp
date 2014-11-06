#include "drc_shared/yarp_single_chain_interface.h"
#include <algorithm>
#include <assert.h>

using namespace walkman::drc;
using namespace yarp::dev;

yarp_single_chain_interface::yarp_single_chain_interface(std::string kinematic_chain,
                                                         std::string module_prefix_with_no_slash,
                                                         std::string robot_name,
                                                         bool useSI,
                                                         const int controlModeVocab):
    module_prefix(robot_name + "/" + module_prefix_with_no_slash),
    kinematic_chain(kinematic_chain),
    isAvailable(internal_isAvailable),
    _useSI(useSI),
    _controlMode(controlModeVocab),
    _robot_name(robot_name)
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
        temp=temp&&polyDriver.view(positionControl);
        temp=temp&&polyDriver.view(positionDirect);
        temp=temp&&polyDriver.view(impedancePositionControl);
        temp=temp&&polyDriver.view(torqueControl);
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

    if (controlModeVocab == VOCAB_CM_NONE) return;
    switch(controlModeVocab) {
        case VOCAB_CM_TORQUE:
            std::cout<<"Initializing "<<kinematic_chain<<" with VOCAB_CM_TORQUE"<<std::endl;
            if(!setTorqueMode())
                std::cout<<"PROBLEM Initializing "<<kinematic_chain<<" with VOCAB_CM_TORQUE"<<std::endl;
            break;
        case VOCAB_CM_IMPEDANCE_POS:
            std::cout<<"Initializing "<<kinematic_chain<<" with VOCAB_CM_IMPEDANCE_POS"<<std::endl;
            if(!setImpedanceMode())
                std::cout<<"PROBLEM Initializing "<<kinematic_chain<<" with VOCAB_CM_IMPEDANCE_POS"<<std::endl;
            break;
        case VOCAB_CM_POSITION_DIRECT:
            std::cout<<"Initializing "<<kinematic_chain<<" with VOCAB_CM_POSITION_DIRECT"<<std::endl;
            if(!setPositionDirectMode())
                std::cout<<"PROBLEM Initializing "<<kinematic_chain<<" with VOCAB_CM_POSITION_DIRECT"<<std::endl;
            break;
        case VOCAB_CM_POSITION:
            std::cout<<"Initializing "<<kinematic_chain<<" with VOCAB_CM_POSITION"<<std::endl;
            if(!setPositionMode())
                std::cout<<"PROBLEM Initializing "<<kinematic_chain<<" with VOCAB_CM_POSITION"<<std::endl;
            break;
        case VOCAB_CM_IDLE:
        default:
            std::cout<<"Initializing "<<kinematic_chain<<" with VOCAB_CM_IDLE"<<std::endl;
            if(!setIdleMode())
                std::cout<<"PROBLEM Initializing "<<kinematic_chain<<" with VOCAB_CM_IDLE"<<std::endl;

    }
    
}


bool yarp_single_chain_interface::setReferenceSpeeds( const yarp::sig::Vector& maximum_velocity )
{
    yarp::sig::Vector maximum_velocity_deg;

    // get joints number
    int joint_number = this->getNumberOfJoints();

    assert(maximum_velocity.size() == joint_number);
    if(this->getControlMode() != VOCAB_CM_POSITION) {
        std::cout << "Tryng to set Reference Speed for chain " << this->getChainName()
                  << " which is not in Position mode" << std::endl;
        return false;
    }

    if(_useSI) maximum_velocity_deg = convertMotorCommandFromSI(maximum_velocity);
    else maximum_velocity_deg = maximum_velocity;

    bool set_success = true;
    for( int i = 0; i < joint_number && set_success; i++ ) {
        set_success = set_success && positionControl->setRefSpeed( i, maximum_velocity_deg[i] );
    }
    return set_success;

//    // set the speed references
//    return positionControl->setRefSpeeds( maximum_velocity_deg.data() );
}

bool yarp_single_chain_interface::setReferenceSpeed( const double& maximum_velocity )
{
    // get joints number
    int num_joints = this->getNumberOfJoints();

    // set the speed references
    yarp::sig::Vector maximum_velocities = yarp::sig::Vector( num_joints,
                                                              maximum_velocity );

    return this->setReferenceSpeeds(maximum_velocities);
}

bool walkman::drc::yarp_single_chain_interface::setImpedance(const yarp::sig::Vector &Kq, const yarp::sig::Vector &Dq)
{
    // get joints number
    int joint_number = std::min(Kq.size(),
                                Dq.size());

    assert(joint_number == this->getNumberOfJoints());
    if(this->getControlMode() != VOCAB_CM_IMPEDANCE_POS) {
        std::cout << "Tryng to set Impedance for chain " << this->getChainName()
                  << "which is not in Impedance mode" << std::endl;
        return false;
    }

    bool set_success = true;
    for(unsigned int i = 0; i < joint_number; ++i) {
        double Kqi;
        double Dqi;
        if(_useSI) {
            Kqi = convertMotorCommandFromSI(Kq[i]);
            Dqi = convertMotorCommandFromSI(Dq[i]);
        } else {
            Kqi = Kq[i];
            Dqi = Dq[i];
        }

            set_success = set_success && impedancePositionControl->setImpedance(i, Kqi, Dqi);
    }
    return set_success;
}

bool walkman::drc::yarp_single_chain_interface::getImpedance(yarp::sig::Vector &Kq, yarp::sig::Vector &Dq)
{
    // get joints number
    int joint_number = this->getNumberOfJoints();

    if(Kq.size() < joint_number)
        Kq.resize(this->getNumberOfJoints());
    if(Dq.size() > joint_number)
        Dq.resize(this->getNumberOfJoints());

    bool set_success = true;
    for(unsigned int i = 0; i < joint_number; ++i) {
        set_success = set_success && impedancePositionControl->getImpedance(i, &Kq[i], &Dq[i]);
    }

    if(_useSI) {
        convertEncoderToSI(Kq);
        convertEncoderToSI(Dq);
    }

    return set_success && (this->getControlMode() == VOCAB_CM_IMPEDANCE_POS);
}


bool yarp_single_chain_interface::setIdleMode()
{
    bool check = true;
    for(unsigned int i = 0; i < joint_numbers; ++i)
        check = check && controlMode->setControlMode(i, VOCAB_CM_IDLE);

    if(check) {
        _controlMode = VOCAB_CM_IDLE;
        std::cout<< "Setting "<<kinematic_chain<<" to VOCAB_CM_IDLE mode"<<std::endl;
    }
    else
        std::cout<< "ERROR setting "<<kinematic_chain<<" to VOCAB_CM_IDLE mode"<<std::endl;
    return check;
}

bool walkman::drc::yarp_single_chain_interface::isInIdleMode() const
{
    return this->getControlMode() == VOCAB_CM_IDLE;
}

bool yarp_single_chain_interface::setTorqueMode()
{
    bool check = true;
    for(unsigned int i = 0; i < joint_numbers; ++i)
        check = check && controlMode->setControlMode(i, VOCAB_CM_TORQUE);

    if(check) {
        _controlMode = VOCAB_CM_TORQUE;
        std::cout<< "Setting "<<kinematic_chain<<" to VOCAB_CM_TORQUE mode"<<std::endl;
    }
    else
        std::cout<< "ERROR setting "<<kinematic_chain<<" to VOCAB_CM_TORQUE mode"<<std::endl;
    return check;
}

bool walkman::drc::yarp_single_chain_interface::isInTorqueMode() const
{
    return this->getControlMode() == VOCAB_CM_TORQUE;
}

bool yarp_single_chain_interface::setPositionMode()
{
    bool check = true;
    for(unsigned int i = 0; i < joint_numbers; ++i)
    {
        check = check && controlMode->setControlMode(i, VOCAB_CM_POSITION) &&
            interactionMode->setInteractionMode(i,VOCAB_IM_STIFF);
    }
    if(check) {
        _controlMode = VOCAB_CM_POSITION;
        std::cout<< "Setting "<<kinematic_chain<<" to VOCAB_CM_POSITION mode"<<std::endl;
    }
    else
        std::cout<< "ERROR setting "<<kinematic_chain<<" to VOCAB_CM_POSITION mode"<<std::endl;
    return check;
}

bool walkman::drc::yarp_single_chain_interface::isInPositionMode() const
{
    return this->getControlMode() == VOCAB_CM_POSITION;
}

bool yarp_single_chain_interface::setImpedanceMode()
{
    bool check = true;
    for(unsigned int i = 0; i < joint_numbers; ++i)
    {
        check = check && controlMode->setControlMode(i, VOCAB_CM_POSITION_DIRECT) &&
            interactionMode->setInteractionMode(i,VOCAB_IM_COMPLIANT);
    }
    if(check) {
        _controlMode = VOCAB_CM_IMPEDANCE_POS;
        std::cout<< "Setting "<<kinematic_chain<<" to VOCAB_CM_IMPEDANCE_POS mode"<<std::endl;
    }
    else
        std::cout<< "ERROR setting "<<kinematic_chain<<" to VOCAB_CM_IMPEDANCE_POS mode"<<std::endl;
    return check;
}

bool walkman::drc::yarp_single_chain_interface::isInImpedanceMode() const
{
    return this->getControlMode() == VOCAB_CM_IMPEDANCE_POS;
}

const int walkman::drc::yarp_single_chain_interface::getControlMode() const
{
    return _controlMode;
}

bool walkman::drc::yarp_single_chain_interface::useSI() const
{
    return _useSI;
}

bool yarp_single_chain_interface::setPositionDirectMode()
{
    bool check = true;
    for(unsigned int i = 0; i < joint_numbers; ++i)
    {
        check = check && controlMode->setControlMode(i, VOCAB_CM_POSITION_DIRECT) &&
            interactionMode->setInteractionMode(i,VOCAB_IM_STIFF);
    }
    if(check) {
        _controlMode = VOCAB_CM_POSITION_DIRECT;
        std::cout<< "Setting "<<kinematic_chain<<" to VOCAB_CM_POSITION_DIRECT mode"<<std::endl;
    }
    else
        std::cout<< "ERROR setting "<<kinematic_chain<<" to VOCAB_CM_POSITION_DIRECT mode"<<std::endl;
    return check;
}

bool walkman::drc::yarp_single_chain_interface::isInPositionDirectMode() const
{
    return this->getControlMode() == VOCAB_CM_POSITION_DIRECT;
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

void yarp_single_chain_interface::move(const yarp::sig::Vector& u_d)
{
    yarp::sig::Vector u_sent(u_d);

    // We assume that all the joints in the kinemati chain are controlled
    // in the same way, so I check only the control mode of the first one.
    _controlMode = computeControlMode();

    switch (_controlMode)
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
        case VOCAB_CM_IDLE:
        default:
                std::cout<<"Cannot move "<< kinematic_chain <<" using Idle Ctrl"<<std::endl;
            break;
    }
}

int yarp_single_chain_interface::computeControlMode()
{
    int ctrlMode;
    controlMode->getControlMode(0, &ctrlMode);

    yarp::dev::InteractionModeEnum intMode;
    interactionMode->getInteractionMode(0, &intMode);

    if(intMode == VOCAB_IM_COMPLIANT)
        return VOCAB_CM_IMPEDANCE_POS;
    else
        return ctrlMode;
}

const int& yarp_single_chain_interface::getNumberOfJoints() const
{
    return this->joint_numbers;
}

const std::string& yarp_single_chain_interface::getChainName() const
{
    return kinematic_chain;
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
