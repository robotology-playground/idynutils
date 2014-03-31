#ifndef YARP_SINGLE_CHAIN_INTERFACE_H
#define YARP_SINGLE_CHAIN_INTERFACE_H
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <vector>
#include <iostream>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <mutex>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/BufferedPort.h>


namespace walkman{
    namespace drc{

class yarp_single_chain_interface
{
public:
    yarp_single_chain_interface(std::string kinematic_chain, std::string module_prefix_with_no_slash);
    virtual yarp::sig::Vector sense();
    virtual void sense(yarp::sig::Vector& q_sensed);
    virtual void move(const yarp::sig::Vector& q_d);
    inline int getNumberOfJoints()
    {
        return joint_numbers;
    }
    ~yarp_single_chain_interface();
    inline std::string getChainName(){
        return kinematic_chain;
    }
private:
    
    bool createPolyDriver ( const std::string &kinematic_chain, yarp::dev::PolyDriver &polyDriver );
    std::string kinematic_chain;
    int joint_numbers;
    std::string module_prefix;
    yarp::sig::Vector q_buffer;
    bool internal_isAvailable;
    yarp::dev::PolyDriver polyDriver;
public:
    bool& isAvailable;
    
    yarp::dev::IEncodersTimed *encodersMotor;
    yarp::dev::IPositionDirect *positionDirect;
    yarp::dev::IControlMode *controlMode;
    yarp::dev::IPositionControl2 *positionControl;
    yarp::dev::IImpedanceControl *impedancePositionControl;
    yarp::dev::ITorqueControl *torqueControl;


};


}
}


#endif // YARP_SINGLE_CHAIN_INTERFACE_H
