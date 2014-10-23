#ifndef YARP_FT_INTERFACE_H
#define YARP_FT_INTERFACE_H
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include <vector>
#include <iostream>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <mutex>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/BufferedPort.h>
#define FT_ENABLED TRUE

class yarp_ft_interface
{
public:
    /**
     * @brief yarp_ft_interface
     * @param kinematic_chain the name of the kinematic chain where the ft resides, as specified in the sdf
     * (assuming there is a ft sensor at the distal link of a kinematic chain)
     * @param robot_name the name of the robot, will be used to open polydrivers
     * @param module_prefix_with_no_slash the module name
     */
    yarp_ft_interface(std::string kinematic_chain,
                      std::string module_prefix_with_no_slash,
                      std::string robot_name);

    /**
     * @brief sense reads the sensed 6d wrench from the ft yarp port
     * @return the sensed wrench
     */
    yarp::sig::Vector sense();

    /**
     * @brief sense reads the sensed 6d wrench from the ft yarp port
     * @param wrench_sensed the sensed wrench
     */
    void sense(yarp::sig::Vector& wrench_sensed);
    
    
private:
    int ft_channels;
    yarp::sig::Vector input;
    
    #if (FT_ENABLED == TRUE)
    yarp::dev::IAnalogSensor *FT_sensor;    
    yarp::dev::PolyDriver polyDriver_FT;
    #endif
    
};

#endif // YARP_FT_INTERFACE_H
