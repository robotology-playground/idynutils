#ifndef YARP_IMU_INTERFACE_H
#define YARP_IMU_INTERFACE_H
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <vector>
#include <iostream>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <kdl/frames.hpp>

class yarp_IMU_interface
{
public:

    /**
     * @brief yarp_IMU_interface creates a reader for the IMU port
     * @param readerName a unique ID for the reader
     * @param useSI do we want to use SI units? (default to true)
     */
    yarp_IMU_interface(std::string readerName,
                       bool useSI = true);

    ~yarp_IMU_interface();
    /**
     * @brief sense
     * @return a 9 element vector with:
     *         3x1 orientation vector (RPY),
     *         3x1 linear acceleration vector, [m/s^2]
     *         3x1 angular velocity vector [rad/s]
     */
    yarp::sig::Vector sense();
    /**
     * @brief sense
     * @param orientation 3x1 orientation vector in Euler angles ZYX (RPY)
     * @param linearAcceleration 3x1 linear acceleration vector [m/s^2]
     * @param angularVelocity 3x1 angular velocity vector [rad/s]
     */
    void sense(yarp::sig::Vector& orientation,
               yarp::sig::Vector& linearAcceleration,
               yarp::sig::Vector& angularVelocity);

    /**
     * @brief sense
     * @param orientation KDL orientation vector
     * @param linearAcceleration linear acceleration vector [m/s^2]
     * @param angularVelocity angular velocity vector [rad/s]
     */
    void sense(KDL::Rotation &orientation,
               KDL::Vector &linearAcceleration,
               KDL::Vector &angularVelocity);
    
    
private:
    void _sense();

    /**
     * @brief _output buffer for read data. Default to zeroes
     */
    yarp::sig::Vector _output;
    /**
     * @brief imuReader buffered port, connected to the /inertial port
     * @TODO  check that the /inertial port exists for this to work!
     */
    yarp::os::BufferedPort<yarp::os::Bottle> imuReader;

    /**
     * @brief _ok is the connection to the /inertial port succesfull?
     */
    bool _ok;
    /**
     * @brief _useSI use the SI system for units? (rad instead of degs)
     */
    bool _useSI;
    
};

#endif // YARP_IMU_INTERFACE_H
