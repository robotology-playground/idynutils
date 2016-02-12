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
     * @param robot_name the name of the robot
     * @param useSI do we want to use SI units? (default to true)
     * @param reference_frame the name of the robot
     */
    yarp_IMU_interface(std::string readerName,
                       std::string robot_name,
                       bool useSI = true,
                       std::string reference_frame = "");

    /**
     * @brief yarp_IMU_interface creates a reader for the IMU port
     * @note this is here to support the old interface
     * @param readerName a unique ID for the reader
     * @param useSI do we want to use SI units?
     * @param robot_name the name of the robot
     * @param reference_frame the name of the robot
     */
    yarp_IMU_interface(std::string readerName,
                       bool useSI,
                       std::string robot_name,
                       std::string reference_frame = "");
    
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
    
    /**
     * @brief returns time stamp of the latest data read by sense command
     * @param timeStamp time stamp in [s] 
     */
    void getLastTimeStamp(double &timeStamp);
    
private:
    void _init(std::string readerName,
               std::string robot_name);

    void _sense();

    /**
     * @brief _output buffer for read data. Default to zeroes
     */
    yarp::sig::Vector _output;
    /**
     * @brief imuReader buffered port, connected to the /inertial port
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

    /**
     * @brief _timeStamp stores a time stamp of the last retreived sensor data
     */
    yarp::os::Stamp _timeStamp;
    
    /**
     * @brief _reference_frame where the lectures are taken
     */
    std::string _reference_frame;
    
};

#endif // YARP_IMU_INTERFACE_H
