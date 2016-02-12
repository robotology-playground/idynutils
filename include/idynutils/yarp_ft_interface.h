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

#ifndef YARP_FT_INTERFACE_H
#define YARP_FT_INTERFACE_H
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include <vector>
#include <iostream>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/BufferedPort.h>

class yarp_ft_interface
{
public:
    /**
     * @brief yarp_ft_interface
     * @param deviceId the name of the kinematic chain where the ft resides, as specified in the sdf
     * (assuming there is a ft sensor at the distal link of a kinematic chain)
     * @param robot_name the name of the robot, will be used to open polydrivers
     * @param module_prefix_with_no_slash the module name
     */
    yarp_ft_interface(std::string deviceId,
                      std::string module_prefix_with_no_slash,
                      std::string robot_name,
                      std::string reference_frame = "");

    /**
     * @brief sense reads the sensed 6d wrench from the ft yarp port
     * @return the sensed wrench
     */
    yarp::sig::Vector sense();

    /**
     * @brief sense reads the sensed 6d wrench from the ft yarp port
     * @param wrench_sensed the sensed wrench
     * @return true on success
     */
    bool sense(yarp::sig::Vector& wrench_sensed);

    /**
     * @brief getReferenceFrame where the forces/torques are measured
     * @return a string with the reference frame where the ft are measured
     */
    std::string getReferenceFrame(){return _reference_frame;}
        
    /**
     * @brief returns time stamp of the latest data read by sense command
     * @param timeStamp time stamp in [s] 
     */
    bool getLastTimeStamp(double &timeStamp);
    
private:
    int ft_channels;
    yarp::sig::Vector input;
    std::string _reference_frame;
    
    yarp::dev::IAnalogSensor *FT_sensor;
    yarp::dev::IPreciselyTimed *_timeStampInterface;
    yarp::dev::PolyDriver polyDriver_FT;
    
};

#endif // YARP_FT_INTERFACE_H
