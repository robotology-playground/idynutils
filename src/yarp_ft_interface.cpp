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

#include <idynutils/yarp_ft_interface.h>
#include <yarp/dev/IAnalogSensor.h>

yarp_ft_interface::yarp_ft_interface(std::string deviceId,
                                     std::string module_prefix_with_no_slash,
                                     std::string robot_name, std::string reference_frame)
{
    
    yarp::os::Property FT_prop;
    FT_prop.put("device", "analogsensorclient");
    FT_prop.put("robotName", robot_name.c_str());
    FT_prop.put("remote", "/"+robot_name+"/"+deviceId+"/analog:o/forceTorque");
    FT_prop.put("local", "/"+robot_name+"/"+module_prefix_with_no_slash+"/"+deviceId+"/analog:i/forceTorque");
    FT_prop.put("rate", 1);
    
    polyDriver_FT.open(FT_prop);
    if(!polyDriver_FT.isValid())
        printf("error opening analogSensor");
    
    polyDriver_FT.view(this->FT_sensor);
    ft_channels = FT_sensor->getChannels();

    _reference_frame = reference_frame;
}



yarp::sig::Vector yarp_ft_interface::sense()
{
    FT_sensor->read(input);
    return input;
}

bool yarp_ft_interface::sense(yarp::sig::Vector &wrench_sensed)
{
    return ( FT_sensor->read(wrench_sensed) == yarp::dev::IAnalogSensor::AS_OK );
}
