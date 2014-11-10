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

#include <idynutils/yarp_IMU_interface.h>
#include <iCub/iDynTree/yarp_kdl.h>

yarp_IMU_interface::yarp_IMU_interface(std::string readerName,
                                       bool useSI)
    : _output(1), _useSI(useSI), _ok(false)
{
    _output.resize(9,0.0);

    std::string portName = "/" + readerName + "/inertial:i";
    if(imuReader.open(portName)) {
        if(yarp::os::Network::connect("/inertial",portName.c_str())) {
            _ok = true;
        }
    }
}

yarp_IMU_interface::~yarp_IMU_interface()
{
    try {
        imuReader.interrupt();
    } catch(...) {}
    imuReader.close();
}


void yarp_IMU_interface::_sense()
{
    yarp::os::Bottle* bottleData;
    if( bottleData = imuReader.read())
        if(bottleData->size() == 12)
            for(unsigned int i = 0; i < 12; ++i )
                _output[i] = bottleData->get(i).asDouble();
    if(_useSI) {
        for(unsigned int i = 0; i < 3; ++i)
            _output[i] = _output[i] * M_PI / 180.0;
        for(unsigned int i = 6; i < 8; ++i)
            _output[i] = _output[i] * M_PI / 180.0;
    }


}

yarp::sig::Vector yarp_IMU_interface::sense()
{
    this->_sense();
    return _output;
}

void yarp_IMU_interface::sense(yarp::sig::Vector &orientation,
                               yarp::sig::Vector &linearAcceleration,
                               yarp::sig::Vector &angularVelocity)
{
    this->sense();

    orientation.resize(3);
    orientation = _output.subVector(0,2);
    linearAcceleration.resize(3);
    linearAcceleration = _output.subVector(3,5);
    angularVelocity.resize(3);
    angularVelocity = _output.subVector(6,8);
}

void yarp_IMU_interface::sense(KDL::Rotation &orientation,
                               KDL::Vector &linearAcceleration,
                               KDL::Vector &angularVelocity)
{
    yarp::sig::Vector yOrientation;
    yarp::sig::Vector yLinearAcceleration;
    yarp::sig::Vector yAngularVelocity;

    this->sense(yOrientation,
                yLinearAcceleration,
                yAngularVelocity);
    orientation.Identity();
    orientation.RPY(yOrientation(1),
                    yOrientation(2),
                    yOrientation(3));
    YarptoKDL(yLinearAcceleration, linearAcceleration);
    YarptoKDL(yAngularVelocity, angularVelocity);
}

