/*
 * Copyright (C) 2014 Walkman
 * Author: Mirko Ferrati, Enrico Mingo, Alessio Rocchi
 * email:  mirko.ferrati@gmail.com, enrico.mingo@iit.it, alessio.rocchi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef COMANUTILS_H
#define COMANUTILS_H

#include "drc_shared/yarp_single_chain_interface.h"
#include "drc_shared/idynutils.h"
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>

class ComanUtils
{
public:
    ComanUtils(const std::string moduleName,
               const int controlModeVocab = VOCAB_CM_POSITION);

    walkman::drc::yarp_single_chain_interface right_arm, left_arm;
    walkman::drc::yarp_single_chain_interface torso;
    walkman::drc::yarp_single_chain_interface right_leg, left_leg;
    iDynUtils idynutils;

    void sense(yarp::sig::Vector& q,
               yarp::sig::Vector& qdot,
               yarp::sig::Vector& tau);

    yarp::sig::Vector& sensePosition();
    yarp::sig::Vector& senseVelocity();
    yarp::sig::Vector& senseTorque();

private:
    yarp::sig::Vector q_sensed;

    yarp::sig::Vector q_sensed_left_arm;
    yarp::sig::Vector q_sensed_right_arm;
    yarp::sig::Vector q_sensed_left_leg;
    yarp::sig::Vector q_sensed_right_leg;
    yarp::sig::Vector q_sensed_torso;

    yarp::sig::Vector qdot_sensed;

    yarp::sig::Vector qdot_sensed_left_arm;
    yarp::sig::Vector qdot_sensed_right_arm;
    yarp::sig::Vector qdot_sensed_left_leg;
    yarp::sig::Vector qdot_sensed_right_leg;
    yarp::sig::Vector qdot_sensed_torso;

    yarp::sig::Vector tau_sensed;

    yarp::sig::Vector tau_sensed_left_arm;
    yarp::sig::Vector tau_sensed_right_arm;
    yarp::sig::Vector tau_sensed_left_leg;
    yarp::sig::Vector tau_sensed_right_leg;
    yarp::sig::Vector tau_sensed_torso;

    void fromRobotToIdyn(yarp::sig::Vector& _right_arm,
                         yarp::sig::Vector& _left_arm,
                         yarp::sig::Vector& _torso,
                         yarp::sig::Vector& _right_leg,
                         yarp::sig::Vector& _left_leg,
                         yarp::sig::Vector& _q);


};

#endif // COMANUTILS_H
