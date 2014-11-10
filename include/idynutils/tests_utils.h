/*
 * Copyright (C) 2014 Walkman
 * Author:  Enrico Mingo, Alessio Rocchi
 * email:   enrico.mingo@iit.it, alessio.rocchi@iit.it
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

#ifndef _TESTS_UTILS_H_
#define _TESTS_UTILS_H_

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <kdl/frames.hpp>
#include <yarp/sig/Vector.h>


class tests_utils
{
public:
    /**
     * @brief initializeIfNeeded recreates random seeds for random functions
     */
    static void initializeIfNeeded();

    /**
     * @brief getRandomAngle return a random angle in [-M_PI, M_PI)
     * @return random angle between [-M_PI, M_PI)
     */
    static double getRandomAngle();

    /**
     * @brief getRandomAngle return a vector of angles with element i in [min[i], max[i]]
     * @param min a vector of minimum angles
     * @param max a vector of maximum angles
     * @param size size of the output vector
     * @return a vector of random angles with element i in [min[i], max[i]]
     */
    static yarp::sig::Vector getRandomAngles(const yarp::sig::Vector& min,
                                             const yarp::sig::Vector& max,
                                             const int size);

    /**
     * @brief getRandomAngle return a random angle in [min, max]
     * @param min >= -M_PI
     * @param max <= M_PI
     * @return random angle in [min, max]
     */
    static double getRandomAngle(const double min, const double max);

    /**
     * @brief getRandomLength returns a random length in [min, max]
     * @param min minimum distance
     * @param max maximum distance
     * @return random length in [min, max]
     */
    static double getRandomLength(const double min, const double max);

    /**
     * @brief getRandomVector returns a random vector with elements in [min, max]
     * @param min minimum length of each element
     * @param max maximum length of each element
     * @return random vector with each element in [min, max]
     */
    static KDL::Vector getRandomVector(const double min, const double max);

    /**
     * @brief getRandomRotation returns a random rotation with each rotation in [min, max]
     * @param min minimum value of each rotation
     * @param max maximum value of each rotation
     * @return random RPY rotation with R, P, Y in [min, max]
     */
    static KDL::Rotation getRandomRotation(const double min, const double max);

    /**
     * @brief getRandomFrame returns a random frame with random length and orientation
     * @param lengthMin minimum distance for each element in the origin vector of the frame
     * @param lengthMax maximum distance for each element in the origin vector of the frame
     * @param rotMin minimum value of R,P,Y in the RPY orientation of the frame
     * @param rotMax maximum value of R,P,Y in the RPY orientation of the frame
     * @return
     */
    static KDL::Frame getRandomFrame(const double lengthMin, const double lengthMax,
                                     const double rotMin, const double rotMax);

    static bool startYarpServer();

    static bool stopYarpServer();

};

#endif
