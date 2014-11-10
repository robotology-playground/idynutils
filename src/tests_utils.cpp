/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo, Alessio Rocchi
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it
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

#include <idynutils/tests_utils.h>
#include <time.h>
#include <iostream>

void tests_utils::initializeIfNeeded()
{
    static bool is_initialized = false;

    if(!is_initialized) {
        time_t seed = time(NULL);
        seed48((unsigned short*)(&seed));
        srand((unsigned int)(seed));

        is_initialized = true;
    }

}

double tests_utils::getRandomAngle()
{
    tests_utils::initializeIfNeeded();
    return drand48()*2.0*M_PI-M_PI;
}

yarp::sig::Vector tests_utils::getRandomAngles(const yarp::sig::Vector& min,
                                               const yarp::sig::Vector& max,
                                               const int size)
{
    tests_utils::initializeIfNeeded();
    yarp::sig::Vector q(size);
    assert(min.size() >= size);
    assert(max.size() >= size);
    for(unsigned int i = 0; i < size; ++i)
        q(i) = getRandomAngle(min[i],max[i]);
    return q;
}

double tests_utils::getRandomAngle(const double min, const double max)
{
    tests_utils::initializeIfNeeded();
    assert(min <= max);
    if(min < -M_PI || max > M_PI)
        return getRandomAngle();

    return (double)rand()/RAND_MAX * (max-min) + min;
}

double tests_utils::getRandomLength(const double min, const double max)
{
    tests_utils::initializeIfNeeded();
    assert(min <= max);
    return (double)rand()/RAND_MAX * (max-min) + min;
}

KDL::Vector tests_utils::getRandomVector(const double min, const double max)
{
    tests_utils::initializeIfNeeded();

    double x = tests_utils::getRandomLength(min,max);
    double y = tests_utils::getRandomLength(min,max);
    double z = tests_utils::getRandomLength(min,max);
    KDL::Vector p(x,y,z);
    return p;
}

KDL::Rotation tests_utils::getRandomRotation(const double min, const double max)
{
    tests_utils::initializeIfNeeded();

    double R = tests_utils::getRandomAngle(min,max);
    double Pmin = min; double Pmax = max;
    if(Pmin < -M_PI_2 || Pmax > M_PI_2) { Pmin /= 2; Pmax /= 2; }
    double P = tests_utils::getRandomAngle(Pmin,Pmax);
    double Y = tests_utils::getRandomAngle(min,max);

    KDL::Rotation rot = KDL::Rotation::RPY(R,P,Y);
    return rot;
}

KDL::Frame tests_utils::getRandomFrame(const double lengthMin, const double lengthMax,
                                       const double rotMin, const double rotMax)
{
    tests_utils::initializeIfNeeded();

    KDL::Vector p = tests_utils::getRandomVector(lengthMin,
                                                 lengthMax);
    KDL::Rotation r = tests_utils::getRandomRotation(rotMin,
                                                     rotMax);
    KDL::Frame f(r,p);

    return f;
}

bool tests_utils::startYarpServer()
{
    int exit_ = system("yarpserver --write&");
    if (exit_ == -1) return false;
    return true;
}
bool tests_utils::stopYarpServer()
{
    int exit_ = system("killall yarpserver&");
    if (exit_ == -1) return false;
    return true;
}

