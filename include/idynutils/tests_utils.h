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
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <gtest/gtest.h>
#include <gtest/internal/gtest-internal.h>

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

    /**
     * @brief startYarpServer start a YARP server in a separate process
     * @return false if can not start
     */
    static bool startYarpServer();

    /**
     * @brief stopYarpServer stop an existing YARP server
     * @return false if can not stop or not existing YARP server
     */
    static bool stopYarpServer();

    /**
     * @brief matrixAreEqual check that two matrices are the same
     * @param m0 first matrix
     * @param m1 second matrix
     * @return true if equal
     */
    static inline bool matrixAreEqual(const yarp::sig::Matrix& m0,
                                      const yarp::sig::Matrix& m1)
    {
        bool sizeAreCompatible = (m0.rows() == m1.rows() &&
                                  m0.cols() == m1.cols());
        EXPECT_TRUE(sizeAreCompatible) << "Size of compared matrices "
                                       << "are not compatible";
        if(!sizeAreCompatible)
            return false;

        bool areEqual = true;
        for(unsigned int r = 0; r < m0.rows(); ++r)
            for(unsigned int c = 0; c < m0.cols(); ++c) {
                EXPECT_DOUBLE_EQ(m0(r,c), m1(r,c)) << "Elements in ("
                                                   << r << "," << c
                                                   << ") are not equal";

                using namespace testing::internal;
                bool elementAreEqual;
                FloatingPoint<double> lhs(m0(r,c));
                FloatingPoint<double> rhs(m1(r,c));
                elementAreEqual = lhs.AlmostEquals(rhs);

                areEqual = areEqual & elementAreEqual;

            }
        return areEqual;
    }

    /**
     * @brief vectorAreEqual check that two vectors are the same
     * @param v0 first vector
     * @param v1 second vector
     * @return true if equal
     */
    static inline bool vectorAreEqual(const yarp::sig::Vector& v0,
                                      const yarp::sig::Vector& v1)
    {
        bool sizeAreCompatible = (v0.size() == v1.size());
        EXPECT_TRUE(sizeAreCompatible) << "Size of compared vectors "
                                       << "are not equal";
        if(!sizeAreCompatible)
            return false;

        bool areEqual = true;
        for(unsigned int s = 0; s < v0.size(); ++s) {
            EXPECT_DOUBLE_EQ(v0(s), v1(s)) << "Elements in  ("
                                               << s
                                               << ") are not equal";

            using namespace testing::internal;
            bool elementAreEqual;
            FloatingPoint<double> lhs(v0(s));
            FloatingPoint<double> rhs(v1(s));
            elementAreEqual = lhs.AlmostEquals(rhs);

            areEqual = areEqual & elementAreEqual;

        }
        return areEqual;
    }

    /**
     * @brief startGazebo start a GAZEBO simulation in another process. Both gzserver
     * and gzclient are sarted so the simulation will be visualized in a gui
     * @param world_file a world file to load
     * @return false if not possible to start simulation
     */
    static bool startGazebo(const std::string& world_file = "");
    /**
     * @brief stopGazebo stop a GAZEBO simulation
     * @return false if not possible or not existing
     */
    static bool stopGazebo();
    /**
     * @brief startGZServer start a GAZEBO simulation in another process. Only gzserver
     * is sarted so the simulation will not be visualized in a gui
     * @param world_file a world file to load
     * @return false if not possible to start simulation
     */
    static bool startGZServer(const std::string& world_file = "");
    /**
     * @brief stopGZServer stop a GAZEBO simulation
     * @return false if not possible or not existing
     */
    static bool stopGZServer();

    /**
     * @brief _kbhit capture any input in a asynchrounous way.
     * Based on the implementation in:
     *      http://www.flipcode.com/archives/_kbhit_for_Linux.shtml
     * @return 1 if a key is pressed
     */
    static int _kbhit();

    /**
     * @brief KDLFramesAreEqual perform GTEST check in 2 frames
     * @param a first frame
     * @param b second frame
     */
    static inline void KDLFramesAreEqual(const KDL::Frame& a, const KDL::Frame& b)
    {
        EXPECT_DOUBLE_EQ(a.p.x(), b.p.x());
        EXPECT_DOUBLE_EQ(a.p.y(), b.p.y());
        EXPECT_DOUBLE_EQ(a.p.z(), b.p.z());

        double x,y,z,w; a.M.GetQuaternion(x,y,z,w);
        double xx,yy,zz,ww; b.M.GetQuaternion(xx,yy,zz,ww);

        EXPECT_DOUBLE_EQ(x,xx);
        EXPECT_DOUBLE_EQ(y,yy);
        EXPECT_DOUBLE_EQ(z,zz);
        EXPECT_DOUBLE_EQ(w,ww);
    }
};

#endif
