/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo, Alessio Rocchi,
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

#ifndef _CARTESIAN_UTILS_H_
#define _CARTESIAN_UTILS_H_

#include <yarp/sig/all.h>
#include <kdl/frames.hpp>
#include <vector>
#include <list>
#include <urdf/model.h>

/**
  This class implements quaternion error as in the paper:
    "Operational Space Control: A Theoretical and Empirical Comparison"
  Authors: Jun Nakanishi, Rick Cory, Michael Mistry, Jan Peters and Stefan Schaal
  The International Journal of Robotics Research, Vol. 27, No. 6, June 2008, pp. 737–757

  REMEMBER: if e is the quaternion error, the orientation error is defined as:
                o_error = -Ke
            with K positive definite!
  **/
class quaternion
{
public:
    double x;
    double y;
    double z;
    double w;

    quaternion():
        x(0.0),
        y(0.0),
        z(0.0),
        w(1.0)
    {

    }

    quaternion(double _x, double _y, double _z, double _w):
        x(_x),
        y(_y),
        z(_z),
        w(_w)
    {

    }

    /**
     * @brief dot product between two quaternions
     * @param a first quaternion
     * @param b second quaternion
     * @return a scalar
     */
    static double dot(const quaternion& a, const quaternion& b)
    {
        return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    }

    /**
     * @brief operator * product between a quaternion and a scalar
     * @param a scalar
     * @return a quaternion
     */
    quaternion operator*(const double a)
    {
        quaternion q(x, y, z, w);

        q.x *= a;
        q.y *= a;
        q.z *= a;
        q.w *= a;

        return q;
    }

    /**
     * @brief skew operator
     * @return the skew matrix of the quaternion
     */
    KDL::Rotation skew()
    {
        KDL::Rotation s(0.0,  -z,   y,
                          z, 0.0,  -x,
                         -y,   x, 0.0);
        return s;
    }

    /**
     * @brief error compute the error between two quaternion to be usable in orientation control
     * @param q actual quaternion
     * @param qd desired quaternion
     * @return an error vector [3x1]
     *
     * REMEMBER: if e is the quaternion error, the orientation error is defined as:
                o_error = -Ke
            with K positive definite!
     *
     */
    static KDL::Vector error(quaternion& q, quaternion& qd)
    {
        KDL::Vector e(0.0, 0.0, 0.0);

        KDL::Vector eps(q.x, q.y, q.z);
        KDL::Vector epsd(qd.x, qd.y, qd.z);

        e = qd.w*eps - q.w*epsd + qd.skew()*eps;

        return e;
    }
};

class cartesian_utils
{
public:
    /**
     * @brief computeFootZMP compute the MEASURED ZMP for a foot in contact, given forces and torques measured
     * from an FT sensor. The formula used is based on:
     *      "Introduction to Humanoid Robots" by Shuuji Kajita et al., pag. 79-80
     *
     *          if fz > fz_threshold
     *              ZMPx = (-tau_y -fx*d)/fz
     *              ZMPy = (tau_x - fy*d)/fz
     *              ZMPz = -d
     *          else
     *              ZMPx = 0
     *              ZMPy = 0
     *              ZMPz = 0
     *
     * where d is the height of the sensor w.r.t. the sole.
     * NOTE: The ZMP position is computed w.r.t. the sensor frame.
     *
     * @param forces vector of forces measured from the FT sensor
     * @param torques vector of torques measured from the FT sensor
     * @param d height of the sensor w.r.t. the sole
     * @param fz_threshold if fz goes over this threshold then ZMP is computed
     * @return a vector with the ZMP position
     */
    static yarp::sig::Vector computeFootZMP(const yarp::sig::Vector& forces, const yarp::sig::Vector& torques,
                                            const double d, const double fz_threshold);

    /**
     * @brief computeZMP compute the MEASURED ZMP for BOTH the feet in contact, given the two ZMPs in the SAME
     * reference frame and the measured force on z.
     * The formula used is
     *
     *      if(fLz > fz_threshold || fRz > fz_threshod)
     *          ZMPx = (ZMPLx*fLz + ZMPRx*fRz)(fLz + fRz)
     *          ZMPy = (ZMPLy*fLz + ZMPRy*fRz)(fLz + fRz)
     *          ZMPz = ZMPLz
     *      else
     *          ZMPx = 0
     *          ZMPy = 0
     *          ZMPz = 0
     *
     *  where ZMP, ZMPL and ZMPR are all expressed in the same reference frame.
     *
     * @param Lforces
     * @param Ltorques
     * @param Rforces
     * @param Rtorques
     * @param fz_threshold
     * @return
     */
    static yarp::sig::Vector computeZMP(const double Lforce_z, const double Rforce_z,
                                        const yarp::sig::Vector& ZMPL, const yarp::sig::Vector& ZMPR,
                                        const double fz_threshold);

    /**
     * @brief computePanTiltMatrix given a gaze vector computes the Homogeneous Matrix to control the
     * YAW-PITCH angles.
     * The algorithm used is based on the paper: "Adaptive Predictive Gaze Control of a Redundant Humanoid
     * Robot Head, IROS2011".
     *
     * @param gaze vector [3x1]
     * @param pan_tilt_matrix Homogeneous Matrix [4x4] in the same reference frame of the gaze vector
     */
    static void computePanTiltMatrix(const yarp::sig::Vector& gaze, yarp::sig::Matrix& pan_tilt_matrix);

    /**
     * @brief computeCartesianError orientation and position error
     * @param T actual pose Homogeneous Matrix [4x4]
     * @param Td desired pose Homogeneous Matrix [4x4]
     * @param position_error position error [3x1]
     * @param orientation_error orientation error [3x1]
     */
    static void computeCartesianError(const yarp::sig::Matrix &T,
                                      const yarp::sig::Matrix &Td,
                                      yarp::sig::Vector& position_error,
                                      yarp::sig::Vector& orientation_error);

    /**
     * @brief homogeneousMatrixFromRPY compute Homogeneous Matrix from position [x, y, z] and orientation [Roll, Pitch, Yaw]
     * @param T pose Homogeneous Matrix [4x4]
     * @param x position
     * @param y position
     * @param z position
     * @param R orientation
     * @param P orientation
     * @param Y orientation
     */
    static void homogeneousMatrixFromRPY(yarp::sig::Matrix& T,
                                         const double x, const double y, const double z,
                                         const double R, const double P, const double Y);

    /**
     * @brief homogeneousMatrixFromQuaternion ompute Homogeneous Matrix from position [x, y, z] and orientation [x, y, z, w]
     * @param T pose Homogeneous Matrix [4x4]
     * @param x position
     * @param y position
     * @param z position
     * @param quaternion_x orientation
     * @param quaternion_y orientation
     * @param quaternion_z orientation
     * @param quaternion_w orientation
     */
    static void homogeneousMatrixFromQuaternion(yarp::sig::Matrix& T,
                                                const double x, const double y, const double z,
                                                const double quaternion_x, const double quaternion_y, const double quaternion_z, const double quaternion_w);

    /**
     * @brief fromKDLFrameToYARPMatrix convert a KDL::Frame in a yarp::sig::Matrix
     * @param Ti KDL::Frame
     * @param To yarp::sig::Matrix
     */
    static void fromKDLFrameToYARPMatrix(const KDL::Frame& Ti, yarp::sig::Matrix& To);

    /**
     * @brief fromKDLTwistToYARPVector convert a KDL::Twist in a yarp::sig::Vector
     * @param vi KDL::Twist
     * @param vo yarp::sig::Vector
     */
    static void fromKDLTwistToYARPVector(const KDL::Twist& vi, yarp::sig::Vector& vo);

    /**
     * @brief fromYARPMatrixtoKDLFrame convert a yarp::sig::Matrix in a KDL::Frame
     * @param Ti yarp::sig::Matrix
     * @param To KDL::Frame
     */
    static void fromYARPMatrixtoKDLFrame(const yarp::sig::Matrix& Ti, KDL::Frame& To);

    /**
     * @brief fromYARPVectortoKDLTwist convert a yarp::sig::Vector in a KDL::Twist
     * @param vi yarp::sig::Vector
     * @param vo KDL::Twist
     */
    static void fromYARPVectortoKDLTwist(const yarp::sig::Vector& vi, KDL::Twist& vo);

    /**
     * @brief printHomogeneousTransform print on terminal a yarp::sig::Matrix used as pose Homogeneous Transform [4x4]
     * @param T yarp::sig::Matrix [4x4]
     */
    static void printHomogeneousTransform(const yarp::sig::Matrix& T);

    /**
     * @brief fromYarpVectortoKDLWrench convert a yarp::sig::Vector in a KDL::Wrench
     * @param wi yarp::sig::Vector
     * @param wo KDL::Wrench
     */
    static void fromYarpVectortoKDLWrench(const yarp::sig::Vector& wi, KDL::Wrench& wo);

    /**
     * @brief fromKDLWrenchtoYarpVector convert a KDL::Wrench in a yarp::sig::Vector
     * @param wi yarp::sig::Vector
     * @param wo KDL::Wrench
     */
    static void fromKDLWrenchtoYarpVector(const KDL::Wrench& wi, yarp::sig::Vector& wo);

    /**
     * @brief printKDLFrame print a KDL::Frame
     * @param T KDL::Frame
     */
    static void printKDLFrame(const KDL::Frame& T);

    /**
     * @brief printKDLTwist print a KDL::Twist
     * @param v KDL::Twist
     */
    static void printKDLTwist(const KDL::Twist& v);

    /**
     * @brief printVelocityVector print a yarp::sig::Vector of 6 elements used as linear and angular velocity [1x6]
     * @param v yarp::sig::Vector [1x6]
     */
    static void printVelocityVector(const yarp::sig::Vector& v);

    /**
     * @brief The CostFunction class pure virtual function used to describe functions for computeGradient method.
     */
    class CostFunction {
    public:
        /**
         * @brief compute value of function in x
         * @param x
         * @return scalar
         */
        virtual double compute(const yarp::sig::Vector &x) = 0;
    };

    /**
     * @brief The GradientVector class pure virtual function used to describe functions for computeHessian method.
     */
    class GradientVector {
        double _size;
    public:
        GradientVector(const double x_size) : _size(x_size) {}
        /**
         * @brief compute value of function in x
         * @param x
         * @return scalar
         */
        virtual yarp::sig::Vector compute(const yarp::sig::Vector &x) = 0;
        double size() { return _size; }
    };

    /**
     * @brief computeGradient compute numerical gradient of a function using 2 points formula:
     *
     *           f(x+h) - f(x-h)
     *   df(x)= ----------------
     *                2h
     * @param x points around gradient is compute
     * @param fun function to derive
     * @param step step of gradient
     * @return vector of gradient
     */
    static yarp::sig::Vector computeGradient(const yarp::sig::Vector &x,
                                              CostFunction &fun,
                                              const double &step = 1E-3);

    /**
     * @brief computeGradient compute numerical gradient of a function using 2 points formula:
     *
     *           f(x+h) - f(x-h)
     *   df(x)= ----------------
     *                2h
     * @param x points around gradient is compute
     * @param fun function to derive
     * @param jointMask the joints over which we want to compute the gradient
     * @param step step of gradient
     * @return vector of gradient
     */
    static yarp::sig::Vector computeGradient(const yarp::sig::Vector &x,
                                              CostFunction &fun,
                                              const std::vector<bool>& jointMask,
                                              const double &step = 1E-3);


    /**
     * @brief computeGradient compute numerical gradient of a function using 2 points formula:
     *
     *           f(x+h) - f(x-h)
     *   df(x)= ----------------
     *                2h
     * @param x points around gradient is compute
     * @param fun function to derive
     * @param step step of gradient
     * @return vector of gradient
     */
    static yarp::sig::Matrix computeHessian( const yarp::sig::Vector &x,
                                              GradientVector &vec,
                                              const double &step = 1E-3);

    /**
     * @brief computeRealLinksFromFakeLinks given a list of links (fake or real) it outputs a list of only real links
     * @param input_links input list of links
     * @param _urdf urdf of the robot
     * @param output_links output list of links
     */
    static void computeRealLinksFromFakeLinks(const std::list<std::string>& input_links,
                                       const boost::shared_ptr<urdf::Model> _urdf,
                                       std::list<std::string>& output_links);

};

#endif
