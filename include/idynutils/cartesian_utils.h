#ifndef _CARTESIAN_UTILS_H_
#define _CARTESIAN_UTILS_H_

#include <yarp/sig/all.h>
#include <kdl/frames.hpp>

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

    class CostFunction {
    public:
        virtual double compute(const yarp::sig::Vector &x) = 0;
    };

    /**
     * @brief computeCartesianError orientation and position error
     * @param T actual pose Homogeneous Matrix [4x4]
     * @param Td desired pose Homogeneous Matrix [4x4]
     * @param position_error position error [3x1]
     * @param orientation_error orientation error [3x1]
     */
    static void computeCartesianError(yarp::sig::Matrix &T,
                                      yarp::sig::Matrix &Td,
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

    static void fromKDLFrameToYARPMatrix(const KDL::Frame& Ti, yarp::sig::Matrix& To);

    static void fromYARPMatrixtoKDLFrame(const yarp::sig::Matrix& Ti, KDL::Frame& To);

    static void printHomogeneousTransform(const yarp::sig::Matrix& T);

    static void printKDLFrame(const KDL::Frame& T);

    static yarp::sig::Vector computeGradient(const yarp::sig::Vector &x,
                                             CostFunction &fun,
                                             const double &step = 1E-3);
};

#endif
