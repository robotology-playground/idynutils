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

#include <idynutils/cartesian_utils.h>
#include <yarp/math/Math.h>
#include <boost/shared_ptr.hpp>

using namespace yarp::math;

#define toDeg(X) (X*180.0/M_PI)
yarp::sig::Vector cartesian_utils::computeFootZMP(const yarp::sig::Vector &forces, const yarp::sig::Vector &torques,
                                                  const double d, const double fz_threshold)
{
    yarp::sig::Vector ZMP(3, 0.0);

    if(forces[2] > fz_threshold && fz_threshold >= 0.0){
        ZMP[0] = -1.0 * (torques[1] + forces[0]*d)/forces[2];
        ZMP[1] = (torques[0] - forces[1]*d)/forces[2];
        ZMP[2] = -d;}
    return ZMP;
}

yarp::sig::Vector cartesian_utils::computeZMP(const double Lforce_z, const double Rforce_z,
                                              const yarp::sig::Vector& ZMPL, const yarp::sig::Vector& ZMPR,
                                              const double fz_threshold)
{
    yarp::sig::Vector ZMP(3, 0.0);

    if((Lforce_z > fz_threshold || Rforce_z > fz_threshold) &&
        fz_threshold >= 0.0){

        ZMP = (ZMPL*Lforce_z + ZMPR*Rforce_z)/(Lforce_z+Rforce_z);
        ZMP[2] = ZMPL[2];
    }
    return ZMP;
}

void cartesian_utils::computePanTiltMatrix(const yarp::sig::Vector& gaze, yarp::sig::Matrix& pan_tilt_matrix)
{
    double pan = std::atan2(gaze[1], gaze[0]);
    double tilt = std::atan2(gaze[2],sqrt(gaze[1]*gaze[1] + gaze[0]*gaze[0]));

    KDL::Frame T; T.Identity();
    T.M.DoRotZ(pan);
    T.M.DoRotY(-tilt);

    pan_tilt_matrix.resize(4,4);
    cartesian_utils::fromKDLFrameToYARPMatrix(T, pan_tilt_matrix);
}

void cartesian_utils::homogeneousMatrixFromRPY(yarp::sig::Matrix& T,
                              const double x, const double y, const double z,
                              const double R, const double P, const double Y)
{
    KDL::Frame tmp(KDL::Rotation::RPY(R, P, Y), KDL::Vector(x, y, z));

    fromKDLFrameToYARPMatrix(tmp, T);
}

void cartesian_utils::homogeneousMatrixFromQuaternion(yarp::sig::Matrix &T,
                                                      const double x, const double y, const double z,
                                                      const double quaternion_x, const double quaternion_y, const double quaternion_z, const double quaternion_w)
{
    KDL::Frame tmp(KDL::Rotation::Quaternion(quaternion_x, quaternion_y, quaternion_z, quaternion_w), KDL::Vector(x, y, z));

    fromKDLFrameToYARPMatrix(tmp, T);
}

void cartesian_utils::computeCartesianError(const yarp::sig::Matrix &T,
                                            const yarp::sig::Matrix &Td,
                                            yarp::sig::Vector& position_error,
                                            yarp::sig::Vector& orientation_error)
{   
    position_error.resize(3, 0.0);
    orientation_error.resize(3, 0.0);

    KDL::Frame x; // ee pose
    x.Identity();
    fromYARPMatrixtoKDLFrame(T, x);
    quaternion q;
    x.M.GetQuaternion(q.x, q.y, q.z, q.w);

    KDL::Frame xd; // ee desired pose
    xd.Identity();
    fromYARPMatrixtoKDLFrame(Td, xd);
    quaternion qd;
    xd.M.GetQuaternion(qd.x, qd.y, qd.z, qd.w);

    //This is needed to move along the short path in the quaternion error
    if(quaternion::dot(q, qd) < 0.0)
        q = q.operator *(-1.0); //che cagata...

    KDL::Vector xerr_p; // Cartesian position error
    KDL::Vector xerr_o; // Cartesian orientation error

    xerr_p = xd.p - x.p;
    xerr_o = quaternion::error(q, qd);

    position_error[0] = xerr_p.x();
    position_error[1] = xerr_p.y();
    position_error[2] = xerr_p.z();

    orientation_error[0] = xerr_o.x();
    orientation_error[1] = xerr_o.y();
    orientation_error[2] = xerr_o.z();
}

void cartesian_utils::fromYarpVectortoKDLWrench(const yarp::sig::Vector& wi, KDL::Wrench& wo)
{
    wo.force.data[0] = wi[0];
    wo.force.data[1] = wi[1];
    wo.force.data[2] = wi[2];
    wo.torque.data[0] = wi[3];
    wo.torque.data[1] = wi[4];
    wo.torque.data[2] = wi[5];
}

void cartesian_utils::fromKDLWrenchtoYarpVector(const KDL::Wrench& wi, yarp::sig::Vector& wo)
{
    wo.resize(6,0.0);

    wo[0] = wi.force.x();
    wo[1] = wi.force.y();
    wo[2] = wi.force.z();
    wo[3] = wi.torque.x();
    wo[4] = wi.torque.y();
    wo[5] = wi.torque.z();
}

void cartesian_utils::fromKDLFrameToYARPMatrix(const KDL::Frame &Ti, yarp::sig::Matrix &To)
{
    To.resize(4,4);
    To.eye();
    To(0,0) = Ti.M.UnitX().x(); To(0,1) = Ti.M.UnitY().x(); To(0,2) = Ti.M.UnitZ().x(); To(0,3) = Ti.p.x();
    To(1,0) = Ti.M.UnitX().y(); To(1,1) = Ti.M.UnitY().y(); To(1,2) = Ti.M.UnitZ().y(); To(1,3) = Ti.p.y();
    To(2,0) = Ti.M.UnitX().z(); To(2,1) = Ti.M.UnitY().z(); To(2,2) = Ti.M.UnitZ().z(); To(2,3) = Ti.p.z();
}

void cartesian_utils::fromKDLTwistToYARPVector(const KDL::Twist& vi, yarp::sig::Vector& vo)
{
    vo.resize(6, 0.0);
    vo[0] = vi.vel.x(); vo[1] = vi.vel.y(); vo[2] = vi.vel.z();
    vo[3] = vi.rot.x(); vo[4] = vi.rot.y(); vo[5] = vi.rot.z();
}

// We copy the homogeneous matrix because we do not want to pass quaternion since we use
// the notation from "Operational Space Control: A Theoretical and Empirical Comparison"
void cartesian_utils::fromYARPMatrixtoKDLFrame(const yarp::sig::Matrix &Ti, KDL::Frame &To)
{
    To.p.data[0] = Ti(0,3); To.p.data[1] = Ti(1,3); To.p.data[2] = Ti(2,3);
    To.M.data[0] = Ti(0,0); To.M.data[1] = Ti(0,1); To.M.data[2] = Ti(0,2);
    To.M.data[3] = Ti(1,0); To.M.data[4] = Ti(1,1); To.M.data[5] = Ti(1,2);
    To.M.data[6] = Ti(2,0); To.M.data[7] = Ti(2,1); To.M.data[8] = Ti(2,2);
}

void cartesian_utils::fromYARPVectortoKDLTwist(const yarp::sig::Vector& vi, KDL::Twist& vo)
{
    vo.vel.data[0] = vi[0]; vo.vel.data[1] = vi[1]; vo.vel.data[2] = vi[2];
    vo.rot.data[0] = vi[3]; vo.rot.data[1] = vi[4]; vo.rot.data[2] = vi[5];
}

void cartesian_utils::printHomogeneousTransform(const yarp::sig::Matrix &T)
{
    KDL::Frame p(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0)); // ee desired pose
    fromYARPMatrixtoKDLFrame(T, p);

    printKDLFrame(p);
}

void cartesian_utils::printKDLFrame(const KDL::Frame& T)
{
    double R = 0.0;
    double P = 0.0;
    double Y = 0.0;
    T.M.GetRPY(R,P,Y);

    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 0.0;
    T.M.GetQuaternion(qx, qy, qz, qw);

    std::cout<<"Position: [ "<<T.p.x()<<" "<<T.p.y()<<" "<<T.p.z()<<" ] [m]"<<std::endl;
    std::cout<<"RPY: [ "<<toDeg(R)<<" "<<toDeg(P)<<" "<<toDeg(Y)<<" ] [deg]"<<std::endl;
    std::cout<<"Quaternion: [ "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<" ]"<<std::endl;
}

void cartesian_utils::printKDLTwist(const KDL::Twist &v)
{
    std::cout<<"Linear Velocity: ["<<v.vel.x()<<" "<<v.vel.y()<<" "<<v.vel.z()<<" ] [m/sec]"<<std::endl;
    std::cout<<"Angular Velocity: ["<<v.rot.x()<<" "<<v.rot.y()<<" "<<v.rot.z()<<" ] [rad/sec]"<<std::endl;
}

void cartesian_utils::printVelocityVector(const yarp::sig::Vector& v)
{
    std::cout<<"Linear Velocity: ["<<v[0]<<" "<<v[1]<<" "<<v[2]<<" ] [m/sec]"<<std::endl;
    std::cout<<"Angular Velocity: ["<<v[3]<<" "<<v[4]<<" "<<v[5]<<" ] [rad/sec]"<<std::endl;
}

yarp::sig::Vector cartesian_utils::computeGradient(const yarp::sig::Vector &x,
                                                    CostFunction& fun,
                                                    const double& step) {
    std::vector<bool> jointMask(x.size(), true);
    return computeGradient(x, fun, jointMask, step);
}

yarp::sig::Vector cartesian_utils::computeGradient(const yarp::sig::Vector &x,
                                                    CostFunction& fun,
                                                    const std::vector<bool>& jointMask,
                                                    const double& step) {
    yarp::sig::Vector gradient(x.size(),0.0);
    yarp::sig::Vector deltas(x.size(),0.0);
    assert(jointMask.size() == x.size() &&
           "jointMask must have the same size as x");
    const double h = step;
    for(unsigned int i = 0; i < gradient.size(); ++i)
    {
        if(jointMask[i])
        {
            deltas[i] = h;
            double fun_a = fun.compute(x+deltas);
            double fun_b = fun.compute(x-deltas);

            gradient[i] = (fun_a - fun_b)/(2.0*h);
            deltas[i] = 0.0;
        } else
            gradient[i] = 0.0;
    }

    return gradient;
}

yarp::sig::Matrix cartesian_utils::computeHessian(const yarp::sig::Vector &x,
                                                   GradientVector& vec,
                                                   const double& step) {
    yarp::sig::Matrix hessian(vec.size(),x.size());
    yarp::sig::Vector deltas(x.size(),0.0);
    const double h = step;
    for(unsigned int i = 0; i < vec.size(); ++i)
    {
        deltas[i] = h;
        yarp::sig::Vector gradient_a = vec.compute(x+deltas);
        yarp::sig::Vector gradient_b = vec.compute(x-deltas);
        yarp::sig::Vector gradient(vec.size());
        for(unsigned int j = 0; j < vec.size(); ++j)
            gradient[j] = (gradient_a[j] - gradient_b[j])/(2.0*h);
        hessian.setCol(i,gradient);
        deltas[i] = 0.0;
    }

    return hessian;
}

void cartesian_utils::computeRealLinksFromFakeLinks(const std::list<std::string>& input_links,
                                                    const boost::shared_ptr<urdf::Model> _urdf,
                                                    std::list<std::string>& output_links)
{
    boost::shared_ptr<urdf::Model> urdf = _urdf;

    std::list<std::string>::const_iterator iter;
    for(iter = input_links.begin();
        iter != input_links.end();
        iter++)
    {
        std::string link_name = *iter;
        boost::shared_ptr<const urdf::Link> link = urdf->getLink(link_name);

        bool true_link = false;
        std::string body_name = "";
        while(!true_link){
            if(!(link->inertial)) //Then the link is a "fake" link and we need the parent
                link = link->getParent();
            else{ //The link is a "true" link
                true_link = true;
                body_name = link->name;
            }

            if(true_link)
            {
                if(!(std::find(output_links.begin(), output_links.end(), body_name) != output_links.end()))
                    output_links.push_back(body_name);
            }

        }
    }
}
