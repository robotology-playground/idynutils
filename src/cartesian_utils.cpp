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
#include <eigen_conversions/eigen_kdl.h>

using namespace yarp::math;

#define toDeg(X) (X*180.0/M_PI)

yarp::sig::Vector cartesian_utils::computeCapturePoint(const yarp::sig::Vector& floating_base_velocity,
                                                       const yarp::sig::Vector& com_velocity,
                                                       const yarp::sig::Vector& com_pose)
{
    yarp::sig::Vector cp(3,0.0);

    yarp::sig::Vector com_velocity_ = com_velocity + floating_base_velocity;

    double g = 9.81;

    cp[0] = com_pose[0] + com_velocity_[0]*sqrt(com_pose[2]/g);
    cp[1] = com_pose[1] + com_velocity_[1]*sqrt(com_pose[2]/g);
    cp[2] = 0.0;

    return cp;
}

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

void  cartesian_utils::computePanTiltMatrix(const Eigen::VectorXd &gaze, KDL::Frame &pan_tilt_matrix)
{
    double pan = std::atan2(gaze[1], gaze[0]);
    double tilt = std::atan2(gaze[2],sqrt(gaze[1]*gaze[1] + gaze[0]*gaze[0]));

    pan_tilt_matrix.Identity();
    pan_tilt_matrix.M.DoRotZ(pan);
    pan_tilt_matrix.M.DoRotY(-tilt);
}

void cartesian_utils::computePanTiltMatrix(const yarp::sig::Vector& gaze, yarp::sig::Matrix& pan_tilt_matrix)
{
    KDL::Frame T;
    computePanTiltMatrix(cartesian_utils::toEigen(gaze), T);
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

void cartesian_utils::computeCartesianError(const Eigen::MatrixXd &T,
                                  const Eigen::MatrixXd &Td,
                                  Eigen::VectorXd& position_error,
                                  Eigen::VectorXd& orientation_error)
{
    position_error.setZero(3);
    orientation_error.setZero(3);

    KDL::Frame x; // ee pose
    x.Identity();
    Eigen::Matrix4d tmp = T;
    tf::transformEigenToKDL(Eigen::Affine3d(tmp),x);
    quaternion q;
    x.M.GetQuaternion(q.x, q.y, q.z, q.w);

    KDL::Frame xd; // ee desired pose
    xd.Identity();
    tmp = Td;
    tf::transformEigenToKDL(Eigen::Affine3d(tmp),xd);
    quaternion qd;
    xd.M.GetQuaternion(qd.x, qd.y, qd.z, qd.w);

    //This is needed to move along the short path in the quaternion error
    if(quaternion::dot(q, qd) < 0.0)
        q = q.operator *(-1.0); //che cagata...

    KDL::Vector xerr_p; // Cartesian position error
    KDL::Vector xerr_o; // Cartesian orientation error

    xerr_p = xd.p - x.p;
    xerr_o = quaternion::error(q, qd);


    position_error(0) = xerr_p.x();
    position_error(1) = xerr_p.y();
    position_error(2) = xerr_p.z();

    orientation_error(0) = xerr_o.x();
    orientation_error(1) = xerr_o.y();
    orientation_error(2) = xerr_o.z();
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

void cartesian_utils::fromKDLRotationToYARPMatrix(const KDL::Rotation& Ri, yarp::sig::Matrix& Ro)
{
    Ro.resize(3,3);
    Ro.eye();
    Ro(0,0) = Ri.UnitX().x(); Ro(0,1) = Ri.UnitY().x(); Ro(0,2) = Ri.UnitZ().x();
    Ro(1,0) = Ri.UnitX().y(); Ro(1,1) = Ri.UnitY().y(); Ro(1,2) = Ri.UnitZ().y();
    Ro(2,0) = Ri.UnitX().z(); Ro(2,1) = Ri.UnitY().z(); Ro(2,2) = Ri.UnitZ().z();
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

void cartesian_utils::fromYARPMatrixtoKDLRotation(const yarp::sig::Matrix& Ri, KDL::Rotation& Ro)
{
    Ro.data[0] = Ri(0,0); Ro.data[1] = Ri(0,1); Ro.data[2] = Ri(0,2);
    Ro.data[3] = Ri(1,0); Ro.data[4] = Ri(1,1); Ro.data[5] = Ri(1,2);
    Ro.data[6] = Ri(2,0); Ro.data[7] = Ri(2,1); Ro.data[8] = Ri(2,2);
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

void cartesian_utils::printKDLRotation(const KDL::Rotation& R)
{
    double r = 0.0;
    double P = 0.0;
    double Y = 0.0;
    R.GetRPY(r,P,Y);

    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 0.0;
    R.GetQuaternion(qx, qy, qz, qw);

    std::cout<<"RPY: [ "<<toDeg(r)<<" "<<toDeg(P)<<" "<<toDeg(Y)<<" ] [deg]"<<std::endl;
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

Eigen::VectorXd cartesian_utils::computeGradient(const Eigen::VectorXd &x,
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

Eigen::VectorXd cartesian_utils::computeGradient(const Eigen::VectorXd &x,
                                                    CostFunction& fun,
                                                    const std::vector<bool>& jointMask,
                                                    const double& step) {
    Eigen::VectorXd gradient(x.rows());
    gradient.setZero(x.rows());
    Eigen::VectorXd deltas(x.rows());
    deltas.setZero(x.rows());
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

Eigen::MatrixXd cartesian_utils::computeHessian(const Eigen::VectorXd &x,
                                                   GradientVector& vec,
                                                   const double& step) {
    Eigen::MatrixXd hessian(vec.size(),x.size());
    Eigen::VectorXd deltas(x.rows());
    deltas.setZero(x.rows());
    const double h = step;
    for(unsigned int i = 0; i < vec.size(); ++i)
    {
        deltas[i] = h;
        Eigen::VectorXd gradient_a = vec.compute(x+deltas);
        Eigen::VectorXd gradient_b = vec.compute(x-deltas);
        Eigen::VectorXd gradient(vec.size());
        for(unsigned int j = 0; j < vec.size(); ++j)
            gradient[j] = (gradient_a[j] - gradient_b[j])/(2.0*h);

        hessian.block(0,i,hessian.rows(),1) = gradient;

        //hessian.setCol(i,gradient);
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

yarp::sig::Matrix cartesian_utils::fromEigentoYarp(const Eigen::MatrixXd& M)
{
    yarp::sig::Matrix tmp(M.rows(), M.cols());
    for(unsigned int i = 0; i < M.rows(); ++i)
        for(unsigned int j = 0; j < M.cols(); ++j)
            tmp(i,j) = M(i,j);
    return tmp;
}

yarp::sig::Vector cartesian_utils::fromEigentoYarp(const Eigen::VectorXd& v)
{
    yarp::sig::Vector tmp(v.rows(), 0.0);
    for(unsigned int i = 0; i < v.rows(); ++i)
        tmp(i) = v(i);
    return tmp;
}

KDL::Wrench cartesian_utils::toKDLWrench(const Eigen::VectorXd& v)
{
    KDL::Wrench tmp; tmp.Zero();
    if(v.rows() == 6){
        tmp.force.x(v[0]);
        tmp.force.y(v[1]);
        tmp.force.z(v[2]);
        tmp.torque.x(v[3]);
        tmp.torque.y(v[4]);
        tmp.torque.z(v[5]);
    }
    return tmp;
}

KDL::Twist cartesian_utils::toKDLTwist(const Eigen::VectorXd& v)
{
    KDL::Twist tmp; tmp.Zero();
    if(v.rows() == 6){
        tmp.vel.x(v[0]);
        tmp.vel.y(v[1]);
        tmp.vel.z(v[2]);
        tmp.rot.x(v[3]);
        tmp.rot.y(v[4]);
        tmp.rot.z(v[5]);
    }
    return tmp;
}

KDL::Frame cartesian_utils::toKDLFrame(const Eigen::MatrixXd& T)
{
    KDL::Frame tmp; tmp.Identity();
    if(T.rows() == 4 && T.cols() == 4)
    {
        tmp.M(0,0) = T(0,0); tmp.M(0,1) = T(0,1); tmp.M(0,2) = T(0,2);
        tmp.M(1,0) = T(1,0); tmp.M(1,1) = T(1,1); tmp.M(1,2) = T(1,2);
        tmp.M(2,0) = T(2,0); tmp.M(2,1) = T(2,1); tmp.M(2,2) = T(2,2);

        tmp.p.x(T(0,3));
        tmp.p.y(T(1,3));
        tmp.p.z(T(2,3));
    }
    return tmp;
}

Eigen::VectorXd cartesian_utils::toEigen(const KDL::Wrench& w)
{
    Eigen::VectorXd tmp(6); tmp.setZero(6);
    tmp[0] = w.force.x();
    tmp[1] = w.force.y();
    tmp[2] = w.force.z();
    tmp[3] = w.torque.x();
    tmp[4] = w.torque.y();
    tmp[5] = w.torque.z();
    return tmp;
}

Eigen::VectorXd cartesian_utils::toEigen(const KDL::Twist& v)
{
    Eigen::VectorXd tmp(6); tmp.setZero(6);
    tmp[0] = v.vel.x();
    tmp[1] = v.vel.y();
    tmp[2] = v.vel.z();
    tmp[3] = v.rot.x();
    tmp[4] = v.rot.y();
    tmp[5] = v.rot.z();
    return tmp;
}

Eigen::MatrixXd cartesian_utils::toEigen(const KDL::Frame& T)
{
    Eigen::MatrixXd tmp(4,4); tmp.setIdentity(4,4);
    tmp(0,0) = T.M(0,0); tmp(0,1) = T.M(0,1); tmp(0,2) = T.M(0,2);
    tmp(1,0) = T.M(1,0); tmp(1,1) = T.M(1,1); tmp(1,2) = T.M(1,2);
    tmp(2,0) = T.M(2,0); tmp(2,1) = T.M(2,1); tmp(2,2) = T.M(2,2);

    tmp(0,3) = T.p.x();
    tmp(1,3) = T.p.y();
    tmp(2,3) = T.p.z();
    return tmp;
}

