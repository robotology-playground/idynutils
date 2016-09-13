/*
 * Copyright (C) 2014-2016 Walkman
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

#ifndef _OCTOMAP_UTILS_H_
#define _OCTOMAP_UTILS_H_

#include <idynutils/idynutils.h>
#include <idynutils/cartesian_utils.h>

#include <kdl/frames_io.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <tf_conversions/tf_eigen.h>

#include <iostream>
#include <cstdlib>

namespace tf
{
    octomath::Pose6D poseEigenToOctomap(Eigen::Affine3d transform);

}

namespace octomap_utils
{
    void octomapWithPoseToOctomap(octomap_msgs::OctomapWithPose& octomap_msg);

    void filterOctomap(octomap_msgs::Octomap& octomap_msg,
                       octomath::Vector3 min, octomath::Vector3 max,
                       Eigen::Affine3d transform = Eigen::Affine3d::Identity());


    void transformOctomap(octomap_msgs::Octomap& octomap_msg,
                          Eigen::Affine3d transform);

    octomap_msgs::Octomap transformAndFilterOctomap(const octomap_msgs::Octomap& octomap_msg,
                                                    octomath::Vector3 min, octomath::Vector3 max,
                                                    Eigen::Affine3d transform);
}

#endif
