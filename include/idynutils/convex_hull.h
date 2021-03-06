/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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

#ifndef _CONVEX_HULL_H_
#define _CONVEX_HULL_H_

#include <idynutils/idynutils.h>
#include <idynutils/cartesian_utils.h>
#include <moveit/robot_model/robot_model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <kdl/frames.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <yarp/sig/Vector.h>

namespace idynutils
{

class convex_hull
{
public:
    convex_hull();
    ~convex_hull();

    /**
     * @brief getConvexHull returns a minimum representation of the convex hull
     * @param points a list of points representing the convex hull
     * @param ch a list of points which are the vertices of the convex hull
     * @return true on success
     */
    bool getConvexHull(const std::list<KDL::Vector>& points,
                             std::vector<KDL::Vector>& ch);
    //void setRansacDistanceThr(const double x){_ransac_distance_thr = x;}

private:
    double _ransac_distance_thr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pointCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _projectedPointCloud;

    /**
     * @brief fromPCLPointXYZ2KDLVector converts a pcl::PointXZY to a KDL::Vector
     * @param point the source point represented as pcl point
     * @return a kdl vector representing the input point
     */
    static KDL::Vector fromPCLPointXYZ2KDLVector(const pcl::PointXYZ &point);
    /**
     * @brief fromKDLVector2PCLPointXYZ converts a KDL::Vector to a pcl::PointXZY
     * @param point he source point represented as kdl vector
     * @return a pcl point representing the input vector
     */
    static pcl::PointXYZ fromKDLVector2PCLPointXYZ(const KDL::Vector& point);

    /**
     * @brief fromSTDList2PCLPointCloud converts a list of kdl vectors into a pointcloud
     * @param points a list of kdl vectors
     * @param point_cloud the equivalent point cloud
     */
    static void fromSTDList2PCLPointCloud(const std::list<KDL::Vector>& points,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

    /**
     * @brief projectPCL2Plane projects a point cloud on a plane
     * @param cloud the input cloud
     * @param ransac_distance_thr threshold for the ransac algorithm to detect outliers
     * @param projected_point_cloud the output (projected) point cloud
     */
    void projectPCL2Plane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                          const double ransac_distance_thr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr projected_point_cloud);

    void printIndexAndPointsInfo(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull,
                                 const std::vector<pcl::Vertices>& indicesOfVertexes);
};

}

#endif
