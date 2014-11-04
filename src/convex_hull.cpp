/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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


#include "drc_shared/utils/convex_hull.h"
#include <pcl/surface/convex_hull.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <ros/ros.h>

using namespace drc_shared;

convex_hull::convex_hull():
    _ransac_distance_thr(0.001),
    _pointCloud(new pcl::PointCloud<pcl::PointXYZ>()),
    _projectedPointCloud(new pcl::PointCloud<pcl::PointXYZ>())
{

}

convex_hull::~convex_hull()
{

}

bool convex_hull::getConvexHull(const std::list<KDL::Vector>& points,
                                      std::vector<KDL::Vector>& convex_hull)
{
    fromSTDList2PCLPointCloud(points, _pointCloud);

    //Filtering
    projectPCL2Plane(_pointCloud, _ransac_distance_thr, _projectedPointCloud);


    pcl::PointCloud<pcl::PointXYZ> pointsInConvexHull;
    std::vector<pcl::Vertices> indicesOfVertexes;

    // hullVertices.vertices is the list of vertices...
    // by taking each point and the consequent in the list
    // (i.e. vertices[1]-vertices[0] it is possible to compute
    // bounding segments for the hull
    pcl::ConvexHull<pcl::PointXYZ> huller;
    huller.setInputCloud (_projectedPointCloud);
    huller.reconstruct(pointsInConvexHull, indicesOfVertexes);
    if(indicesOfVertexes.size() != 1) {
        ROS_ERROR("Error: more than one polygon found!");
    }
    pcl::Vertices hullVertices = indicesOfVertexes[0];

    //printIndexAndPointsInfo(pointsInConvexHull, indicesOfVertexes);

    const pcl::Vertices& vs = indicesOfVertexes[0];
    for(unsigned int j = 0; j < vs.vertices.size(); ++j)
    {
        pcl::PointXYZ pointXYZ = pointsInConvexHull.at(vs.vertices[j]);
        convex_hull.push_back(fromPCLPointXYZ2KDLVector(pointXYZ));
    }

    _pointCloud->clear();
    _projectedPointCloud->clear();

    return true;
}

pcl::PointXYZ convex_hull::fromKDLVector2PCLPointXYZ(const KDL::Vector &point)
{
    pcl::PointXYZ p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    return p;
}

KDL::Vector convex_hull::fromPCLPointXYZ2KDLVector(const pcl::PointXYZ &point)
{
    return KDL::Vector(point.x,point.y,point.z);
}

void convex_hull::fromSTDList2PCLPointCloud(const std::list<KDL::Vector> &points, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    for(std::list<KDL::Vector>::const_iterator i = points.begin(); i != points.end(); ++i)
        point_cloud->push_back(fromKDLVector2PCLPointXYZ(*i));
}



void convex_hull::projectPCL2Plane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const double ransac_distance_thr,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr projected_point_cloud)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    //We projects ALL the points in the plane (0 0 1)
    coefficients->values.clear();
    coefficients->values.resize(4, 0.0);
    coefficients->values[0] = 0.0;
    coefficients->values[1] = 0.0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0.0;

//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (ransac_distance_thr);
//    seg.setInputCloud (cloud);
//    seg.segment (*inliers, *coefficients);


    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected_point_cloud);
}

void convex_hull::printIndexAndPointsInfo(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull, const std::vector<pcl::Vertices>& indicesOfVertexes)
{
    ROS_WARN("Indices of vertex has size %i", indicesOfVertexes.size());
    for(unsigned int i = 0; i < indicesOfVertexes.size(); ++i){
        pcl::Vertices vertices = indicesOfVertexes[i];
        for(unsigned int ii = 0; ii < vertices.vertices.size(); ++ii)
            ROS_INFO("vertex %i (%f, %f, %f) has index %i ", ii,
                     pointsInConvexHull.at(vertices.vertices[ii]).x, pointsInConvexHull.at(vertices.vertices[ii]).y, pointsInConvexHull.at(vertices.vertices[ii]).z,
                     vertices.vertices[ii]);
    }
}

void convex_hull::getSupportPolygonPoints(iDynUtils &robot,
                                          std::list<KDL::Vector>& points)
{
    std::vector<std::string> names;
    names.push_back("l_foot_lower_left_link");
    names.push_back("l_foot_lower_right_link");
    names.push_back("l_foot_upper_left_link");
    names.push_back("l_foot_upper_right_link");
    names.push_back("r_foot_lower_left_link");
    names.push_back("r_foot_lower_right_link");
    names.push_back("r_foot_upper_left_link");
    names.push_back("r_foot_upper_right_link");

    KDL::Frame waist_T_CoM;
    KDL::Frame waist_T_point;
    KDL::Frame CoM_T_point;
    for(unsigned int i = 0; i < names.size(); ++i)
    {
        // get points in world frame
        waist_T_point = robot.iDyn3_model.getPositionKDL(robot.iDyn3_model.getLinkIndex(names[i]));
        // get CoM in the world frame
        YarptoKDL(robot.iDyn3_model.getCOM(), waist_T_CoM.p);

        CoM_T_point = waist_T_CoM.Inverse() * waist_T_point;
        points.push_back(CoM_T_point.p);
    }
}
