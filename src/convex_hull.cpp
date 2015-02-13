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
// #define PCL_FOUND TRUE
#include <ros/ros.h>
#include <idynutils/convex_hull.h>
#include <kdl/frames.hpp>
#include <eigen3/Eigen/Core>
#ifdef PCL_FOUND
#include <pcl/surface/convex_hull.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#else
#include <idynutils/convex_hull2D.h>
#endif

using namespace idynutils;
#ifdef PCL_FOUND
namespace pcl_convex_hull
{
/**
    * @brief fromPCLPointXYZ2KDLVector converts a pcl::PointXZY to a KDL::Vector
    * @param point the source point represented as pcl point
    * @return a kdl vector representing the input point
    */
KDL::Vector fromPCLPointXYZ2KDLVector(const pcl::PointXYZ &point);
/**
    * @brief fromKDLVector2PCLPointXYZ converts a KDL::Vector to a pcl::PointXZY
    * @param point he source point represented as kdl vector
    * @return a pcl point representing the input vector
    */
pcl::PointXYZ fromKDLVector2PCLPointXYZ(const KDL::Vector& point);

/**
    * @brief projectPCL2Plane projects a point cloud on a plane
    * @param cloud the input cloud
    * @param projected_point_cloud the output (projected) point cloud
    */
void projectPCL2Plane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr projected_point_cloud,KDL::Vector projection_vector);

/**
    * @brief fromSTDList2PCLPointCloud converts a list of kdl vectors into a pointcloud
    * @param points a list of kdl vectors
    * @param point_cloud the equivalent point cloud
    */
void fromSTDList2PCLPointCloud(const std::list<KDL::Vector>& points,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

void printIndexAndPointsInfo(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull,
                                const std::vector<pcl::Vertices>& indicesOfVertexes);
}
using namespace pcl_convex_hull;

#endif
convex_hull::convex_hull()
{

}

convex_hull::~convex_hull()
{

}

bool convex_hull::getConvexHull(const std::list<KDL::Vector>& points,
                                      std::vector<KDL::Vector>& convex_hull)
{
#ifdef PCL_FOUND
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    convex_hull.clear();
    fromSTDList2PCLPointCloud(points, point_cloud);

    pcl::PointCloud<pcl::PointXYZ> pointsInConvexHull;
    std::vector<pcl::Vertices> indicesOfVertexes;

    // hullVertices.vertices is the list of vertices...
    // by taking each point and the consequent in the list
    // (i.e. vertices[1]-vertices[0] it is possible to compute
    // bounding segments for the hull
    pcl::ConvexHull<pcl::PointXYZ> huller;
    huller.setInputCloud (point_cloud);
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
#else
    auto convex_hull_points=convex_hull2D::compute(points);
    std::vector<KDL::Vector> convex_hull1;
    for (auto point:convex_hull_points)
    {
        convex_hull1.push_back(KDL::Vector(point.x,point.y,0));
    }
    convex_hull.swap(convex_hull1);
#endif

    return true;
}

std::list< KDL::Vector > convex_hull::projectKDL2Plane(const std::list<KDL::Vector> &points,KDL::Vector projection_vector)
{
#ifdef PCL_FOUND
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr _projectedPointCloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    fromSTDList2PCLPointCloud(points,point_cloud);
    std::list<KDL::Vector> result;
    projectPCL2Plane(point_cloud,_projectedPointCloud,projection_vector);
    for(unsigned int j = 0; j < point_cloud->size(); ++j)
    {
        result.push_back(fromPCLPointXYZ2KDLVector(point_cloud->at(j)));
    }
    return result;
#else
    KDL::Vector x,y,z,unit_x,unit_y;
    KDL::Frame temp;
    unit_x=KDL::Vector(1,0,0);
    unit_y=KDL::Vector(0,1,0);
    z=projection_vector;
    z.Normalize();
    y=unit_x*z;
    if (y.Norm()<0.01)
        y=unit_y*z;
    y.Normalize();
    x=y*z;
    temp.M.UnitX(x);
    temp.M.UnitY(y);
    temp.M.UnitZ(z);
    return convex_hull2D::projection(points,temp);
#endif
}

#ifdef PCL_FOUND
pcl::PointXYZ pcl_convex_hull::fromKDLVector2PCLPointXYZ(const KDL::Vector &point)
{
    pcl::PointXYZ p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    return p;
}

KDL::Vector pcl_convex_hull::fromPCLPointXYZ2KDLVector(const pcl::PointXYZ &point)
{
    return KDL::Vector(point.x,point.y,point.z);
}

void pcl_convex_hull::fromSTDList2PCLPointCloud(const std::list<KDL::Vector> &points, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    for(std::list<KDL::Vector>::const_iterator i = points.begin(); i != points.end(); ++i)
        point_cloud->push_back(fromKDLVector2PCLPointXYZ(*i));
}



void pcl_convex_hull::projectPCL2Plane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr projected_point_cloud, KDL::Vector projection_vector)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    //We projects ALL the points in the plane (projection_vector)
    coefficients->values.clear();
    coefficients->values.resize(4, 0.0);
    coefficients->values[0] = projection_vector.x();
    coefficients->values[1] = projection_vector.y();
    coefficients->values[2] = projection_vector.z();
    coefficients->values[3] = 0.0;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected_point_cloud);
}

void pcl_convex_hull::printIndexAndPointsInfo(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull, const std::vector<pcl::Vertices>& indicesOfVertexes)
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
#endif