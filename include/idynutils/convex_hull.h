#ifndef _CONVEX_HULL_H_
#define _CONVEX_HULL_H_

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

#include "drc_shared/idynutils.h"
#include "drc_shared/cartesian_utils.h"
#include <moveit/robot_model/robot_model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <kdl/frames.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <yarp/sig/Vector.h>

namespace drc_shared
{

class convex_hull
{
public:
    convex_hull();
    ~convex_hull();

    void getConvexHull(const std::list<KDL::Vector>& points, yarp::sig::Matrix& A, yarp::sig::Vector& b);
    //void setRansacDistanceThr(const double x){_ransac_distance_thr = x;}

/**
  * @brief getSupportPolygonPoints
  * @param points return a list of points express in a frame F
  *        oriented like the world frame and with origin
  *        on the CoM projection in the support polygon
  */
 static void getSupportPolygonPoints( iDynUtils &robot,
                                      std::list<KDL::Vector>& points);
private:
    double _ransac_distance_thr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pointCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _projectedPointCloud;

    pcl::PointXYZ fromKDLVector2PCLPointXYZ(const KDL::Vector& point);
    void fromSTDList2PCLPointCloud(const std::list<KDL::Vector>& points, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    void projectPCL2Plane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const double ransac_distance_thr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr projected_point_cloud);
    void printIndexAndPointsInfo(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull, const std::vector<pcl::Vertices>& indicesOfVertexes);
    void getLineCoefficients(const pcl::PointXYZ& p0, const pcl::PointXYZ& p1, double &a, double& b, double &c);
    void getConstraints(const pcl::PointCloud<pcl::PointXYZ>& pointsInConvexHull, const std::vector<pcl::Vertices>& indicesOfVertexes,
                        yarp::sig::Matrix& A, yarp::sig::Vector& b);
};

}

#endif
