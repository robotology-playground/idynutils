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

#include <kdl/frames.hpp>
#include <list>
#include <vector>
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

    /**
     * @brief projectKDL2Plane projects a set of points on a plane
     * @param points the input set of points
     * @param projection_vector the vector representing the plane
     * @return a set of projected points
     */
    static std::list< KDL::Vector > projectKDL2Plane(const std::list<KDL::Vector> &points,KDL::Vector projection_vector);

};

}

#endif
