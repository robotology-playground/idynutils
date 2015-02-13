/*
 * Copyright (C) 2014 Walkman
 * Author: Mirko Ferrati
 * email:  mirko.ferrati@gmail.com
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


#ifndef CONVEX_HULL_2D
#define CONVEX_HULL_2D

#include <vector>
#include <list>
#include <kdl/frames.hpp>


namespace idynutils
{

struct Point {
    double x, y;

    Point(double x = 0, double y = 0);

    static double CrossProduct(const Point &a, const Point &b);

    // 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
    // Returns a positive value, if OAB makes a counter-clockwise turn,
    // negative for clockwise turn, and zero if the points are collinear.
    static double cross(const Point &O, const Point &A, const Point &B);

    Point operator + (const Point &other) const;

    Point operator - (const Point &other) const;

    bool operator <(const Point &p) const;
};

class convex_hull2D
{

public:
    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    static std::list<KDL::Vector> smartZfilter(std::vector<KDL::Vector> GravityWorld_points3D, double tolerance=0.05);
    static std::vector<Point> compute(std::vector<Point> P);
    static std::vector<Point> compute(std::list<KDL::Vector> P);
    static std::list< KDL::Vector > projection(const std::list< KDL::Vector >& IMULink_points3D, KDL::Frame IMULink_GravityWorld);

private:

};

}


#endif //CONVEX_HULL_2D