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


// Implementation of Andrew's monotone chain 2D convex hull algorithm.
// Asymptotic complexity: O(n log n).
// Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.
#include <algorithm>
#include <vector>
#include<iostream>
#include<cmath>
#include <kdl/frames.hpp>

#include <idynutils/convex_hull2D.h>

using namespace std;
using namespace idynutils;

Point::Point(double x, double y)
{
    this->x = x;
    this->y = y;
}

double Point::CrossProduct(const Point &a, const Point &b)
{
    return a.x * b.y - a.y * b.x;
}

double Point::cross(const Point &O, const Point &A, const Point &B)
{
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

Point Point::operator + (const Point &other) const
{
    return Point(this->x + other.x, this->y + other.y);
}

Point Point::operator - (const Point &other) const
{
    return Point(this->x - other.x, this->y - other.y);
}

bool Point::operator <(const Point &p) const {
    return x < p.x || (x == p.x && y < p.y);
}

struct Zgreater
{
    bool operator()( const KDL::Vector& lx, const KDL::Vector& rx ) const {
        return lx.z() < rx.z();
    }
};

list< KDL::Vector > convex_hull2D::projection(const list< KDL::Vector >& IMULink_points3D, KDL::Frame IMULink_GravityWorld)
{
    double err=0;
    std::list<KDL::Vector> result;
    for (const KDL::Vector& IMULink_point:IMULink_points3D)
    {
        auto temp=IMULink_GravityWorld.Inverse()*IMULink_point;
        //         KDL::Vector p;
        //         p.x=temp.x();
        //         p.y=temp.y();
        //         p.z=temp.z();
        err+=temp.z();
        temp.z(0.0);
        result.push_back(temp);
    }
    for (KDL::Vector& point:result)
    {
        point=IMULink_GravityWorld*point;
    }
//     std::cout<<"global error with respect to the world plane: "<<err<<std::endl;
    return result;
}

std::list<KDL::Vector> convex_hull2D::smartZfilter(vector<KDL::Vector> GravityWorld_points3D, double tolerance)
{
    assert(GravityWorld_points3D.size());
    list<KDL::Vector> results;
    std::sort(GravityWorld_points3D.begin(),GravityWorld_points3D.end(),Zgreater());
    double startingZ=GravityWorld_points3D.front().z();
    for (auto point:GravityWorld_points3D)
    {
        if (point.z()<startingZ+tolerance)
            results.push_back(point);
        else
            break;
    }
    return results;
}

vector<Point> convex_hull2D::compute(list<KDL::Vector> P)
{
    std::vector<Point> temp;
    for (auto point:P)
    {
        Point a;
        a.x=point.x();
        a.y=point.y();
        temp.push_back(a);
    }
    return compute(temp);
}

class atan2comparison
{
public:
    atan2comparison(Point medianPoint)
    {
        this->median=medianPoint;
    }
    bool operator()( const Point& a, const Point& b ) const
    {
        return atan2(a.y-median.y,a.x-median.x) < atan2(b.y-median.y,b.x-median.x);
    }
private:
    Point median;
};

vector<Point> convex_hull2D::compute(vector<Point> P)
{
    int n = P.size(), k = 0;
//     std::cout<<"original: ";
    for (auto point: P)
    {
//         std::cout<<"("<<point.x<<" "<<point.y<<") ";
    }
//     std::cout<<std::endl;
    vector<Point> H(2*n);
    Point medianPoint;
    // Sort points lexicographically
    sort(P.begin(), P.end());
    for (auto point: P)
    {
        medianPoint.x+=point.x;
        medianPoint.y+=point.y;
    }
    medianPoint.x/=P.size();
    medianPoint.y/=P.size();
    // Build lower hull
    for (int i = 0; i < n; ++i) {
        while (k >= 2 && Point::cross(H[k-2], H[k-1], P[i]) <= 0) k--;
        H[k++] = P[i];
    }
    
    // Build upper hull
    for (int i = n-2, t = k+1; i >= 0; i--) {
        while (k >= t && Point::cross(H[k-2], H[k-1], P[i]) <= 0) k--;
        H[k++] = P[i];
    }
    
    H.resize(k-1);
    std::sort( H.begin(), H.end(), atan2comparison(medianPoint) );
    std::reverse(H.begin(),H.end());
//     std::cout<<"sorted: ";
    
    
    for (auto point: H)
    {
//         std::cout<<"("<<point.x<<" "<<point.y<<") ";
    }
//     std::cout<<std::endl;
    return H;
}
