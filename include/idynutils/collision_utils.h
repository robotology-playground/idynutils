/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo, Alessio Rocchi, Cheng Fang
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

#ifndef _COLLISION_UTILS_H_
#define _COLLISION_UTILS_H_

#include <kdl/frames.hpp>
#include <idynutils/idynutils.h>
#include <limits>
#include <list>
#include <string>
#include <utility>

/**
 * @brief The LinkPairDistance class represents the minimum distance information between two links.
 *        The two links must have shape (i.e. collision) information.
 *        Instances of this class will contain the name of both links in the pair,
 *        the actual minimum distance between the two, and the homogeneous transforms
 *        in the respective link frame reference system of the minimum distance point
 *        in each shape.
 */
class LinkPairDistance {
private:
    /**
     * @brief linkPair the pair of link names in alphabetic order
     */
    std::pair<std::string, std::string> linkPair;
    /**
     * @brief link_T_closestPoint is a pair of homogeneous transformations.
     *        The first transformation will express the pose of the closes point on the first link shape
     *        in the link reference frame, i.e. link1_T_closestPoint1.
     *        The second transformation will express the pose of the closes point on the second link shape
     *        in the link reference frame, i.e. link2_T_closestPoint2.
     */
    std::pair<KDL::Frame, KDL::Frame> link_T_closestPoint;
    /**
     * @brief distance the minimum distance between the two link shapes, i.e.
     * ||w_T_closestPoint1.p - w_T_closesPoint2.p||
     */
    double distance;

public:
    /**
     * @brief LinkPairDistance creates an instance of a link pair distance data structure
     * @param link1 the first link name
     * @param link2 the second link name
     * @param link1_T_closestPoint1 the transform from the first link frame to the closest point on its shape
     * @param link2_T_closesPoint2 the transform from the second link frame to the closest point on its shape
     * @param distance the distance between the two minimum distance points
     */
    LinkPairDistance(const std::string& link1, const std::string& link2,
                     const KDL::Frame& link1_T_closestPoint1, const KDL::Frame& link2_T_closestPoint2,
                     const double& distance);

    /**
     * @brief getDistance returns the minimum distance between the two link shapes
     * @return a double representing the minimum distance
     */
    inline const double& getDistance() const;

    /**
     * @brief getTransforms returns a pair of homogeneous transformation from a
     *        link reference frame to the closest point on the respective link shape
     *        link_T_closestPoint
     * @return a pair of homogeneous transformations
     */
    inline const std::pair<KDL::Frame, KDL::Frame>& getTransforms() const;

    /**
     * @brief getLinkNames returns the pair of links between which we want express the distance information
     * @return a pair of strings
     */
    inline const std::pair<std::string, std::string>& getLinkNames() const;

    /**
     * @brief operator < is the comparison operator between two LinkDIstancePairs.
     *        It returns true if the first link pair is closer than the second pair.
     *        If the distance is exactly the same, the pairs will be sorted by
     *        alphabetic order using the first link name in each pair
     * @param second the second link pair
     * @return true if first pair is closer than second pair
     */
    bool operator <(const LinkPairDistance& second) const;
};

class ComputeLinksDistance {
public:
    ComputeLinksDistance(iDynUtils& model);

    /**
     * @brief getLinkDistances returns a list of distances between all link pairs which are enabled for checking.
     *                         If detectionThreshold is not infinity, the list will be clamped to contain only
     *                         the pairs whose distance is smaller than the detection threshold.
     * @param detectionThreshold the maximum distance which we use to look for link pairs.
     * @return a sorted list of linkPairDistances
     */
    std::list<LinkPairDistance> getLinkDistances(double detectionThreshold = std::numeric_limits<double>::infinity());
};

#endif
