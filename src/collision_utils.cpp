#include <idynutils/collision_utils.h>


ComputeLinksDistance::ComputeLinksDistance(iDynUtils &model)
{

}

std::list<LinkPairDistance> ComputeLinksDistance::getLinkDistances(double detectionThreshold)
{
    return std::list<LinkPairDistance>();
}


LinkPairDistance::LinkPairDistance(const std::string &link1, const std::string &link2,
                                   const KDL::Frame &link1_T_closestPoint1,
                                   const KDL::Frame &link2_T_closestPoint2,
                                   const double &distance) :
    linkPair(link1 < link2 ? link1:link2,
             link1 < link2 ? link2:link1),
    link_T_closestPoint(link1 < link2 ? link1_T_closestPoint1:link2_T_closestPoint2,
                        link1 < link2 ? link2_T_closestPoint2 :link1_T_closestPoint1),
    distance(distance)
{

}

const double &LinkPairDistance::getDistance() const
{
    return distance;
}

const std::pair<KDL::Frame, KDL::Frame> &LinkPairDistance::getTransforms() const
{
    return link_T_closestPoint;
}

const std::pair<std::string, std::string> &LinkPairDistance::getLinkNames() const
{
    return linkPair;
}

bool LinkPairDistance::operator <(const LinkPairDistance &second) const
{
    if(this->distance < second.distance) return true;
    else return (this->linkPair.first < second.linkPair.first);
}
