#include <idynutils/octomap_utils.h>

void octomap_utils::transformOctomap(octomap_msgs::Octomap &octomap_msg, Eigen::Affine3d transform)
{

    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octomap_msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

    octomap::OcTree* newOctree = new octomap::OcTree(octomap_msg.resolution);

    octomath::Pose6D t = tf::poseEigenToOctomap(transform);

    std::cout << "starting octomap transform ...";
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end=octree->end_leafs(); it!= end; ++it)
    {
        //manipulate node, e.g.:
        octomath::Vector3 newCoordinate = t.transform(it.getCoordinate());
        newOctree->updateNode(newCoordinate,
                              it->getValue());

    }
    std::cout << "done" << std::endl;

    octomap_msgs::fullMapToMsg(*newOctree, octomap_msg);

    delete(newOctree);
    delete(octree);
}


octomap_msgs::Octomap octomap_utils::transformAndFilterOctomap(const octomap_msgs::Octomap &octomap_msg, octomath::Vector3 min, octomath::Vector3 max, Eigen::Affine3d transform)
{
    octomap_msgs::Octomap msg;
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octomap_msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

    octomap::OcTree* newOctree = new octomap::OcTree(octomap_msg.resolution);

    octomath::Pose6D t = tf::poseEigenToOctomap(transform);

    std::cout << "starting octomap transform and filter...";
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end=octree->end_leafs(); it!= end; ++it)
    {
        //manipulate node, e.g.:
        octomath::Vector3 newCoord = t.transform(it.getCoordinate());
        if(newCoord.x() > min.x() &&
                newCoord.x() < max.x() &&
                newCoord.y() > min.y() &&
                newCoord.y() < max.y() &&
                newCoord.z() > min.z() &&
                newCoord.z() < max.z())
            newOctree->updateNode(newCoord,
                                  it->getValue());

    }
    std::cout << "done" << std::endl;


    octomap_msgs::fullMapToMsg(*newOctree, msg);

    delete(newOctree);
    delete(octree);

    return msg;
}


void octomap_utils::filterOctomap(octomap_msgs::Octomap &octomap_msg,
                                  octomath::Vector3 min, octomath::Vector3 max,
                                  Eigen::Affine3d transform)
{
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octomap_msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    octomap::OcTree* newOctree = new octomap::OcTree(octomap_msg.resolution);

    octomath::Pose6D t = tf::poseEigenToOctomap(transform);

    std::cout << "starting octomap filtering ...";
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end=octree->end_leafs(); it!= end; ++it)
    {
        //manipulate node, e.g.:
        octomath::Vector3 coord = it.getCoordinate();
        octomath::Vector3 newCoord = t.transform(coord);
        if(newCoord.x() > min.x() &&
                newCoord.x() < max.x() &&
                newCoord.y() > min.y() &&
                newCoord.y() < max.y() &&
                newCoord.z() > min.z() &&
                newCoord.z() < max.z())
            newOctree->updateNode(coord,
                                  it->getValue());
    }

    std::cout << "done" << std::endl;

    octomap_msgs::fullMapToMsg(*newOctree, octomap_msg);

    delete(newOctree);
    delete(octree);
}


octomath::Pose6D tf::poseEigenToOctomap(Eigen::Affine3d transform)
{
    Eigen::Quaterniond qd(transform.rotation());
    octomath::Quaternion q(qd.w(), qd.x(), qd.y(), qd.z());
    octomath::Vector3 v(transform.translation().x(),
                        transform.translation().y(),
                        transform.translation().z());
    octomath::Pose6D t(v,q);
    return t;

    /* can be done also as:
               tf::Pose tf;
               tf::poseEigenToTF(transform, tf)
               return tf::poseTfToOctomap(tf);
            */
}


void octomap_utils::octomapWithPoseToOctomap(octomap_msgs::OctomapWithPose &octomap_msg)
{
    Eigen::Affine3d origin;
    tf::poseMsgToEigen(octomap_msg.origin, origin);

    octomap_utils::transformOctomap(octomap_msg.octomap,
                                    origin);
    origin.setIdentity();
    tf::poseEigenToMsg(origin, octomap_msg.origin);
    return;
}
