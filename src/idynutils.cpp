/*
 * Copyright (C) 2014 Walkman
 * Author: Mirko Ferrati, Enrico Mingo, Alessio Rocchi, Federico Moro
 * email:  mirko.ferrati@gmail.com, enrico.mingo@iit.it, alessio.rocchi@iit.it
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

#include <idynutils/idynutils.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <idynutils/yarp_single_chain_interface.h>
#include <yarp/math/SVD.h>
#include <idynutils/cartesian_utils.h>
#include <moveit/robot_model/joint_model.h>

using namespace iCub::iDynTree;
using namespace yarp::math;

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

iDynUtils::iDynUtils(const std::string robot_name_,
		     const std::string urdf_path,
		     const std::string srdf_path) :
//     right_arm(walkman::robot::right_arm),
    right_leg(walkman::robot::right_leg),
//     left_arm(walkman::robot::left_arm),
    left_leg(walkman::robot::left_leg),
//     torso(walkman::robot::torso),
    robot_name(robot_name_),
    g(3,0.0),
    anchor_name("l_sole"),
    world_is_inited(false)
{
    worldT.resize(4,4);
    worldT.eye();

    g[2] = 9.81;

    std::string folder = std::string(robot_name_+"_folder");
    // initialize the path for urdf
    if( urdf_path != "" ) {
	robot_urdf_folder = urdf_path;
    }
    else {
	std::string robot_folder = std::string(getenv(folder.c_str()));	//NOTE do the getenv only if needed TODO check NULL
	robot_urdf_folder = robot_folder+"/urdf/"+robot_name_+".urdf";
    }
    
    // initialize the path for srdf
    if( srdf_path != "" ) {
	robot_srdf_folder = srdf_path;
    }
    else {
	std::string robot_folder = std::string(getenv(folder.c_str()));	//NOTE do the getenv only if needed TODO check NULL
	robot_srdf_folder = robot_folder+"/srdf/"+robot_name_+".srdf";
    }
	
    
    bool iDyn3Model_loaded = iDyn3Model();
    if(!iDyn3Model_loaded){
        std::cout<<"Problem Loading iDyn3Model"<<std::endl;
        assert(iDyn3Model_loaded);}

    bool setJointNames_ok = setJointNames();
    if(!setJointNames_ok){
        std::cout<<"Problems Setting Joint names"<<std::endl;
        assert(setJointNames_ok);}

    setControlledKinematicChainsJointNumbers();

    zeros.resize(iDyn3_model.getNrOfDOFs(),0.0);

    links_in_contact.push_back("l_foot_lower_left_link");
    links_in_contact.push_back("l_foot_lower_right_link");
    links_in_contact.push_back("l_foot_upper_left_link");
    links_in_contact.push_back("l_foot_upper_right_link");
    links_in_contact.push_back("r_foot_lower_left_link");
    links_in_contact.push_back("r_foot_lower_right_link");
    links_in_contact.push_back("r_foot_upper_left_link");
    links_in_contact.push_back("r_foot_upper_right_link");
}

const std::vector<std::string>& iDynUtils::getJointNames() const {
    return this->joint_names;
}

const std::vector<std::string>& iDynUtils::getFixedJointNames() const {
    return this->fixed_joint_names;
}

bool iDynUtils::findGroupChain(const std::vector<std::string>& chain_list, const std::vector<srdf::Model::Group>& groups,std::string chain_name, int& group_index)
{
    for (auto iterator_chain:chain_list)
    {
        if (iterator_chain==chain_name)
        {
            int index=0;
            for(auto group:groups)
            {
                if (group.name_==chain_name)
                {
                    group_index=index;
                    return true;
                }
                index++;
            }
        }
    }
    group_index=-1;
    return false;
}

bool iDynUtils::setChainJointNames(const srdf::Model::Group& group, kinematic_chain& k_chain)
{
    const KDL::Tree& robot = iDyn3_model.getKDLTree();
    if (group.chains_.size()==1)
    {
        auto chain=group.chains_[0];
        KDL::Chain temp;
        
        robot.getChain(chain.first,chain.second,temp);
        if (!setChainIndex(chain.second,k_chain)) return false;
        for (KDL::Segment& segment: temp.segments)
        {
            if (segment.getJoint().getType()==KDL::Joint::None)
                k_chain.fixed_joint_names.push_back(segment.getJoint().getName());
            else
                k_chain.joint_names.push_back(segment.getJoint().getName());
        }

        std::vector<std::string> explicit_joints = group.joints_;
        for(unsigned int i = 0; i < explicit_joints.size(); ++i)
        {
            if(moveit_robot_model->getJointModel(explicit_joints[i])->getType() == moveit::core::JointModel::FIXED)
            {
                auto it = std::find (k_chain.fixed_joint_names.begin(), k_chain.fixed_joint_names.end(), explicit_joints[i]);
                if (it == k_chain.fixed_joint_names.end())
                    k_chain.fixed_joint_names.push_back(explicit_joints[i]);
            }
            else
            {
                auto it = std::find (k_chain.joint_names.begin(), k_chain.joint_names.end(), explicit_joints[i]);
                if (it == k_chain.joint_names.end())
                    k_chain.joint_names.push_back(explicit_joints[i]);
            }
        }

        std::cout<<GREEN<<" "<<group.name_<<DEFAULT<<std::endl;
        for(unsigned int i = 0; i < k_chain.joint_names.size(); ++i)
            std::cout<<"    "<<k_chain.joint_names[i]<<"  Active"<<std::endl;
        for(unsigned int i = 0; i < k_chain.fixed_joint_names.size(); ++i)
            std::cout<<"    "<<k_chain.fixed_joint_names[i]<<"  Fixed"<<std::endl;

        return true;
    }
    return false;
}


bool iDynUtils::setJointNames()
{
    //Index 0 is a string that do not exists!
    for(unsigned int i = 1; i < moveit_robot_model->getJointModels().size(); ++i){
        if(moveit_robot_model->getJointModels()[i]->getType() == moveit::core::JointModel::FIXED)
            fixed_joint_names.push_back(moveit_robot_model->getJointModels()[i]->getName());
        else
            joint_names.push_back(moveit_robot_model->getJointModels()[i]->getName());}


    bool expected_left_arm = false;
    bool expected_right_arm = false;
    bool expected_left_leg = false;
    bool expected_right_leg = false;
    bool expected_torso = false;

    std::cout<<GREEN<<"KINEMATICS CHAINS & JOINTS:"<<DEFAULT<<std::endl;
    std::vector<srdf::Model::Group> coman_groups = robot_srdf->getGroups();
    for(auto group: coman_groups)
    {
        if (group.name_==walkman::robot::chains)
        {
            int group_index=-1;
//             if (findGroupChain(group.subgroups_,coman_groups,walkman::robot::left_arm,group_index))
//                 expected_left_arm=setChainJointNames(coman_groups[group_index],left_arm);
            if (findGroupChain(group.subgroups_,coman_groups,walkman::robot::left_leg,group_index))
                expected_left_leg=setChainJointNames(coman_groups[group_index],left_leg);
//             if (findGroupChain(group.subgroups_,coman_groups,walkman::robot::right_arm,group_index))
//                 expected_right_arm=setChainJointNames(coman_groups[group_index],right_arm);
            if (findGroupChain(group.subgroups_,coman_groups,walkman::robot::right_leg,group_index))
                expected_right_leg=setChainJointNames(coman_groups[group_index],right_leg);
//             if (findGroupChain(group.subgroups_,coman_groups,walkman::robot::torso,group_index))
//                 expected_torso=setChainJointNames(coman_groups[group_index],torso);
        }
    }

//     if(!expected_left_arm) std::cout<<walkman::robot::left_arm<<" joint group is missing in SRDF"<<std::endl;
//     if(!expected_right_arm) std::cout<<walkman::robot::right_arm<<" joint group is missing in SRDF"<<std::endl;
    if(!expected_left_leg) std::cout<<walkman::robot::left_leg<<" joint group is missing in SRDF"<<std::endl;
    if(!expected_right_leg) std::cout<<walkman::robot::right_leg<<" joint group is missing in SRDF"<<std::endl;
//     if(!expected_torso) std::cout<<walkman::robot::torso<<" joint group is missing in SRDF"<<std::endl;

    if(
// 	  expected_left_arm  &&
       expected_left_leg  &&
//        expected_right_arm &&
       expected_right_leg //&&
//        expected_torso
	  ) return true;
    return false;

}


bool iDynUtils::iDyn3Model()
{
    /// iDyn3 Model creation
    // Giving name to references for FT sensors and IMU
    std::vector<std::string> joint_sensor_names;
    std::string base_link_name;

    urdf_model.reset(new urdf::Model());
    std::string model_folder, srdf_folder;
    std::cout<<" - USING ROBOT "<<robot_name<<" - "<<std::endl;

    if (!urdf_model->initFile(robot_urdf_folder))
    {
        std::cout<<"Failed to parse urdf robot model"<<std::endl;
        return false;
    }
    else
    {
        robot_srdf.reset(new srdf::Model());
        if(!robot_srdf->initFile(*urdf_model, robot_srdf_folder))
        {
            std::cout<<"Failed to parse SRDF robot model!"<<std::endl;
            return false;
        }
        else
        {
            moveit_robot_model.reset(new robot_model::RobotModel(urdf_model, robot_srdf));
            std::ostringstream robot_info;
            moveit_robot_model->printModelInfo(robot_info);
            //ROS_INFO(robot_info.str().c_str());
        }
    }
    
    std::vector<srdf::Model::Group> groups = robot_srdf->getGroups();

    for(auto group: groups)
    {
        if (group.name_==walkman::robot::force_torque_sensors)
        {
            for (auto joint:group.joints_)
            {
                if (moveit_robot_model->getJointModel(joint)->getType() == moveit::core::JointModel::FIXED)
                    joint_sensor_names.push_back(joint);
                else
                    assert(false && "joint inside the force torque sensor list of the srdf has to be fixed!!");
            }
        }
        if (group.name_==walkman::robot::base)
            base_link_name=group.links_[0];
    }
    
    if (!kdl_parser::treeFromUrdfModel(*urdf_model, robot_kdl_tree)){
        std::cout<<"Failed to construct kdl tree"<<std::endl;
        return false;}
    
    // Here the iDyn3 model of the robot is generated
    iDyn3_model.constructor(robot_kdl_tree, joint_sensor_names, base_link_name);
    std::cout<<"Loaded"<<robot_name<<"in iDynTree!"<<std::endl;
    
    int nJ = iDyn3_model.getNrOfDOFs(); //29
    yarp::sig::Vector qMax; qMax.resize(nJ,0.0);
    yarp::sig::Vector qMin; qMin.resize(nJ,0.0);
    
    std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator i;
    for(i = urdf_model->joints_.begin(); i != urdf_model->joints_.end(); ++i) {
        int jIndex = iDyn3_model.getDOFIndex(i->first);
        if(jIndex != -1) {
            qMax[jIndex] = i->second->limits->upper;
            qMin[jIndex] = i->second->limits->lower;
        }
    }
    
    iDyn3_model.setJointBoundMax(qMax);
    iDyn3_model.setJointBoundMin(qMin);
    
    yarp::sig::Vector tauMax; tauMax.resize(nJ,1.0);
    for(i = urdf_model->joints_.begin(); i != urdf_model->joints_.end(); ++i) {
        int jIndex = iDyn3_model.getDOFIndex(i->first);
        if(jIndex != -1) {
            tauMax[jIndex] = i->second->limits->effort;
        }
    }
    
    //std::cout<<"Setting torque MAX"<<std::endl;

   iDyn3_model.setJointTorqueBoundMax(tauMax);
    

    /*std::cout<<"MAX TAU: [ "<<iDyn3_model.getJointTorqueMax().toString()<<std::endl;
    std::cout<<"Loaded COMAN in iDyn3!"<<std::endl;
    //std::cout<<"#Links: "<<iDyn3_model.getNrOfLinks()<<std::endl;
    */
    return true;
}

bool iDynUtils::setChainIndex(std::string endeffector_name,kinematic_chain& chain)
{
    chain.end_effector_name=endeffector_name;
    chain.end_effector_index = iDyn3_model.getLinkIndex(chain.end_effector_name);
    if(chain.end_effector_index == -1){
        std::cout << "Failed to get link index for "<<chain.chain_name<< std::endl;
        return false;}
    return true;
}

void iDynUtils::fromRobotToIDyn(const yarp::sig::Vector& q_chain,
                                yarp::sig::Vector& q_out,
                                kinematic_chain& chain)
{
    for(unsigned int i = 0; i < chain.joint_numbers.size(); ++i)
    {
        q_out[chain.joint_numbers[i]] = q_chain[i];
    }
}

void iDynUtils::fromIDynToRobot(const yarp::sig::Vector& q,
                                yarp::sig::Vector& q_chain_out,
                                kinematic_chain& chain)
{
    for(unsigned int i = 0; i < chain.joint_numbers.size(); ++i)
    {
        q_chain_out[i] = q[chain.joint_numbers[i]];
    }
}

void iDynUtils::initWorldPose()
{
    // saving old values of Ang,DAng,D2Ang
    yarp::sig::Vector Ang,DAng,D2Ang;
    Ang = iDyn3_model.getAng();
    DAng = iDyn3_model.getDAng();
    D2Ang = iDyn3_model.getD2Ang();

    // setting joints to default configuration
    iDyn3_model.setAng(zeros);
    iDyn3_model.setDAng(zeros);
    iDyn3_model.setD2Ang(zeros);
    iDyn3_model.kinematicRNEA();
    iDyn3_model.computePositions();

    this->anchor_T_world = this->setWorldPose(anchor_name);

    // restoring old values for Ang,DAng,D2Ang
    iDyn3_model.setAng(Ang);
    iDyn3_model.setDAng(DAng);
    iDyn3_model.setD2Ang(D2Ang);
}

void iDynUtils::updateWorldPose()
{
    assert(this->anchor_name.length() > 0);
    this->setWorldPose(anchor_T_world, anchor_name);
}



KDL::Frame iDynUtils::setWorldPose(const std::string& anchor)
{
    // worldT = anchor_T_baselink
    worldT = iDyn3_model.getPosition(iDyn3_model.getLinkIndex(anchor),0);

    // anchor_T_world is the offset between the anchor link and the inertial frame,
    //                which is in this case the projection of the base link
    //                ( the base link position vector in anchor frame )
    //                on to the plane determined by the anchor frame x and y axes
    KDL::Frame anchor_T_world;
    anchor_T_world.Identity();
    anchor_T_world.p[0] = worldT(0,3);
    anchor_T_world.p[1] = worldT(1,3);

    worldT(0,3) = 0.0;
    worldT(1,3) = 0.0;

    if(iDyn3_model.getFloatingBaseLink() != 0) {
        worldT = worldT*iDyn3_model.getPosition(0,iDyn3_model.getFloatingBaseLink());
    }

    iDyn3_model.setWorldBasePose(worldT);

    return anchor_T_world;
}

void iDynUtils::setWorldPose(const KDL::Frame& anchor_T_world, const std::string& anchor)
{
    if(iDyn3_model.getLinkIndex(anchor) != iDyn3_model.getFloatingBaseLink()) {
        worldT =    KDLtoYarp_position(
                        anchor_T_world.Inverse()
                        *
                        iDyn3_model.getPositionKDL(iDyn3_model.getLinkIndex(anchor),iDyn3_model.getFloatingBaseLink())
                    );
    } else {
        worldT = KDLtoYarp_position(anchor_T_world.Inverse());
    }

    iDyn3_model.setWorldBasePose(worldT);
}

bool iDynUtils::getWorldPose(KDL::Frame &anchor_T_world, std::string &anchor) const
{
    if(!world_is_inited) return false;
    assert(this->anchor_name.size() > 0);
    anchor_T_world = this->anchor_T_world;
    anchor = this->anchor_name;
}

const KDL::Frame iDynUtils::getAnchor_T_World() const
{
    return this->anchor_T_world;
}

void iDynUtils::setAnchor_T_World(const KDL::Frame& anchor_T_world)
{
    world_is_inited = true;
    this->anchor_T_world = anchor_T_world;
}

void iDynUtils::updateiDyn3Model(const yarp::sig::Vector& q,
                                 const bool set_world_pose) {
    this->updateiDyn3Model(q,zeros,zeros, set_world_pose);
}

void iDynUtils::updateiDyn3Model(const yarp::sig::Vector& q,
                                 const yarp::sig::Vector& dq,
                                 const bool set_world_pose) {
    this->updateiDyn3Model(q,dq,zeros, set_world_pose);
}

void iDynUtils::updateiDyn3Model(const yarp::sig::Vector& q,
                                 const yarp::sig::Vector& dq_ref,
                                 const yarp::sig::Vector& ddq_ref,
                                 const bool set_world_pose)
{
    // Here we set these values in our internal model
    iDyn3_model.setAng(q);
    iDyn3_model.setDAng(dq_ref);
    iDyn3_model.setD2Ang(ddq_ref);

    // setting the world pose

    if(set_world_pose) {
            if(!world_is_inited) {
                this->initWorldPose();
                world_is_inited = true;
            } this->updateWorldPose();
        }

    // This is the fake Inertial Measure
    g.zero();
    g[2] = 9.81;

    // get the rotational part of worldT (w_R_b),
    // compute the inverse (b_R_w = w_R_b^T) and multiply by w_g
    // to obtain g expressed in base link coordinates, b_g
    g = (worldT * iDyn3_model.getPosition(0,iDyn3_model.getFloatingBaseLink())).submatrix(0,2,0,2).transposed() * g;

    yarp::sig::Vector o(3,0.0);
    iDyn3_model.setInertialMeasure(o, o, g);

    iDyn3_model.kinematicRNEA();

    iDyn3_model.dynamicRNEA();

    iDyn3_model.computePositions();
}

void iDynUtils::setJointNumbers(kinematic_chain& chain)
{
    for(auto joint_name: chain.joint_names)
        chain.joint_numbers.push_back(iDyn3_model.getDOFIndex(joint_name));
}

void iDynUtils::setControlledKinematicChainsJointNumbers()
{
//     setJointNumbers(right_arm);
    setJointNumbers(right_leg);
//     setJointNumbers(left_arm);
    setJointNumbers(left_leg);
//     setJointNumbers(torso);
}

yarp::sig::Matrix iDynUtils::computeFloatingBaseProjector(const int contacts) {
    yarp::sig::Matrix J_left_foot, J_right_foot, J_left_hand, J_right_hand;
    yarp::sig::Matrix J_contacts;

    if(contacts & CONTACT_LEFT_FOOT) {
        this->iDyn3_model.getJacobian(this->left_leg.end_effector_index, J_left_foot);
        J_contacts = J_left_foot;
    }

    if(contacts & CONTACT_RIGHT_FOOT) {
        this->iDyn3_model.getJacobian(this->right_leg.end_effector_index, J_right_foot);
        if(J_contacts.rows() == 0)
            J_contacts = J_right_foot;
        else
            J_contacts = pile(J_contacts, J_right_foot);
    }

//     if(contacts & CONTACT_LEFT_HAND) {
//         this->iDyn3_model.getJacobian(this->left_arm.end_effector_index, J_left_hand);
//         if(J_contacts.rows() == 0)
//             J_contacts = J_left_hand;
//         else
//             J_contacts = pile(J_contacts, J_left_hand);
//     }

//     if(contacts & CONTACT_RIGHT_HAND) {
//         this->iDyn3_model.getJacobian(this->right_arm.end_effector_index, J_right_hand);
//         if(J_contacts.rows() == 0)
//             J_contacts = J_right_hand;
//         else
//             J_contacts = pile(J_contacts, J_right_hand);
//     }

    return computeFloatingBaseProjector(J_contacts);

}

yarp::sig::Matrix iDynUtils::computeFloatingBaseProjector(const yarp::sig::Matrix& JContacts) {
    int nJ = this->iDyn3_model.getNrOfDOFs();
    /**
     * @brief nullContacts is a R^{n+6xn+6} matrix that projects into
     *                     the null space of J_contacts
     */
    yarp::sig::Matrix nullContacts = nullspaceProjection(JContacts, 1E-6);
    assert(nullContacts.cols() == nullContacts.rows());
    assert(nullContacts.cols() == nJ + 6);

    yarp::sig::Matrix floatingBaseProjector;

    floatingBaseProjector = pinv(nullContacts.submatrix(6,6 + nJ-1,
                                                        6,6 + nJ-1),1E-7)
                                                        *
                                nullContacts.submatrix(6,6 + nJ -1,
                                                       0,6 + nJ -1);

    assert(floatingBaseProjector.cols() == 6+nJ);
    assert(floatingBaseProjector.rows() == nJ);

    return floatingBaseProjector;
}

bool iDynUtils::switchAnchor(const std::string& new_anchor)
{
    int link_index = iDyn3_model.getLinkIndex(new_anchor);
    if(link_index != -1)
    {
        anchor_name = new_anchor;
        anchor_T_world = iDyn3_model.getPositionKDL(link_index, true);
        setWorldPose(anchor_T_world, anchor_name);

        return true;
    }
    return false;
}

const std::string iDynUtils::getAnchor() const
{
    return this->anchor_name;
}

bool iDynUtils::setFloatingBaseLink(const std::string new_base)
{
    int new_fb_index = iDyn3_model.getLinkIndex(new_base);
    int old_fb_index = iDyn3_model.getFloatingBaseLink();
    if(new_fb_index != -1 &&
       old_fb_index != -1)
    {
        if(iDyn3_model.setFloatingBaseLink(new_fb_index))
        {
            setWorldPose(anchor_T_world, anchor_name);
            return true;
        }
    }
    return false;
}

bool iDynUtils::switchAnchorAndFloatingBase(const std::string new_anchor)
{
    return switchAnchor(new_anchor) && setFloatingBaseLink(new_anchor);
}

unsigned int kinematic_chain::getNrOfDOFs() const {
    return joint_numbers.size();
}

const std::string iDynUtils::getRobotName() const
{
    return robot_name;
}

const std::string iDynUtils::getRobotURDFPath() const
{
    return robot_urdf_folder;
}

const std::string iDynUtils::getRobotSRDFPath() const
{
    return robot_srdf_folder;
}

const std::list<std::string>& iDynUtils::getLinksInContact(){
    return links_in_contact;
}

void iDynUtils::setLinksInContact(const std::list<std::string>& list_links_in_contact){
    std::list<std::string> tmp_list;
    for(std::list<std::string>::const_iterator it = list_links_in_contact.begin(); it != list_links_in_contact.end(); it++)
    {
        int link_index = iDyn3_model.getLinkIndex(*it);
        if(!(link_index == -1))
            tmp_list.push_back(*it);
    }

    if(!tmp_list.empty())
        links_in_contact = tmp_list;
}


bool iDynUtils::getSupportPolygonPoints(std::list<KDL::Vector>& points)
{
    if(links_in_contact.empty())
        return false;

    KDL::Frame waist_T_CoM;
    KDL::Frame waist_T_point;
    KDL::Frame CoM_T_point;
    for(std::list<std::string>::iterator it = links_in_contact.begin(); it != links_in_contact.end(); it++)
    {
        // get points in world frame
        waist_T_point = iDyn3_model.getPositionKDL(iDyn3_model.getLinkIndex(*it));
        // get CoM in the world frame
        YarptoKDL(iDyn3_model.getCOM(), waist_T_CoM.p);

        CoM_T_point = waist_T_CoM.Inverse() * waist_T_point;
        points.push_back(CoM_T_point.p);
    }
    return true;
}
