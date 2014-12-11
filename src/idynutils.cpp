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

using namespace iCub::iDynTree;
using namespace yarp::math;


iDynUtils::iDynUtils(const std::string robot_name_,
		     const std::string urdf_path,
		     const std::string srdf_path) :
    right_arm(walkman::robot::right_arm),
    right_leg(walkman::robot::right_leg),
    left_arm(walkman::robot::left_arm),
    left_leg(walkman::robot::left_leg),
    torso(walkman::robot::torso),
    robot_name(robot_name_),
    g(3,0.0)
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
}

// iDynUtils::iDynUtils(const std::string& robot_name_, 
// 		     const std::string& urdf_path,
// 		     const std::string& srdf_path):
//     right_arm(walkman::robot::right_arm),
//     right_leg(walkman::robot::right_leg),
//     left_arm(walkman::robot::left_arm),
//     left_leg(walkman::robot::left_leg),
//     torso(walkman::robot::torso),
//     robot_name(robot_name_),
//     robot_urdf_folder(urdf_path),
//     robot_srdf_folder(srdf_path),
//     g(3,0.0)
// {
//     std::cout << "Current robot_urdf_folder : " << robot_urdf_folder << std::endl;
//     std::cout << "Current robot_srdf_folder : " << robot_srdf_folder << std::endl;
//     
//     worldT.resize(4,4);
//     worldT.eye();
// 
//     g[2] = 9.81;
// 
//     bool iDyn3Model_loaded = iDyn3Model();
//     if(!iDyn3Model_loaded){
//         std::cout<<"Problem Loading iDyn3Model"<<std::endl;
//         assert(iDyn3Model_loaded);}
// 
//     bool setJointNames_ok = setJointNames();
//     if(!setJointNames_ok){
//         std::cout<<"Problems Setting Joint names"<<std::endl;
//         assert(setJointNames_ok);}
// 
//     setControlledKinematicChainsJointNumbers();
// 
//     zeros.resize(iDyn3_model.getNrOfDOFs(),0.0);
// }

const std::vector<std::string>& iDynUtils::getJointNames() const {
    return this->joint_names;
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
        std::cout<<group.name_<<std::endl;
        
        robot.getChain(chain.first,chain.second,temp);
        if (!setChainIndex(chain.second,k_chain)) return false;
        for (KDL::Segment& segment: temp.segments)
        {
            if (segment.getJoint().getType()==KDL::Joint::None) continue;
            std::cout<<segment.getJoint().getName()<<std::endl;
            k_chain.joint_names.push_back(segment.getJoint().getName());
        }
        return true;
    }
    return false;
}


bool iDynUtils::setJointNames()
{
    
    std::vector<std::string> temp_joint_names = this->moveit_robot_model->getJointModelNames();
    this->joint_names.resize(iDyn3_model.getNrOfDOFs(), "unknown_joint_name");
    for(unsigned int i = 0; i < temp_joint_names.size(); ++i) {
        int jointIndex = iDyn3_model.getDOFIndex(temp_joint_names[i]);
        if(jointIndex>=0 &&
           jointIndex <= iDyn3_model.getNrOfDOFs())
            joint_names[jointIndex] = temp_joint_names[i];
    }

    bool expected_left_arm = false;
    bool expected_right_arm = false;
    bool expected_left_leg = false;
    bool expected_right_leg = false;
    bool expected_torso = false;

    std::vector<srdf::Model::Group> coman_groups = robot_srdf->getGroups();
    for(auto group: coman_groups)
    {
        std::cout<<group.name_<<std::endl;
        if (group.name_==walkman::robot::chains)
        {
            int group_index=-1;
            if (findGroupChain(group.subgroups_,coman_groups,walkman::robot::left_arm,group_index))
                expected_left_arm=setChainJointNames(coman_groups[group_index],left_arm);
            if (findGroupChain(group.subgroups_,coman_groups,walkman::robot::left_leg,group_index))
                expected_left_leg=setChainJointNames(coman_groups[group_index],left_leg);
            if (findGroupChain(group.subgroups_,coman_groups,walkman::robot::right_arm,group_index))
                expected_right_arm=setChainJointNames(coman_groups[group_index],right_arm);
            if (findGroupChain(group.subgroups_,coman_groups,walkman::robot::right_leg,group_index))
                expected_right_leg=setChainJointNames(coman_groups[group_index],right_leg);
            if (findGroupChain(group.subgroups_,coman_groups,walkman::robot::torso,group_index))
                expected_torso=setChainJointNames(coman_groups[group_index],torso);
        }
    }

    if(!expected_left_arm) std::cout<<walkman::robot::left_arm<<" joint group is missing in SRDF"<<std::endl;
    if(!expected_right_arm) std::cout<<walkman::robot::right_arm<<" joint group is missing in SRDF"<<std::endl;
    if(!expected_left_leg) std::cout<<walkman::robot::left_leg<<" joint group is missing in SRDF"<<std::endl;
    if(!expected_right_leg) std::cout<<walkman::robot::right_leg<<" joint group is missing in SRDF"<<std::endl;
    if(!expected_torso) std::cout<<walkman::robot::torso<<" joint group is missing in SRDF"<<std::endl;

    if(expected_left_arm  &&
       expected_left_leg  &&
       expected_right_arm &&
       expected_right_leg &&
       expected_torso) return true;
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
        if (group.name_==walkman::robot::sensors)
        {
            for (auto joint:group.joints_)
            {
                joint_sensor_names.push_back(joint);
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

void iDynUtils::initWorldPose(const std::string& anchor)
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

    this->anchor_name = anchor;
    this->anchor_T_world = this->setWorldPose(anchor);

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


KDL::Frame iDynUtils::setWorldPose(const yarp::sig::Vector& q,
                             const yarp::sig::Vector& dq_ref,
                             const yarp::sig::Vector& ddq_ref,
                             const std::string& anchor)
{
    updateiDyn3Model(q,dq_ref,ddq_ref);
    
//     yarp::sig::Matrix worldT(4,4);
//     worldT.eye();
//     iDyn3_model.setWorldBasePose(worldT);
//     yarp::sig::Vector foot_pose(3);
//     foot_pose = iDyn3_model.getPosition(iDyn3_model.getLinkIndex("r_sole")).getCol(3).subVector(0,2);
//     worldT(2,3) = -foot_pose(2);
//     //std::cout<<"World Base Pose: "<<std::endl; cartesian_utils::printHomogeneousTransform(worldT);std::cout<<std::endl;
//     iDyn3_model.setWorldBasePose(worldT);
//     iDyn3_model.computePositions();
    return setWorldPose(anchor);
}

/**
 * THIS IS AN EXAMPLE OF A MODULE USING THESE UTILS
 * 
 * q=yarp_interface.sense();
 * updateiDyn3Model(q);
 * receiveTargetPosition(waist_target_position);
 *  KDL::Frame waist_target_position;
 *  KDL::Frame from_waist_to_end_effector = fromYARPMatrixtoKDLFrame(iDyn3_model.getPosition(chain.index,true));
 *  KDL::Vector position_error=-from_waist_to_end_effector*waist_target_position;
 *  J=getSmallJacobian()
 *  q_dot=pinv(J)*(k*position_error) , k>0, q_dot.size=chain.size
 *  yarp_interface.moveSpeed(q_dot);
 */

yarp::sig::Matrix iDynUtils::getSimpleChainJacobian(const kinematic_chain chain, bool world_frame)
{    
    yarp::sig::Matrix J_chain(6, chain.getNrOfDOFs()); J_chain.zero();
    yarp::sig::Matrix J_whole;

    int waist = 0;

    std::cout.flush();

    if(!iDyn3_model.getRelativeJacobian(chain.end_effector_index, waist, J_whole, world_frame)) {
        std::cout << "Error computing Jacobian for chain " << chain.chain_name << std::endl;
        return J_chain;
    }

    for(unsigned int i = 0; i < chain.getNrOfDOFs(); i++)
    {
        J_chain.setCol(i, J_whole.getCol(chain.joint_numbers[i]));
    }

    return J_chain;
}

void iDynUtils::updateiDyn3Model(const yarp::sig::Vector& q,
                                 const bool set_world_pose,
                                 const std::string &support_foot) {
    this->updateiDyn3Model(q,zeros,zeros, set_world_pose, support_foot);
}

void iDynUtils::updateiDyn3Model(const yarp::sig::Vector& q,
                                 const yarp::sig::Vector& dq,
                                 const bool set_world_pose,
                                 const std::string &support_foot) {
    this->updateiDyn3Model(q,dq,zeros, set_world_pose, support_foot);
}

void iDynUtils::updateiDyn3Model(const yarp::sig::Vector& q,
                                 const yarp::sig::Vector& dq_ref,
                                 const yarp::sig::Vector& ddq_ref,
                                 const bool set_world_pose,
                                 const std::string &support_foot)
{
    // Here we set these values in our internal model
    iDyn3_model.setAng(q);
    iDyn3_model.setDAng(dq_ref);
    iDyn3_model.setD2Ang(ddq_ref);

    // setting the world pose

    if(set_world_pose) {
        if(anchor_name.length() == 0)
            this->initWorldPose(support_foot);
        this->updateWorldPose();
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
    //std::cout<<chain.end_effector_name<<" joint indices: \n";
    for(auto joint_name: chain.joint_names){
        //std::cout<<iDyn3_model.getDOFIndex(joint_name)<<" ";
        chain.joint_numbers.push_back(iDyn3_model.getDOFIndex(joint_name));
    }
//    std::cout<<std::endl;
}

void iDynUtils::setControlledKinematicChainsJointNumbers()
{
    setJointNumbers(right_arm);
    setJointNumbers(right_leg);
    setJointNumbers(left_arm);
    setJointNumbers(left_leg);
    setJointNumbers(torso);
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

    if(contacts & CONTACT_LEFT_HAND) {
        this->iDyn3_model.getJacobian(this->left_arm.end_effector_index, J_left_hand);
        if(J_contacts.rows() == 0)
            J_contacts = J_left_hand;
        else
            J_contacts = pile(J_contacts, J_left_hand);
    }

    if(contacts & CONTACT_RIGHT_HAND) {
        this->iDyn3_model.getJacobian(this->right_arm.end_effector_index, J_right_hand);
        if(J_contacts.rows() == 0)
            J_contacts = J_right_hand;
        else
            J_contacts = pile(J_contacts, J_right_hand);
    }

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


unsigned int kinematic_chain::getNrOfDOFs() const {
    return joint_numbers.size();
}
