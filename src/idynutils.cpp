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
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames_io.hpp>

using namespace iCub::iDynTree;
using namespace yarp::math;

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

iDynUtils::iDynUtils(const std::string robot_name_,
		     const std::string urdf_path,
		     const std::string srdf_path) :
    right_arm(walkman::robot::right_arm),
    right_leg(walkman::robot::right_leg),
    left_arm(walkman::robot::left_arm),
    left_leg(walkman::robot::left_leg),
    torso(walkman::robot::torso),
    head(walkman::robot::head),
    robot_name(robot_name_),
    g(3,0.0),
    anchor_name(""),  // temporary value. Will get updated as soon as we load kinematic chains
    world_is_inited(false),
    _computeDynamics(true)
{
    worldT.resize(4,4);
    worldT.eye();

    g[2] = 9.81;

    if( urdf_path != "" )
        robot_urdf_folder = urdf_path;
    else
        std::cout << "The urdf path is empty!" << std::endl;

    if( srdf_path != "" )
        robot_srdf_folder = srdf_path;
    else
        std::cout << "The srdf path is empty!" << std::endl;
    
    bool iDyn3Model_loaded = iDyn3Model();
    if(!iDyn3Model_loaded){
        std::cout<<"Problem Loading iDyn3Model"<<std::endl;
        assert(iDyn3Model_loaded);}

    bool setJointNames_ok = setJointNames();
    if(!setJointNames_ok){
        std::cout<<"Problems Setting Joint names"<<std::endl;
        assert(setJointNames_ok && "No chains found!");}
    anchor_name = left_leg.end_effector_name;

    setControlledKinematicChainsJointNumbers();

    zeros.resize(iDyn3_model.getNrOfDOFs(),0.0);
    zerosXd.setZero(iDyn3_model.getNrOfDOFs());

    links_in_contact.push_back("l_foot_lower_left_link");
    links_in_contact.push_back("l_foot_lower_right_link");
    links_in_contact.push_back("l_foot_upper_left_link");
    links_in_contact.push_back("l_foot_upper_right_link");
    links_in_contact.push_back("r_foot_lower_left_link");
    links_in_contact.push_back("r_foot_lower_right_link");
    links_in_contact.push_back("r_foot_upper_left_link");
    links_in_contact.push_back("r_foot_upper_right_link");

    readForceTorqueSensorsNames();
    readIMUSensorsNames();
    _w_R_imu.first = "";
    _w_R_imu.second = yarp::sig::Matrix();
}

const std::vector<std::string>& iDynUtils::getJointNames() const {
    return this->joint_names;
}

const std::vector<std::string>& iDynUtils::getFixedJointNames() const {
    return this->fixed_joint_names;
}

bool iDynUtils::findGroupChain(const std::vector<std::string>& chain_list, const std::vector<srdf::Model::Group>& groups,std::string chain_name, int& group_index)
{
    for (std::vector<std::string>::const_iterator it_chain = chain_list.begin();
         it_chain != chain_list.end();
         ++it_chain)
    {
        const std::string& iterator_chain = *it_chain;
        if (iterator_chain==chain_name)
        {
            int index=0;
            for(std::vector<srdf::Model::Group>::const_iterator it_groups = groups.begin();
                it_groups != groups.end();
                ++it_groups)
            {
                const srdf::Model::Group& group = *it_groups;
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
        std::pair<std::string, std::string> chain=group.chains_[0];
        KDL::Chain temp;
        
        robot.getChain(chain.first,chain.second,temp);
        if (!setChainIndex(chain.second,k_chain)) return false;
        for (std::vector<KDL::Segment>::iterator it_segments = temp.segments.begin();
             it_segments != temp.segments.end();
             ++it_segments)
        {
            KDL::Segment& segment = *it_segments;
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
                std::vector<std::string>::iterator it = std::find (k_chain.fixed_joint_names.begin(), k_chain.fixed_joint_names.end(), explicit_joints[i]);
                if (it == k_chain.fixed_joint_names.end())
                    k_chain.fixed_joint_names.push_back(explicit_joints[i]);
            }
            else
            {
                std::vector<std::string>::iterator it = std::find (k_chain.joint_names.begin(), k_chain.joint_names.end(), explicit_joints[i]);
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
    bool expected_head = false;

    std::cout<<GREEN<<"KINEMATICS CHAINS & JOINTS:"<<DEFAULT<<std::endl;
    std::vector<srdf::Model::Group> robot_groups = robot_srdf->getGroups();
    for(std::vector<srdf::Model::Group>::iterator it_groups = robot_groups.begin();
        it_groups != robot_groups.end();
        ++it_groups)
    {
        srdf::Model::Group& group = *it_groups;
        if (group.name_==walkman::robot::chains)
        {
            int group_index=-1;
            if (findGroupChain(group.subgroups_,robot_groups,walkman::robot::left_arm,group_index))
                expected_left_arm=setChainJointNames(robot_groups[group_index],left_arm);
            if (findGroupChain(group.subgroups_,robot_groups,walkman::robot::left_leg,group_index))
                expected_left_leg=setChainJointNames(robot_groups[group_index],left_leg);
            if (findGroupChain(group.subgroups_,robot_groups,walkman::robot::right_arm,group_index))
                expected_right_arm=setChainJointNames(robot_groups[group_index],right_arm);
            if (findGroupChain(group.subgroups_,robot_groups,walkman::robot::right_leg,group_index))
                expected_right_leg=setChainJointNames(robot_groups[group_index],right_leg);
            if (findGroupChain(group.subgroups_,robot_groups,walkman::robot::torso,group_index))
                expected_torso=setChainJointNames(robot_groups[group_index],torso);
            if (findGroupChain(group.subgroups_,robot_groups,walkman::robot::head,group_index))
                expected_head=setChainJointNames(robot_groups[group_index],head);
        }
    }

    if(!expected_left_arm) std::cout<<walkman::robot::left_arm<<" joint group left_arm is missing in SRDF"<<std::endl;
    if(!expected_right_arm) std::cout<<walkman::robot::right_arm<<" joint group right_arm is missing in SRDF"<<std::endl;
    if(!expected_left_leg) std::cout<<walkman::robot::left_leg<<" joint group left_leg is missing in SRDF"<<std::endl;
    if(!expected_right_leg) std::cout<<walkman::robot::right_leg<<" joint group right_leg is missing in SRDF"<<std::endl;
    if(!expected_torso) std::cout<<walkman::robot::torso<<" joint group torso is missing in SRDF"<<std::endl;
    if(!expected_head) std::cout<<walkman::robot::head<<" joint group head is missing in SRDF"<<std::endl;

    if(expected_left_arm  ||
       expected_left_leg  ||
       expected_right_arm ||
       expected_right_leg ||
       expected_torso     ||
       expected_head) return true;
    return false;

}



bool iDynUtils::iDyn3Model()
{
    /// iDyn3 Model creation
    // Giving name to references for FT sensors and IMU
    std::vector<std::string> joint_ft_sensor_names;
    std::vector<std::string> imu_link_names;

    urdf_model.reset(new urdf::Model());
    std::cout<<" - USING ROBOT "<<robot_name<<" - "<<std::endl;

    if (!urdf_model->initFile(robot_urdf_folder))
    {
        std::cout<<"Failed to parse urdf robot model"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"URDF LOADED"<<std::endl;
        robot_srdf.reset(new srdf::Model());
        if(!robot_srdf->initFile(*urdf_model, robot_srdf_folder))
        {
            std::cout<<"Failed to parse SRDF robot model!"<<std::endl;
            return false;
        }
        else
        {
            std::cout<<"SRDF LOADED"<<std::endl;

            moveit_planning_scene.reset(new planning_scene::PlanningScene(urdf_model, robot_srdf));
            moveit_robot_model = moveit_planning_scene->getRobotModel();
            std::ostringstream robot_info;
            moveit_robot_model->printModelInfo(robot_info);
            moveit_collision_robot = moveit_planning_scene->getCollisionRobotNonConst();
            std::cout<<"ROBOT LOADED in MOVEIT!"<<std::endl;
        }
    }
    
    std::vector<srdf::Model::Group> groups = robot_srdf->getGroups();

    for(std::vector<srdf::Model::Group>::iterator it_groups = groups.begin();
        it_groups != groups.end();
        ++it_groups)
    {
        srdf::Model::Group& group = *it_groups;
        if (group.name_==walkman::robot::force_torque_sensors)
        {
            for (std::vector<std::string>::iterator it_joints = group.joints_.begin();
                 it_joints != group.joints_.end();
                 ++it_joints)
            {
                std::string& joint = *it_joints;
                if (moveit_robot_model->getJointModel(joint)->getType() == moveit::core::JointModel::FIXED)
                    joint_ft_sensor_names.push_back(joint);
                else
                    assert(false && "joint inside the force torque sensor list of the srdf has to be fixed!!");
            }

        }

        if(group.name_==walkman::robot::imu_sensors)
        {
            for(unsigned int i = 0; i < group.links_.size(); ++i)
                imu_link_names.push_back(group.links_[i]);
        }

        if (group.name_==walkman::robot::base)
            base_link_name=group.links_[0];
    }
    
    if (!kdl_parser::treeFromUrdfModel(*urdf_model, robot_kdl_tree)){
        std::cout<<"Failed to construct kdl tree"<<std::endl;
        return false;}
    std::cout<<"ROBOT LOADED in KDL"<<std::endl;
    
    // Here the iDyn3 model of the robot is generated
    std::string imu_link_idyntree = "";
    if(!imu_link_names.empty())
        imu_link_idyntree = imu_link_names[0]; //The first IMU is used in the constructor of idyntree
    else
        imu_link_idyntree = base_link_name; //The base_link is used as imu_link in idyntree
    iDyn3_model.constructor(robot_kdl_tree, joint_ft_sensor_names, imu_link_idyntree);
    std::cout<<"Loaded"<<robot_name<<"in iDynTree!"<<std::endl;
    
    int nJ = iDyn3_model.getNrOfDOFs();
    yarp::sig::Vector qMax; qMax.resize(nJ,0.0);
    yarp::sig::Vector qMin; qMin.resize(nJ,0.0);
    
    std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator i;
    for(i = urdf_model->joints_.begin(); i != urdf_model->joints_.end(); ++i) {
        int jIndex = iDyn3_model.getDOFIndex(i->first);
        if(jIndex != -1) {
            if (i->second->type != urdf::Joint::CONTINUOUS)
            {
                if (i->second->limits)
                {
                    qMax[jIndex] = i->second->limits->upper;
                    qMin[jIndex] = i->second->limits->lower;
                }
                else
                {
                    std::cout<<"missing joint limits for joint "<<i->first<<std::endl;
                    qMax[jIndex] = M_PI;
                    qMin[jIndex] = -M_PI;
                }
            }
        }
    }
    
    iDyn3_model.setJointBoundMax(qMax);
    iDyn3_model.setJointBoundMin(qMin);
    
    yarp::sig::Vector tauMax; tauMax.resize(nJ,1.0);
    for(i = urdf_model->joints_.begin(); i != urdf_model->joints_.end(); ++i) {
        int jIndex = iDyn3_model.getDOFIndex(i->first);
        if(jIndex != -1) {
            if (i->second->type != urdf::Joint::CONTINUOUS)
            {
                if (i->second->limits)
                    tauMax[jIndex] = i->second->limits->effort;
                else
                    tauMax[jIndex] = 0;
            }
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

yarp::sig::Vector iDynUtils::fromJointStateMsgToiDyn(const sensor_msgs::JointStateConstPtr &msg)
{
    yarp::sig::Vector q(iDyn3_model.getNrOfDOFs());

    for(unsigned int i = 0; i < msg->position.size(); ++i) {
        q[iDyn3_model.getDOFIndex(msg->name[i])]=msg->position[i];
    }

    return q;
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

void iDynUtils::updateiDyn3Model(const Eigen::VectorXd& q,
                                 const bool set_world_pose) {
    this->updateiDyn3Model(q,zerosXd,zerosXd, set_world_pose);
}

void iDynUtils::updateiDyn3Model(const yarp::sig::Vector &q,
                                 const std::vector<ft_measure> &force_torque_measurement,
                                 const bool set_world_pose)
{
    this->updateiDyn3Model(q, zeros, zeros, set_world_pose);

    for(unsigned int i = 0; i < force_torque_measurement.size(); ++i)
        updateForceTorqueMeasurement(force_torque_measurement[i]);
}

void iDynUtils::updateiDyn3Model(const yarp::sig::Vector& q,
                                 const yarp::sig::Vector& dq,
                                 const bool set_world_pose) {
    this->updateiDyn3Model(q,dq,zeros, set_world_pose);
}

void iDynUtils::updateiDyn3Model(const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& dq,
                                 const bool set_world_pose) {
    this->updateiDyn3Model(q,dq,zerosXd, set_world_pose);
}

void iDynUtils::updateiDyn3Model(const yarp::sig::Vector &q,
                                 const yarp::sig::Vector &dq,
                                 const std::vector<ft_measure> &force_torque_measurement,
                                 const bool set_world_pose)
{
    this->updateiDyn3Model(q, dq, zeros, set_world_pose);

    for(unsigned int i = 0; i < force_torque_measurement.size(); ++i)
        updateForceTorqueMeasurement(force_torque_measurement[i]);
}

void iDynUtils::updateiDyn3Model(const yarp::sig::Vector &q,
                                 const yarp::sig::Vector &dq,
                                 const yarp::sig::Vector &ddq_ref,
                                 const std::vector<ft_measure> &force_torque_measurement,
                                 const bool set_world_pose)
{
    this->updateiDyn3Model(q, dq, ddq_ref, set_world_pose);

    for(unsigned int i = 0; i < force_torque_measurement.size(); ++i)
        updateForceTorqueMeasurement(force_torque_measurement[i]);
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

    if(set_world_pose)
    {
            if(!world_is_inited) {
                this->initWorldPose();
                world_is_inited = true;
            } this->updateWorldPose();

            // here we check if the user set also IMU orientation measurements
            if(_w_R_imu.first.compare("") != 0 && _w_R_imu.second.rows() == 3 && _w_R_imu.second.cols() == 3)
                updateWorldOrientationWithIMU();
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

    if(_computeDynamics)
        iDyn3_model.dynamicRNEA();

    iDyn3_model.computePositions();
}

void iDynUtils::updateiDyn3Model(const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& dq,
                                 const Eigen::VectorXd& ddq,
                                 const bool set_world_pose)
{
    this->updateiDyn3Model(cartesian_utils::fromEigentoYarp(q),
                           cartesian_utils::fromEigentoYarp(dq),
                           cartesian_utils::fromEigentoYarp(ddq),
                           set_world_pose);
}

void iDynUtils::setJointNumbers(kinematic_chain& chain)
{
    for(std::vector<std::string>::const_iterator joint_name = chain.joint_names.begin();
        joint_name != chain.joint_names.end(); ++joint_name)
        chain.joint_numbers.push_back(iDyn3_model.getDOFIndex(*joint_name));
}

void iDynUtils::setControlledKinematicChainsJointNumbers()
{
    setJointNumbers(right_arm);
    setJointNumbers(right_leg);
    setJointNumbers(left_arm);
    setJointNumbers(left_leg);
    setJointNumbers(torso);
    setJointNumbers(head);
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
    if(list_links_in_contact.empty())
        links_in_contact.clear();
    else
    {
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

}

bool iDynUtils::checkCollisionWithWorld()
{
    return this->checkCollisionWithWorldAt(iDyn3_model.getAng());
}

bool iDynUtils::checkCollisionWithWorldAt(const yarp::sig::Vector& q)
{
    this->updateRobotState(q);
    return moveit_planning_scene->isStateColliding();
}

void iDynUtils::resetOccupancyMap()
{
    this->moveit_planning_scene->
        getWorldNonConst()->
            removeObject(
                planning_scene::PlanningScene::OCTOMAP_NS);
}

void iDynUtils::updateOccupancyMap(const octomap_msgs::Octomap& octomapMsg)
{
    this->updateRobotState(iDyn3_model.getAng());
    moveit_planning_scene->processOctomapMsg(octomapMsg);
    return;
}

void iDynUtils::updateOccupancyMap(const octomap_msgs::OctomapWithPose& octomapMsgWithPose)
{
    this->updateRobotState(iDyn3_model.getAng());
    moveit_planning_scene->processOctomapMsg(octomapMsgWithPose);
    return;
}

void iDynUtils::updateOccupancyMap(const octomap_msgs::Octomap& octomapMsg, const yarp::sig::Vector& q)
{
    this->updateRobotState(q);
    moveit_planning_scene->processOctomapMsg(octomapMsg);
    this->updateRobotState(iDyn3_model.getAng());
    return;
}

void iDynUtils::updateOccupancyMap(const octomap_msgs::OctomapWithPose& octomapMsgWithPose, const yarp::sig::Vector& q)
{
    this->updateRobotState(q);
    moveit_planning_scene->processOctomapMsg(octomapMsgWithPose);
    this->updateRobotState(iDyn3_model.getAng());
    return;
}

bool iDynUtils::checkSelfCollision()
{
    return checkSelfCollisionAt(iDyn3_model.getAng());
}

bool iDynUtils::checkSelfCollisionAt(const yarp::sig::Vector& q,
                                     std::list< std::pair<std::string,std::string> >* collisionPairs)
{
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    this->updateRobotState(q);
    moveit_planning_scene->getCurrentStateNonConst().updateCollisionBodyTransforms();
    if(collisionPairs != NULL)
    {
        req.contacts = true;
        req.max_contacts = 100;
    }
    moveit_collision_robot->checkSelfCollision(req, res,
                                               moveit_planning_scene->getCurrentStateNonConst(),
                                               moveit_planning_scene->getAllowedCollisionMatrixNonConst());

    if(collisionPairs != NULL)
    {
        for(collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin();
            it != res.contacts.end();
            ++it)
        {
            std::pair<std::string, std::string> collisionPair(
                        it->first.first.c_str(),
                        it->first.second.c_str());
            if(find(collisionPairs->begin(), collisionPairs->end(), collisionPair) == collisionPairs->end())
                collisionPairs->push_back(collisionPair);
        }
    }

    return res.collision;

}

bool iDynUtils::checkCollision()
{
    if(hasOccupancyMap())
        return checkCollisionWithWorld();
    else
        return checkSelfCollision();
}

bool iDynUtils::hasOccupancyMap()
{
    return (bool)moveit_planning_scene->
                    getWorld()->
                        getObject(
                            planning_scene::PlanningScene::OCTOMAP_NS);
}

void iDynUtils::loadDisabledCollisionsFromSRDF(collision_detection::AllowedCollisionMatrixPtr acm)
{
    loadDisabledCollisionsFromSRDF(*this->robot_srdf, acm);
}

void iDynUtils::loadDisabledCollisionsFromSRDF(srdf::Model& srdf,
                                               collision_detection::AllowedCollisionMatrixPtr acm)
{
    for( std::vector<srdf::Model::DisabledCollision>::const_iterator dc = srdf.getDisabledCollisionPairs().begin();
         dc != srdf.getDisabledCollisionPairs().end();
         ++dc)
        acm->setEntry(dc->link1_, dc->link2_, true);
}



bool iDynUtils::getSupportPolygonPoints(std::list<KDL::Vector>& points,
                                        const std::string referenceFrame)
{
    if(referenceFrame != "COM" &&
       referenceFrame != "world" &&
       iDyn3_model.getLinkIndex(referenceFrame) < 0)
        std::cerr << "ERROR: "
                  << "trying to get support polygon points in "
                  << "unknown reference frame "
                  << referenceFrame << std::endl;

    if(links_in_contact.empty() ||
       (referenceFrame != "COM" &&
        referenceFrame != "world" &&
        iDyn3_model.getLinkIndex(referenceFrame) < 0))
        return false;

    KDL::Frame world_T_CoM;
    KDL::Frame world_T_point;
    KDL::Frame referenceFrame_T_point;
    KDL::Frame CoM_T_point;
    for(std::list<std::string>::iterator it = links_in_contact.begin(); it != links_in_contact.end(); it++)
    {
        if(referenceFrame == "COM" ||
           referenceFrame == "world")
            // get points in world frame
            world_T_point = iDyn3_model.getPositionKDL(
                        iDyn3_model.getLinkIndex(*it));
        else
            referenceFrame_T_point = iDyn3_model.getPositionKDL(
                        iDyn3_model.getLinkIndex(referenceFrame),
                        iDyn3_model.getLinkIndex(*it));

        if(referenceFrame == "COM")
        {
            // get CoM in the world frame
            YarptoKDL(iDyn3_model.getCOM(), world_T_CoM.p);

            CoM_T_point = world_T_CoM.Inverse() * world_T_point;
            points.push_back(CoM_T_point.p);
        } else if(referenceFrame == "world")
            points.push_back(world_T_point.p);
        else
            points.push_back(referenceFrame_T_point.p);
    }
    return true;
}


void iDynUtils::updateiDyn3ModelFromJoinStateMsg(const sensor_msgs::JointStateConstPtr &msg)
{
    yarp::sig::Vector q = this->fromJointStateMsgToiDyn(msg);

    this->updateiDyn3Model(q, true);
}

moveit_msgs::DisplayRobotState iDynUtils::getDisplayRobotStateMsg()
{
    //std::string VJOINT_NAME = "virtual_joint";
    this->updateRobotState();
    moveit_msgs::DisplayRobotState msg;

    robot_state::robotStateToRobotStateMsg(moveit_planning_scene->getCurrentState(),
                                           msg.state);
    return msg;
}

std::string iDynUtils::getBaseLink()
{
    return this->base_link_name;
}

moveit_msgs::DisplayRobotState iDynUtils::getDisplayRobotStateMsgAt(const yarp::sig::Vector &q)
{
    std::string VJOINT_NAME = "virtual_joint";
    this->updateRobotState(q);
    moveit_msgs::DisplayRobotState msg;
    
        if(moveit_planning_scene->
            getCurrentStateNonConst().
                getRobotModel()->
                    hasJointModel(VJOINT_NAME))
    {
        robot_state::RobotState robot_state(moveit_planning_scene->getCurrentState());
        Eigen::Affine3d offset = moveit_planning_scene->
                                    getCurrentState().
                                        getFrameTransform(this->getBaseLink());

        robot_state.setVariablePosition(VJOINT_NAME + "/trans_x", offset.translation().x());
        robot_state.setVariablePosition(VJOINT_NAME + "/trans_y", offset.translation().y());
        robot_state.setVariablePosition(VJOINT_NAME + "/trans_z", offset.translation().z());

        // Apply rotation
        Eigen::Quaterniond q(offset.rotation());
        robot_state.setVariablePosition(VJOINT_NAME + "/rot_x", q.x());
        robot_state.setVariablePosition(VJOINT_NAME + "/rot_y", q.y());
        robot_state.setVariablePosition(VJOINT_NAME + "/rot_z", q.z());
        robot_state.setVariablePosition(VJOINT_NAME + "/rot_w", q.w());
        robot_state::robotStateToRobotStateMsg(robot_state, msg.state);

    } else
        robot_state::robotStateToRobotStateMsg(moveit_planning_scene->getCurrentState(),
                                               msg.state);    
    return msg;
}

moveit_msgs::PlanningScene iDynUtils::getPlanningSceneMsg()
{
    moveit_msgs::PlanningScene scene;
    moveit_msgs::PlanningSceneComponents components;
    components.components = components.OCTOMAP |
                            components.ROBOT_STATE |
                            components.TRANSFORMS |
                            components.WORLD_OBJECT_GEOMETRY |
                            components.WORLD_OBJECT_NAMES;
    this->moveit_planning_scene->getPlanningSceneMsg(scene, components);
    #ifdef RVIZ_DOES_NOT_TRANSFORM_OCTOMAP
    if(this->moveit_planning_scene->getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS)) {
        octomap_utils::octomapWithPoseToOctomap(scene.world.octomap);
    }
    #endif
    return scene;
}

void iDynUtils::updateRobotState()
{
    this->updateRobotState(iDyn3_model.getAng());
}

void iDynUtils::disableDynamicsUpdate()
{
    this->_computeDynamics = false;
}

void iDynUtils::enableDynamicsUpdate()
{
    this->_computeDynamics = true;
}

void iDynUtils::updateRobotState(const yarp::sig::Vector& q)
{
    for(unsigned int i = 0; i < joint_names.size(); ++i) {
        // TODO once we are sure joint_names are ALWAYS in joint order, this becomes faster
        moveit_planning_scene->
            getCurrentStateNonConst().setJointPositions(joint_names[i],
                                                        &q[iDyn3_model.getDOFIndex(joint_names[i])]);
    }
    
    moveit_planning_scene->getCurrentStateNonConst().updateLinkTransforms();
    Eigen::Affine3d world_T_anchor;
    tf::transformKDLToEigen(this->anchor_T_world.Inverse(), world_T_anchor);
    Eigen::Affine3d scene_T_base_link =
        moveit_planning_scene->getCurrentState()
            .getFrameTransform(this->getBaseLink());
    Eigen::Affine3d scene_T_anchor =
        moveit_planning_scene->getCurrentState()
            .getFrameTransform(anchor_name);
    // computed so that map == world
    Eigen::Affine3d scene_T_base_link_desired =
        world_T_anchor * scene_T_anchor.inverse() * scene_T_base_link;
    if(moveit_robot_model->getRootJoint()->getType() == moveit::core::JointModel::FLOATING)
    {
        Eigen::Affine3d scene_T_root_link =
            moveit_planning_scene->getCurrentState()
                .getFrameTransform(moveit_robot_model->getRootLinkName());
        Eigen::Affine3d base_link_T_root_link =
                scene_T_base_link.inverse() * scene_T_root_link ;
        moveit_planning_scene->
            getCurrentStateNonConst().
                setJointPositions(
                    moveit_robot_model->getRootJoint(),
                    scene_T_base_link_desired * base_link_T_root_link);
        moveit_planning_scene->
            getCurrentStateNonConst().update();
    }
    else
        moveit_planning_scene->getCurrentStateNonConst().
                updateStateWithLinkAt(this->getBaseLink(),
                                      scene_T_base_link_desired);
}

bool iDynUtils::updateForceTorqueMeasurement(const ft_measure& force_torque_measurement)
{
    const moveit::core::LinkModel* ft_link = moveit_robot_model->getLinkModel(
                force_torque_measurement.first);

    int ft_index = iDyn3_model.getFTSensorIndex(
                ft_link->getParentJointModel()->getName());

    if(iDyn3_model.setSensorMeasurement(ft_index, force_torque_measurement.second))
        return true;

    return false;
}

//TODO: ADD CHECK THAT JOINT EXISTS
bool iDynUtils::readForceTorqueSensorsNames()
{
    std::vector<srdf::Model::Group> robot_groups = robot_srdf->getGroups();
    for(std::vector<srdf::Model::Group>::iterator it_groups = robot_groups.begin();
        it_groups != robot_groups.end();
        ++it_groups)
    {
        if (it_groups->name_ == walkman::robot::force_torque_sensors)
        {
            if(it_groups->joints_.size() > 0) {
                for(int i = 0; i < it_groups->joints_.size(); i++)
                {
                    std::cout << "ft sensor found on joint " << it_groups->joints_[i];

                    std::string reference_frame = moveit_robot_model->getJointModel(it_groups->joints_[i])->
                            getChildLinkModel()->getName();

                    std::cout << " on frame " << reference_frame <<std::endl; std::cout.flush();

                    _ft_sensor_frames.push_back(reference_frame);
                }
                return true;
            }
        }
    }
    

    std::cout << "Robot does not have any ft sensor" << std::endl;
    return false;
}

bool iDynUtils::readIMUSensorsNames()
{
    std::vector<srdf::Model::Group> robot_groups = robot_srdf->getGroups();
    for(unsigned int i = 0; i < robot_groups.size(); ++i)
    {
        srdf::Model::Group group = robot_groups[i];
        if(group.name_ == walkman::robot::imu_sensors){
            if(group.links_.size() > 0){
                for(unsigned int j = 0; j < group.links_.size(); ++j)
                {
                    std::string link = group.links_[j];
                    std::cout << "imu sensor found on link "<<link<<std::endl;

                    _imu_sensor_frames.push_back(link);

                }
                return true;
            }
        }
    }
    std::cout << "Robot does not have any imu sensor" << std::endl;
    return false;
}

bool iDynUtils::setIMUOrientation(const yarp::sig::Matrix& world_R_imu, const std::string& reference_frame)
{
    if(std::find(_imu_sensor_frames.begin(), _imu_sensor_frames.end(), reference_frame) != _imu_sensor_frames.end())
    {
       _w_R_imu.first = reference_frame;
       _w_R_imu.second = world_R_imu;
       return true;
    }
    return false;
}

void iDynUtils::updateWorldOrientationWithIMU()
{
    KDL::Frame world_T_imu; world_T_imu.Identity();
    cartesian_utils::fromYARPMatrixtoKDLRotation(_w_R_imu.second, world_T_imu.M);

    KDL::Frame anchor_T_imu = iDyn3_model.getPositionKDL(
                iDyn3_model.getLinkIndex(anchor_name),
                iDyn3_model.getLinkIndex(std::string(_w_R_imu.first)));

    KDL::Frame anchor_T_world_ = anchor_T_imu * world_T_imu.Inverse();
    anchor_T_world.M = anchor_T_world_.M;

    this->setWorldPose(anchor_T_world, anchor_name);
}

KDL::Frame iDynUtils::getPose(const std::string& first_link, const std::string& second_link)
{
    int first_link_id = iDyn3_model.getLinkIndex(first_link);
    if(first_link_id == -1){
        yarp::sig::Vector zeros(16, 0.0);
        KDL::Frame T;
        T.Make4x4(zeros.data());
        return T;}

    int second_link_id = iDyn3_model.getLinkIndex(second_link);
    if(second_link_id == -1){
        yarp::sig::Vector zeros(16, 0.0);
        KDL::Frame T;
        T.Make4x4(zeros.data());
        return T;}

    return iDyn3_model.getPositionKDL(first_link_id, second_link_id);
}

KDL::Frame iDynUtils::getPose(const std::string& link)
{
    int link_id = iDyn3_model.getLinkIndex(link);
    if(link_id == -1){
        yarp::sig::Vector zeros(16, 0.0);
        KDL::Frame T;
        T.Make4x4(zeros.data());
        return T;}

    return iDyn3_model.getPositionKDL(link_id);
}

KDL::Vector iDynUtils::getCoM(const std::string& link)
{
    if(link.compare("world") == 0)
        return iDyn3_model.getCOMKDL();

    int link_id = iDyn3_model.getLinkIndex(link);
    if(link_id == -1){
        KDL::Vector v;
        v.Zero();
        return v;}
    return iDyn3_model.getCOMKDL(link_id);
}

bool iDynUtils::getJacobian(const int distal_link_index, Eigen::MatrixXd& J)
{
    yarp::sig::Matrix tmp;
    bool a = iDyn3_model.getJacobian(distal_link_index, tmp);
    if(a)
        J = cartesian_utils::toEigen(tmp);
    return a;
}

bool iDynUtils::getCOMJacobian(Eigen::MatrixXd& JCoM)
{
    yarp::sig::Matrix tmp;
    bool a = iDyn3_model.getCOMJacobian(tmp);
    if(a)
        JCoM = cartesian_utils::toEigen(tmp);
    return a;
}

bool iDynUtils::getRelativeJacobian(const int distal_link_index,
                         const int base_link_index,
                         Eigen::MatrixXd& J,
                         bool global)
{
    yarp::sig::Matrix tmp;
    bool a = iDyn3_model.getRelativeJacobian(distal_link_index, base_link_index,
                                             tmp, global);
    if(a)
        J = cartesian_utils::toEigen(tmp);
    return a;
}

Eigen::VectorXd iDynUtils::getCOM(const int link_index)
{
    return cartesian_utils::toEigen(iDyn3_model.getCOM(link_index));
}

Eigen::MatrixXd iDynUtils::getPosition(const int link_index, bool inverse)
{
    return cartesian_utils::toEigen(iDyn3_model.getPosition(link_index, inverse));
}

Eigen::MatrixXd iDynUtils::getPosition(const int first_link, const int second_link)
{
    return cartesian_utils::toEigen(iDyn3_model.getPosition(first_link, second_link));
}

Eigen::VectorXd iDynUtils::getJointBoundMin()
{
    return cartesian_utils::toEigen(iDyn3_model.getJointBoundMin());
}

Eigen::VectorXd iDynUtils::getJointBoundMax()
{
    return cartesian_utils::toEigen(iDyn3_model.getJointBoundMax());
}

Eigen::VectorXd iDynUtils::getJointTorqueMax()
{
    return cartesian_utils::toEigen(iDyn3_model.getJointTorqueMax());
}

Eigen::VectorXd iDynUtils::getTorques()
{
    return cartesian_utils::toEigen(iDyn3_model.getTorques());
}

Eigen::VectorXd iDynUtils::getAng()
{
    return cartesian_utils::toEigen(iDyn3_model.getAng());
}

Eigen::VectorXd iDynUtils::getDAng()
{
    return cartesian_utils::toEigen(iDyn3_model.getDAng());
}

Eigen::VectorXd iDynUtils::setAng(const Eigen::VectorXd& q)
{
    return cartesian_utils::toEigen(iDyn3_model.setAng(
                                        cartesian_utils::fromEigentoYarp(q)));
}

Eigen::VectorXd iDynUtils::getVelCOM()
{
    return cartesian_utils::toEigen(iDyn3_model.getVelCOM());
}

bool iDynUtils::getFloatingBaseMassMatrix(Eigen::MatrixXd & fb_mass_matrix)
{
    yarp::sig::Matrix M(iDyn3_model.getNrOfDOFs()+6, iDyn3_model.getNrOfDOFs()+6);
    bool a = iDyn3_model.getFloatingBaseMassMatrix(M);
    if(a)
        fb_mass_matrix = cartesian_utils::toEigen(M);
    return a;
}

Eigen::VectorXd iDynUtils::getCentroidalMomentum()
{
    return cartesian_utils::toEigen(iDyn3_model.getCentroidalMomentum());
}

bool iDynUtils::getSensorMeasurement(const int sensor_index, Eigen::VectorXd &ftm)
{
    yarp::sig::Vector tmp;
    bool a = iDyn3_model.getSensorMeasurement(sensor_index, tmp);
    if(a)
        ftm = cartesian_utils::toEigen(tmp);
    return a;
}
