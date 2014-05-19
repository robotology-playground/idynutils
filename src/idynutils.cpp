#include "drc_shared/idynutils.h"


using namespace iCub::iDynTree;
using namespace yarp::math;

// Here is the path to the URDF model
const std::string coman_model_folder = std::string(getenv("YARP_WORKSPACE")) + "/coman_yarp_apps/coman_urdf/coman.urdf";


iDynUtils::iDynUtils():right_arm("right arm"),right_leg("right leg"),left_arm("left arm"),left_leg("left leg"),torso("torso")
{
    iDyn3Model();
    setJointNames();   
    setControlledKinematicChainsLinkIndex();
    setControlledKinematicChainsJointNumbers();
    
}


void iDynUtils::setJointNames()
{
    right_arm.joint_names.push_back("RShSag");
    right_arm.joint_names.push_back("RShLat");
    right_arm.joint_names.push_back("RShYaw");
    right_arm.joint_names.push_back("RElbj");
    right_arm.joint_names.push_back("RForearmPlate");
    right_arm.joint_names.push_back("RWrj1");
    right_arm.joint_names.push_back("RWrj2");
    
    left_arm.joint_names.push_back("LShSag");
    left_arm.joint_names.push_back("LShLat");
    left_arm.joint_names.push_back("LShYaw");
    left_arm.joint_names.push_back("LElbj");
    left_arm.joint_names.push_back("LForearmPlate");
    left_arm.joint_names.push_back("LWrj1");
    left_arm.joint_names.push_back("LWrj2");
    
    right_leg.joint_names.push_back("RHipSag");
    right_leg.joint_names.push_back("RHipLat");
    right_leg.joint_names.push_back("RHipYaw");
    right_leg.joint_names.push_back("RKneeSag");
    right_leg.joint_names.push_back("RAnkLat");
    right_leg.joint_names.push_back("RAnkSag");
    
    left_leg.joint_names.push_back("LHipSag");
    left_leg.joint_names.push_back("LHipLat");
    left_leg.joint_names.push_back("LHipYaw");
    left_leg.joint_names.push_back("LKneeSag");
    left_leg.joint_names.push_back("LAnkLat");
    left_leg.joint_names.push_back("LAnkSag");
    
    torso.joint_names.push_back("WaistSag");
    torso.joint_names.push_back("WaistLat");
    torso.joint_names.push_back("WaistYaw");
}


void iDynUtils::iDyn3Model()
{
    /// iDyn3 Model creation
    // Giving name to references for FT sensors and IMU
    std::vector<std::string> joint_sensor_names;
    joint_sensor_names.push_back("l_ankle_joint");
    joint_sensor_names.push_back("r_ankle_joint");
    std::string waist_link_name = "Waist";
    
    if (!coman_model.initFile(coman_model_folder))
        std::cout<<"Failed to parse urdf robot model"<<std::endl;
    
    if (!kdl_parser::treeFromUrdfModel(coman_model, coman_tree))
        std::cout<<"Failed to construct kdl tree"<<std::endl;
    
    // Here the iDyn3 model of the robot is generated
    coman_iDyn3.constructor(coman_tree, joint_sensor_names, waist_link_name);
    std::cout<<"Loaded COMAN in iDyn3!"<<std::endl;
    
    int nJ = coman_iDyn3.getNrOfDOFs(); //29
    yarp::sig::Vector qMax; qMax.resize(nJ,0.0);
    yarp::sig::Vector qMin; qMin.resize(nJ,0.0);
    
    std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator i;
    for(i = coman_model.joints_.begin(); i != coman_model.joints_.end(); ++i) {
        int jIndex = coman_iDyn3.getDOFIndex(i->first);
        if(jIndex != -1) {
            qMax[jIndex] = i->second->limits->upper;
            qMin[jIndex] = i->second->limits->lower;
        }
    }
    
    coman_iDyn3.setJointBoundMax(qMax);
    coman_iDyn3.setJointBoundMin(qMin);
    std::cout<<"Loaded COMAN in iDyn3!"<<std::endl;
    
    std::cout<<"#DOFS: "<<coman_iDyn3.getNrOfDOFs()<<std::endl;
    std::cout<<"#Links: "<<coman_iDyn3.getNrOfLinks()<<std::endl;
}

void iDynUtils::setChainIndex(std::string endeffector_name,kinematic_chain& chain)
{
    chain.name=endeffector_name;
    chain.index = coman_iDyn3.getLinkIndex(chain.name);
    if(chain.index == -1)
        std::cout << "Failed to get link index for "<<chain.chain_name<< std::endl;
}

void iDynUtils::setControlledKinematicChainsLinkIndex()
{
    setChainIndex("r_wrist",right_arm);
    setChainIndex("l_wrist",left_arm);
    setChainIndex("r_sole",right_leg);
    setChainIndex("l_sole",left_leg);
    setChainIndex("Waist",torso);    
}

void iDynUtils::fromRobotToIDyn(const yarp::sig::Vector& q_chain_radiands,yarp::sig::Vector& q_out,kinematic_chain& chain)
{
    for(unsigned int i = 0; i < chain.joint_numbers.size(); ++i)
    {
        q_out[chain.joint_numbers[i]] = q_chain_radiands[i];
    }
}


void iDynUtils::setWorldPose(const yarp::sig::Vector& q,const yarp::sig::Vector& dq_ref,const yarp::sig::Vector& ddq_ref)
{
    updateiDyn3Model(q,dq_ref,ddq_ref);
    
    yarp::sig::Matrix worldT(4,4);
    worldT.eye();
    coman_iDyn3.setWorldBasePose(worldT);
    yarp::sig::Vector foot_pose(3);
    foot_pose = coman_iDyn3.getPosition(coman_iDyn3.getLinkIndex("r_sole")).getCol(3).subVector(0,2);
    worldT(2,3) = -foot_pose(2);
    //std::cout<<"World Base Pose: "<<std::endl; cartesian_utils::printHomogeneousTransform(worldT);std::cout<<std::endl;
    coman_iDyn3.setWorldBasePose(worldT);
    coman_iDyn3.computePositions();
}

/**
 * THIS IS AN EXAMPLE OF A MODULE USING THESE UTILS
 * 
 * q=yarp_interface.sense();
 * updateiDyn3Model(q);
 * receiveTargetPosition(waist_target_position);
 *  KDL::Frame waist_target_position;
 *  KDL::Frame from_waist_to_end_effector = fromYARPMatrixtoKDLFrame(coman_iDyn3.getPosition(chain.index,true));
 *  KDL::Vector position_error=-from_waist_to_end_effector*waist_target_position;
 *  J=getSmallJacobian()
 *  q_dot=pinv(J)*(k*position_error) , k>0, q_dot.size=chain.size
 *  yarp_interface.moveSpeed(q_dot);
 */

yarp::sig::Matrix iDynUtils::getSimpleChainJacobian(const kinematic_chain chain,bool world_frame)
{    
    yarp::sig::Matrix temp;
    if(!coman_iDyn3.getRelativeJacobian(chain.index,torso.index,temp,world_frame))
        std::cout << "Error computing Jacobian for chain "<<chain.chain_name << std::endl;
    for(unsigned int i = temp.cols();i>0; i--)
    {
        bool set_zero = true;
        for(unsigned int j = 0; j <chain.joint_names.size(); ++j){
            if(i-1 == chain.joint_numbers[j])
            {
                set_zero = false;
                break;
            }
        }
        if (set_zero)
            temp.removeCols(i-1,1);
    }
    return temp;
}

void iDynUtils::updateiDyn3Model(const yarp::sig::Vector& q,const yarp::sig::Vector& dq_ref,const yarp::sig::Vector& ddq_ref)
{
    // Here we set these values in our internal model
    coman_iDyn3.setAng(q);
    coman_iDyn3.setDAng(dq_ref);
    coman_iDyn3.setD2Ang(ddq_ref);
    // This is the fake Inertial Measure
    yarp::sig::Vector g(3);
    g[0] = 0; g[1] = 0; g[2] = 9.81;
    yarp::sig::Vector o(3);
    o[0] = 0; o[1] = 0; o[2] = 0;
    coman_iDyn3.setInertialMeasure(o, o, g);
    
    coman_iDyn3.kinematicRNEA();
    coman_iDyn3.computePositions();
}

void iDynUtils::setJointNumbers(kinematic_chain& chain)
{
    std::cout<<chain.name<<" joint indices: \n";
    for(auto joint_name: chain.joint_names){
        std::cout<<coman_iDyn3.getDOFIndex(joint_name)<<" ";
        chain.joint_numbers.push_back(coman_iDyn3.getDOFIndex(joint_name));
    }
    std::cout<<std::endl;    
}

void iDynUtils::setControlledKinematicChainsJointNumbers()
{
    setJointNumbers(right_arm);
    setJointNumbers(right_leg);
    setJointNumbers(left_arm);
    setJointNumbers(left_leg);
    setJointNumbers(torso);
}