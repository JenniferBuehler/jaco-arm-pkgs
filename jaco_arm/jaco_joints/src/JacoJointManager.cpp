#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Manages the jaco joint names as specified in the URDF file.

   Copyright (C) 2015 Jennifer Buehler

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/
#endif

#include <jaco_joints/JacoJointManager.h>
#include <map>
#include <string>
#include <vector>

#define J0_NAME  "arm_0_joint"
#define J1_NAME  "arm_1_joint"
#define J2_NAME  "arm_2_joint"
#define J3_NAME  "arm_3_joint"
#define J4_NAME  "arm_4_joint"
#define J5_NAME  "arm_5_joint"

#define JF0_NAME  "finger_joint_0"
#define JF1_NAME  "finger_joint_2"
#define JF2_NAME  "finger_joint_4"


#define J0_INIT 4.5532045
#define J1_INIT 4.5506868
#define J2_INIT 0.7038534
#define J3_INIT 5.4655337
#define J4_INIT 1.506298
#define J5_INIT 3.135861
// #define JF_INIT -0.0043633
#define JF_INIT 0


#define HAND_LINK "6_hand_limb"
#define ARM_LINKS "0_baseA", "0_base_limb", "1_shoulder_limb", "2_upperarm_limb",\
    "3_forearm_limb", "4_upperwrist_limb", "5_lowerwrist_limb", "ring_1", "ring_2", "ring_3", "ring_4", "ring_5", "ring_6"
#define FINGER_LINKS "grippers_base_link", "7_gripper_mount_index", "8_gripper_index",\
    "9_gripper_index_tip", "7_gripper_mount_thumb", "8_gripper_thumb", "9_gripper_thumb_tip",\
    "7_gripper_mount_pinkie", "8_gripper_pinkie", "9_gripper_pinkie_tip"

/**
 *
 */
JacoJointManager::JacoJointManager(const std::string& _robot_namespace):
    robot_namespace(_robot_namespace)
{
/*    arm_joints.resize(6);
    arm_joint_init.resize(6);
    gripper_joints.resize(3);
    gripper_joint_init.resize(3);

    arm_joints[0] = J0_NAME;
    arm_joints[1] = J1_NAME;
    arm_joints[2] = J2_NAME;
    arm_joints[3] = J3_NAME;
    arm_joints[4] = J4_NAME;
    arm_joints[5] = J5_NAME;

    gripper_joints[0] = JF0_NAME;
    gripper_joints[1] = JF1_NAME;
    gripper_joints[2] = JF2_NAME;

    arm_joint_init[0] = J0_INIT;
    arm_joint_init[1] = J1_INIT;
    arm_joint_init[2] = J2_INIT;
    arm_joint_init[3] = J3_INIT;
    arm_joint_init[4] = J4_INIT;
    arm_joint_init[5] = J5_INIT;

    gripper_joint_init[0] = JF_INIT;
    gripper_joint_init[1] = JF_INIT;
    gripper_joint_init[2] = JF_INIT;
*/
    // get joint names from parameter server
    readJointNamesFromParameters();
}

JacoJointManager::JacoJointManager(const JacoJointManager& o):
    palm_link(o.palm_link),
    arm_joints(o.arm_joints),
    arm_links(o.arm_links),
    gripper_joints(o.gripper_joints),
    gripper_links(o.gripper_links),
    arm_joint_init(o.arm_joint_init),
    gripper_joint_init(o.gripper_joint_init)
{
}

void JacoJointManager::readJointNamesFromParameters()
{
    ros::NodeHandle robot_nh(robot_namespace);
    ROS_INFO_STREAM("JacoJointManager reading parameters from namespace: "<<robot_nh.getNamespace());

    // ROS_INFO_STREAM("Reading palm_link:");
    robot_nh.getParam("palm_link", palm_link);
    if (palm_link.empty())
    {
      ROS_ERROR("Parameter palm_link should be specified");
    }
    
    // --- arm parameters
    
    // ROS_INFO_STREAM("Reading arm_joints:");
    robot_nh.getParam("arm_joints", arm_joints);
    if (arm_joints.empty())
    {
      ROS_ERROR("Parameter arm_joints should be specified as an array");
    }
    // for (int i=0; i < arm_joints.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_joints[i]);}

    // ROS_INFO_STREAM("Reading arm_joint_init:");
    robot_nh.getParam("arm_joint_init", arm_joint_init);
    if (arm_joint_init.empty())
    {
      ROS_ERROR("Parameter arm_joint_init should be specified as an array");
    }
    // for (int i=0; i < arm_joint_init.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_joint_init[i]);}
 
    
    // ROS_INFO_STREAM("Reading arm_links:");
    robot_nh.getParam("arm_links", arm_links);
    if (arm_links.empty())
    {
      ROS_ERROR("Parameter arm_links should be specified as an array");
    }
    // for (int i=0; i < arm_links.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_links[i]);}

    // controllers

    // ROS_INFO_STREAM("Reading arm_position_controller_names:");
    robot_nh.getParam("arm_position_controller_names", arm_position_controller_names);
    if (arm_position_controller_names.empty())
    {
      ROS_INFO("INFO: Parameter arm_position_controller_names has not been specified");
    }
    // for (int i=0; i < arm_position_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_position_controller_names[i]);}

    // ROS_INFO_STREAM("Reading arm_velocity_controller_names:");
    robot_nh.getParam("arm_velocity_controller_names", arm_velocity_controller_names);
    if (arm_velocity_controller_names.empty())
    {
      ROS_INFO("INFO: Parameter arm_velocity_controller_names has not been specified");
    }
    // for (int i=0; i < arm_velocity_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_velocity_controller_names[i]);}

    // ROS_INFO_STREAM("Reading arm_effort_controller_names:");
    robot_nh.getParam("arm_effort_controller_names", arm_effort_controller_names);
    if (arm_effort_controller_names.empty())
    {
      ROS_INFO("INFO: Parameter arm_effort_controller_names has not been specified");
    }
    // for (int i=0; i < arm_effort_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << arm_effort_controller_names[i]);}



    // --- gripper parameters

    // ROS_INFO_STREAM("Reading gripper_joints:");
    robot_nh.getParam("gripper_joints", gripper_joints);
    if (gripper_joints.empty())
    {
      ROS_ERROR("Parameter gripper_joints should be specified as an array");
    }
    // for (int i=0; i < gripper_joints.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_joints[i]);}

    // ROS_INFO_STREAM("Reading gripper_joint_init:");
    robot_nh.getParam("gripper_joint_init", gripper_joint_init);
    if (gripper_joint_init.empty())
    {
      ROS_ERROR("Parameter gripper_joint_init should be specified as an array");
    }
    // for (int i=0; i < gripper_joint_init.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_joint_init[i]);}
 

    // ROS_INFO_STREAM("Reading gripper_links:");
    robot_nh.getParam("gripper_links", gripper_links);
    if (gripper_links.empty())
    {
      ROS_ERROR("Parameter gripper_links should be specified as an array");
    }
    // for (int i=0; i < gripper_links.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_links[i]);}

    // controllers

    // ROS_INFO_STREAM("Reading gripper_position_controller_names:");
    robot_nh.getParam("gripper_position_controller_names", gripper_position_controller_names);
    if (gripper_position_controller_names.empty())
    {
      ROS_INFO("INFO: Parameter gripper_position_controller_names has not been specified");
    }
    // for (int i=0; i < gripper_position_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_position_controller_names[i]);}

    // ROS_INFO_STREAM("Reading gripper_velocity_controller_names:");
    robot_nh.getParam("gripper_velocity_controller_names", gripper_velocity_controller_names);
    if (gripper_velocity_controller_names.empty())
    {
      ROS_INFO("INFO: Parameter gripper_velocity_controller_names has not been specified");
    }
    // for (int i=0; i < gripper_velocity_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_velocity_controller_names[i]);}

    // ROS_INFO_STREAM("Reading gripper_effort_controller_names:");
    robot_nh.getParam("gripper_effort_controller_names", gripper_effort_controller_names);
    if (gripper_effort_controller_names.empty())
    {
      ROS_INFO("INFO: Parameter gripper_effort_controller_names has not been specified");
    }
    // for (int i=0; i < gripper_effort_controller_names.size(); ++i) { ROS_INFO_STREAM("idx " << i << ": " << gripper_effort_controller_names[i]);}


/*    robot_nh.param<std::string>("arm_joint_0_name", arm_joints[0], arm_joints[0]);
    // ROS_INFO_STREAM("Joint 0: " << arm_joints[0]);

    robot_nh.param<std::string>("arm_joint_1_name", arm_joints[1], arm_joints[1]);
    // ROS_INFO_STREAM("Joint 1: " << arm_joints[1]);

    robot_nh.param<std::string>("arm_joint_2_name", arm_joints[2], arm_joints[2]);
    // ROS_INFO_STREAM("Joint 2: " << arm_joints[2]);

    robot_nh.param<std::string>("arm_joint_3_name", arm_joints[3], arm_joints[3]);
    // ROS_INFO_STREAM("Joint 3: " << arm_joints[3]);

    robot_nh.param<std::string>("arm_joint_4_name", arm_joints[4], arm_joints[4]);
    // ROS_INFO_STREAM("Joint 4: " << arm_joints[4]);

    robot_nh.param<std::string>("arm_joint_5_name", arm_joints[5], arm_joints[5]);
    // ROS_INFO_STREAM("Joint 5: " << arm_joints[5]);

    robot_nh.param<std::string>("gripper_joint_0_name", gripper_joints[0], gripper_joints[0]);
    // ROS_INFO_STREAM("Gripper joint 0: " << gripper_joints[0]);

    robot_nh.param<std::string>("gripper_joint_1_name", gripper_joints[1], gripper_joints[1]);
    // ROS_INFO_STREAM("Gripper joint 1: " << gripper_joints[1]);

    robot_nh.param<std::string>("gripper_joint_2_name", gripper_joints[2], gripper_joints[2]);
    // ROS_INFO_STREAM("Gripper joint 2: " << gripper_joints[2]);

    robot_nh.param<float>("arm_joint_0_init", arm_joint_init[0], arm_joint_init[0]);
    // ROS_INFO_STREAM("Init joint 0: " << arm_joint_init[0]);

    robot_nh.param<float>("arm_joint_1_init", arm_joint_init[1], arm_joint_init[1]);
    // ROS_INFO_STREAM("Init joint 1: " << arm_joint_init[1]);

    robot_nh.param<float>("arm_joint_2_init", arm_joint_init[2], arm_joint_init[2]);
    // ROS_INFO_STREAM("Init joint 2: " << arm_joint_init[2]);

    robot_nh.param<float>("arm_joint_3_init", arm_joint_init[3], arm_joint_init[3]);
    // ROS_INFO_STREAM("Init joint 3: " << arm_joint_init[3]);

    robot_nh.param<float>("arm_joint_4_init", arm_joint_init[4], arm_joint_init[4]);
    // ROS_INFO_STREAM("Init joint 4: " << arm_joint_init[4]);

    robot_nh.param<float>("arm_joint_5_init", arm_joint_init[5], arm_joint_init[5]);
    // ROS_INFO_STREAM("Init joint 5: " << arm_joint_init[5]);

    robot_nh.param<float>("gripper_joint_0_init", gripper_joint_init[0], gripper_joint_init[0]);
    // ROS_INFO_STREAM("Init gripper joint 0: " << gripper_joint_init[0]);

    robot_nh.param<float>("gripper_joint_1_init", gripper_joint_init[1], gripper_joint_init[1]);
    // ROS_INFO_STREAM("Init gripper joint 1: " << gripper_joint_init[1]);

    robot_nh.param<float>("gripper_joint_2_init", gripper_joint_init[2], gripper_joint_init[2]);
    // ROS_INFO_STREAM("Init gripper joint 2: " << gripper_joint_init[2]);
*/
}


void JacoJointManager::ReadPIDValues(const std::string& pidParameterName, float& kp, float& ki, float& kd) const
{
    static const std::string pid_param_name = "pid";

    std::map<std::string, float> pid;
    ros::NodeHandle pub("");
    if (!pub.getParam(robot_namespace + "/" + pidParameterName + "/" + pid_param_name, pid))
    {
        ROS_WARN_STREAM(pidParameterName << " was not on parameter server. Keeping default values.");
    }
    else
    {
        kp = pid["p"];
        ki = pid["i"];
        kd = pid["d"];
    }
}

bool JacoJointManager::GetPosGains(const std::string& jointName, float& kp, float& ki, float& kd) const
{
    std::vector<std::string>::const_iterator jnt = std::find(arm_joints.begin(), arm_joints.end(), jointName);
    if (jnt==arm_joints.end())
    {
        jnt = std::find(gripper_joints.begin(), gripper_joints.end(), jointName);
        if (jnt==gripper_joints.end())
        {
            ROS_ERROR_STREAM("JacoJointManager does not maintain joint name '"<<jointName<<"'");
            return false;
        }
        int idx = jnt - gripper_joints.begin();
        if (gripper_position_controller_names.size() <= idx)
        {
            ROS_ERROR_STREAM("JacoJointManager does have the name for position controller '"<<jointName<<"'");
            return false;
        }
        ReadPIDValues(gripper_position_controller_names[idx], kp, ki, kd);
    }
    else
    {
        int idx = jnt - arm_joints.begin();
        if (arm_position_controller_names.size() <= idx)
        {
            ROS_ERROR_STREAM("JacoJointManager does have the name for position controller '"<<jointName<<"'");
            return false;
        }
        ReadPIDValues(arm_position_controller_names[idx], kp, ki, kd);
    }
    return true;

    /*if (jointName == arm_joints[0]) ReadPIDValues("arm_0_joint_position_controller", kp, ki, kd);
    else if (jointName == arm_joints[1]) ReadPIDValues("arm_1_joint_position_controller", kp, ki, kd);
    else if (jointName == arm_joints[2]) ReadPIDValues("arm_2_joint_position_controller", kp, ki, kd);
    else if (jointName == arm_joints[3]) ReadPIDValues("arm_3_joint_position_controller", kp, ki, kd);
    else if (jointName == arm_joints[4]) ReadPIDValues("arm_4_joint_position_controller", kp, ki, kd);
    else if (jointName == arm_joints[5]) ReadPIDValues("arm_5_joint_position_controller", kp, ki, kd);
    else if (jointName == gripper_joints[0]) ReadPIDValues("gripper_joint_0_position_controller", kp, ki, kd);
    else if (jointName == gripper_joints[1]) ReadPIDValues("gripper_joint_2_position_controller", kp, ki, kd);
    else if (jointName == gripper_joints[2]) ReadPIDValues("gripper_joint_4_position_controller", kp, ki, kd);
    else return false;
    return true;*/
}

bool JacoJointManager::GetVelGains(const std::string& jointName, float& kp, float& ki, float& kd) const
{
    std::vector<std::string>::const_iterator jnt = std::find(arm_joints.begin(), arm_joints.end(), jointName);
    if (jnt==arm_joints.end())
    {
        jnt = std::find(gripper_joints.begin(), gripper_joints.end(), jointName);
        if (jnt==gripper_joints.end())
        {
            ROS_ERROR_STREAM("JacoJointManager does not maintain joint name '"<<jointName<<"'");
            return false;
        }
        int idx = jnt - gripper_joints.begin();
        if (gripper_velocity_controller_names.size() <= idx)
        {
            ROS_ERROR_STREAM("JacoJointManager does have the name for velocity controller '"<<jointName<<"'");
            return false;
        }
        ReadPIDValues(gripper_velocity_controller_names[idx], kp, ki, kd);
    }
    else
    {
        int idx = jnt - arm_joints.begin();
        if (arm_velocity_controller_names.size() <= idx)
        {
            ROS_ERROR_STREAM("JacoJointManager does have the name for velocity controller '"<<jointName<<"'");
            return false;
        }
        ReadPIDValues(arm_velocity_controller_names[idx], kp, ki, kd);
    }
    return true;

/*    if (jointName == arm_joints[0]) ReadPIDValues("arm_0_joint_velocity_controller", kp, ki, kd);
    else if (jointName == arm_joints[1]) ReadPIDValues("arm_1_joint_velocity_controller", kp, ki, kd);
    else if (jointName == arm_joints[2]) ReadPIDValues("arm_2_joint_velocity_controller", kp, ki, kd);
    else if (jointName == arm_joints[3]) ReadPIDValues("arm_3_joint_velocity_controller", kp, ki, kd);
    else if (jointName == arm_joints[4]) ReadPIDValues("arm_4_joint_velocity_controller", kp, ki, kd);
    else if (jointName == arm_joints[5]) ReadPIDValues("arm_5_joint_velocity_controller", kp, ki, kd);
    else if (jointName == gripper_joints[0]) ReadPIDValues("gripper_joint_0_velocity_controller", kp, ki, kd);
    else if (jointName == gripper_joints[1]) ReadPIDValues("gripper_joint_2_velocity_controller", kp, ki, kd);
    else if (jointName == gripper_joints[2]) ReadPIDValues("gripper_joint_4_velocity_controller", kp, ki, kd);
    else return false;
    return true; */
}


void JacoJointManager::getJointNames(std::vector<std::string>& joint_names, bool withGripper, const std::string& prepend) const
{
    joint_names.insert(joint_names.begin(), arm_joints.begin(), arm_joints.end());
    if (!withGripper) return;
    joint_names.insert(joint_names.end(), gripper_joints.begin(), gripper_joints.end());
    if (!prepend.empty())
    {
        for (std::vector<std::string>::iterator it = joint_names.begin();
                it != joint_names.end(); ++it)
        {
            *it = prepend + *it;
        }
    }
}

const std::vector<std::string>& JacoJointManager::getGripperLinks() const
{
    return gripper_links;
}

const std::string& JacoJointManager::getPalmLink() const
{
    return palm_link;
}

const std::vector<std::string>& JacoJointManager::getArmLinks() const
{
    return arm_links;
}


std::vector<std::string> JacoJointManager::getDefaultGripperLinks() const
{
    static std::string arr[] = {FINGER_LINKS};
    int arrSize=sizeof(arr) / sizeof(std::string);

    std::vector<std::string> vec;
    for (int i=0; i<arrSize; ++i) vec.push_back(arr[i]);
    return vec;
}

std::string JacoJointManager::getDefaultPalmLink() const
{
    return HAND_LINK;
}

std::vector<std::string> JacoJointManager::getDefaultArmLinks() const
{
    static std::string arr[] = {ARM_LINKS, HAND_LINK};
    int arrSize=sizeof(arr) / sizeof(std::string);

    std::vector<std::string> vec;
    for (int i=0; i<arrSize; ++i) vec.push_back(arr[i]);
    return vec;
}



const std::vector<std::string>& JacoJointManager::getArmJoints() const
{
    return arm_joints;
}
const std::vector<std::string>& JacoJointManager::getGripperJoints() const
{
    return gripper_joints;
}

const std::vector<float>& JacoJointManager::getArmJointsInitPose() const
{
    return arm_joint_init;
}
const std::vector<float>& JacoJointManager::getGripperJointsInitPose() const
{
    return gripper_joint_init;
}

void JacoJointManager::initJointState(sensor_msgs::JointState& js, bool withGripper, const std::vector<float> * init_poses) const
{
    getJointNames(js.name, withGripper);
    int num = 9;
    if (!withGripper) num = 6;
    js.position.resize(num, 0);
    js.velocity.resize(num, 0);
    js.effort.resize(num, 0);
    if (init_poses)
    {
        for (int i = 0; i < num; ++i)
        {
            js.position[i] = (*init_poses)[i];
        }
    }
}


int JacoJointManager::getJointIndices(const std::vector<std::string>& joint_names, std::vector<int>& idx)
{
    typedef std::vector<std::string>::const_iterator It;

    It jnt1 = std::find(joint_names.begin(), joint_names.end(), arm_joints[0]);
    It jnt2 = std::find(joint_names.begin(), joint_names.end(), arm_joints[1]);
    It jnt3 = std::find(joint_names.begin(), joint_names.end(), arm_joints[2]);
    It jnt4 = std::find(joint_names.begin(), joint_names.end(), arm_joints[3]);
    It jnt5 = std::find(joint_names.begin(), joint_names.end(), arm_joints[4]);
    It jnt6 = std::find(joint_names.begin(), joint_names.end(), arm_joints[5]);
    It fjnt1 = std::find(joint_names.begin(), joint_names.end(), gripper_joints[0]);
    It fjnt2 = std::find(joint_names.begin(), joint_names.end(), gripper_joints[1]);
    It fjnt3 = std::find(joint_names.begin(), joint_names.end(), gripper_joints[2]);

    bool armIncomplete = (jnt1 == joint_names.end()) ||
                         (jnt2 == joint_names.end()) ||
                         (jnt3 == joint_names.end()) ||
                         (jnt4 == joint_names.end()) ||
                         (jnt5 == joint_names.end()) ||
                         (jnt6 == joint_names.end());
    bool grippersIncomplete = (fjnt1 == joint_names.end()) || (fjnt2 == joint_names.end()) || (fjnt3 == joint_names.end());

    if (armIncomplete && grippersIncomplete)
    {
        // ROS_INFO("JacoJointNames::getJointIndices: Not all joint names present in trajectory. List:");
        // for (std::vector<std::string>::const_iterator it=joint_names.begin(); it!=joint_names.end(); ++it) ROS_INFO("%s",it->c_str());
        return -1;
    }
    if (!armIncomplete)
    {
        idx.push_back(jnt1 - joint_names.begin());
        idx.push_back(jnt2 - joint_names.begin());
        idx.push_back(jnt3 - joint_names.begin());
        idx.push_back(jnt4 - joint_names.begin());
        idx.push_back(jnt5 - joint_names.begin());
        idx.push_back(jnt6 - joint_names.begin());
    }
    else
    {
        idx.insert(idx.end(), 6, -1);
    }
    if (!grippersIncomplete)
    {
        idx.push_back(fjnt1 - joint_names.begin());
        idx.push_back(fjnt2 - joint_names.begin());
        idx.push_back(fjnt3 - joint_names.begin());
    }
    else
    {
        idx.insert(idx.end(), 3, -1);
    }

    return grippersIncomplete ? 1 : armIncomplete ? 2 : 0;
}

bool JacoJointManager::isGripper(const std::string& name) const
{
    std::vector<std::string>::const_iterator it;
    for (it = gripper_joints.begin(); it != gripper_joints.end(); ++it)
    {
        if (*it == name) return true;
    }
    return false;
}

int JacoJointManager::armJointNumber(const std::string& name) const
{
    for (int i = 0; i < arm_joints.size(); ++i)
    {
        const std::string& name_i = arm_joints[i];
        if (name_i == name) return i;
    }
    return -1;
}

int JacoJointManager::gripperJointNumber(const std::string& name) const
{
    for (int i = 0; i < gripper_joints.size(); ++i)
    {
        const std::string& name_i = gripper_joints[i];
        if (name_i == name) return i;
    }
    return -1;
}


double JacoJointManager::capToPI(const double value)
{
    static const double pi_2 = 2.0 * M_PI;
    double v = value;
    if (v <= -M_PI || v > M_PI)
    {
        v = fmod(v, pi_2);
        if (v <= -M_PI)
            v += pi_2;
        else if (v > M_PI)
            v -= pi_2;
    }
    return v;
}

double JacoJointManager::limitsToTwoPI(const double value, const double lowLimit, const double highLimit) 
{
    double ret = value;
    if (value > highLimit) ret = value - 2*M_PI;
    if (value < lowLimit) ret = value + 2*M_PI; 
    return ret;
}



double JacoJointManager::angleDistance(const double _f1, const double _f2) 
{
    double f1 = capToPI(_f1);
    double f2 = capToPI(_f2);
    double diff = f2 - f1;
    diff = capToPI(diff);
    /*ROS_INFO("Cap1 %f to %f",_f1,f1);
    ROS_INFO("Cap2 %f to %f",_f2,f2);
    ROS_INFO("DIFF %f",diff);*/
    /*      if (diff > M_PI) diff=M_PI-diff;
        else if (diff < -M_PI) diff=-M_PI+diff;
        ROS_INFO("DIFF2 %f",diff);*/
    return diff;
}


