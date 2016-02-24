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

/**
 *
 */
JacoJointManager::JacoJointManager():
    priv("~"),
    pub("")
{
    arm_joints.resize(6);
    arm_joint_init.resize(6);
    finger_joints.resize(3);
    finger_joint_init.resize(3);

    arm_joints[0] = J0_NAME;
    arm_joints[1] = J1_NAME;
    arm_joints[2] = J2_NAME;
    arm_joints[3] = J3_NAME;
    arm_joints[4] = J4_NAME;
    arm_joints[5] = J5_NAME;

    finger_joints[0] = JF0_NAME;
    finger_joints[1] = JF1_NAME;
    finger_joints[2] = JF2_NAME;

    arm_joint_init[0] = J0_INIT;
    arm_joint_init[1] = J1_INIT;
    arm_joint_init[2] = J2_INIT;
    arm_joint_init[3] = J3_INIT;
    arm_joint_init[4] = J4_INIT;
    arm_joint_init[5] = J5_INIT;

    finger_joint_init[0] = JF_INIT;
    finger_joint_init[1] = JF_INIT;
    finger_joint_init[2] = JF_INIT;

    // get joint names from parameter server
    readJointNamesFromParameters();
}

JacoJointManager::JacoJointManager(const std::vector<std::string>& _arm_joints,
                                   const std::vector<std::string>& _finger_joints,
                                   const std::vector<double>& _arm_joint_init,
                                   const std::vector<double>& _finger_joint_init):
    arm_joints(_arm_joints),
    finger_joints(_finger_joints),
    arm_joint_init(_arm_joint_init),
    finger_joint_init(_finger_joint_init),
    priv("~"),
    pub("")
{
}


JacoJointManager::JacoJointManager(const JacoJointManager& o):
    arm_joints(o.arm_joints),
    finger_joints(o.finger_joints),
    arm_joint_init(o.arm_joint_init),
    finger_joint_init(o.finger_joint_init),
    priv(o.priv),
    pub(o.pub)
{
}

void JacoJointManager::readJointNamesFromParameters()
{
    // get joint names from parameter server

    // ROS_INFO_STREAM("JacoJointManager reading parameters from namespace: "<<priv.getNamespace());

    priv.param<std::string>("arm_joint_0_name", arm_joints[0], arm_joints[0]);
    // ROS_INFO_STREAM("Joint 0: " << arm_joints[0]);

    priv.param<std::string>("arm_joint_1_name", arm_joints[1], arm_joints[1]);
    // ROS_INFO_STREAM("Joint 1: " << arm_joints[1]);

    priv.param<std::string>("arm_joint_2_name", arm_joints[2], arm_joints[2]);
    // ROS_INFO_STREAM("Joint 2: " << arm_joints[2]);

    priv.param<std::string>("arm_joint_3_name", arm_joints[3], arm_joints[3]);
    // ROS_INFO_STREAM("Joint 3: " << arm_joints[3]);

    priv.param<std::string>("arm_joint_4_name", arm_joints[4], arm_joints[4]);
    // ROS_INFO_STREAM("Joint 4: " << arm_joints[4]);

    priv.param<std::string>("arm_joint_5_name", arm_joints[5], arm_joints[5]);
    // ROS_INFO_STREAM("Joint 5: " << arm_joints[5]);

    priv.param<std::string>("finger_joint_0_name", finger_joints[0], finger_joints[0]);
    // ROS_INFO_STREAM("Finger joint 0: " << finger_joints[0]);

    priv.param<std::string>("finger_joint_1_name", finger_joints[1], finger_joints[1]);
    // ROS_INFO_STREAM("Finger joint 1: " << finger_joints[1]);

    priv.param<std::string>("finger_joint_2_name", finger_joints[2], finger_joints[2]);
    // ROS_INFO_STREAM("Finger joint 2: " << finger_joints[2]);

    priv.param<double>("arm_joint_0_init", arm_joint_init[0], arm_joint_init[0]);
    // ROS_INFO_STREAM("Init joint 0: " << arm_joint_init[0]);

    priv.param<double>("arm_joint_1_init", arm_joint_init[1], arm_joint_init[1]);
    // ROS_INFO_STREAM("Init joint 1: " << arm_joint_init[1]);

    priv.param<double>("arm_joint_2_init", arm_joint_init[2], arm_joint_init[2]);
    // ROS_INFO_STREAM("Init joint 2: " << arm_joint_init[2]);

    priv.param<double>("arm_joint_3_init", arm_joint_init[3], arm_joint_init[3]);
    // ROS_INFO_STREAM("Init joint 3: " << arm_joint_init[3]);

    priv.param<double>("arm_joint_4_init", arm_joint_init[4], arm_joint_init[4]);
    // ROS_INFO_STREAM("Init joint 4: " << arm_joint_init[4]);

    priv.param<double>("arm_joint_5_init", arm_joint_init[5], arm_joint_init[5]);
    // ROS_INFO_STREAM("Init joint 5: " << arm_joint_init[5]);

    priv.param<double>("finger_joint_0_init", finger_joint_init[0], finger_joint_init[0]);
    // ROS_INFO_STREAM("Init finger joint 0: " << finger_joint_init[0]);

    priv.param<double>("finger_joint_1_init", finger_joint_init[1], finger_joint_init[1]);
    // ROS_INFO_STREAM("Init finger joint 1: " << finger_joint_init[1]);

    priv.param<double>("finger_joint_2_init", finger_joint_init[2], finger_joint_init[2]);
    // ROS_INFO_STREAM("Init finger joint 2: " << finger_joint_init[2]);
}



void JacoJointManager::ReadPIDValues(const std::string& pidParameterName, float& kp, float& ki, float& kd) const
{
    static const std::string ns_prefix = "/jaco";
    static const std::string pid_param_name = "pid";

    std::map<std::string, float> pid;
    if (!pub.getParam(ns_prefix + "/" + pidParameterName + "/" + pid_param_name, pid))
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
    if (jointName == arm_joints[0]) ReadPIDValues("arm_0_joint_position_controller", kp, ki, kd);
    else if (jointName == arm_joints[1]) ReadPIDValues("arm_1_joint_position_controller", kp, ki, kd);
    else if (jointName == arm_joints[2]) ReadPIDValues("arm_2_joint_position_controller", kp, ki, kd);
    else if (jointName == arm_joints[3]) ReadPIDValues("arm_3_joint_position_controller", kp, ki, kd);
    else if (jointName == arm_joints[4]) ReadPIDValues("arm_4_joint_position_controller", kp, ki, kd);
    else if (jointName == arm_joints[5]) ReadPIDValues("arm_5_joint_position_controller", kp, ki, kd);
    else if (jointName == finger_joints[0]) ReadPIDValues("finger_joint_0_position_controller", kp, ki, kd);
    else if (jointName == finger_joints[1]) ReadPIDValues("finger_joint_2_position_controller", kp, ki, kd);
    else if (jointName == finger_joints[2]) ReadPIDValues("finger_joint_4_position_controller", kp, ki, kd);
    else return false;
    return true;
}

bool JacoJointManager::GetVelGains(const std::string& jointName, float& kp, float& ki, float& kd) const
{
    if (jointName == arm_joints[0]) ReadPIDValues("arm_0_joint_velocity_controller", kp, ki, kd);
    else if (jointName == arm_joints[1]) ReadPIDValues("arm_1_joint_velocity_controller", kp, ki, kd);
    else if (jointName == arm_joints[2]) ReadPIDValues("arm_2_joint_velocity_controller", kp, ki, kd);
    else if (jointName == arm_joints[3]) ReadPIDValues("arm_3_joint_velocity_controller", kp, ki, kd);
    else if (jointName == arm_joints[4]) ReadPIDValues("arm_4_joint_velocity_controller", kp, ki, kd);
    else if (jointName == arm_joints[5]) ReadPIDValues("arm_5_joint_velocity_controller", kp, ki, kd);
    else if (jointName == finger_joints[0]) ReadPIDValues("finger_joint_0_velocity_controller", kp, ki, kd);
    else if (jointName == finger_joints[1]) ReadPIDValues("finger_joint_2_velocity_controller", kp, ki, kd);
    else if (jointName == finger_joints[2]) ReadPIDValues("finger_joint_4_velocity_controller", kp, ki, kd);
    else return false;
    return true;
}




void JacoJointManager::getJointNames(std::vector<std::string>& joint_names, bool withFingers, const std::string& prepend)
{
    joint_names.insert(joint_names.begin(), arm_joints.begin(), arm_joints.end());
    if (!withFingers) return;
    joint_names.insert(joint_names.end(), finger_joints.begin(), finger_joints.end());
    if (!prepend.empty())
    {
        for (std::vector<std::string>::iterator it = joint_names.begin();
                it != joint_names.end(); ++it)
        {
            *it = prepend + *it;
        }
    }
}

const std::vector<std::string>& JacoJointManager::getArmJoints() const
{
    return arm_joints;
}
const std::vector<std::string>& JacoJointManager::getFingerJoints() const
{
    return finger_joints;
}

const std::vector<double>& JacoJointManager::getArmJointsInitPose() const
{
    return arm_joint_init;
}
const std::vector<double>& JacoJointManager::getFingerJointsInitPose() const
{
    return finger_joint_init;
}

void JacoJointManager::initJointState(sensor_msgs::JointState& js, bool withFingers, const std::vector<float> * init_poses)
{
    getJointNames(js.name, withFingers);
    int num = 9;
    if (!withFingers) num = 6;
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
    It fjnt1 = std::find(joint_names.begin(), joint_names.end(), finger_joints[0]);
    It fjnt2 = std::find(joint_names.begin(), joint_names.end(), finger_joints[1]);
    It fjnt3 = std::find(joint_names.begin(), joint_names.end(), finger_joints[2]);

    bool armIncomplete = (jnt1 == joint_names.end()) ||
                         (jnt2 == joint_names.end()) ||
                         (jnt3 == joint_names.end()) ||
                         (jnt4 == joint_names.end()) ||
                         (jnt5 == joint_names.end()) ||
                         (jnt6 == joint_names.end());
    bool fingersIncomplete = (fjnt1 == joint_names.end()) || (fjnt2 == joint_names.end()) || (fjnt3 == joint_names.end());

    if (armIncomplete && fingersIncomplete)
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
    if (!fingersIncomplete)
    {
        idx.push_back(fjnt1 - joint_names.begin());
        idx.push_back(fjnt2 - joint_names.begin());
        idx.push_back(fjnt3 - joint_names.begin());
    }
    else
    {
        idx.insert(idx.end(), 3, -1);
    }

    return fingersIncomplete ? 1 : armIncomplete ? 2 : 0;
}

bool JacoJointManager::isFinger(const std::string& name) const
{
    std::vector<std::string>::const_iterator it;
    for (it = finger_joints.begin(); it != finger_joints.end(); ++it)
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

int JacoJointManager::fingerJointNumber(const std::string& name) const
{
    for (int i = 0; i < finger_joints.size(); ++i)
    {
        const std::string& name_i = finger_joints[i];
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


