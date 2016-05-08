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


JacoJointManager::JacoJointManager(const std::string& _robot_namespace, bool _readParams):
    arm_components_name_manager::ArmComponentsNameManager(_robot_namespace, _readParams)
{
}

JacoJointManager::JacoJointManager(const JacoJointManager& o):
    arm_components_name_manager::ArmComponentsNameManager(o)
{
}

std::vector<std::string> JacoJointManager::getDefaultGripperLinks() const
{
    static std::string arr[] = {FINGER_LINKS};
    int arrSize = sizeof(arr) / sizeof(std::string);

    std::vector<std::string> vec;
    for (int i = 0; i < arrSize; ++i) vec.push_back(arr[i]);
    return vec;
}

std::string JacoJointManager::getDefaultPalmLink() const
{
    return HAND_LINK;
}

std::vector<std::string> JacoJointManager::getDefaultArmLinks() const
{
    static std::string arr[] = {ARM_LINKS, HAND_LINK};
    int arrSize = sizeof(arr) / sizeof(std::string);

    std::vector<std::string> vec;
    for (int i = 0; i < arrSize; ++i) vec.push_back(arr[i]);
    return vec;
}

std::vector<std::string> JacoJointManager::getDefaultArmJoints() const
{
    std::vector<std::string> _arm_joints;
    _arm_joints.resize(6);
    _arm_joints[0] = J0_NAME;
    _arm_joints[1] = J1_NAME;
    _arm_joints[2] = J2_NAME;
    _arm_joints[3] = J3_NAME;
    _arm_joints[4] = J4_NAME;
    _arm_joints[5] = J5_NAME;
    return _arm_joints;
}

std::vector<std::string> JacoJointManager::getDefaultGripperJoints() const
{
    std::vector<std::string> _gripper_joints;
    _gripper_joints.resize(3);
    _gripper_joints[0] = JF0_NAME;
    _gripper_joints[1] = JF1_NAME;
    _gripper_joints[2] = JF2_NAME;
    return _gripper_joints;
}

std::vector<float> JacoJointManager::getDefaultArmJointsInitPose() const
{
    std::vector<float> _arm_joint_init;
    _arm_joint_init.resize(6);
    _arm_joint_init[0] = J0_INIT;
    _arm_joint_init[1] = J1_INIT;
    _arm_joint_init[2] = J2_INIT;
    _arm_joint_init[3] = J3_INIT;
    _arm_joint_init[4] = J4_INIT;
    _arm_joint_init[5] = J5_INIT;
    return _arm_joint_init;
}

std::vector<float> JacoJointManager::getDefaultGripperJointsInitPose() const
{
    std::vector<float> _gripper_joint_init;
    _gripper_joint_init.resize(3);
    _gripper_joint_init[0] = JF_INIT;
    _gripper_joint_init[1] = JF_INIT;
    _gripper_joint_init[2] = JF_INIT;
    return _gripper_joint_init;
}
