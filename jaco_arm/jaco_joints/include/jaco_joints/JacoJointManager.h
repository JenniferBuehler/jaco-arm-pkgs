#ifndef JACO_JOINTS_JACOJOINTMANAGER_H
#define JACO_JOINTS_JACOJOINTMANAGER_H

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


#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

/**
 * \brief Manages the jaco joint names as specified in the URDF file.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class JacoJointManager
{
public:
    /**
     * Initializes joint names by reading from parameter server.
     * If the parameters are not set on the parameter server, the
     * default names are used for initialization.
     * Parameters:
     * - "arm_joint_<i>_name" [i=0..5]: names of the arm joints
     * - "finger_joint_<i>_name" [i=0..2]: names of the finger joints
     * - "arm_joint_<i>_init" [i=0..5]: initial positions of the arm joints
     * - "finger_joint_<i>_init" [i=0..2]: initial positions of the finger joints
     */
    JacoJointManager();

    /**
     * \param _arm_joints       joint names of the arm joints: joints 1..6 (at indices 0..4)
     * \param _finger_joints    joint names of the finger joints: joints 1..3 (at indices 0..2)
     */
    JacoJointManager(const std::vector<std::string>& _arm_joints,
                     const std::vector<std::string>& _finger_joints,
                     const std::vector<float>& _arm_joint_init,
                     const std::vector<float>& _finger_joint_init);

    JacoJointManager(const JacoJointManager& o);

    /**
     * Reads from the private parameter server:
     * - "arm_joint_<i>_name" [i=0..5]: names of the arm joints
     * - "finger_joint_<i>_name" [i=0..2]: names of the finger joints
     * - "arm_joint_<i>_init" [i=0..5]: initial positions of the arm joints
     * - "finger_joint_<i>_init" [i=0..2]: initial positions of the finger joints
     */
    void readJointNamesFromParameters();

    /**
     * Joint names as to be used in the default order (e.g. for JointState messsages)
     * \param prepend if not empty, this string is going to be prepended to the joint names
     */
    void getJointNames(std::vector<std::string>& joint_names,
            bool withFingers, const std::string& prepend = std::string()) const;

    const std::vector<std::string>& getArmJoints() const;

    const std::vector<std::string>& getFingerJoints() const;

    const std::vector<float>& getArmJointsInitPose() const;

    const std::vector<float>& getFingerJointsInitPose() const;

    /**
     * Joint names as to be used in the default order (e.g. for JointState messsages)
     * \param init_poses (optional) if specified, you can set the target angles of the
     * joint positions here. The order has to be standard, as returned by getJointNames().
     */
    void initJointState(sensor_msgs::JointState& js, bool withFingers = true,
            const std::vector<float> * init_poses = NULL) const;

    /**
     * Helper function to find out the order of joints given in a names vector. An index vector
     * is returned, of size 9, which contains indices to the respective joint names into the input vector.
     * \return in vector idx the indices of actuator and finger joints in the joint_names vector.
     * Indices are given in order of first all arm joints and then all finger joints.
     * 
     * Returns -1 if not all arm or all finger joints are present in joint_names.
     * Required are either all arm joints, or all finger joints, or both arm and finger joints.
     * Returns 0 if all joints present, 1 if only arm joints present, and 2 if only finger joints present.
     * If only one group is present, the indices for the other group in idx are going to be set to -1.
     */
    int getJointIndices(const std::vector<std::string>& joint_names, std::vector<int>& idx);

    bool isFinger(const std::string& name) const;

    /**
     * Returns the number of the joint in the arm, or -1 if this is no arm joint.
     * Numbering starts with 0.
     */
    int armJointNumber(const std::string& name) const;

    /**
     * Returns the number of the joint of the fingers, or -1 if this is no finger joint.
     * Numbering starts with 0.
     */
    int fingerJointNumber(const std::string& name) const;

    /**
     * enforces the bound of the angle to be between -PI and PI
     * this is a useful helper for lots of places because the
     * native jaco joint angles may go beyond PI.
     */
    static double capToPI(const double value);

    template<typename Flt>
    static void capToPI(std::vector<Flt>& v)
    {
        for (typename std::vector<Flt>::iterator it = v.begin(); it != v.end(); ++it)
        {
            *it = capToPI(*it);
        }
    }


    /**
     * Returns shortest distace between two angles (in rad), specifically
     * when going from \e _f2 to \e _f1
     */
    static double angleDistance(const double _f1, const double _f2);

    /**
     * Transforms a value between -PI and PI to a value between -2*PI and +2*PI such
     * that the joint limits are respected. It is assumed all values i have to be between
     * between lower bound l and higher bound h, and always l <= i <= h.
     * So e.g. l=-3.9 .. h=0.8 instead of
     * l=2.38 .. h=0.8. So lowLimit can *always* be smaller than highlimit, and there
     * never is a jump from PI to -PI or the other way round.
     */
    static double limitsToTwoPI(const double value, const double lowLimit, const double highLimit);

    /**
     * Reads PID parameters from the ROS parameter server, where they are stored as a dictionary
     * as follows:
     *
     *     pid:
     *         p: <p-value>
     *         i: <i-value>
     *         d: <d-value>
     */
    void ReadPIDValues(const std::string& pidParameterName, float& kp, float& ki, float& kd) const;

    /**
     * Calls ReadPIDValues() for the position controller for the joint specified in jointName.
     * If the joint is not found, the values of
     * paramters kp,ki and kd are left as they are, so ideally they should be
     * initialzied with defaults before calling this function.
     * \return false if the joint 'jointName' is not maintained by this joint manager.
     */
    bool GetPosGains(const std::string& jointName, float& kp, float& ki, float& kd) const;

    /**
     * Like GetPosGains(), but for velocity controller.
     */
    bool GetVelGains(const std::string& jointName, float& kp, float& ki, float& kd) const;

private:
    // contains joint names of the arm joints: joints 1..6 (at indices 0..4)
    std::vector<std::string> arm_joints;
    // contains joint names of the finger joints: joints 1..3 (at indices 0..2)
    std::vector<std::string> finger_joints;

    // contains joint names of the arm joint initial position: joints 1..6 (at indices 0..4)
    std::vector<float> arm_joint_init;
    // contains joint names of the finger joint initial position: joints 1..3 (at indices 0..2)
    std::vector<float> finger_joint_init;

    ros::NodeHandle priv;
    ros::NodeHandle pub;
};

#endif  // JACO_JOINTS_JACOJOINTMANAGER_H
