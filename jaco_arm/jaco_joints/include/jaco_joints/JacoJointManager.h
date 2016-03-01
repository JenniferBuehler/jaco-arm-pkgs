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


#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

/**
 * Contains information about the for a robotic "arm" as defined in the URDF file.
 * The "robotic arm" is the kinematic chain
 * which is used to reach to objects and grasp or manipulate them.
 * It usually includes a gripper or hand to grasp the objects.
 * This manager can be used to manage all sorts of properties which are specified
 * per-joint or per-link. For example, it can also be used to manage PID values.
 *
 * Read in the paramters as follows from a launch file *before* an instance of this
 * class is created (the parameters are read in the constructor of this class)!
 *
 * ```
 *   <arg name="names_config_file" default="$(find jaco_joints)/config/JacoJoints.yaml"/>
 *   <rosparam command="load" file="$(arg names_config_file)"/>
 * ```
 * 
 * Parameters have to be specified as in this documented .yaml template:
 * 
 * ``rosed jaco_joints JacoJoints.yaml``
 * 
 * To use this class to manage PID values (which is optional),
 * you have to specify the *effort_controller_names*,
 * *position_controller_names* and *velocity_controller_names* lists in the yaml file.
 * These "controller names" are to be the ones used in another configuration file,
 * which has to be in the format also used by ros_control. For example,
 *  for a velocity controller, this could be a PID values config file (lets name it PIDConfig.yaml):
 *
 * ```
 * <joint_name>_velocity_controller:
 *   type: velocity_controllers/JointVelocityController
 *   joint: <joint_name> 
 *   pid: {p: 10, i: 0.0001, d: 0.005}
 * ```                                         
 *
 * In this example, the configuration file (i.e. JacoJoints.yaml) has to include
 * this:
 *
 * ```
 *    velocity_controller_names:
 *       - <joint_name>_velocity_controller
 * ```
 * 
 * The names config file (JacoJoints.yaml) only needs a list of those joint controller names,
 * not the actual PID values. It is assumed that each robot only has *one* velocity, effort or
 * position controller per joint, which is then referred to globally with the same name.
 * The actual PID values may be loaded on the parameter server *after* an instance
 * of this class has already been created.
 * The names in the lists are only needed to look up the right controller values at the
 * time when functions GetPosGains() and GetVelGains() are called to read from the parameter
 * server.    
 * Current limitation: The namespace in the JacoJoints.yaml config file has to be the same as
 * the namespace used in the PID values PIDConfig.yaml file.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class JacoJointManager
{
public:
    /**
     * Initializes joint names by reading from parameter server.
     * \namespace of the robot which is used in the YAML file
     */
    JacoJointManager(const std::string& robot_namespace="jaco");
    JacoJointManager(const JacoJointManager& o);

    /**
     * Reads from the private parameter server according to specification in the example .yaml file.
     */
    void readJointNamesFromParameters();

    /**
     * Joint names as to be used in the default order (e.g. for JointState messsages)
     * \param prepend if not empty, this string is going to be prepended to the joint names
     */
    void getJointNames(std::vector<std::string>& joint_names,
            bool withGripper, const std::string& prepend = std::string()) const;

    const std::vector<std::string>& getArmJoints() const;

    const std::vector<std::string>& getGripperJoints() const;

    const std::vector<float>& getArmJointsInitPose() const;

    const std::vector<float>& getGripperJointsInitPose() const;
    
    /**
     * returns all gripper links *excluding* the palm link link
     */
    const std::vector<std::string>& getGripperLinks() const;
    std::vector<std::string> getDefaultGripperLinks() const;

    /**
     * Returns the link which counts as the "palm", which is
     * the link to which all gripper joints are attached.
     */
    const std::string& getPalmLink() const;
    std::string getDefaultPalmLink() const;

    /**
     * Get all links for the arm, *including* the palm link
     */
    const std::vector<std::string>& getArmLinks() const;
    std::vector<std::string> getDefaultArmLinks() const;


    /**
     * Joint names as to be used in the default order (e.g. for JointState messsages)
     * \param init_poses (optional) if specified, you can set the target angles of the
     * joint positions here. The order has to be standard, as returned by getJointNames().
     */
    void initJointState(sensor_msgs::JointState& js, bool withGripper = true,
            const std::vector<float> * init_poses = NULL) const;

    /**
     * Helper function to find out the order of joints given in a names vector. An index vector
     * is returned, of size 9, which contains indices to the respective joint names into the input vector.
     * \return in vector idx the indices of actuator and gripper joints in the joint_names vector.
     * Indices are given in order of first all arm joints and then all gripper joints.
     * 
     * Returns -1 if not all arm or all gripper joints are present in joint_names.
     * Required are either all arm joints, or all gripper joints, or both arm and gripper joints.
     * Returns 0 if all joints present, 1 if only arm joints present, and 2 if only gripper joints present.
     * If only one group is present, the indices for the other group in idx are going to be set to -1.
     */
    int getJointIndices(const std::vector<std::string>& joint_names, std::vector<int>& idx);

    bool isGripper(const std::string& name) const;

    /**
     * Returns the number of the joint in the arm, or -1 if this is no arm joint.
     * Numbering starts with 0.
     */
    int armJointNumber(const std::string& name) const;

    /**
     * Returns the number of the joint of the grippers, or -1 if this is no gripper joint.
     * Numbering starts with 0.
     */
    int gripperJointNumber(const std::string& name) const;

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

protected:
    // JacoJointManager();
   
    /** 
     * Names of the arm joints *without* the gripper joints.
     * These are the joints which are used to move the arm in
     * pre-grasp state, but which are *not* part of the acutal
     * gripper (essentially an arm without the grippers)
     */
    std::vector<std::string> arm_joints;

    /**
     * All links which are in-between (and directly before and after)
     * the arm_joints. It does however *not* include the palm_link
     * because this is specified separately.
     */
    std::vector<std::string> arm_links;

    /**
     * All joints of the "gripper". The gripper is the part of the
     * arm used to grasp/grip objects. 
     * Essentially, they are the "gripper joints".
     */
    std::vector<std::string> gripper_joints;


    /**
     * All links which are in-between (and directly before and after)
     * the gripper_joints. It does however *not* include the palm_link
     * because this is specified separately.
     */
    std::vector<std::string> gripper_links;

    /**
     * Name of the palm link. This is the link to which objects
     * that this end effector grasps are symbolically "attached"
     * when the object is grasped.
     * This link should be the link directly before the gripper joints.
     * There should be **no other movable joints between the palm link
     * and the gripper joints**.
     * The palm link lies between the last joint in arm_joints and
     * before all joints in gripper_joints.
     */
    std::string palm_link;

    /**
     * initial ("Home") pose of the arm joints. Has to be the same
     * order as arm_joints.
     */
    std::vector<float> arm_joint_init;
    
    /**
     * initial ("Home") pose of the gripper joints. Has to be the same
     * order as gripper_joints.
     */
    std::vector<float> gripper_joint_init;

    std::vector<std::string> arm_effort_controller_names;
    std::vector<std::string> arm_velocity_controller_names;
    std::vector<std::string> arm_position_controller_names;
    
    std::vector<std::string> gripper_effort_controller_names;
    std::vector<std::string> gripper_velocity_controller_names;
    std::vector<std::string> gripper_position_controller_names;

    std::string robot_namespace;
};

#endif  // JACO_JOINTS_JACOJOINTMANAGER_H
