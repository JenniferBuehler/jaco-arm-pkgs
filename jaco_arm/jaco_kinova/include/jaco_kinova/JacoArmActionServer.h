/**
 */
#ifndef _JACO_ARM_ANGLES_ACTION_H_
#define _JACO_ARM_ANGLES_ACTION_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <kinova_msgs/JointAngles.h>
#include <jaco_ros/jaco_joints.h>

namespace jaco_joints
{

/**
 * Implements functionality for ArmJointAngles action of the Kinova message interface
 */
class JacoArmActionServer
{
public:
    /**
     * \param _targetA pointer to target angles. This reference will be manipulated by this class to set the target angles for the arm.
     * \param _joints reference to the joints objects currently active in gazebo (to obtain current state)
     * \param action_topic_name topic name for ArmJointAngles
     */
    JacoArmActionServer(
        ros::NodeHandle &n,
        std::string& action_topic_name,
        kinova_msgs::JointAngles& _targetA,
        JacoJoints& _joints, boost::mutex& _lock);

    ~JacoArmActionServer();
    void ActionCallback(const kinova_msgs::ArmJointAnglesGoalConstPtr &);

private:

    bool equalFlt(float first, float second, float tolerance);
    bool equalAngles(const kinova_msgs::JointAngles& first, const kinova_msgs::JointAngles & second, float tolerance);


    boost::mutex& lock; //to lock access to the fields joints and targetA

    //reference to structure to store target angles at
    kinova_msgs::JointAngles& targetA;
    //reference to structure storing current joint values
    JacoJoints& joints;

    actionlib::SimpleActionServer<kinova_msgs::ArmJointAnglesAction> action_server;

};

}  // namespace
#endif // _JACO_ANGLES_ACTION_H_

