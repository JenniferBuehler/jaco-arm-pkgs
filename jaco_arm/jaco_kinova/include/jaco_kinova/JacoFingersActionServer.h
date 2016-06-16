/**
 */
#ifndef _JACO_FINGERANGLES_ACTION_H_
#define _JACO_FINGERANGLES_ACTION_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <jaco_msgs/SetFingersPositionAction.h>

#include <jaco_ros/jaco_joints.h>

#include <jaco_msgs/FingerPosition.h>

namespace jaco_joints
{

/**
 * Implements functionality for SetFingerPosition action of the kinova message interface
 */
class JacoFingersActionServer
{
public:
    /**
     * \param _targetF pointer to target angles. This reference will be manipulated by this class to set the target angles for the fingers.
     * \param _joints reference to the joints objects currently active in gazebo (to obtain current state)
     * \param _lock mutex locking access to _targetF
     * \param action_topic_name topic name for SetFingerPosition
     */
    JacoFingersActionServer(
        ros::NodeHandle &n,
        std::string& action_topic_name,
        jaco_msgs::FingerPosition& _targetF,
        JacoJoints& _joints, boost::mutex& _lock);

    ~JacoFingersActionServer();
    void ActionCallback(const jaco_msgs::SetFingersPositionGoalConstPtr&);

private:

    bool equalFlt(float first, float second, float tolerance);
    bool equalAngles(const jaco_msgs::FingerPosition& first, const jaco_msgs::FingerPosition & second, float tolerance);

    boost::mutex& lock; //to lock access to the fields joints and targetA
    jaco_msgs::FingerPosition& targetF;
    JacoJoints& joints;
    actionlib::SimpleActionServer<jaco_msgs::SetFingersPositionAction> action_server;

};

}  // namespace


#endif // _GAZEBO_JACO_FINGERANGLES_ACTION_H_

