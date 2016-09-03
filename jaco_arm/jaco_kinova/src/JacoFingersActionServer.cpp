/**
 */
#include <jaco_ros/jaco_fingers_action.h>

using jaco_joints::JacoFingersActionServer;

JacoFingersActionServer::JacoFingersActionServer(
    ros::NodeHandle &n,
    std::string& action_topic_name,
    kinova_msgs::FingerPosition& _targetF,
    JacoJoints& _joints,
    boost::mutex& _lock) :

    targetF(_targetF),
    joints(_joints),
    action_server(n, action_topic_name, boost::bind(&JacoFingersActionServer::ActionCallback, this, _1), false),
    lock(_lock)
{
    action_server.start();
}

JacoFingersActionServer::~JacoFingersActionServer()
{

}


bool JacoFingersActionServer::equalFlt(float first, float second, float tolerance)
{
    return ((first <= second + tolerance) && (first >= second - tolerance));
}


bool JacoFingersActionServer::equalAngles(const kinova_msgs::FingerPosition& first, const kinova_msgs::FingerPosition & second, float tolerance)
{
    return equalFlt(first.finger1, second.finger1, tolerance)
           && equalFlt(first.finger2, second.finger2, tolerance)
           && equalFlt(first.finger3, second.finger3, tolerance);
}

void JacoFingersActionServer::ActionCallback(const kinova_msgs::SetFingersPositionGoalConstPtr &goal)
{
    kinova_msgs::SetFingersPositionFeedback feedback;
    kinova_msgs::SetFingersPositionResult result;

    ROS_DEBUG("Got an angular goal for the fingers. Joints: %s", joints.toString().c_str());

    /*if (arm.Stopped())
    {
        arm.GetAngles(cur_position);
        result.angles = cur_position.Angles();

        action_server.setAborted(result);
        return;
    }*/

    kinova_msgs::FingerPosition target = goal->fingers;

    //ROS_INFO("Target: %f",target.finger1);

    lock.lock();
    targetF.finger1 = target.finger1;
    targetF.finger2 = target.finger2;
    targetF.finger3 = target.finger3;
    lock.unlock();

    ros::Rate r(15);

    const float tolerance = 0.1;    //dead zone for position (Radians)

    //while we have not timed out
    while (true)
    {
        ros::spinOnce();
        if (action_server.isPreemptRequested() || !ros::ok())
        {
            ROS_WARN("Aborting FingersAction execution because cancellation was requested");
            //arm.Stop();
            //arm.Start();
            action_server.setPreempted();
            return;
        }

        //ROS_INFO("Correctign: %s",joints.toString().c_str());
        //ROS_INFO("Target: %f",targetF.finger1);

        lock.lock();
        bool valid;
        kinova_msgs::FingerPosition curr_angles = joints.Fingers(valid);
        lock.unlock();

        feedback.fingers = curr_angles;

        /*if (arm.Stopped())
        {
            result.angles = cur_position.Angles();
            action_server.setAborted(result);
            return;
        }*/

        action_server.publishFeedback(feedback);

        if (equalAngles(curr_angles, target, tolerance))
        {
            //ROS_INFO("Angular Control Complete.");

            result.fingers = curr_angles;
            action_server.setSucceeded(result);
            return;
        }

        ROS_DEBUG_STREAM("Jaco finger angles action: Not at target yet. Current angles: " << curr_angles);

        r.sleep();
    }
}
