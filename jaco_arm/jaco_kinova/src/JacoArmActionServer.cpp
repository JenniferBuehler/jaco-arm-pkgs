#include <jaco_ros/jaco_arm_action.h>
#include <boost/math/constants/constants.hpp>

using jaco_joints::JacoArmActionServer;

JacoArmActionServer::JacoArmActionServer(
    ros::NodeHandle &n, std::string& action_topic_name,
    kinova_msgs::JointAngles& _targetA,
    JacoJoints& _joints, boost::mutex& _lock) :

    targetA(_targetA), joints(_joints),
    action_server(n, action_topic_name, boost::bind(&JacoArmActionServer::ActionCallback, this, _1), false),
    lock(_lock)
{
    //action_server.registerPreemptCallback(boost::bind<&JacoArmActionServer::PreemptCallback, this);
    action_server.start();
}

JacoArmActionServer::~JacoArmActionServer()
{

}


bool JacoArmActionServer::equalFlt(float first, float second, float tolerance)
{
    return ((first <= second + tolerance) && (first >= second - tolerance));
}


bool JacoArmActionServer::equalAngles(const kinova_msgs::JointAngles& first, const kinova_msgs::JointAngles & second, float tolerance)
{
    return equalFlt(first.joint1, second.joint1, tolerance)
           && equalFlt(first.joint2, second.joint2, tolerance)
           && equalFlt(first.joint3, second.joint3, tolerance)
           && equalFlt(first.joint4, second.joint4, tolerance)
           && equalFlt(first.joint5, second.joint5, tolerance)
           && equalFlt(first.joint6, second.joint6, tolerance);
}

//enforces the bound of the angle to be between -PI and PI
template<typename Flt>
void capToPI(Flt& v)
{
    if (v <= -boost::math::constants::pi<Flt>() || v > boost::math::constants::pi<Flt>())
    {
        v = fmod(v, 2.0 * boost::math::constants::pi<Flt>());
        if (v <= -boost::math::constants::pi<Flt>())
            v += 2.0 * boost::math::constants::pi<Flt>();
        else if (v > boost::math::constants::pi<Flt>())
            v -= 2.0 * boost::math::constants::pi<Flt>();
    }
}
void capToPI(kinova_msgs::JointAngles& v)
{
    capToPI(v.joint1);
    capToPI(v.joint2);
    capToPI(v.joint3);
    capToPI(v.joint4);
    capToPI(v.joint5);
    capToPI(v.joint6);
}





void JacoArmActionServer::ActionCallback(const kinova_msgs::ArmJointAnglesGoalConstPtr &goal)
{
    kinova_msgs::ArmJointAnglesFeedback feedback;
    kinova_msgs::ArmJointAnglesResult result;

    ROS_INFO("Got an angular goal for the arm.");// Joints: %s",joints.toString().c_str());

    /*if (arm.Stopped())
    {
        arm.GetAngles(cur_position);
        result.angles = cur_position.Angles();

        action_server.setAborted(result);
        return;
    }*/

    kinova_msgs::JointAngles target = goal->angles;

    capToPI(target);

    //ROS_INFO("Target: %f",target.joint1);

    lock.lock();
    targetA.joint1 = target.joint1;
    targetA.joint2 = target.joint2;
    targetA.joint3 = target.joint3;
    targetA.joint4 = target.joint4;
    targetA.joint5 = target.joint5;
    targetA.joint6 = target.joint6;
    lock.unlock();

    ros::Rate r(10);

    const float tolerance = 0.1;    //dead zone for position (Radians)

    //while we have not timed out
    while (true)
    {
        ros::spinOnce();
        if (action_server.isPreemptRequested() || !ros::ok())
        {
            ROS_WARN("Aborting ArmAction execution because cancellation was requested");
            //arm.Stop();
            //arm.Start();
            action_server.setPreempted();
            return;
        }

        //ROS_INFO("Correctign: %s",joints.toString().c_str());
        //ROS_INFO("Getting current angles");
        lock.lock();
        bool valid;
        kinova_msgs::JointAngles curr_angles = joints.Angles(valid);
        lock.unlock();

        capToPI(curr_angles);

        //ROS_INFO("Target: %f - curr %f ",targetA.joint1,curr_angles.joint1);

        feedback.angles = curr_angles;

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

            result.angles = curr_angles;
            action_server.setSucceeded(result);
            return;
        }

        r.sleep();
    }
}

