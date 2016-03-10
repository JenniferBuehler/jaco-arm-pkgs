#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Simple test client to test JointTrajectory execution

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


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>

#include <jaco_joints/JacoJointManager.h>
#include <convenience_math_functions/MathFunctions.h>

#define JOINT_TRAJECTORY_ACTION_TOPIC "/jaco/joint_trajectory_action"
#define JOINT_STATES_TOPIC "/jaco/joint_state"

#include <string>
#include <vector>

sensor_msgs::JointState js;
bool received_js;

using convenience_math_functions::MathFunctions;

void jointStateCallback(const sensor_msgs::JointState& jointstate)
{
    js = jointstate;
    received_js = true;
}

sensor_msgs::JointState getCurrentJointState(std::string& topicName, ros::NodeHandle& n)
{
    received_js = false;
    ros::Subscriber jsub = n.subscribe(topicName, 10, jointStateCallback);
    ROS_INFO("Waiting until current joint state arrives...");
    while (!received_js)
    {
        ros::spinOnce();
    }
    ROS_INFO("Joint state received.");
//  ROS_INFO_STREAM(js);
    return js;
}

/**
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaco_joint_trajectory_client");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(JOINT_TRAJECTORY_ACTION_TOPIC, true);

    ROS_INFO("Waiting for action server to start: %s", JOINT_TRAJECTORY_ACTION_TOPIC);
    // wait for the action server to start
    ac.waitForServer();  // will wait for infinite time
    ROS_INFO("Action server started.");

    ros::NodeHandle n("");

    JacoJointManager joints;

    std::string jointStateTopic(JOINT_STATES_TOPIC);
    if (argc > 1) jointStateTopic=std::string(argv[1]);

    sensor_msgs::JointState currentState = getCurrentJointState(jointStateTopic, n);
    std::vector<int> idx;
    if (joints.getJointIndices(currentState.name, idx) != 0)
    {
        ROS_ERROR("Current state does not have all arm joint names.");
        return 1;
    }

    ROS_INFO("Now constructing goal");

    // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal goal;

    std::vector<std::string> joint_names, arm_joint_names, gripper_joint_names;
    arm_joint_names = joints.getArmJoints();
    gripper_joint_names = joints.getGripperJoints();
    std::string prepend = "";
    joints.getJointNames(joint_names, true, prepend);

    trajectory_msgs::JointTrajectory trajectory;

    trajectory.joint_names = joint_names;

    int pathLen = 3;
    trajectory.points.resize(pathLen);
    int numJoints = joint_names.size();
    float timeOffset = 0;
    float timeStep = 0.5;
    for (int i = 0; i < pathLen; ++i)
    {
        trajectory.points[i].positions.resize(numJoints);
        trajectory.points[i].velocities.resize(numJoints);
        trajectory.points[i].accelerations.resize(numJoints);
        trajectory.points[i].effort.resize(numJoints);
        trajectory.points[i].time_from_start = ros::Duration(timeOffset + timeStep * i);
        for (int k = 0; k < numJoints; ++k)
        {
            trajectory.points[i].positions[k] = currentState.position[idx[k]];
            trajectory.points[i].velocities[k] = 0.0;
            trajectory.points[i].accelerations[k] = 0;
            trajectory.points[i].effort[k] = 0;
        }
    }

    trajectory.points[1].positions[0]+=0.4;
    trajectory.points[2].positions[0]+=0.8;

    trajectory.points[1].positions[6] += 0.2;
    trajectory.points[1].positions[7] += 0.2;
    trajectory.points[1].positions[8] += 0.2;

    trajectory.points[2].positions[6] += 0.4;
    trajectory.points[2].positions[7] += 0.4;
    trajectory.points[2].positions[8] += 0.4;

    goal.trajectory = trajectory;

    ROS_INFO("Now sending goal");
    ROS_INFO_STREAM(goal);

    ac.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_WARN("Action did not finish before the time out.");

    currentState = getCurrentJointState(jointStateTopic, n);

    float trajPos = MathFunctions::capToPI(trajectory.points[2].positions[0]);
    float currPos = MathFunctions::capToPI(currentState.position[idx[0]]);
    ROS_INFO("Final accuracy of joint 0: %f", MathFunctions::capToPI(trajPos - currPos));

    return 0;
}
