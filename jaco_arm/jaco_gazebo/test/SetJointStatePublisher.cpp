#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Sends a sensor_msgs/JointState message to a topic in order to set the joint state.

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

#include <jaco_joints/JacoJointManager.h>
#include <string>
#include <vector>

#define JOINT_STATES_TOPIC "/jaco/joint_state"
#define JOINT_CONTROL_TOPIC "/jaco/joint_control"

sensor_msgs::JointState js;
bool received_js;

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
 * Sends a sensor_msgs/JointState message to a topic in order to set the joint state.
 * Holds the command for 5 seconds and then stops the arm.
 */
int main(int argc, char **argv)
{
    if (argc < 3)
    {
        ROS_ERROR_STREAM("Not enough arguments, specify (1) joint number [0..8] and (2) target velocity. "
                         << "Optional: joint states topic name (3) to get state and (4) to publish on!");
        return 1;
    }
    ros::init(argc, argv, "set_joint_state_publisher");

    ros::NodeHandle n("");

    int set_joint_i = atoi(argv[1]);
    float vel = atof(argv[2]);

    JacoJointManager joints;

    std::string jointStateTopic(JOINT_STATES_TOPIC);
    if (argc >= 4) jointStateTopic = std::string(argv[3]);

    std::string jointStatePublishTopic(JOINT_CONTROL_TOPIC);
    if (argc >= 4) jointStatePublishTopic = std::string(argv[4]);

    sensor_msgs::JointState currentState = getCurrentJointState(jointStateTopic, n);
    std::vector<int> idx;
    if (joints.getJointIndices(currentState.name, idx) != 0)
    {
        ROS_ERROR("Current state does not have all arm joint names.");
        return 1;
    }

    ROS_INFO_STREAM("Publishing on " << jointStatePublishTopic);
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>(jointStatePublishTopic, 1000, true);

    float maintain_secs = 5;

    sensor_msgs::JointState newState = currentState;
    for (int i = 0; i < newState.velocity.size(); ++i) newState.velocity[i] = 0;
    newState.velocity[idx[set_joint_i]] = vel;

    /*ros::Rate loop_rate(1);
    while (ros::ok()) {
        //newState.header.stamp=ros::Time::now();
        ROS_INFO_STREAM("Publishing "<<newState);
        pub.publish(newState);
        ros::spinOnce();
        loop_rate.sleep();
    }*/

    while (pub.getNumSubscribers() == 0)
    {
        ROS_INFO_STREAM("SetJointStatePublisher: Waiting for subscribers to "
                        << jointStatePublishTopic << "...");
        ros::Duration(1).sleep();
    }

    ROS_INFO_STREAM("Publishing " << newState);
    pub.publish(newState);

    ros::Duration(maintain_secs).sleep();

/*
    // reset velocity
    newState = getCurrentJointState(jointStateTopic, n);
    for (int i = 0; i < newState.velocity.size(); ++i) newState.velocity[i] = 0;
    newState.velocity[idx[set_joint_i]] = 0;


    while (pub.getNumSubscribers() == 0)
    {
        ROS_INFO_STREAM("SetJointStatePublisher: Waiting for subscribers to "
                        << jointStatePublishTopic << "...");
        ros::Duration(1).sleep();
    }
    pub.publish(newState);

    // wait to make sure the message arrives
    ros::Duration(1).sleep();
*/
    return 0;
}
