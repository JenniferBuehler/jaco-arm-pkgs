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
    return js;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_joint_state_cube_publisher");

    ros::NodeHandle n("");
    
    bool doUngrasp=false;
    if (argc > 1) 
    {
        std::string arg=argv[1];
        if (arg=="--ungrasp")
        {
            ROS_INFO("ungrasping cube");
            doUngrasp=true;
        }
    }

    JacoJointManager joints;
    std::string jointStateTopic(JOINT_STATES_TOPIC);
    std::string jointStatePublishTopic(JOINT_CONTROL_TOPIC);

    ROS_INFO_STREAM("Publishing on " << jointStatePublishTopic);
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>(jointStatePublishTopic, 1000, true);

    // get the current state, as other joints of jaco have to remain the same
    sensor_msgs::JointState currentState = getCurrentJointState(jointStateTopic, n);
    std::vector<int> idx;
    if ((joints.getJointIndices(currentState.name, idx) != 0) || (idx.size()!=9))
    {
        ROS_ERROR("Current state does not have all arm and finger joint names.");
        return 1;
    }
        
    sensor_msgs::JointState grasp_state = currentState;
    if (!grasp_state.velocity.empty())
    {   // all velocities zero
        for (int i = 0; i < grasp_state.velocity.size(); ++i) grasp_state.velocity[i] = 0;
    }

    while (pub.getNumSubscribers() == 0)
    {
        ROS_INFO("GraspCube: waiting for subscribers...");
        ros::Duration(1).sleep();
    }

    if (!doUngrasp)
    {

        float grasp_position=0.53;
        for (int i = 6; i < idx.size(); ++i) grasp_state.position[idx[i]] = grasp_position;
        ROS_INFO_STREAM("Publishing grasp state" << grasp_state);
        pub.publish(grasp_state);
    }
    else
    {
        float ungrasp_position=0.05;
        for (int i = 6; i < idx.size(); ++i) grasp_state.position[idx[i]] = ungrasp_position;

        ROS_INFO_STREAM("Publishing ungrasp state" << grasp_state);
        pub.publish(grasp_state);
    }

    // wait to allow that the last message arrives before quitting node
    ros::Duration(1).sleep();
    return 0;
}
