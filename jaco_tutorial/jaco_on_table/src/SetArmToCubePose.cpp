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

// cube spawned as follows on jaco_on_table:
// rosrun gazebo_test_tools cube_spawner cube1 0.2 0 0.76 robot_base
void getJointStateCube1(const JacoJointManager& joints, sensor_msgs::JointState& pre_grasp_state, sensor_msgs::JointState& grasp_state)
{
    
    std::vector<float> p;
    p.push_back(-1.4434142800407566);
    p.push_back(-0.5);
    p.push_back(-0.41735973772675905);
    p.push_back(-2.7749829126295795);
    p.push_back(2.227743057190299);
    p.push_back(-2.5657326478268896);
    p.push_back(0.0030029308999246496);
    p.push_back(0.003514033567878272);
    p.push_back(0.005490284397680512);
    joints.copyToJointState(pre_grasp_state, 0, &p, 0, true);
    pre_grasp_state.velocity.resize(p.size(),0);

    p.clear();
    p.push_back(-1.4434142800407566);
    p.push_back(-0.30078138329085613);
    p.push_back(-0.41735973772675905);
    p.push_back(-2.7749829126295795);
    p.push_back(2.227743057190299);
    p.push_back(-2.5657326478268896);
    p.push_back(0.0030029308999246496);
    p.push_back(0.003514033567878272);
    p.push_back(0.005490284397680512);
    joints.copyToJointState(grasp_state, 0, &p, 0, true);
    grasp_state.velocity.resize(p.size(),0);
}

sensor_msgs::JointState getHomeState(const JacoJointManager& joints)
{
    const std::vector<float>& arm_init = joints.getArmJointsInitPose();
    sensor_msgs::JointState ret;
    joints.copyToJointState(ret, 0, &arm_init, 0, true);
    ret.velocity.resize(arm_init.size(),0);
    return ret;
}
            
void resetFingers(sensor_msgs::JointState& s, const sensor_msgs::JointState& currPosState)
{
    for (int i=6; i<9; ++i) s.position[i]=currPosState.position[i];
}

/**
 * Sends a sensor_msgs/JointState message to a topic in order to set the joint state
 * to the fixed position where it can grasp a cube. Helper functions are available
 * for cubes spawned on jaco_on_table at a certain pose.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_joint_state_cube_publisher");

    ros::NodeHandle n("");
    
    bool setHome=false;
    bool skipFingers=false;
    if (argc > 1) 
    {
        for (int i=1; i<argc; ++i)
        {
            std::string arg=argv[i];
            if (arg=="--home")
            {
                ROS_INFO("Setting jaco to home position.");
                setHome=true;
            }
            if (arg=="--no-fingers")
            {
                ROS_INFO("Setting jaco to home position.");
                skipFingers=true;
            }
        }
    }


    JacoJointManager joints;
    std::string jointStateTopic(JOINT_STATES_TOPIC);
    std::string jointStatePublishTopic(JOINT_CONTROL_TOPIC);

    ROS_INFO_STREAM("Publishing on " << jointStatePublishTopic);
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>(jointStatePublishTopic, 1000, true);

        
    sensor_msgs::JointState currPosState;
    // in case we skip the fingers, we'll need the actual current state of the robot
    // to initialize new joint states later on.
    if (skipFingers)
    {
        std::vector<std::string> jointNames;
        joints.getJointNames(jointNames,true);
        // get the current state, as other joints of jaco have to remain the same
        sensor_msgs::JointState _currPosState = getCurrentJointState(jointStateTopic, n);
        std::vector<int> idx;
        if ((joints.getJointIndices(_currPosState.name, idx) != 0) || (idx.size()!=jointNames.size()))
        {
            ROS_ERROR("Current state does not have all arm and finger joint names.");
            return 1;
        }
        currPosState.position.resize(idx.size());
        for (int i=0; i<idx.size(); ++i) currPosState.position[i] = _currPosState.position[idx[i]];
    }


    while (pub.getNumSubscribers() == 0)
    {
        ROS_INFO("SetArmToCubePose: waiting for subscribers...");
        ros::Duration(1).sleep();
    }

    if (!setHome)
    {
        sensor_msgs::JointState grasp_state;
        sensor_msgs::JointState pre_grasp_state;
        getJointStateCube1(joints,pre_grasp_state, grasp_state);
        if (skipFingers)
        {
            resetFingers(pre_grasp_state, currPosState);
            resetFingers(grasp_state, currPosState);
        }
        ROS_INFO_STREAM("Publishing pre-grasp state" << pre_grasp_state);
        pub.publish(pre_grasp_state);

        // Wait for the arm to arrive. This could be improved by waiting
        // for the exact state to be reached, but it's only a test at this
        // stage anyway.
        ros::Duration(3).sleep();
        
        ROS_INFO_STREAM("Publishing grasp state" << grasp_state);
        pub.publish(grasp_state);
    }
    else
    {
        sensor_msgs::JointState home_state=getHomeState(joints);
        if (skipFingers) resetFingers(home_state, currPosState);
        ROS_INFO_STREAM("Publishing grasp state" << home_state);
        pub.publish(home_state);
    }

    // wait to allow that the last message arrives before quitting node
    ros::Duration(1).sleep();
    return 0;
}
