#ifndef JACO_GAZEBO_JACOGAZEBOJOINTSTATECLIENT_H
#define JACO_GAZEBO_JACOGAZEBOJOINTSTATECLIENT_H

/**
   Gazebo plugin which accepts sensor_msgs/JointState messages to set the joint targets in the models JointController.

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

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <jaco_joints/JacoJointManager.h>
#include <jaco_gazebo/JointController.h>

namespace gazebo
{

/**
 * \brief Gazebo plugin which accepts sensor_msgs/JointState messages to set the joint targets in the models JointController.
 *
 * **Requirements**
 *
 * Requires another gazebo plugin running which adds a gazebo::physics::JointControllerThreadsafe instance as a child to the model
 * and controls the joints. This can be for example an instance of gazebo::JacoGazeboJointControl.
 *
 * **ROS parameters**
 *
 * Reads the topic to subscribe to from ROS parameter "jaco_joint_state_control_topic".
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class JacoGazeboJointStateClient : public ModelPlugin
{
public:
    JacoGazeboJointStateClient();

    virtual ~JacoGazeboJointStateClient();

    /**
     *
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
    virtual void UpdateChild();

    bool isGripper(const physics::JointPtr& joint) const;

    physics::ModelPtr model;
    JacoJointManager joints;

private:
    void JointStateCallback(sensor_msgs::JointStateConstPtr _joints);

    physics::JointControllerThreadsafePtr jointController;

    std::string jointStateTopic;
    ros::Subscriber jsSub;
    ros::NodeHandle nh;
};

}  // namespace gazebo

#endif  // JACO_GAZEBO_JACOGAZEBOJOINTSTATECLIENT_H
