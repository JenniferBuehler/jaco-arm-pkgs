#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
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


#include <jaco_gazebo/JacoGazeboJointStateClient.h>

#include <ros/ros.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>

#include <map>
#include <string>
#include <algorithm>
#include <vector>

#define DEFAULT_JOINT_STATE_TOPIC "jaco/joint_control"

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(JacoGazeboJointStateClient);

JacoGazeboJointStateClient::JacoGazeboJointStateClient():
    joints(),  // use default constructor to read from parameters
    jointStateTopic(DEFAULT_JOINT_STATE_TOPIC)
{
    ROS_INFO("Creating JacoGazeboJointStateClient plugin");
    nh.param("jaco_joint_state_control_topic", jointStateTopic, jointStateTopic);
    ROS_INFO_STREAM("Joint state subscription topic: " << jointStateTopic);
}

JacoGazeboJointStateClient::~JacoGazeboJointStateClient()
{
}



void JacoGazeboJointStateClient::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
                         << "in the gazebo_ros package)");
        return;
    }

    physics::BasePtr jcChild = _parent->GetChild(physics::JointControllerThreadsafe::UniqueName());
    if (!jcChild.get())
    {
        ROS_ERROR("Cannot load JacoGazeboJointStateClient if no default JointControllerThreadsafe is set for the model");
        throw std::string("Cannot load JacoGazeboJointStateClient if no default JointController is set for the model");
    }
    physics::JointControllerThreadsafePtr ptr =
        boost::dynamic_pointer_cast<physics::JointControllerThreadsafe>(jcChild);
    if (!ptr.get())
    {
        ROS_ERROR_STREAM("Cannot load JacoGazeboJointStateClient if child '"
                         << physics::JointControllerThreadsafe::UniqueName()
                         << "' is not of class JointControllerThreadsafe");
        throw std::string("Cannot load JacoGazeboJointStateClient if child '"
                          + physics::JointControllerThreadsafe::UniqueName()
                          + "' is not of class JointControllerThreadsafe");
    }
    jointController = ptr;

    // get joint names from parameters
    std::vector<std::string> joint_names;
    joints.getJointNames(joint_names, true);
    const std::vector<float>& arm_init = joints.getArmJointsInitPose();
    const std::vector<float>& gripper_init = joints.getGripperJointsInitPose();

    if (joint_names.size() != 9)
    {
        ROS_ERROR("JacoGazeboJointStateClient: joint names have to be of size 9!");
        throw std::string("JacoGazeboJointStateClient: joint names have to be of size 9!");
    }

    // Print the joint names to help debugging
    std::map<std::string, physics::JointPtr > jntMap = jointController->GetJoints();
    for (std::map< std::string, physics::JointPtr>::iterator it = jntMap.begin(); it != jntMap.end(); ++it)
    {
        physics::JointPtr j = it->second;
        // ROS_INFO_STREAM("Gazebo joint: '"<<j->GetName()<<"' is registered as '"<<it->first<<"'");
    }


    // check if the joint names maintained in 'joints' match the names in gazebo,
    // that the joints can be used by this class, and if yes, load PID controllers.
    int i = 0;
    for (std::vector<std::string>::iterator it = joint_names.begin();
            it != joint_names.end(); ++it)
    {
        // ROS_INFO_STREAM("Local joint name: '"<<*it<<"'");

        physics::JointPtr joint = _parent->GetJoint(*it);
        if (!joint.get())
        {
            ROS_FATAL_STREAM("Joint name " << *it << " not found as robot joint");
            throw std::string("Joint not found");
        }

        std::string scopedName = joint->GetScopedName();
        std::map<std::string, physics::JointPtr >::iterator jit = jntMap.find(scopedName);
        if (jit == jntMap.end())
        {
            ROS_ERROR_STREAM("Joint name " << *it << " not found in joint controller joints.");
            throw std::string("Joint not found");
        }

        ++i;
    }

    model = _parent;

    jsSub = nh.subscribe(jointStateTopic, 1, &JacoGazeboJointStateClient::JointStateCallback, this);
}


void JacoGazeboJointStateClient::JointStateCallback(sensor_msgs::JointStateConstPtr _joints)
{
    // ROS_INFO_STREAM("Joint state callback: "<<*_joints);
    if (!jointController.get())
    {
        ROS_ERROR("Cannot load JacoGazeboJointStateCallback if no default JointController is set for the model");
        return;
    }

    // number of joints we are going to process is the larger of velocity or position
    size_t jointCount = std::min(_joints->name.size(), std::max(_joints->velocity.size(), _joints->position.size()));
    if (jointCount == 0)
    {
        ROS_ERROR("JacoGazeboJointStateClient: Must at least provide one position or velocity");
        return;
    }

    boost::unique_lock<boost::recursive_mutex> lck = jointController->GetLock();

    std::map<std::string, double> forces = jointController->GetForces();
    std::map<std::string, double> positions = jointController->GetPositions();
    std::map<std::string, double> velocities = jointController->GetVelocities();
    std::map<std::string, physics::JointPtr > jntMap = jointController->GetJoints();


    // clear out all possible current targets. Necessary e.g. to clear out
    // exitsing velocity targets, when only positions are set now...
    jointController->Reset();

    for (size_t i = 0; i < jointCount; ++i)
    {
        bool isGripper = false;
        std::string jointName = _joints->name[i];


        // ROS_INFO_STREAM("Processing joint '"<<jointName<<"'");
        physics::JointPtr joint = model->GetJoint(jointName);
        if (!joint.get())
        {
            ROS_ERROR_STREAM("Joint name " << jointName << " not found as robot joint");
            continue;
        }

        std::string scopedName = joint->GetScopedName();
        std::map<std::string, physics::JointPtr >::iterator jit = jntMap.find(scopedName);
        if (jit == jntMap.end())
        {
            // ROS_WARN_STREAM("Joint name "<<scopedName<<" not found in joint controller joints.");
            continue;
        }

        if (i < _joints->position.size())
        {
            double currTargetPosVal = positions[scopedName];
            // set current target position of joint i, if it is not already achieved:
            double targetPosVal = _joints->position[i];
            static double eps = 1e-06;

            if (fabs(currTargetPosVal - targetPosVal) > eps)
            {
                // ROS_INFO_STREAM("Setting position target "<<scopedName<<": "<<target);
                jointController->SetPositionTarget(scopedName, targetPosVal);
            }
            else
            {
                // ROS_INFO_STREAM("Leaving position target "<<scopedName<<": "<<currTargetPosVal);
                jointController->SetPositionTarget(scopedName, currTargetPosVal);
            }
        }
        if (i < _joints->velocity.size())
        {
            double targetVelVal = _joints->velocity[i];

            static double eps = 1e-03;
            bool zeroTargetVel = (fabs(targetVelVal) < eps);

            double targetPosVal = 0;
            if (i < _joints->position.size())
            {
                targetPosVal = _joints->position[i];
            }
            else if (zeroTargetVel)
            {
                ROS_ERROR_STREAM("If you specify 0 velocity for joint " << scopedName
                                 << ", you also have to specify a position for the joint to remain at");
                continue;
            }

            // ROS_INFO_STREAM("JacoGazeboJointStateClient: Setting 'fallback' position of " << scopedName << " to " << targetPosVal);
            // set the position target to the current value to start with. The velocity
            // should overwrite this, but in case the velocity is set to 0, this position
            // should be held.
            jointController->SetPositionTarget(scopedName, targetPosVal);

            // set current target position of joint i, if it is not already achieved:
            double currTargetVelVal = velocities[scopedName];

            if (fabs(currTargetVelVal - targetVelVal) > eps)
            {
                // ROS_INFO_STREAM("JacoGazeboJointStateClient: Setting velocity target "<<scopedName<<": "<<targetVelVal);
                jointController->SetVelocityTarget(scopedName, targetVelVal);
            }
            else
            {
                // ROS_INFO_STREAM("JacoGazeboJointStateClient: Leaving velocity target "<<scopedName<<": "<<targetVelVal);
                jointController->SetVelocityTarget(scopedName, currTargetVelVal);
            }
        }
    }
}



bool JacoGazeboJointStateClient::isGripper(const physics::JointPtr& joint) const
{
    return joints.isGripper(joint->GetName()) || joints.isGripper(joint->GetScopedName());
}



void JacoGazeboJointStateClient::UpdateChild()
{
    ROS_INFO("UpdateChild()");
}

}  // namespace gazebo
