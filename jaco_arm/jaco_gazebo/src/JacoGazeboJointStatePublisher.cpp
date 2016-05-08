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


#include <jaco_gazebo/JacoGazeboJointStatePublisher.h>
#include <convenience_math_functions/MathFunctions.h>

#include <ros/ros.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>

#include <string>
#include <vector>

#define DEFAULT_JOINT_STATE_TOPIC "jaco/joint_control"


// XXX HACK: We need to publish values NOT between -PI and PI for arm joints 1 and 2, because
// its lower limit is < -PI. When using Gazebo, it was not possible to use lower limit > higher
// limit in the URDF, e.g. 2.5..0.73.
// MoveIt however will detect an angle of 3.0 as invalid if we have limits such as -3.9..0.73.
// So we need to publish angles between -3.9..0.73. This has to be done for all revolute joints
// with such limits where lower > higher.
#define DO_JOINT_1_2_PUBLISH_FIX



// set to true if all joints are to be published.
// If set to false, only the arm joints are published, not including gripper joints.
#define PUBLISH_ALL_JOINT_STATES true

using convenience_math_functions::MathFunctions;

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(JacoGazeboJointStatePublisher);

JacoGazeboJointStatePublisher::JacoGazeboJointStatePublisher():
    joints(),  // use default constructor to read from parameters
    jointStateTopic(DEFAULT_JOINT_STATE_TOPIC),
    publishAllJoints(PUBLISH_ALL_JOINT_STATES)
{
    ROS_INFO("Creating JacoGazeboJointStatePublisher plugin");
    nh.param("jaco_joint_states_topic", jointStateTopic, jointStateTopic);
    ROS_INFO_STREAM("Joint state publish topic: " << jointStateTopic);
}

JacoGazeboJointStatePublisher::~JacoGazeboJointStatePublisher()
{
}



void JacoGazeboJointStatePublisher::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    // get joint names from parameters
    std::vector<std::string> joint_names;
    joints.getJointNames(joint_names, true);
    const std::vector<float>& arm_init = joints.getArmJointsInitPose();
    const std::vector<float>& gripper_init = joints.getGripperJointsInitPose();

    if (joint_names.size() != 9)
    {
        ROS_ERROR("JacoGazeboJointStatePublisher: joint names have to be of size 9!");
        throw std::string("JacoGazeboJointStatePublisher: joint names have to be of size 9!");
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

        ++i;
    }

    model = _parent;
    jsPub = nh.advertise<sensor_msgs::JointState>(jointStateTopic, 1000);
    update_connection =
        event::Events::ConnectWorldUpdateBegin(boost::bind(&JacoGazeboJointStatePublisher::WorldUpdate, this));
}



void JacoGazeboJointStatePublisher::WorldUpdate()
{
    sensor_msgs::JointState js;
    readJointStates(js);
    jsPub.publish(js);
}


void JacoGazeboJointStatePublisher::readJointStates(sensor_msgs::JointState& js)
{
    // Add timestamp to message (so robot_state_publisher does not ignore it)
    js.header.stamp = ros::Time::now();
    js.header.frame_id = "world";

    gazebo::physics::Joint_V::const_iterator it;
    for (it = model->GetJoints().begin(); it != model->GetJoints().end(); ++it)
    {
        physics::JointPtr joint = *it;
        std::string _jointName = joint->GetName();

        // ROS_INFO("Getting %s",_jointName.c_str());

        int armJointNumber = joints.armJointNumber(_jointName);
        int gripperJointNumber = joints.gripperJointNumber(_jointName);

        unsigned int axis = 0;
        if (joint->GetAngleCount() != 1)
        {
            ROS_FATAL("Only support 1 axis");
            exit(1);
        }

        double currAngle = joint->GetAngle(axis).Radian();
        currAngle = MathFunctions::capToPI(currAngle);

#ifdef DO_JOINT_1_2_PUBLISH_FIX
        // XXX TODO use convenience_math_functions::MathFunctions::limitsToTwoPI instead!
        if ((armJointNumber == 1) || (armJointNumber == 2))
        {
            // simply overwrite the "capToPi" correction from above
            currAngle = joint->GetAngle(axis).Radian();
        }
#endif

        double currEff = joint->GetForce(axis);
        double currVel = joint->GetVelocity(axis);

        // ROS_INFO("Joint %s (%u) %f %f %f", _jointName.c_str(), i, currAngle, currEff, currVel);

        bool isJacoJoint = (gripperJointNumber >= 0) || (armJointNumber >= 0);

        if (publishAllJoints || isJacoJoint)
        {
            js.name.push_back(_jointName);
            js.position.push_back(currAngle);
            js.velocity.push_back(currVel);
            js.effort.push_back(currEff);
        }
    }
}


/*
void readJointStates(jaco_msgs::JointAngles& as, jaco_msgs::GripperPosition& fs) {
    gazebo::physics::Joint_V::const_iterator it;
    for (it=model->GetJoints().begin(); it!=model->GetJoints().end(); ++it) {
        physics::JointPtr joint = *it;
        std::string _jointName=joint->GetName();

        //ROS_INFO("Getting %s",_jointName.c_str());

        int armJointNumber=joints.armJointNumber(_jointName);
        int gripperJointNumber=joints.gripperJointNumber(_jointName);

        unsigned int axis=0;
        if (joint->GetAngleCount()!=1) {
            ROS_FATAL("Only support 1 axis");
            exit(1);
        }

        double currAngle=joint->GetAngle(axis).Radian();
        currAngle=MathFunctions::capToPI(currAngle);

// XXX TODO Check again: do we need this for real jaco arm too?
#ifdef DO_JOINT_1_2_PUBLISH_FIX
        // XXX check if we need to do the fix here too. Copy from other readJointStates().
#endif
        if ((armJointNumber >=0) || (gripperJointNumber>=0)) {
            //ROS_INFO("Joint %s (%u) %f %f %f", _jointName.c_str(), i, currAngle, currEff, currVel);
            // Save gripper angles in radians
            if (gripperJointNumber==0) { fs.gripper1=currAngle; }
            else if (gripperJointNumber==1) { fs.gripper2=currAngle; }
            else if (gripperJointNumber==2) { fs.gripper3=currAngle; }
            else if (armJointNumber==0) { as.joint1=currAngle; }
            else if (armJointNumber==1) { as.joint2=currAngle; }
            else if (armJointNumber==2) { as.joint3=currAngle; }
            else if (armJointNumber==3) { as.joint4=currAngle; }
            else if (armJointNumber==4) { as.joint5=currAngle; }
            else if (armJointNumber==5) { as.joint6=currAngle; }
            else ROS_ERROR("Consistency in gazebo plugin!");

        }
    }
}
*/



bool JacoGazeboJointStatePublisher::isGripper(const physics::JointPtr& joint) const
{
    return joints.isGripper(joint->GetName()) || joints.isGripper(joint->GetScopedName());
}



void JacoGazeboJointStatePublisher::UpdateChild()
{
    ROS_INFO("UpdateChild()");
}

}  // namespace gazebo
