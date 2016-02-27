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

#include <jaco_gazebo/JacoGazeboJointControl.h>
#include <jaco_gazebo/JacoGazeboControlConstants.h>

#include <ros/ros.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>

#include <map>
#include <string>
#include <algorithm>
#include <vector>

// A value of 5 for MAX_FORCE proved to work with high
// KP values so the hand could just stay up. So shouldn't
// be much higher than that...
#define DEFAULT_MAX_FORCE 5
#define DEFAULT_MAX_FORCE_FINGER 2.5

// maximum velocity for arm joints (rads/sec)
#define DEFAULT_MAX_VEL 1.5
// maximum velocity for arm finger (rads/sec)
#define DEFAULT_MAX_VEL_FINGER 1.5

#define KP_POS 200
#define KI_POS 10
#define KD_POS 1

#define KP_VEL 10000
#define KI_VEL 100
#define KD_VEL 0

#define INIT_WITH_POS false

#define DISABLE_GRAVITY false

#define USE_FORCE true

// don't use home pose as initial pose, but 0 angles instead
// #define USE_0_INIT_POSE

#define UPDATE_RATE 1000

// the major version of Gazebo from which on 
// SetVelocityLimits() works
#define GAZEBO_MAJOR_MAXVALS_WORKING 3



namespace gazebo
{

using physics::JointControllerThreadsafe;

JacoGazeboJointControl::JacoGazeboJointControl():
    joints(),  // use default constructor to read from parameters
    nh(""),
    loadedVelocityControllers(false)
{
    ROS_INFO("Creating JacoGazeboJointControl plugin");

}

JacoGazeboJointControl::~JacoGazeboJointControl()
{
}

void JacoGazeboJointControl::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
    
    bool loadVelocityControllers = true;
    nh.param<bool>("gazebo/load_velocity_controllers",loadVelocityControllers,loadVelocityControllers);

    physics::JointControllerPtr _modelJointController = _parent->GetJointController();
    if (!_modelJointController.get())
    {
        ROS_ERROR("Cannot load JacoGazeboJointControl if no default JointController is set for the model");
        throw std::string("Cannot load JacoGazeboJointControl if no default JointController is set for the model");
    }

    JointControllerThreadsafe::JointControllerImplPtr _newControllerPtr(new JointControllerThreadsafe::JointControllerImpl(_parent));
    jointController = physics::JointControllerThreadsafePtr(
                          new JointControllerThreadsafe(_parent, _newControllerPtr));
    //jointController = physics::JointControllerThreadsafePtr(
    //                      new physics::JointControllerThreadsafe(_parent, _modelJointController));

    // get joint names from parameters
    std::vector<std::string> joint_names;
    joints.getJointNames(joint_names, true);
    const std::vector<float>& arm_init = joints.getArmJointsInitPose();
    const std::vector<float>& finger_init = joints.getFingerJointsInitPose();

    if (joint_names.size() != 9)
    {
        ROS_ERROR("JacoGazeboJointControl: joint names have to be of size 9!");
        throw std::string("JacoGazeboJointControl: joint names have to be of size 9!");
    }

#if GAZEBO_MAJOR_VERSION >= GAZEBO_ADVANCED_JOINTCONTROLLER_VERSION
    // TODO take this out, not really needed any more as we are using a new Jointcontroller anyway
    // Print the joint names to help debugging
    std::map<std::string, physics::JointPtr > jntMap = _modelJointController->GetJoints();
    for (std::map< std::string, physics::JointPtr>::iterator it = jntMap.begin(); it != jntMap.end(); ++it)
    {
        physics::JointPtr j = it->second;
        ROS_INFO_STREAM("Gazebo joint: '" << j->GetName() << "' is registered as '" << it->first << "'");
    }
#endif

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

        if (joint->GetAngleCount() != 1)
        {
            ROS_FATAL("JacoGazeboJointControl: joints must have exactly one axis");
            throw std::string("JacoGazeboJointControl: joints must have exactly one axis");
        }


        jointController->AddJoint(joint);
        
        bool isFinger = i > 5;

        float max_force, max_velocity;
        GetMaxVals(*it, max_force, max_velocity);

        ROS_INFO_STREAM("Setting max vals for joint " << *it << ": f=" << max_force << ", v=" << max_velocity);
        joint->SetEffortLimit(0, max_force);

#if GAZEBO_MAJOR_VERSION >= GAZEBO_JADE_VERSION
        joint->SetVelocityLimit(0, max_velocity);
#endif

#if GAZEBO_MAJOR_VERSION > 2
        // joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
        // going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
        // going to be called.
        // XXX TODO investigate: This was deprecated in ODE gazebo 5? Also works without it..
        if (!UseForce()) {
            joint->SetParam("fmax", 0, max_force);
        }
#else
        joint->SetMaxForce(0,max_force);
#endif
        
        if (DisableGravity())
        {
            physics::LinkPtr link = joint->GetChild();
            // physics::LinkPtr link = joint->GetJointLink(0);
            if (link.get())
            {
                ROS_INFO_STREAM("Disable gravity for link " << link->GetName());
                link->SetGravityMode(false);
            }
        }

        float max_val = UseForce() ? max_force : max_velocity;
        
        std::string scopedName = joint->GetScopedName();

        float imin = 0;
        float imax = 0;
        float kp, ki, kd;
        GetPosGains(*it, kp, ki, kd);
        jointController->SetPositionPID(scopedName, gazebo::common::PID(kp, ki, kd, imin, imax, max_val, -max_val));

        if (loadVelocityControllers)
        {
            GetVelGains(*it, kp, ki, kd);
            //imax = 0.2;
            jointController->SetVelocityPID(scopedName, gazebo::common::PID(kp, ki, kd, imin, imax, max_val, -max_val));
        }

        double initVal = isFinger ? finger_init[i - 6] : arm_init[i];
        initVal = joints.capToPI(initVal);

        ROS_INFO_STREAM("Setting initial position for " << joint_names[i] << ": " << initVal);

        double lowLimit = joint->GetLowerLimit(0).Radian();
        double upLimit = joint->GetUpperLimit(0).Radian();
        // ROS_INFO("Joint limits: %f, %f",lowLimit,upLimit);
        if (initVal > upLimit) initVal = upLimit;
        if (initVal < lowLimit) initVal = lowLimit;

#ifdef USE_0_INIT_POSE
        initVal = 0;
#endif

        jointController->SetJointPosition(scopedName, initVal);

        if (!jointController->SetPositionTarget(scopedName, initVal))
        {
            ROS_ERROR_STREAM("Could not set position target for " << joint_names[i]);
        }
        ROS_INFO_STREAM("Set position target for " << joint_names[i]);
        ++i;
    }

    _parent->AddChild(jointController);
    model = _parent;

    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&JacoGazeboJointControl::WorldUpdate, this));
    // ros::NodeHandle nh;
    // update_connection = nh.createTimer(ros::Duration(1.0 / UPDATE_RATE), &JacoGazeboJointControl::WorldUpdate, this);
   
    loadedVelocityControllers = loadVelocityControllers;
}

double JacoGazeboJointControl::capTargetVel(const physics::JointPtr joint, const float targetVel,
        const bool useEndTargetPos, const float endTargetPos, const float tolerance) const
{
    if (joint.get() == NULL)
    {
        ROS_FATAL("Passing NULL joint!");
        return targetVel;
    }

    float velocity = targetVel;
    int axis = 0;

#if GAZEBO_MAJOR_VERSION > GAZEBO_MAJOR_MAXVALS_WORKING
    float maxVel = fabs(joint->GetVelocityLimit(axis));  
    float maxForce = fabs(joint->GetEffortLimit(axis));
#else
    float maxForce, maxVel;
    GetMaxVals(joint->GetName(), maxForce, maxVel);
  // double maxForce=fabs(joint->GetMaxForce(axis));
#endif
  // ROS_INFO_STREAM("Max vel "<<joint->GetName()<<": "<<maxVel);
  // ROS_INFO_STREAM("Max f "<<joint->GetName()<<": "<<maxForce);

    double distToTargetPos = 0;
    bool closeToTargetPos = false;

    double currAngle = joint->GetAngle(axis).Radian();

    if (useEndTargetPos)
    {
        float target = joints.capToPI(endTargetPos);

        double _currAngle = joints.capToPI(currAngle);
        distToTargetPos = (target - _currAngle);
        distToTargetPos = joints.capToPI(distToTargetPos);

        closeToTargetPos = fabs(distToTargetPos) < tolerance;
    }

    if (closeToTargetPos)
    {
//        ROS_INFO_STREAM("Close to target: "<<joint->GetName()<<" low, pos "<<currAngle);
        velocity = 0;
    }

    velocity = std::min(velocity, maxVel);
    velocity = std::max(velocity, -maxVel);

    // Now, prevent the joints to get close to any possible limits, because
    // Gazebo then has weird issues.

    double lowLimit = joint->GetLowerLimit(axis).Radian();
    double upLimit = joint->GetUpperLimit(axis).Radian();
    // ROS_INFO_STREAM("Low limit "<<joint->GetName()<<": "<<lowLimit<<" curr: "<<currAngle);
    // ROS_INFO_STREAM("High limit "<<joint->GetName()<<": "<<upLimit<<" curr: "<<currAngle);
    // ROS_INFO_STREAM("Current vel: "<<velocity);

    // XXX TODO. This is not a safe hack to prevent getting too close, because it does not consider
    // the update rate. Setting a high velocity when update rate is low
    // can very well exceed the joint limits.
    if ((fabs(currAngle - lowLimit) < JOINTLIMIT_PADDING) && (velocity < 0))
    {
        // ROS_INFO_STREAM("Skipping "<<joint->GetName()<<" low, pos "<<currAngle);
        velocity = 0;
    }
    if ((fabs(upLimit - currAngle) < JOINTLIMIT_PADDING) && (velocity > 0))
    {
        // ROS_INFO_STREAM("Skipping "<<joint->GetName()<<" high, pos "<<currAngle);
        velocity = 0;
    }

    return velocity;
}



double JacoGazeboJointControl::capTargetForce(const physics::JointPtr joint, const float targetForce, const bool considerJointLimits) const
{
    if (joint.get() == NULL)
    {
        ROS_FATAL("Passing NULL joint!");
        return targetForce;
    }

    float force = targetForce;
    int axis = 0;
  
#if GAZEBO_MAJOR_VERSION > GAZEBO_MAJOR_MAXVALS_WORKING
    float maxVel = fabs(joint->GetVelocityLimit(axis));  
    float maxForce = fabs(joint->GetEffortLimit(axis));
#else
    float maxForce, maxVel;
    GetMaxVals(joint->GetName(), maxForce, maxVel);
    // double maxForce=fabs(joint->GetMaxForce(axis));
#endif
    // ROS_INFO_STREAM("Max vel "<<joint->GetName()<<": "<<maxVel);
    // ROS_INFO_STREAM("Max f "<<joint->GetName()<<": "<<maxForce);

    force = std::min(force, maxForce);
    force = std::max(force, -maxForce);
    
    double currAngle = joint->GetAngle(axis).Radian();

    // Now, prevent the joints to get close to any possible limits, because
    // Gazebo then has weird issues.

    double lowLimit = joint->GetLowerLimit(axis).Radian();
    double upLimit = joint->GetUpperLimit(axis).Radian();
    // ROS_INFO_STREAM("Low limit "<<joint->GetName()<<": "<<lowLimit<<" curr: "<<currAngle);
    // ROS_INFO_STREAM("High limit "<<joint->GetName()<<": "<<upLimit<<" curr: "<<currAngle);
    // ROS_INFO_STREAM("Current vel: "<<force);

    // XXX TODO. This is not a safe hack to prevent getting too close, because actually
    // an opposite force may have to be applied in order for the joint not to "fall down" beyond
    // its limits. But this would need to be handled by PID controller...
    if ((fabs(currAngle - lowLimit) < JOINTLIMIT_PADDING) && (force < 0))
    {
        // ROS_INFO_STREAM("Skipping "<<joint->GetName()<<" low, pos "<<currAngle);
        force = 0;
    }
    if ((fabs(upLimit - currAngle) < JOINTLIMIT_PADDING) && (force > 0))
    {
        // ROS_INFO_STREAM("Skipping "<<joint->GetName()<<" high, pos "<<currAngle);
        force = 0;
    }

    return force;
}




bool JacoGazeboJointControl::UpdateJoints()
{
    if (!UseForce())
    {
        ROS_FATAL_STREAM("Cannot use the base class JacoGazeboJointControl in velocity mode. Always force is used.");
        return false;
    }

    if (!model.get())
    {
        ROS_ERROR("Cannot update JacoGazeboJointControl if no model is set");
        return false;
    }
    if (!jointController.get())
    {
        ROS_ERROR("Cannot load JacoGazeboJointControl if no default JointController is set for the model");
        return false;
    }
    jointController->Update();
    return true;
}


void JacoGazeboJointControl::WorldUpdate()
//void JacoGazeboJointControl::WorldUpdate(const ros::TimerEvent& t)
{
    if (!UpdateJoints())
    {
        throw std::string("Cannot update JacoGazeboJointControl");
    }
}


void JacoGazeboJointControl::UpdateChild()
{
    ROS_INFO("UpdateChild()");
}


bool JacoGazeboJointControl::isFinger(const physics::JointPtr& joint) const
{
    return joints.isFinger(joint->GetName()) || joints.isFinger(joint->GetScopedName());
}

bool JacoGazeboJointControl::DisableGravity() const
{
    return DISABLE_GRAVITY;
}

bool JacoGazeboJointControl::UseForce() const
{
    return USE_FORCE;
}

void JacoGazeboJointControl::GetDefaultPosGains(float& kp, float& ki, float& kd) const
{
    kp = KP_POS;
    ki = KI_POS;
    kd = KD_POS;
}

void JacoGazeboJointControl::GetDefaultVelGains(float& kp, float& ki, float& kd) const
{
    kp = KP_VEL;
    ki = KI_VEL;
    kd = KD_VEL;
}


void JacoGazeboJointControl::GetPosGains(const std::string& jointName, float& kp, float& ki, float& kd) const
{
    GetDefaultPosGains(kp, ki, kd);
    if (!joints.GetPosGains(jointName, kp, ki, kd))
    {
        ROS_ERROR_STREAM("Joint " << jointName << " not maintained by the joint manager");
    }
    ROS_INFO_STREAM("Using position PID values for joint " << jointName << ": p=" << kp << ", i=" << ki << ", d=" << kd);
}


void JacoGazeboJointControl::GetVelGains(const std::string& jointName, float& kp, float& ki, float& kd) const
{
    GetDefaultVelGains(kp, ki, kd);
    if (!joints.GetVelGains(jointName, kp, ki, kd))
    {
        ROS_ERROR_STREAM("Joint " << jointName << " not maintained by the joint manager");
    }

    ROS_INFO_STREAM("Using velocity PID values for joint " << jointName << ": p=" << kp << ", i=" << ki << ", d=" << kd);
}

void JacoGazeboJointControl::GetMaxVals(const std::string& jointName, float& force, float& velocity) const
{
    bool isFinger = joints.isFinger(jointName);

    // initialize default values first:
    force = isFinger ? DEFAULT_MAX_FORCE_FINGER : DEFAULT_MAX_FORCE;
    velocity = isFinger ? DEFAULT_MAX_VEL_FINGER : DEFAULT_MAX_VEL;

    // now read from config file
    const std::vector<std::string>& arm_joints = joints.getArmJoints();

//    ROS_INFO_STREAM("Joint "<<jointName<<" is finger: "<<isFinger);

    if (isFinger)
    {
        force = MAX_FINGER_EFFORT;
        velocity = MAX_FINGER_VELOCITY;
    }
    else if (jointName == arm_joints[0])
    {
        force = MAX_ARM_0_EFFORT;
        velocity = MAX_ARM_0_VELOCITY;
    }
    else if (jointName == arm_joints[1])
    {
        force = MAX_ARM_1_EFFORT;
        velocity = MAX_ARM_1_VELOCITY;
    }
    else if (jointName == arm_joints[2])
    {
        force = MAX_ARM_2_EFFORT;
        velocity = MAX_ARM_2_VELOCITY;
    }
    else if (jointName == arm_joints[3])
    {
        force = MAX_ARM_3_EFFORT;
        velocity = MAX_ARM_3_VELOCITY;
    }
    else if (jointName == arm_joints[4])
    {
        force = MAX_ARM_4_EFFORT;
        velocity = MAX_ARM_4_VELOCITY;
    }
    else if (jointName == arm_joints[5])
    {
        force = MAX_ARM_5_EFFORT;
        velocity = MAX_ARM_5_VELOCITY;
    }
}

}  // namespace gazebo
