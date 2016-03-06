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


#include <jaco_gazebo/JacoGazeboJointTrajectoryServer.h>
#include <convenience_math_functions/MathFunctions.h>

#include <ros/ros.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>

#include <map>
#include <string>
#include <vector>

#define DEFAULT_JOINT_TRAJECTORY_ACTION_TOPIC "joint_trajectory_action"

#define DEFAULT_TRAJECTORY_POSITION_MODE false 

#define _DEG_TO_RAD M_PI/180.0
#define _RAD_TO_DEG 180.0/M_PI

// velocity limitation (in degrees per second) for joints 1,2,3
#define _MAX_SPEED_123_DEG 10

// velocity limitation (in degrees per second) for joints 4,5,6
#define _MAX_SPEED_456_DEG 20
#define _MAX_SPEED_FINGERS_DEG 20

// default ROS namespace to use for reading
// trajectory parameters from ROS parameter server
#define READ_PARAMS_FROM_ROS_NAMESPACE "/jaco_trajectory"


// if defined, then instead of reading gazebo joint
// velocities with Joint::GetVelocity(), the
// JointVelocityTracker is used.
// #define USE_VELOCITY_TRACKER

using convenience_math_functions::MathFunctions;

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(JacoGazeboJointTrajectoryServer);

JacoGazeboJointTrajectoryServer::JacoGazeboJointTrajectoryServer():
    joints(),  // use default constructor to read from parameters
    finalize(false)
{
    ROS_INFO("Creating JacoGazeboJointTrajectoryServer plugin");
}

JacoGazeboJointTrajectoryServer::~JacoGazeboJointTrajectoryServer()
{
}


void JacoGazeboJointTrajectoryServer::InitActionServer()
{
    std::string rosNamespace = READ_PARAMS_FROM_ROS_NAMESPACE;
    ros::NodeHandle n(rosNamespace);
    ROS_INFO_STREAM("Reading joint trajectory parameters from namespace "<<rosNamespace);

    n.param<std::string>("action_topic", joint_trajectory_action_topic, DEFAULT_JOINT_TRAJECTORY_ACTION_TOPIC);
    // ROS_INFO("got joint trajectory action topic name: <%s>", joint_trajectory_action_topic.c_str());

    double goal_angles_tolerance = 0.01;
    n.param<double>("goal_angles_tolerance", goal_angles_tolerance, goal_angles_tolerance);
    // ROS_INFO("got goal angles tolerance: <%f>", goal_angles_tolerance);

    double intermediateTrajectoryAnglesTolerance = 2 * goal_angles_tolerance;
    n.param<double>("goal_angles_intermediate_tolerance", intermediateTrajectoryAnglesTolerance,
            intermediateTrajectoryAnglesTolerance);

    double angles_safety_limit = -1;
    n.param<double>("angle_safety_limit", angles_safety_limit, angles_safety_limit);
    // ROS_INFO("got goal angles safety limit: <%lf>", angles_safety_limit);

    bool trajectory_position_mode = DEFAULT_TRAJECTORY_POSITION_MODE;
    n.param<bool>("use_angle_poses", trajectory_position_mode, trajectory_position_mode);
    // ROS_INFO("using trajectory position mode: <%i>", trajectory_position_mode);

    bool useOnlineVelocityControl = !DEFAULT_TRAJECTORY_POSITION_MODE;
    n.param<bool>("use_online_control", useOnlineVelocityControl, useOnlineVelocityControl);

    exceed_duration_wait_factor = 1.2;
    n.param<double>("exceed_duration_wait_factor", exceed_duration_wait_factor, exceed_duration_wait_factor);
    // ROS_INFO("got exceed duration wait factor: <%f>", exceed_duration_wait_factor);

    /*if (useOnlineVelocityControl && trajectory_position_mode) {
        ROS_WARN("forcing velocity mode for trajectory execution, because online velocity control is enabled");
        trajectory_position_mode=false;
    }*/

    double maxSpeed123, maxSpeed456, maxSpeedGrippers;
    // velocity limitation (in RAD per second) for joints 1,2,3 and 4,5,6
    n.param<double>("max_arm_speed_123", maxSpeed123, _MAX_SPEED_123_DEG * _DEG_TO_RAD);
    n.param<double>("max_arm_speed_456", maxSpeed456, _MAX_SPEED_456_DEG * _DEG_TO_RAD);
    n.param<double>("max_grippers_speed", maxSpeedGrippers, _MAX_SPEED_FINGERS_DEG * _DEG_TO_RAD);
    // ROS_INFO("Speed limits: {123} = %lf {456} = %lf {Grippers} = %lf", maxSpeed123, maxSpeed456, maxSpeedGrippers);

    std::vector<float> maxVel;
    maxVel.insert(maxVel.end(), 3, maxSpeed123);
    maxVel.insert(maxVel.end(), 3, maxSpeed456);
    maxVel.insert(maxVel.end(), 3, maxSpeedGrippers);

    trajectoryPos.resize(9, 0);
    trajectoryVel.resize(9, 0);
    currentAngles.resize(9, 0);
    currentVels.resize(9, 0);
    bool simplifyTrajectories = true;

    trajectory_action_server = JacoTrajectoryActionServerPtr(
                                   new JacoTrajectoryActionServer(
                                       nh, joint_trajectory_action_topic,
                                       trajectoryPos, trajectoryVel,
                                       currentAngles, currentVels,
                                       data_lock,
                                       trajectory_position_mode,
                                       maxVel,
                                       angles_safety_limit,
                                       goal_angles_tolerance,
                                       simplifyTrajectories,
                                       useOnlineVelocityControl,
                                       intermediateTrajectoryAnglesTolerance));

    if (!trajectory_action_server->init())
    {
        ROS_ERROR("Error initializing joint trajectory action server");
    }
    /*else
    {
        ROS_INFO("Joint trajectory action server running.");
    }*/
}


void JacoGazeboJointTrajectoryServer::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    physics::BasePtr jcChild = _parent->GetChild(physics::JointControllerThreadsafe::UniqueName());
    if (!jcChild.get())
    {
        ROS_ERROR("Cannot load JacoGazeboJointTrajectoryServer if no default JointControllerThreadsafe is set for the model");
        throw std::string("Cannot load JacoGazeboJointTrajectoryServer if no default JointController is set for the model");
    }
    physics::JointControllerThreadsafePtr ptr = boost::dynamic_pointer_cast<physics::JointControllerThreadsafe>(jcChild);
    if (!ptr.get())
    {
        ROS_ERROR_STREAM("Cannot load JacoGazeboJointTrajectoryServer if child '"
                         << physics::JointControllerThreadsafe::UniqueName()
                         << "' is not of class JointControllerThreadsafe");
        throw std::string("Cannot load JacoGazeboJointTrajectoryServer if child '"
                          + physics::JointControllerThreadsafe::UniqueName()
                          + "' is not of class JointControllerThreadsafe");
    }
    jointController = ptr;

    // XXX TODO check names maintained in JacoJointManager with loop below

    std::vector<std::string> joint_names;
    std::string prepend = "";
    joints.getJointNames(joint_names, true, prepend);

    // Print the joint names to help debugging
    std::map<std::string, physics::JointPtr > jntMap = jointController->GetJoints();

    // check if the joint names maintained in 'joints' match the names in gazebo,
    // that the joints can be used by this class.
    // If yes, do any initializations that may be necessary as well.
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
       
#ifdef USE_VELOCITY_TRACKER
        int axis=0; 
        velTracker.add(*it,&joint);
#endif
        ++i;
    }

    model = _parent;

    // update_connection =
    //  event::Events::ConnectWorldUpdateBegin(boost::bind(&JacoGazeboJointControl::WorldUpdate, this));
    update_connection =
        nh.createTimer(ros::Duration(1.0 / 1000.0), &JacoGazeboJointTrajectoryServer::WorldUpdate, this);

    InitActionServer();
}



// void JacoGazeboJointControl::WorldUpdate()
void JacoGazeboJointTrajectoryServer::WorldUpdate(const ros::TimerEvent& t)
{
    std::vector<std::string> joint_names;
    std::string prepend = "";
    joints.getJointNames(joint_names, true, prepend);
   
#ifdef USE_VELOCITY_TRACKER
    // update velocity tracker so that we can read the actual physical
    // joint velocity averaged over a brief time. 
    for (std::vector<std::string>::iterator it = joint_names.begin();
            it != joint_names.end(); ++it)
    {
        int axis=0;
        gazebo::physics::JointPtr joint;
        if (!velTracker.getJoint(*it,joint) || !joint.get())
        {
            ROS_ERROR_STREAM("Inconsistency: Joint "<<*it<<" should have been added to velocity tracker.");
            throw std::string("Inconsistency: Joint should have been added to velocity tracker.");
        } 
        velTracker.update(*it, joint->GetAngle(axis).Radian(), ros::Time::now());
    }
#endif

     // read the current joint values and store in vector
    data_lock.lock();
    JacoGazeboJointTrajectoryServer::readJointStates(currentAngles, currentVels);
    data_lock.unlock();


    // set current joint positions in trajectory

    // ROS_INFO_STREAM("Executing trajectory goal!");

    if (!jointController.get())
    {
        ROS_ERROR("Cannot load JacoGazeboJointTrajectoryServer if no JointController is set for the model");
        return;
    }


    if (!trajectory_action_server->goalActive())
    {
        // no trajectory is being executed, so there is no need to update

        // however it may be necessary to ensure the last trajectory point positions
        // are finalised, because now all velocities will be zero. Must at least
        // once ensure current target positions are set in the controller.
        if (finalize) {
            // ROS_INFO_STREAM("Finalizing trajectory");

            boost::unique_lock<boost::recursive_mutex> lck = jointController->GetLock();
            // clear out all possible current targets. Necessary e.g. to clear out
            // exitsing velocity targets, when only positions are set now.
            // This is safe to use here because we are setting new targets for *all*
            // joints of the arm.
            jointController->Reset();
            int i = 0;
            for (std::vector<std::string>::iterator it = joint_names.begin();
                    it != joint_names.end(); ++it)
            {
                // ROS_INFO_STREAM("Local joint name: '"<<*it<<"'");
                physics::JointPtr joint = model->GetJoint(*it);
                if (!joint.get())
                {
                    ROS_FATAL_STREAM("Joint name " << *it << " not found as robot joint");
                    throw std::string("Joint not found");
                }

                std::string scopedName = joint->GetScopedName();
                data_lock.lock();
                // ROS_INFO_STREAM("JacoGazeboJointTrajectoryServer: Setting final position target "<<scopedName<<": "<<trajectoryPos[i]);
                if (!jointController->SetPositionTarget(scopedName, MathFunctions::capToPI(trajectoryPos[i])))
                {
                    ROS_ERROR_STREAM_ONCE("Could not set position target for "<<joint->GetName()<<
                           ". Can't control joint. This message is only printed once.");
                }
                data_lock.unlock();
                ++i;
            }
            finalize=false;
        } 
        return;
    }

 
    finalize=true;

    boost::unique_lock<boost::recursive_mutex> lck = jointController->GetLock();


    // clear out all possible current targets. Necessary e.g. to clear out
    // exitsing velocity targets, when only positions are set now...
    // or because we moved on to next trajectory point and the positions
    // are not valid any more. It's ok to use here because we are setting new
    // targets for every joint anyway.
    jointController->Reset();

    // check if the joint names maintained in 'joints' match the names in gazebo,
    // that the joints can be used by this class, and if yes, load PID controllers.
    int i = 0;
    for (std::vector<std::string>::iterator it = joint_names.begin();
            it != joint_names.end(); ++it)
    {
        // ROS_INFO_STREAM("Local joint name: '"<<*it<<"'");

        physics::JointPtr joint = model->GetJoint(*it);
        if (!joint.get())
        {
            ROS_FATAL_STREAM("Joint name " << *it << " not found as robot joint");
            throw std::string("Joint not found");
        }

        std::string scopedName = joint->GetScopedName();
        // ROS_INFO("Executing %f %f",trajectoryPos[0],trajectoryPos[1]);
        bool usePosition = trajectory_action_server->usePositionMode();
        data_lock.lock();
        if (usePosition)
        {
            // if (fabs(trajectoryPos[i])>0.01) ROS_INFO_STREAM("JacoGazeboJointTrajectoryServer: Setting position target "<<scopedName<<": "<<trajectoryPos[i]);
            if (!jointController->SetPositionTarget(scopedName, MathFunctions::capToPI(trajectoryPos[i])))
            {
                ROS_ERROR_STREAM_ONCE("Could not set position target for "<<joint->GetName()<<
                       ". Can't control joint. This message is only printed once.");
            }
        }
        else
        {
            // if (fabs(trajectoryVel[i])>0.2) ROS_INFO_STREAM("JacoGazeboJointTrajectoryServer: Setting velocity target "<<scopedName<<": "<<trajectoryVel[i]);
            if (!jointController->SetVelocityTarget(scopedName, trajectoryVel[i]))
            {
                ROS_ERROR_STREAM_ONCE("Could not set velocity target for "<<joint->GetName()<<
                       ". Can't control joint. This message is only printed once.");
            }
            // with 0 velocities, also have to maintain the correct arm pose:
            if (fabs(trajectoryVel[i]<1e-04))
            {
                // ROS_INFO_STREAM("JacoGazeboJointTrajectoryServer: Setting position target "<<scopedName<<": "<<currentAngles[i]);
                
                // if the velocity is 0, we can assume that we either are at the next trajectory point already,
                // or this joint didn't move in between trajectory points. Therefore, we can use the next trajectory
                // pose as joint position value. 
                // NOTE: If we use currentAngles[i] here, the arm will slowly collapse. This is because small fluctuations in
                // the current angle values. This is random as the current angles can very well be equivalent
                // to the target position, but more often than not, they are a tiny bit off, caused by the gravity pulling down
                // the arm just a tiny bit. Then setting these wrong new targets will amplify over multiple iterations.
                // jointController->SetPositionTarget(scopedName,MathFunctions::capToPI(currentAngles[i]));
                if (!jointController->SetPositionTarget(scopedName,MathFunctions::capToPI(trajectoryPos[i])))
                {
                    ROS_ERROR_STREAM_ONCE("Could not set position target for "<<joint->GetName()<<
                           ". Can't control joint. This message is only printed once.");
                }
            }
        }
        data_lock.unlock();
        ++i;
    }
}



void JacoGazeboJointTrajectoryServer::readJointStates(std::vector<float>& currAngles, std::vector<float>& currVels)
{
    currAngles.resize(9, 0);
    currVels.resize(9, 0);

    gazebo::physics::Joint_V::const_iterator it;
    for (it = model->GetJoints().begin(); it != model->GetJoints().end(); ++it)
    {
        physics::JointPtr joint = *it;
        std::string jointName = joint->GetName();

        // ROS_INFO("Getting %s",jointName.c_str());

        int armJointNumber = joints.armJointNumber(jointName);
        int gripperJointNumber = joints.gripperJointNumber(jointName);

        bool jacoJoint = (armJointNumber >= 0) || (gripperJointNumber >= 0);
        if (!jacoJoint) continue;

        unsigned int axis = 0;
        if (joint->GetAngleCount() != 1)
        {
            ROS_FATAL("Only support 1 axis");
            exit(1);
        }

        double currAngle = joint->GetAngle(axis).Radian();
        currAngle = MathFunctions::capToPI(currAngle);

        // double currEff=joint->GetForce(axis);
        double currVel = joint->GetVelocity(axis);

#ifdef USE_VELOCITY_TRACKER
        double currGzbVel = currVel;
        if (velTracker.getJointVelocity(jointName, currVel) < 0)
        {
            ROS_ERROR_STREAM("Error obtaining joint velocity for joint "<<jointName);
            // continue;
        }
        // if (fabs(currGzbVel)>0.1) ROS_INFO_STREAM("Velocity compare "<<jointName<<": g="<<currGzbVel<<", t="<<currVel);
#endif
        
        // if (fabs(currVel>0.1)) ROS_INFO_STREAM("Velocity "<<jointName<<": "<<currVel);

        if (gripperJointNumber >= 0)
        {
            // Save gripper angles in radians
            currAngles[6 + gripperJointNumber] = currAngle;
            currVels[6 + gripperJointNumber] = currVel;
        }
        else if (armJointNumber >= 0)
        {
            // Save arm angles in radians
            currAngles[armJointNumber] = currAngle;
            currVels[armJointNumber] = currVel;
        }
    }
}


bool JacoGazeboJointTrajectoryServer::isGripper(const physics::JointPtr& joint) const
{
    return joints.isGripper(joint->GetName()) || joints.isGripper(joint->GetScopedName());
}



void JacoGazeboJointTrajectoryServer::UpdateChild()
{
    ROS_INFO("UpdateChild()");
}

}  // namespace gazebo
