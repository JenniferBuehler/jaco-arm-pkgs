#ifndef JACO_GAZEBO_JACOGAZEBOJOINTTRAJECTORYCLIENT_H
#define JACO_GAZEBO_JACOGAZEBOJOINTTRAJECTORYCLIENT_H

/**
   Gazebo plugin which uses JacoTrajectoryActionServer (in package jaco_joints) to execute trajectories.

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
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <jaco_joints/JacoJointManager.h>
#include <jaco_joints/JacoTrajectoryActionServer.h>
#include <jaco_joints/JointVelocityTracker.hpp>

#include <jaco_gazebo/JointController.h>

#include <boost/thread/mutex.hpp>

namespace gazebo
{


/**
 * \brief Gazebo plugin which uses JacoTrajectoryActionServer (in package jaco_joints) to execute trajectories.
 *
 * **Requirements**
 *
 * Requires another gazebo plugin running which adds a gazebo::physics::JointControllerThreadsafe instance as a child to the model
 * and controls the joints. This can be for example an instance of gazebo::JacoGazeboJointControl.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class JacoGazeboJointTrajectoryServer : public ModelPlugin
{
public:
    JacoGazeboJointTrajectoryServer();

    virtual ~JacoGazeboJointTrajectoryServer();

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
    /**
     */
    void InitActionServer();

    // void WorldUpdate();
    void WorldUpdate(const ros::TimerEvent& t);

    void readJointStates(std::vector<float>& currAngles, std::vector<float>& currVels);

    JacoTrajectoryActionServerPtr trajectory_action_server;

    // target trajectory positions
    std::vector<float> trajectoryPos;
    std::vector<float> trajectoryVel;

    // current angles of arm (size 9, one value for each float)
    std::vector<float> currentAngles;
    std::vector<float> currentVels;

    // to lock access to the fields trajectoryPos/trajectoryVel
    // and currentAngles/currentVels
    boost::mutex data_lock;

    // to track the joint velocities coming from Gazebo.
    // this is to smooth the velocity returned from
    // gazebo::physics::Joint and correct possible inaccuracies.
    // It may not be used by the actual implementation.
    JointVelocityTracker<gazebo::physics::JointPtr> velTracker;

    // flag used to determine when a trajectory has finished playing
    // and one last update of the joint positions is required,
    // when all velocities are zero.
    bool finalize;

    // this is where gazebo will provide the joint trajectory action
    std::string joint_trajectory_action_topic;

    // tolerance for target angles to be considered reached.
    // double goal_angles_tolerance;

    // factor which is allowed on trajectory execution time (vs. expected time)
    double exceed_duration_wait_factor;

    ros::NodeHandle nh;

    // XXX TODO alternatively to using jointController to set targets,
    // it would be good to provide an alternative to send out
    // JointState messages
    physics::JointControllerThreadsafePtr jointController;

//    event::ConnectionPtr update_connection;
    ros::Timer update_connection;
};

}  // namespace gazebo

#endif  // JACO_GAZEBO_JACOGAZEBOJOINTTRAJECTORYCLIENT_H
