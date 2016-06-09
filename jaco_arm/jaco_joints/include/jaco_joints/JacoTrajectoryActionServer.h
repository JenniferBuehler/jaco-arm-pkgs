#ifndef JACO_JOINTS_TRAJECTORYACTIONSERVER_H
#define JACO_JOINTS_TRAJECTORYACTIONSERVER_H
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


#include <joint_trajectory_execution/TrajectoryActionServer.h>


#define JACO_DEFAULT_TRAJECTORY_PLAY_FACTOR 1.0
#define JACO_DEFAULT_ANGLE_SAFETY_LIMIT 0.3
#define JACO_DEFAULT_GOAL_TOLERANCE 0.05


/**
 * \brief Simple extension of JacoTrajectoryActionServer for the Jaco arm.
 *  In particular, this sets defaults and uses JacoJointManager as a specific
 *  implementation of ArmComponentsNameManager.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class JacoTrajectoryActionServer: public TrajectoryActionServer
{
public:
    JacoTrajectoryActionServer(
        std::string& _action_topic_name,
        std::vector<float>& _targetPos,
        std::vector<float>& _targetVel,
        std::vector<float>& _currentAngles,
        std::vector<float>& _currentVels,
        boost::mutex& _lock,
        bool _positionMode,
        std::vector<float>& _maxVelocities,
        float _armAngleSafetyLimit = JACO_DEFAULT_ANGLE_SAFETY_LIMIT,
        float _goalTolerance = JACO_DEFAULT_GOAL_TOLERANCE,
        bool _simplifyTrajectoryVelocities = true,
        bool _useOnlineVelocityControl = true,
        float _interGoalTolerance = 2 * JACO_DEFAULT_GOAL_TOLERANCE):
            TrajectoryActionServer(_action_topic_name,
                JacoJointManager(),  // use default constructor to read from ROS parameter server
                _targetPose,
                _currentAngles,
                _currentVels,
                _lock,
                _positionMode,
                _maxVelocities,
                _armAngleSafetyLimit,
                _goalTolerance,
                _simplifyTrajectoryVelocities,
                _useOnlineVelocityControl,
                _interGoalTolerance),
    {
    }

    ~JacoTrajectoryActionServer(){}
};

#endif  // JACO_JOINTS_TRAJECTORYACTIONSERVER_H
