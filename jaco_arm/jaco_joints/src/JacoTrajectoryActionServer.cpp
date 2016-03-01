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

#include <jaco_joints/JacoTrajectoryActionServer.h>
#include <convenience_math_functions/MathFunctions.h>
#include <algorithm>
#include <vector>
#include <string>

#define MIN_FINGERS_TOLERANCE 3e-02

// epsilon which counts as velocity zero
#define ZERO_TARGET_VEL 1e-05

// general epsilon value for zero comparisons
#define ZERO_EPS 1e-06

// minimum correction velocity to call waitUntiPointReached()
// in the main loop of the online-controlled execution
#define ONLINE_EXE_MIN_CORRECT_VEL 0.07
// maximum of ONLINE_EXE_MIN_CORRECT_VEL
#define ONLINE_EXE_MAX_CORRECT_VEL 0.15


// like ONLINE_EXE_MIN_CORRECT_VEL but for 
// fine-adjustment, so should be lower.
#define ONLINE_EXE_MIN_CORRECT_VEL_FINE 0.05
// maximum of ONLINE_EXE_MIN_CORRECT_VEL_FINE
#define ONLINE_EXE_MAX_CORRECT_VEL_FINE 0.09


// tolerance to accept a velocity as zero
// when waiting for the arm to settle down
// XXX KINOVA: Was 0.05 at one place and 0.015 at another
#define VELOCITY_ZERO_TOLERANCE 0.015

using convenience_math_functions::MathFunctions;

JacoTrajectoryActionServer::JacoTrajectoryActionServer(
    ros::NodeHandle &n,
    std::string& action_topic_name,
    std::vector<float>& _targetPos, std::vector<float>& _targetVel,
    std::vector<float>& _currentAngles,
    std::vector<float>& _currentVels,
    boost::mutex& _lock, bool _positionMode, std::vector<float>& _maxVelocities,
    float _armAngleSafetyLimit, float goalTolerance, bool _simplifyTrajectoryVelocities,
    bool _useOnlineVelocityControl,
    float interGoalTolerance):
    joints(),  // use default constructor to read from parameters
    TRAJECTORY_PLAY_FACTOR(DEFAULT_TRAJECTORY_PLAY_FACTOR),
    playThread(NULL) ,
    GOAL_TOLERANCE(goalTolerance),
    INTER_GOAL_TOLERANCE(interGoalTolerance),
    ANGLE_SAFETY_LIMIT(_armAngleSafetyLimit),
    maxVelocities(_maxVelocities),
    targetPos(_targetPos), targetVel(_targetVel),
    current_joint_angles(_currentAngles),
    current_joint_vels(_currentVels),
    valueLock(_lock),
    positionMode(_positionMode),
    enableMaxAngleDistSafety(_armAngleSafetyLimit > 0),
    has_goal(false), executionIsFinished(false),
    current_traj_group(-1),
    initialized(false),
    simplifyTrajectoryVelocities(_simplifyTrajectoryVelocities),
    useOnlineVelocityControl(_useOnlineVelocityControl)
{
    action_server = new JTActionServerT(n,
                                        action_topic_name,
                                        boost::bind(&JacoTrajectoryActionServer::actionCallback, this, _1),
                                        boost::bind(&JacoTrajectoryActionServer::cancelCallback, this, _1),
                                        false);

    targetPos.resize(9, 0);
    targetVel.resize(9, 0);
    if (current_joint_angles.size() != 9)
    {
        ROS_INFO("Current values are not of size 9");
    }


    if (useOnlineVelocityControl && usePositionMode())
    {
        ROS_WARN("Can only online velocity control in velocity mode!");
        ROS_WARN("Forcing velocity mode for trajectory execution.");
        positionMode = false;
    }
}


JacoTrajectoryActionServer::~JacoTrajectoryActionServer()
{
    shutdown();
    delete action_server;
}

void JacoTrajectoryActionServer::shutdown()
{
    shutdownImpl();
}

bool JacoTrajectoryActionServer::init()
{
    if ((targetPos.size() != 9) || (targetVel.size() != 9) || (current_joint_angles.size() != 9))
    {
        ROS_INFO("Current and target values are not of size 9");
        return false;
    }

    if (maxVelocities.size() != 9)
    {
        ROS_ERROR("Have to have 9 max velocities specified. Pass right array in constructor!");
        return false;
    }


    if (useOnlineVelocityControl && usePositionMode())
    {
        ROS_ERROR("Can only use velocity player in velocity mode!");
        return false;
    }

    if (!initImpl())
    {
        ROS_ERROR("Could not initialize the trajectory action server");
        return false;
    }




    action_server->start();
    ROS_INFO("Action server for Joint Trajectory started. Using position mode: %i", usePositionMode());

    initialized = true;
    return true;
}


void JacoTrajectoryActionServer::cancelCallback(GoalHandle& goal)
{
    ROS_INFO("JacoTrajectoryActionServer: Received action cancel request");
    goalLock.lock();
    bool currentGoal = (goal == current_goal);
    goalLock.unlock();
    if (currentGoal) abortExecution();
    else ROS_ERROR("This goal is not currently being executed in this server");
}



void JacoTrajectoryActionServer::actionCallback(GoalHandle& goal)
{
    if (!initialized)
    {
        ROS_ERROR("Action server not initialised, can't accept goal");
        goal.setRejected();
        return;
    }

    if (hasCurrentGoal())
    {
        ROS_ERROR("Currently executiong a trajectory, can't accept another one");
        goal.setRejected();
        return;
    }

    trajectory_msgs::JointTrajectory traj = goal.getGoal()->trajectory;

    ROS_INFO("Received joint trajectory goal, trajectory size %lu", traj.points.size());
    // ROS_INFO_STREAM(traj);

    if (traj.points.empty())
    {
        ROS_ERROR("Empty trajectory");
        goal.setRejected();
        return;
    }

    // ROS_INFO("Adapted trajectory:");
    // ROS_INFO_STREAM(traj);

    std::vector<int> joint_indices;
    int idxRet = joints.getJointIndices(traj.joint_names, joint_indices);
    if (idxRet < 0)
    {
        ROS_ERROR("Not all joints specified in trajectory, so trajectory not eligible");
        goal.setRejected();
        return;
    }


    double diff = 0.02;
    adaptTrajectoryAngles(traj, diff, 1e-02);
    if (!usePositionMode() && simplifyTrajectoryVelocities)
    {
        if (!adaptTrajectoryVelocitiesToLinear(traj, joint_indices))
        {
            ROS_ERROR("Could not simplify trajectory velocities to linear velocities");
            ROS_INFO_STREAM(traj);
            goal.setRejected();
            return;
        }
    }

    // ROS_INFO("Adapted trajectory: "); ROS_INFO_STREAM(traj);


    if (!checkTrajectory(traj, joint_indices, idxRet))
    {
        ROS_ERROR("Trajectory not eligible from current robot state");
        goal.setRejected();
        return;
    }

    float tolerance = ANGLE_SAFETY_LIMIT;
    if (ANGLE_SAFETY_LIMIT < 0) tolerance=GOAL_TOLERANCE; // if no safety limit is used, just use goal tolerance.
    if (!atTrajectoryStart(traj, joint_indices, idxRet, tolerance))
    {
        ROS_ERROR("Can only accept trajectoryies that start at the current state of the robot");
        // ROS_ERROR_STREAM(traj);
        goal.setRejected();
        return;
    }


    ROS_INFO("Trajectory is eligible for execution, goal accepted");

    goalLock.lock();
    current_goal = goal;
    current_goal.setAccepted();
    has_goal = true;
    current_traj = traj;
    current_traj_idx = joint_indices;
    current_traj_group = idxRet;
    goal = current_goal;
    goalLock.unlock();

    setExecutionFinished(false, false);

    ros::Time start = ros::Time::now();
    if (!playTrajectoryImplementation(traj, joint_indices, idxRet))
    {
        ROS_INFO("Playing the trajectory has failed.");
        goalLock.lock();
        if (has_goal)
        {
            current_goal.setAborted();
            has_goal = false;
        }
        goalLock.unlock();
        return;
    }

    /*
    float expectedDuration=traj.points.back().time_from_start.toSec();

    float maxWait=-1;
    if (exceedDurationWait > 0) maxWait=expectedDuration*exceedDurationWait;

    if (waitForExecution(maxWait)<=0){
        goalLock.lock();
        if (has_goal) {
            has_goal=false;
        }
        goalLock.unlock();
    }else{
        ros::Time end=ros::Time::now();
        ros::Duration dur=end-start;
        float timediff=dur.toSec()-expectedDuration;
        ROS_INFO("Overall execution duration: %f (expected: %f, difference is %f / %f percent)",dur.toSec(),
            expectedDuration,timediff,dur.toSec()*100/expectedDuration);
        goalLock.lock();
        if (has_goal) {
            current_goal.setSucceeded();
            has_goal=false;
        }
        goalLock.unlock();
    }
    */
}


bool JacoTrajectoryActionServer::playTrajectoryImplementation(const trajectory_msgs::JointTrajectory& traj,
    const std::vector<int>& joint_indices, const int group)
{
    if (playThread != NULL)
    {
        playThread->interrupt();
        playThread->join();
        delete playThread;
    }

    if (useOnlineVelocityControl)
    {
        playThread = new boost::thread(boost::bind(&JacoTrajectoryActionServer::playTrajectoryOnlineControlled,
                this, traj, joint_indices, group, INTER_GOAL_TOLERANCE, 0.0));
    }
    else
    {
        playThread = new boost::thread(boost::bind(&JacoTrajectoryActionServer::playTrajectorySimple,
                this, traj, joint_indices, group));
    }

    return true;
}



void JacoTrajectoryActionServer::playTrajectorySimple(const trajectory_msgs::JointTrajectory traj,
        const std::vector<int>& joint_indices, const int group)
{
    ROS_INFO("Playing trajectory!");

    float lastTime = 0.0;
    bool success = true;

    for (int i = 0; i < traj.points.size(); i++)
    {
        float pointTime = traj.points[i].time_from_start.toSec();
        float sleepTime = (pointTime - lastTime) / TRAJECTORY_PLAY_FACTOR;
        lastTime = pointTime;

        updateCurrentState();
        trajectory_msgs::JointTrajectoryPoint point = traj.points[i];

        // The point with the angular target coordinates will be either:
        // 1) in position mode: The current point
        // 2) in velocity mode: The next or last point
        const trajectory_msgs::JointTrajectoryPoint * targetPoint = &point;
        if (!usePositionMode() && (i < traj.points.size() - 1))
        {
            targetPoint = &(traj.points[i + 1]);
            sleepTime = (targetPoint->time_from_start.toSec() - pointTime) / TRAJECTORY_PLAY_FACTOR;
        }

        valueLock.lock();
        setTargetValues(*targetPoint, joint_indices, group, current_joint_angles, targetPos, true);
        if (!usePositionMode()) setTargetValues(point, joint_indices, group, current_joint_angles, targetVel, false);
        valueLock.unlock();

        float tolerance = GOAL_TOLERANCE;

        // ROS_INFO("Waiting for target keypoint %i/%lu to be reached with tolerance %f",i,traj.points.size()-1,tolerance);
        ros::Duration(sleepTime).sleep();


        float extraTime = 0;
        float recheckTime = 0.005;
        while (!currentTargetReached(*targetPoint, joint_indices, group, tolerance))
        {
            // ROS_INFO("Have to wait as current target is not reached yet by arm");
            ros::Duration(recheckTime).sleep();
            extraTime += recheckTime;
            /*if (!goalActive()) {
                ROS_WARN("Action has been considered ended already, so stopping execution");
                success=false;
                break;
            }*/
        }

        if (extraTime > (recheckTime - ZERO_EPS))
            ROS_INFO("Had to wait an extra %f seconds (roughly) until target reached.", extraTime);

        if (!success) break;
    }
    ROS_INFO("Execution of trajectory finished. Success=%i", success);
    setExecutionFinished(true, success);
}


void JacoTrajectoryActionServer::playTrajectoryOnlineControlled(const trajectory_msgs::JointTrajectory traj,
    const std::vector<int>& joint_indices, const int group, const float inter_tolerance, float lagTime)
{
    if (usePositionMode())
    {
        ROS_ERROR("playTrajectoryOnlineControlled works only in velocity mode");
        return;
    }

    if (!simplifyTrajectoryVelocities)
    {
        ROS_ERROR("playTrajectoryOnlineControlled works only with simplified trajectory velocities");
        return;
    }

    setExecutionFinished(false, true);

    ROS_INFO("Playing trajectory on-line controlled, lag time %f", lagTime);

    bool success = true;

    for (int i = 0; i < traj.points.size(); i++)
    {
        // The point with the target coordinates and velocities
        const trajectory_msgs::JointTrajectoryPoint& point = traj.points[i];
        float pointTime = point.time_from_start.toSec();

        float targetPointTime = pointTime;
        // The point with the angular target coordinates will be the next or last point
        const trajectory_msgs::JointTrajectoryPoint * targetPoint = &point;
        if (i < traj.points.size() - 1)
        {
            targetPoint = &(traj.points[i + 1]);
            targetPointTime = targetPoint->time_from_start.toSec();
        }

        float stepTime = (targetPointTime - pointTime) / TRAJECTORY_PLAY_FACTOR;
        if (stepTime < 0.1) stepTime = 0.1;  // step time should be at least a little

        std::vector<float> _lastPos, _targetPos, _targetVel, _currAngles;

        updateCurrentState();
        valueLock.lock();
        _currAngles = current_joint_angles;
        valueLock.unlock();
        MathFunctions::capToPI(_currAngles);

        // copy position values of last trajectory point into _lastPos
        setTargetValues(point, joint_indices, group, _currAngles, _lastPos, true);
        MathFunctions::capToPI(_lastPos);
        // copy position values of target trajectory point into _targetPos
        setTargetValues(*targetPoint, joint_indices, group, _currAngles, _targetPos, true);
        MathFunctions::capToPI(_targetPos);
        // copy velocity values of trajectory point into _targetVel
        setTargetValues(point, joint_indices, group, _currAngles, _targetVel, false);

        // ROS_INFO("target pos fingers: %f %f %f",_targetPos[6],_targetPos[7],_targetPos[8]);

        float sleepTime = 1.0 / 80.0;
        float maxWaitTime = stepTime * 100;
        if (maxWaitTime < 0.1) maxWaitTime = 0.1;  // have to wait at least a little!
        float tolerance = inter_tolerance;
        if (i == traj.points.size() - 1) tolerance = GOAL_TOLERANCE;

        float min_vel = ONLINE_EXE_MIN_CORRECT_VEL;
        float max_vel = ONLINE_EXE_MAX_CORRECT_VEL;

        if (simplifyTrajectoryVelocities) {
            if (i < (traj.points.size() - 1)) ROS_INFO("Waiting to reach trajectory point %i.", i+1);
        } else {
            ROS_INFO("Waiting to reach trajectory point %i.", i);
        }
    
        // set the target position now already. This is required to fall back to 
        // if velocities become 0. Velocities will be adjusted in call to waitUntilPointReached() below.
        valueLock.lock();
        targetPos = _targetPos;
        valueLock.unlock();

        success = waitUntilPointReached(_targetPos, _targetVel, sleepTime, maxWaitTime, tolerance, lagTime, min_vel, max_vel);

        if (!success)
        {
            ROS_ERROR("playTrajectoryOnlineControlled: Could not reach trajectory point %i within the allocated time (%f s). step time: %f. %s",
                i + 1, maxWaitTime, stepTime, __FILE__);
            break;
        }
        // if this is the last point in the trajectory, we should already have reached it in the last
        // iteration, and the last trajectory point served merely to set velocities to 0.
        if (i < traj.points.size() - 1)
        {
            ROS_INFO("Reached trajectory point %i. %f %f %f %f %f %f", i + 1,
                _targetPos[0], _targetPos[1], _targetPos[2], _targetPos[3], _targetPos[4], _targetPos[5]);
        }
    }

    if (success)
    {
        ROS_INFO("##########################################################");
        ROS_INFO("###### Played. Now doing fine adjustments #################");
        ROS_INFO("##########################################################");
    }
    // make sure to reset all velocities to 0
    std::vector<float> _targetVel;
    _targetVel.resize(9, 0);
    for (int k = 0; k < 0; ++k)
    {
        _targetVel[k] = 0;
    }
    valueLock.lock();
    targetVel = _targetVel;
    // targetPos will still be set to the last trajectory point.
    valueLock.unlock();

    float recheckTime = 0.02;
    float maxWaitForZero = 4;
    float zeroTolerance = VELOCITY_ZERO_TOLERANCE; // 0.05 XXX Kinova: was 0.05
    // now we have to wait until the arm actually stops. Due to lag time, this could still take a bit.
    bool backToZero = waitUntilVelocitiesZero(recheckTime, maxWaitForZero, zeroTolerance);
    if (!backToZero)
    {
        ROS_ERROR("Could not wait until velocities zero within allocated time");
    }

    if (!success || !backToZero)
    {
        setExecutionFinished(true, false);
        return;
    }

    // now, we can adjust final accuracy, in case the exact pose has not been reached yet.
    std::vector<float> _currAngles;
    updateCurrentState();
    valueLock.lock();
    _currAngles = current_joint_angles;
    valueLock.unlock();
    MathFunctions::capToPI(_currAngles);

    const trajectory_msgs::JointTrajectoryPoint& point = traj.points.back();
    std::vector<float> _targetPos;
    setTargetValues(point, joint_indices, group, _currAngles, _targetPos, true);
    MathFunctions::capToPI(_targetPos);

    float min_vel = ONLINE_EXE_MIN_CORRECT_VEL_FINE;
    float max_vel = ONLINE_EXE_MAX_CORRECT_VEL_FINE;
    float maxWaitForExact = 2.0;
    int numTries = 10;
    if (!repeatedWaitUntilPointReached(_targetPos, recheckTime, maxWaitForExact, maxWaitForZero,
            GOAL_TOLERANCE, lagTime, min_vel, max_vel, numTries))
    {
        ROS_ERROR("Could not wait for final adjustment");
        success = false;
    }

    float maxAngle;
    int maxJoint;
    maxEndpointDiff(maxAngle, maxJoint, traj, joint_indices, group);
    
    ROS_INFO("##########################################################");
    ROS_INFO("###  Execution of trajectory finished. Success=%i", success);
    ROS_INFO("###  Final accuracy is %f (at joint %i)", maxAngle, maxJoint);
    ROS_INFO("##########################################################");

    setExecutionFinished(true, success);
}




bool JacoTrajectoryActionServer::repeatedWaitUntilPointReached(const std::vector<float>& _targetPos, float recheckTime,
        float maxWaitForExact, float maxWaitForZero, float tolerance, float lagTime,
        float min_correct_vel, float max_correct_vel, int numTries)
{
    bool success = true;

    // now, we can adjust final accuracy, in case the exact pose has not been reached yet.
    std::vector<float> _currAngles;
    updateCurrentState();
    valueLock.lock();
    _currAngles = current_joint_angles;
    valueLock.unlock();
    MathFunctions::capToPI(_currAngles);

    while (!equalJointFloats(_currAngles, _targetPos, tolerance, true))
    {
        ROS_INFO("Fine-adjusting accuracy, tries left: %i", numTries);

        std::vector<float> _targetVel;
        _targetVel.resize(9, 0);  // start off with 0 velocities, and leave adjustments to internal waitUntilPointReached call.

        success = waitUntilPointReached(_targetPos, _targetVel, recheckTime, maxWaitForExact,
                tolerance, lagTime, min_correct_vel, max_correct_vel);
        if (!success)
        {
            ROS_INFO("Could not reach exact angle position within allocated time, tries left: %i", numTries);
        }

        // make sure to reset all velocities to 0
        _targetVel.resize(9, 0);
        for (int k = 0; k < 0; ++k)
        {
            _targetVel[k] = 0;
        }
        valueLock.lock();
        targetVel = _targetVel;
        targetPos = _targetPos;
        valueLock.unlock();

        ROS_INFO("Re-setting to zero velocities to settle the arm...");

        // wait again until arm has reached zero.
        float zeroTolerance = VELOCITY_ZERO_TOLERANCE; // XXX Kinova: was 0.015;
        if (!waitUntilVelocitiesZero(recheckTime, maxWaitForZero, zeroTolerance))
        {
            ROS_INFO("Could not wait until velocities zero within allocated time, tries left: %i", numTries);
        }

        updateCurrentState();
        valueLock.lock();
        _currAngles = current_joint_angles;
        valueLock.unlock();
        MathFunctions::capToPI(_currAngles);

        --numTries;
        if (numTries <= 0)
        {
            ROS_ERROR("Tries exhausted in trying to reach target accuracy");
            success = false;
            break;
        }
    }
    return success;
}



bool JacoTrajectoryActionServer::waitUntilVelocitiesZero(float recheckTime, float maxWaitTime, float tolerance)
{
    bool success = true;
    float timeSlept = 0;
    int numIter = 0;
    std::vector<float> _currVels;
    float failVel = 0;
    while (timeSlept < maxWaitTime)
    {
        ++numIter;

        updateCurrentState();
        valueLock.lock();
        _currVels = current_joint_vels;
        valueLock.unlock();

        bool reachedZero = true;
        for (int k = 0; k < 9; ++k)
        {
            if (fabs(_currVels[k]) > tolerance)
            {
                reachedZero = false;
                failVel = fabs(_currVels[k]);
                break;
            }
        }

        if (reachedZero) break;

        ros::Duration(recheckTime).sleep();
        timeSlept += recheckTime;
    }

    if (timeSlept > maxWaitTime)
    {
        ROS_ERROR("Waiting (max %lfs) until velocities 0: Could not reach point within the allocated time. Failed with velocity %f, tolerance was %f",
                maxWaitTime, failVel, tolerance);
        return false;
    }
    return true;
}

bool JacoTrajectoryActionServer::waitUntilPointReached(const std::vector<float>& _targetPos,
        const std::vector<float>& _initialTargetVel,
        float recheckTime, float maxWaitTime, float tolerance,
        float lagTime, float min_correct_vel, float max_correct_vel)
{
    std::vector<float> _targetVel = _initialTargetVel;

    std::vector<float> _currAngles;

    bool success = true;
    float timeSlept = 0;
    int numIter = 0;
    while (timeSlept < maxWaitTime)
    {
        ++numIter;

        updateCurrentState();
        valueLock.lock();
        _currAngles = current_joint_angles;
        valueLock.unlock();

        MathFunctions::capToPI(_currAngles);

        _targetVel = _initialTargetVel;

        // assuming lag, we have to look ahead in time and assume we are at the position we would
        // be in <lagTime> seconds when keeping this velocities.
        if (lagTime > ZERO_EPS)
        {
            for (int k = 0; k < 9; ++k)
            {
                float currPos = _currAngles[k];
                float nextPoint = _targetPos[k];
                float targetDiff = MathFunctions::capToPI(nextPoint - currPos);
                if (fabs(targetDiff) > tolerance)   
                {
                    // position not reached currently (in which case we don't care), so look ahead in lag
                    // ROS_INFO("Adapting curr Angle %i from %f to %f",k,_currAngles[k],_currAngles[k] + _targetVel[k]*lagTime);
                    _currAngles[k] = _currAngles[k] + _targetVel[k] * lagTime;
                }
            }
            MathFunctions::capToPI(_currAngles);
        }


        // targetPos and _targetVel are now of size 9

        bool allTargetsReached = true;
        // check whether the targets are reached or velocities have to be adapted 
        for (int k = 0; k < 9; ++k)
        {
            float currPos = _currAngles[k];
            float nextPos = _targetPos[k];
            float targetDiff = MathFunctions::capToPI(nextPos - currPos);

            float tol = tolerance;
            if (k >= 6) tol = std::max(tolerance, static_cast<float>(MIN_FINGERS_TOLERANCE));  // fingers can't do higher accuracy than this

            if (fabs(targetDiff) < tol)   // position reached, set velocity to 0
            {
                _targetVel[k] = 0;  // target reached
                // ROS_INFO("%i: Target joint %i reached with accuracy %f (curr=%f, target=%f)",numIter,k,targetDiff,currPos,nextPos);
            }
            else
            {
                allTargetsReached = false;

                // ROS_INFO("Target joint %i not reached! tvel=%f, currPos=%f, nextPos=%f, diff %f (iter %i)",k,_targetVel[k],currPos,nextPos,targetDiff,numIter);

                // float lastPoint=_lastPos[k];
                bool overshot = _targetVel[k] > 0 ? targetDiff < 0 : targetDiff > 0; 

                // if we overshot the goal, we have to correct. Or if velocitiy is 0, we will never get there!
                if (overshot || (fabs(_targetVel[k]) < ZERO_EPS))
                {
                    float oldVel = _targetVel[k];
                    _targetVel[k] = targetDiff / (recheckTime * 10);  // we want to get there in 10 of these steps...
                    // .. but always stay within the correction bounds
                    if (fabs(_targetVel[k]) < min_correct_vel) _targetVel[k] = targetDiff < 0 ? -min_correct_vel : min_correct_vel;
                    if (fabs(_targetVel[k]) > max_correct_vel) _targetVel[k] = targetDiff < 0 ? -max_correct_vel : max_correct_vel;
                    // ROS_WARN("Correct target vel joint %i (overshot: %i) from %f to %f (p-diff %f), pos=%f next=%f",
                       // k, overshot, oldVel, _targetVel[k], targetDiff, currPos, nextPos);
                }
            }
        }

        if (allTargetsReached) break;  // we can move on to next trajectory point

        // ROS_INFO_STREAM("Setting target pos "<<_targetPos[1]<<", "<<_targetPos[2]);
        valueLock.lock();
        targetVel = _targetVel;
        targetPos = _targetPos;
        valueLock.unlock();

        if (!goalActive())
        {
            ROS_WARN_STREAM("Trajectory execution is inactive, maybe has been cancelled.");
            success=false;
            break;
        }

        // ROS_INFO("Waiting for target keypoint %i/%lu to be reached with tolerance %f",i,traj.points.size()-1,tolerance);
        ros::Duration(recheckTime).sleep();
        timeSlept += recheckTime;
    }

    if (timeSlept > maxWaitTime)
    {
        ROS_ERROR("waitUntilPointReached: Could not reach point within the allocated time (%f). %s", maxWaitTime, __FILE__);
        success = false;
    }

    return success;
}


inline int sign(const double n)
{
    return n > 0 ? 1 : -1;
}




bool JacoTrajectoryActionServer::adaptTrajectoryVelocitiesToLinear(trajectory_msgs::JointTrajectory& traj,
        const std::vector<int>& joint_indices) const
{
    if (traj.points.size() < 1) return false;

    float timeCount = 0;

    for (int i = 1; i < traj.points.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint &lastPoint = traj.points[i - 1];
        trajectory_msgs::JointTrajectoryPoint& point = traj.points[i];
        if (lastPoint.velocities.size() != lastPoint.positions.size())
        {
            lastPoint.velocities.resize(lastPoint.positions.size(), 0);
        }
        float timeDiff = point.time_from_start.toSec() - lastPoint.time_from_start.toSec();
        float adaptedTimeDiff = timeDiff;
        for (int j = 0; j < lastPoint.positions.size(); ++j)
        {
            float p1 = lastPoint.positions[j];
            float p2 = point.positions[j];
            float distance = MathFunctions::angleDistance(p1, p2);
            float maxVel = maxVelocities[joint_indices[j]];
            float velocity;

            // distance (now a value between -PI and PI) may have to be made to go the long way 
            // round if a velocity in p1 indicated this...
            float v1 = (lastPoint.velocities.size() >= j) ? lastPoint.velocities[j] : 0; // velocity at last point (if given)
            if ((fabs(v1) > ZERO_EPS) && (sign(distance)!=sign(v1)))
            {
                ROS_INFO_STREAM("Info: velocity in joint trajectory goes 'the long way round': "<<p1<<" -> "<<p2<<" v="<<v1);
                double invDist = 2*M_PI-fabs(distance); 
                distance = distance > 0 ? -invDist: invDist;
            }

            if (fabs(timeDiff) < ZERO_EPS)
            {
                if (fabs(distance) > ZERO_EPS)
                {
                    ROS_WARN("Faulty trajectory: Can't cover distance %f in 0 seconds. Assuming max velocity in adaptTrajectoryVelocitiesToLinear.",distance);
                    velocity = distance > 0 ? maxVel : -maxVel;
                }
                else
                {
                    velocity = 0;
                }
            }
            else
            {
                velocity = distance / timeDiff;
            }

            if (fabs(velocity) >= (maxVel + 1e-03))  // if exceeds max velocity by small tolerance
            {
                ROS_WARN("Trajectory point %i exceeds max velocity of joint %i, this will change timings", i, j);
                velocity = distance > 0 ? maxVel : -maxVel;
            }
            lastPoint.velocities[j] = velocity;
            if (fabs(velocity) > ZERO_EPS)
            {
                // re-calculate time (if velocities haven't been changed, this should stay the same)
                adaptedTimeDiff = std::max(adaptedTimeDiff, distance / velocity);
            }
        }


        // adapt all velocities again. Necessary if one of the joints has stretched the time.
        if (fabs(adaptedTimeDiff - timeDiff) > ZERO_EPS)
        {
            for (int j = 0; j < lastPoint.positions.size(); ++j)
            {
                float p1 = lastPoint.positions[j];
                float p2 = point.positions[j];
                float distance = MathFunctions::angleDistance(p1, p2);
                float velocity = distance / adaptedTimeDiff;
                lastPoint.velocities[j] = velocity;
            }
        }

        timeCount += adaptedTimeDiff;
        // update the time of the trajectory point. If nothing changed in this trajectory point, it should remain the same.
        point.time_from_start = ros::Duration(timeCount);
    }

    // last point has 0 velocity
    trajectory_msgs::JointTrajectoryPoint &lastPoint = traj.points[traj.points.size() - 1];
    if (lastPoint.velocities.size() != lastPoint.positions.size())
        lastPoint.velocities.resize(lastPoint.positions.size(), 0);
    for (int j = 0; j < lastPoint.velocities.size(); ++j) lastPoint.velocities[j] = 0.0;

    return true;
}



void JacoTrajectoryActionServer::adaptTrajectoryAngles(trajectory_msgs::JointTrajectory& traj, double diff, double epsilon) const
{
    if (traj.points.size() < 1) return;

    trajectory_msgs::JointTrajectoryPoint lastPoint = traj.points[0];
    for (int i = 1; i < traj.points.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint& point = traj.points[i];
        if (point.velocities.size() != point.positions.size())
        {
            lastPoint = point;
            continue;  // can't do without velocities!
        }
        for (int j = 0; j < point.positions.size(); ++j)
        {
            float p1 = lastPoint.positions[j];
            float p2 = point.positions[j];
            float distance = MathFunctions::angleDistance(p1, p2);
            if (fabs(distance - M_PI) < epsilon)
            {
                ROS_INFO("Adapting joint %i (trajectory pos %i) which is 180 degrees from last trajectory point", j, i);
                traj.points[i].positions[j] -= sign(lastPoint.velocities[j]) * diff;
            }
        }
        lastPoint = point;
    }
}



bool JacoTrajectoryActionServer::removeIrrelevantStates(const std::vector<int>& idx, std::vector<float>& values) const
{
    if (idx.size() != values.size()) return false;

    std::vector<float>::iterator fit = values.begin();
    for (std::vector<int>::const_iterator it = idx.begin(); it != idx.end(); ++it)
    {
        if (*it < 0)
        {
            fit = values.erase(fit);
        }
        else
        {
            ++fit;
        }
    }
    return true;
}



bool JacoTrajectoryActionServer::checkTrajectory(const trajectory_msgs::JointTrajectory& traj,
        const std::vector<int>& joint_indices, int group)
{
    if (traj.points.empty())
    {
        return true;
    }

    updateCurrentState();

    valueLock.lock();
    std::vector<float> currentAngles = current_joint_angles;
    valueLock.unlock();

    if (!removeIrrelevantStates(joint_indices, currentAngles))
    {
        ROS_ERROR("Inconsistency in array sizes");
        return false;
    }

    ROS_INFO("Checking trajectory's validity");


    // first, check if first point is too far away from current state
    std::vector<float> firstTargetAngles;
    if (!getTargetAngles(traj.points.front(), joint_indices, firstTargetAngles))
    {
        ROS_ERROR("First trajectory point invalid, joint_indices is of size %lu", joint_indices.size());
        return false;
    }

    if ((ANGLE_SAFETY_LIMIT >=0) && !angleDistanceOK(currentAngles, firstTargetAngles, group, ANGLE_SAFETY_LIMIT))
    {
        ROS_ERROR("Too much distance to first trajectory point from current state!");
        return false;
    }




    // Check wheter distances along trajectory are ok for safety limitations
    for (int i = 0; i < traj.points.size(); i++)
    {
        if (traj.points[i].positions.empty())
        {
            ROS_ERROR("No positions given at trajectory point %i", i);
            return false;
        }

        if (enableMaxAngleDistSafety)
        {
            std::vector<float> targetAngles;
            if (!getTargetAngles(traj.points[i], joint_indices, targetAngles))
            {
                ROS_ERROR("Trajectory point %i invalid, joint_indices is of size %lu", i, joint_indices.size());
                return false;
            }

            if (!angleDistanceOK(currentAngles, targetAngles, group, ANGLE_SAFETY_LIMIT))
            {
                ROS_ERROR("Too much distance to trajectory point %i for safety restrictions. Pass finer resolved trajectory!", i);
                return false;
            }
            currentAngles = targetAngles;
        }

        if (!usePositionMode())   // check also for maximum velocities
        {
            std::vector<float> targetVels;
            if (!getTargetVelocities(traj.points[i], joint_indices, targetVels))
            {
                ROS_ERROR("Trajectory point %i invalid, joint_indices is of size %lu", i, joint_indices.size());
                return false;
            }
            if (maxVelExceeded(targetVels, group))
            {
                ROS_ERROR("Maximum velocity in one of the trajectory points exceeded.");
                return false;
            }
        }
    }
    return true;
}

bool JacoTrajectoryActionServer::maxVelExceeded(std::vector<float>& vels, int check, float tolerance_above) const
{
    if (check < 2)
    {
        // Check arm angles
        for (int i = 0; i < 6; ++i)
        {
            float vel = vels[i];
            if (fabs(vel) > (maxVelocities[i] + tolerance_above))
            {
                ROS_ERROR("Velocity of joint %i exceeded: is %f, max is %f", i, vel, maxVelocities[i]);
                return true;
            }
        }
    }

    if (check == 1) return false;

    int init = 6;
    if (check == 2) init = 0;
    int m = 6;
    // Check finger angles:
    for (int i = init; i < (init + 3); ++i)
    {
        float vel = vels[i];
        if (fabs(vel) > (maxVelocities[m] + tolerance_above))
        {
            ROS_ERROR("Velocity of joint %i exceeded: is %f, max is %f", i, vel, maxVelocities[m]);
            return true;
        }
        ++m;
    }

    return false;
}




bool JacoTrajectoryActionServer::angleDistanceOK(std::vector<float>& j1, std::vector<float>& j2,
                int check, float maxAngle) const
{
    if (j1.size() != j2.size())
    {
        ROS_ERROR("Incorrect vector sizes");
        return false;
    }
    if (check < 2)
    {
        // Check arm angles
        for (int i = 0; i < 6; ++i)
        {
            float f1 = MathFunctions::capToPI(j1[i]);
            float f2 = MathFunctions::capToPI(j2[i]);
            float dist = MathFunctions::angleDistance(f1, f2);
            if (fabs(dist) > maxAngle)
            {
                ROS_INFO("Angle %i: %f %f", i, j1[i], j2[i]);
                ROS_ERROR("Limit for joint %i reached: pose1 %f pose2 %f, dist %f (max %f)", i, f1, f2, dist, maxAngle);
                return false;
            }
        }
    }

    if (check == 1) return true;

    int init = 6;
    if (check == 2) init = 0;

    // Check finger angles:
    for (int i = init; i < (init + 3); ++i)
    {
        float f1 = j1[i];
        float f2 = j2[i];
        float dist = fabs(MathFunctions::angleDistance(f1, f2));
        if (dist > maxAngle)
        {
            ROS_ERROR("Safety limit for joint %i reached, %f > %f in moving from %f to %f", i, dist, maxAngle, f1, f2);
            return false;
        }
    }
    return true;
}

void JacoTrajectoryActionServer::abortExecution()
{
    if (playThread)
    {
        playThread->interrupt();
        playThread->join();
    }
}

void JacoTrajectoryActionServer::joinExecutionThread()
{
    if (playThread)
    {
        playThread->join();
    }
}

bool JacoTrajectoryActionServer::goalActive()
{
    goalLock.lock();
    bool hasOneGoal = has_goal;
    bool cancelled = false;
    if (hasOneGoal)
    {
        actionlib_msgs::GoalStatus stat = current_goal.getGoalStatus();
        cancelled = (stat.status != actionlib_msgs::GoalStatus::ACTIVE);
        // (stat.status == actionlib_msgs::GoalStatus::PREEMPTED)
        // || (stat.status == actionlib_msgs::GoalStatus::ABORTED)
        // || (stat.status == actionlib_msgs::GoalStatus::LOST);
    }
    goalLock.unlock();
    return ros::ok() && !cancelled && hasOneGoal;
}

void JacoTrajectoryActionServer::setHasCurrentGoal(bool flag)
{
    goalLock.lock();
    has_goal = flag;
    goalLock.unlock();
}

bool JacoTrajectoryActionServer::hasCurrentGoal()
{
    goalLock.lock();
    bool ret = has_goal;
    goalLock.unlock();
    return ret;
}


void JacoTrajectoryActionServer::setExecutionFinished(bool flag, bool success)
{
    goalLock.lock();
    executionIsFinished = flag;
    executionSuccessful = success;
    if (flag && has_goal)
    {
        if ((current_goal.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED)
                || (current_goal.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE))
        {
            if (success) current_goal.setSucceeded();
            else current_goal.setAborted();
        }
        has_goal = false;
    }
    goalLock.unlock();
}

bool JacoTrajectoryActionServer::executionFinished(bool& success)
{
    goalLock.lock();
    bool ret = executionIsFinished;
    success = executionSuccessful;
    goalLock.unlock();
    return ret;
}

int JacoTrajectoryActionServer::waitForExecution(float timeout_secs)
{
    ROS_INFO("Waiting for execution of trajectory.");
    float waited = 0.0;
    float step_sec = 0.2;
    while (true)
    {
        if ((timeout_secs > 0) && (waited > timeout_secs))
        {
            ROS_WARN("Timeout reached waiting for joint trajectory action to finish. Waited %f secs (maximum %f)",
                waited, timeout_secs);
            abortExecution();
            goalLock.lock();
            if (has_goal)
            {
                current_goal.setAborted();
                has_goal = false;
            }
            goalLock.unlock();
            return -1;
        }

        bool cancelled = false;
        // check that preempt has not been requested by the client
        goalLock.lock();
        if (has_goal)
        {
            actionlib_msgs::GoalStatus stat = current_goal.getGoalStatus();
            cancelled = (stat.status == actionlib_msgs::GoalStatus::PREEMPTED)
                        || (stat.status == actionlib_msgs::GoalStatus::ABORTED)
                        || (stat.status == actionlib_msgs::GoalStatus::LOST);
        }
        bool goalValid = has_goal;
        goalLock.unlock();

        if (cancelled || !ros::ok() || !goalValid)
        {
            ROS_INFO("FollowJointTrajectoryAction: Preempted");
            // set the action state to preempted
            return 0;
        }


        const float tolerance = GOAL_TOLERANCE;
        if (trajectoryFinished(tolerance))
        {
            ROS_INFO("WaitForExecution: Trajectory finished.");
            return 1;
        }

        bool executionSuccess;
        if (executionFinished(executionSuccess))
        {
            bool abortWithError = true;
            if (!executionSuccess)
            {
                ROS_ERROR("Execution was finished with an error.");
            }
            else if (!trajectoryFinished(tolerance))
            {
                float maxAngle;
                int maxJoint;
                maxEndpointDiff(maxAngle, maxJoint, current_traj, current_traj_idx, current_traj_group);
                ROS_WARN("Execution finished successful, but we are not at target (max distance %f at joint %i)",
                    maxAngle, maxJoint);
            }
            else
            {
                abortWithError = false;
            }
            abortExecution();
            if (abortWithError)
            {
                goalLock.lock();
                if (has_goal)
                {
                    current_goal.setAborted();
                    has_goal = false;
                }
                goalLock.unlock();
                return -1;
            }
            break;
        }

        // ROS_INFO("Sleep.");
        ros::Duration(step_sec).sleep();

        waited += step_sec;
    }
    ROS_INFO("Waiting for thread...");
    joinExecutionThread();
    ROS_INFO("Trajectory execution finsished.");
    return 1;
}

bool JacoTrajectoryActionServer::currentTargetReached(const trajectory_msgs::JointTrajectoryPoint& point,
            const std::vector<int>& joint_indices, const int group, float tolerance)
{
    if (!hasCurrentGoal()) return true;
    updateCurrentState();

    std::vector<float> target_angles;

    valueLock.lock();
    std::vector<float> curr_angles = current_joint_angles;
    valueLock.unlock();

    setTargetValues(point, joint_indices, group, curr_angles, target_angles, true);

    // for (int i=0; i<curr_angles.size(); ++i) ROS_INFO("%i Curr: %f , Targ: %f (tol %f)",i, curr_angles[i],target_angles[i],tolerance);

    return equalJointFloats(curr_angles, target_angles, tolerance, true);
}

bool JacoTrajectoryActionServer::currentTargetReached(const std::vector<float>& target_angles, float tolerance)
{
    if (!hasCurrentGoal()) return true;
    updateCurrentState();
    valueLock.lock();
    std::vector<float> curr_angles = current_joint_angles;
    valueLock.unlock();
    return equalJointFloats(curr_angles, target_angles, tolerance, true);
}


bool JacoTrajectoryActionServer::atTrajectoryStart(const trajectory_msgs::JointTrajectory& traj,
            const std::vector<int>& joint_indices, int group, float tolerance)
{
    updateCurrentState();

    std::vector<float> target_angles;

    valueLock.lock();
    std::vector<float> curr_angles = current_joint_angles;
    valueLock.unlock();
    goalLock.lock();
    if (traj.points.empty())
    {
        ROS_ERROR("No current trajectory points!");
    }
    else
    {
        setTargetValues(traj.points[0], joint_indices, group, curr_angles, target_angles, true);
    }
    goalLock.unlock();
    MathFunctions::capToPI(curr_angles);
    MathFunctions::capToPI(target_angles);
    bool reached = equalJointFloats(curr_angles, target_angles, tolerance, true);
    if (!reached) {
        ROS_ERROR("atTrajectoryStart:: not at start angles, using tolerance %f",tolerance);
        for (int i=0; i<curr_angles.size(); ++i)
            ROS_ERROR("%i Curr: %f , Targ: %f",i,
                curr_angles[i],target_angles[i]);
    }
    return reached;
}


void JacoTrajectoryActionServer::maxEndpointDiff(float& maxAngle, int& maxJoint, const trajectory_msgs::JointTrajectory& traj,
        const std::vector<int>& joint_indices, const int group)
{
    updateCurrentState();
    valueLock.lock();
    std::vector<float> curr_angles = current_joint_angles;
    valueLock.unlock();

    goalLock.lock();
    std::vector<float> target_angles;
    if (traj.points.empty())
    {
        ROS_ERROR("No current trajectory points!");
    }
    else
    {
        setTargetValues(traj.points.back(), joint_indices, group, curr_angles, target_angles, true);
    }
    goalLock.unlock();
    MathFunctions::capToPI(curr_angles);
    MathFunctions::capToPI(target_angles);
    maxAngle = 0;
    maxJoint = -1;
    maxAngleDiff(curr_angles, target_angles, maxAngle, maxJoint);
}



bool JacoTrajectoryActionServer::trajectoryFinished(float tolerance)
{
    updateCurrentState();
    valueLock.lock();
    std::vector<float> curr_angles = current_joint_angles;
    valueLock.unlock();

    goalLock.lock();
    std::vector<float> target_angles;
    if (current_traj.points.empty())
    {
        ROS_ERROR("No current trajectory points!");
    }
    else
    {
        setTargetValues(current_traj.points.back(), current_traj_idx, current_traj_group, curr_angles, target_angles, true);
    }
    goalLock.unlock();
    MathFunctions::capToPI(curr_angles);
    MathFunctions::capToPI(target_angles);
    bool reached = equalJointFloats(curr_angles, target_angles, tolerance, true);
    /*if (reached) {
        ROS_INFO("trajectoryFinished: reached target angles with tolerance %f",tolerance);
        for (int i=0; i<curr_angles.size(); ++i) ROS_INFO("%i Curr: %f , Targ: %f (tol %f)",i,
            curr_angles[i],target_angles[i],tolerance);
    }*/
    return reached;
}

bool JacoTrajectoryActionServer::getTargetAngles(const trajectory_msgs::JointTrajectoryPoint& p,
        const std::vector<int>& j_idx, std::vector<float>& targetAngles) const
{
    if (j_idx.size() != 9)
    {
        return false;
    }

    // Have to convert to an int, otherwise comparisons below fail if
    // index < 0, because they are converted to unsigned long int.
    int nJoints = p.positions.size();

    if ((j_idx[0] >= nJoints) ||
            (j_idx[1] >= nJoints) ||
            (j_idx[2] >= nJoints) ||
            (j_idx[3] >= nJoints) ||
            (j_idx[4] >= nJoints) ||
            (j_idx[5] >= nJoints) ||
            (j_idx[6] >= nJoints) ||
            (j_idx[7] >= nJoints) ||
            (j_idx[8] >= nJoints))
    {
        ROS_ERROR("One of the indices is out of range (positions size %i): %i %i %i %i %i %i %i %i %i",
                  nJoints, j_idx[0], j_idx[1], j_idx[2], j_idx[3], j_idx[4], j_idx[5], j_idx[6], j_idx[7], j_idx[8]);
        return false;
    }

    targetAngles.clear();
    if (j_idx[0] >= 0) targetAngles.push_back(p.positions[j_idx[0]]);
    if (j_idx[1] >= 0) targetAngles.push_back(p.positions[j_idx[1]]);
    if (j_idx[2] >= 0) targetAngles.push_back(p.positions[j_idx[2]]);
    if (j_idx[3] >= 0) targetAngles.push_back(p.positions[j_idx[3]]);
    if (j_idx[4] >= 0) targetAngles.push_back(p.positions[j_idx[4]]);
    if (j_idx[5] >= 0) targetAngles.push_back(p.positions[j_idx[5]]);
    if (j_idx[6] >= 0) targetAngles.push_back(p.positions[j_idx[6]]);
    if (j_idx[7] >= 0) targetAngles.push_back(p.positions[j_idx[7]]);
    if (j_idx[8] >= 0) targetAngles.push_back(p.positions[j_idx[8]]);

    return true;
}


bool JacoTrajectoryActionServer::getTargetVelocities(const trajectory_msgs::JointTrajectoryPoint& p,
    const std::vector<int>& j_idx, std::vector<float>& targetVels) const
{
    if (j_idx.size() != 9)
    {
        return false;
    }

    // Have to convert to an int, otherwise comparisons below fail if index < 0,
    // because they are converted to unsigned long int.
    int nJoints = p.positions.size();

    if ((j_idx[0] >= nJoints) ||
            (j_idx[1] >= nJoints) ||
            (j_idx[2] >= nJoints) ||
            (j_idx[3] >= nJoints) ||
            (j_idx[4] >= nJoints) ||
            (j_idx[5] >= nJoints) ||
            (j_idx[6] >= nJoints) ||
            (j_idx[7] >= nJoints) ||
            (j_idx[8] >= nJoints))
    {
        ROS_ERROR("One of the indices is out of range (positions size %i): %i %i %i %i %i %i %i %i %i",
                  nJoints, j_idx[0], j_idx[1], j_idx[2], j_idx[3], j_idx[4], j_idx[5], j_idx[6], j_idx[7], j_idx[8]);
        return false;
    }

    targetVels.clear();
    if (j_idx[0] >= 0) targetVels.push_back(p.velocities[j_idx[0]]);
    if (j_idx[1] >= 0) targetVels.push_back(p.velocities[j_idx[1]]);
    if (j_idx[2] >= 0) targetVels.push_back(p.velocities[j_idx[2]]);
    if (j_idx[3] >= 0) targetVels.push_back(p.velocities[j_idx[3]]);
    if (j_idx[4] >= 0) targetVels.push_back(p.velocities[j_idx[4]]);
    if (j_idx[5] >= 0) targetVels.push_back(p.velocities[j_idx[5]]);
    if (j_idx[6] >= 0) targetVels.push_back(p.velocities[j_idx[6]]);
    if (j_idx[7] >= 0) targetVels.push_back(p.velocities[j_idx[7]]);
    if (j_idx[8] >= 0) targetVels.push_back(p.velocities[j_idx[8]]);

    return true;
}



void JacoTrajectoryActionServer::setTargetValues(const trajectory_msgs::JointTrajectoryPoint& p,
        const std::vector<int>& joint_indices,
        const int group, const std::vector<float>& currentState,
        std::vector<float>& targetState, bool usePositions)
{
    if (joint_indices.size() != 9)
    {
        ROS_ERROR("Can't set target values because index array is not of size 9");
        return;
    }
    if (currentState.size() != 9)
    {
        ROS_ERROR("Can't set target values because current state is not of size 9");
        return;
    }

    if (targetState.size() != 9)
    {
        if (usePositions)
        {
            targetState.clear();
            targetState.insert(targetState.begin(), currentState.begin(), currentState.end());
        }
        else
        {
            targetState.resize(9, 0);
        }
    }
    /*  if ((group==0) && (targetState.size()!=9)) targetState.resize(9,0);
        else if ((group==1) && (targetState.size()<6)) targetState.resize(6,0);
        else if ((group==2) && (targetState.size()<3)) targetState.resize(3,0);
    */

    const std::vector<double> * val = &(p.positions);
    double mult = 1.0;
    if (!usePositions)
    {
        val = &(p.velocities);
        mult = TRAJECTORY_PLAY_FACTOR;
    }

    if (group < 2)   // arm joints specified
    {
        // for (int k=0; k<targetState.size(); ++k) ROS_INFO("TAngle1%i: %f",k,p.positions[k]);
        // for (int k=0; k<targetState.size(); ++k) ROS_INFO("TAngle2%i: %f",k,(*val)[k]);
        for (int k=0; k<6; ++k)
        {
            double v = (*val)[joint_indices[k]];
            targetState[k] = v * mult;
        }
    }
    else   // no arm joints specified
    {
        if (usePositions)
        {
            if (currentState.size() != 9)
            {
                ROS_ERROR("Current state out of bounds, needs at least 9 entries");
            }
            targetState = currentState;
        }
        else   // set zero velocities
        {
            for (int k=0; k<6; ++k)
            {
                targetState[k] = 0.0;
            }
        }
    }


    if (group != 1)  // finger joints specified
    {
        for (int k=6; k<9; ++k) 
        {
            double v = (*val)[joint_indices[k]];
            targetState[k] = v * mult;
        }
    }
    else   // no finger joints specified
    {
        if (usePositions)
        {
            for (int k=6; k<9; ++k) 
            {
                targetState[k] = currentState[k];
            }
        }
        else    // set 0 velocities
        {
            for (int k=6; k<9; ++k) 
            {
                targetState[k] = 0.0;
            }
        }
    }
}


bool JacoTrajectoryActionServer::interpolate(const std::vector<float>& v1, const std::vector<float>& v2,
        const float t, std::vector<float>& result)
{
    if (v1.size() != v2.size())
    {
        ROS_ERROR("Can't interpolate vectors of different size");
        return false;
    }

    result.clear();
    for (int i = 0; i != v1.size(); ++i)
    {
        result.push_back((1.0 - t)*v1[i] + t * v2[i]);
    }

    return true;
}



bool JacoTrajectoryActionServer::equalFlt(float first, float second, float tolerance)
{
    return fabs(first - second) < tolerance;
}


bool JacoTrajectoryActionServer::equalJointFloats(const std::vector<float>& first, const std::vector<float>& second,
        float tolerance, bool useMinFingers)
{
    if (first.size() != second.size())
    {
        ROS_ERROR("Inconsistency, containers are different size! %lu %lu", first.size(), second.size());
        return false;
    }
    float minSize = std::min(first.size(), second.size());
    for (int i = 0; i < minSize; ++i)
    {
        // ROS_INFO("Vals: %f %f",first[i],second[i]);
        float tol = tolerance;
        if (useMinFingers && (i >= 6)) tol = std::max(tolerance,
            static_cast<float>(MIN_FINGERS_TOLERANCE));  // fingers can't handle very fine accuracies
        if (fabs(MathFunctions::angleDistance(first[i], second[i])) > tol) return false;
    }
    return true;
}

void JacoTrajectoryActionServer::maxAngleDiff(const std::vector<float>& first,
        const std::vector<float>& second, float& maxVal, int& idx)
{
    int minSize = std::min(first.size(), second.size());
    maxVal = 0;
    idx = -1;
    if (minSize == 0) return;
    idx = 0;
    maxVal = fabs(MathFunctions::angleDistance(first[0], second[0]));
    // ROS_INFO("Vals: %f %f",first[0],second[0]);
    for (int i = 1; i < minSize; ++i)
    {
        // ROS_INFO("Vals: %f %f",first[i],second[i]);
        float _max = std::max(maxVal, static_cast<float>(fabs(MathFunctions::angleDistance(first[i], second[i]))));
        if (_max > maxVal)
        {
            maxVal = _max;
            idx = i;
        }
    }
}

