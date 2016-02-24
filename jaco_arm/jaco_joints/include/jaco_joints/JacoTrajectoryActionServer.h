#ifndef JACO_JOINTS_JACOTRAJECTORYACTIONSERVER_H
#define JACO_JOINTS_JACOTRAJECTORYACTIONSERVER_H
/**

   Publishes a ROS control_msgs/JoinTrajectoryAction and provides target joint position / velocity to set at the times of playing the trajectory.

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

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <sensor_msgs/JointState.h>

#include <jaco_joints/JacoJointManager.h>

#include <boost/shared_ptr.hpp>

#define DEFAULT_TRAJECTORY_PLAY_FACTOR 1.0
#define DEFAULT_ANGLE_SAFETY_LIMIT 0.3
#define DEFAULT_GOAL_TOLERANCE 0.05

/**
 * \brief Provides a ROS control_msgs/JoinTrajectoryAction and computes target joint position / velocity to set at the times of playing the trajectory.
 *
 * This class exists mainly in order to provide a means to execute a trajectory with some control
 * over the accuracy of execution. The velocity actuators of the real jaco arm are not as accurate
 * as may be required: when executing a trajectory, it may be extremely important to execute it
 * as accurate as possible, as little inaccuracies of only one joint during intermediate trajectory points
 * reflects on the overall pose of the arm. Some motion planners may plan a path such that the arm
 * gets close to obstacles. Little inaccuracies in trajectory execution can then cause collisions with the arm.
 *
 * Also with the newer Gazebo versions (>2), since Joint::SetVelocity() doesn't set the velocity that accurately
 * any more and PID controllers can only do so much, it is necessary to control the trajectory execution and
 * adjust velocities, e.g. backtrack when a position goal has been overshot.
 *
 * The disadvantage of the increased accuracy control is that the execution of the trajectory may not be as
 * smooth. 
 *
 * **Prerequisite**    
 * This implementation only works well if the joint velocities in the robot can be met fairly well
 * according to the target values provided by this class. As a rule of thumb, an error margin of
 * max. 15-20% *can* be acceptable, but the greater the error, the more often the trajectory execution
 * may get stuck and not reach the target points. Underlying PID controllers should be used in the
 * arm controllers to ensure that the error margin is low.
 *
 * **Usage**    
 * A controller which maintains an instance of this class may read the target
 * joint positions/velocities and control the arm accordingly.
 * An array is passed to the constructor (along with a mutex) which will have the current target
 * values updated by this action server as the trajectory is playing.
 * Please refer to the constructor documentation which provides more details on how this
 * class works through the documentation of paramters.
 *
 * **Important TODO**    
 * This class has recently been migrated from JointTrajectoryAction to FollowJointTrajectoryAction.
 * Accordingly, this can be improved to read the tolerances from FollowJointTrajectoryAction. In
 * the current implementation, tolerances are set globally for all joints.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class JacoTrajectoryActionServer
{
protected:
    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTActionServerT;
    typedef JTActionServerT::GoalHandle GoalHandle;

public:
    /**
     * \param _n Node to use for providing the action server
     * \param _action_topic_name topic name for the control_msgs/FollowJointTrajectoryAction.
     * \param _targetPos pointer to target values array, 9 floats for arm and fingers.
     *      The most recent target values while trajectory is played can be read from there.
     *      After the trajectory has finished, this will remain set to the last used values.
     * \param _targetVel pointer to target velocities array, 9 floats for arm and fingers.
     *      The most recent target values while trajectory is played can be read from there.
     *      After the trajectory has finished, this will remain set to the last used values.
     * \param _currentAngles pointer to current position values, 9 floats for arm and
     *      finger angles (not velocities).
     *      This is to determine when the trajectory has finished playing.
     *      The values must externally be kept up-to-date at all times.
     * \param _currentVels pointer to current velocity values, 9 floats for arm and finger velocities. 
     *      This is to determine when the trajectory has finished playing.
     *      The values must externally be kept up-to-date at all times.
     * \param _lock lock to use for \e _targetPos, \e _targetVel, \e _currentAngles and \e _currentVels.
     * \param _positionMode If set to true, the joint trajectory is to be played
     *      using joint positions, and the next trajectory point target position are stored in \e _targetPos,
     *      while \e _targetVel remains undefined.
     *      If set to false, target values will be velocities (stored in \e _targetVel), *and* the
     *      current target trajectory point position values will be stored in \e _targetPos.
     * \param _maxVelocities array of size 9 which contains maximum velocities for all jaco joints.
     *     This only applies if \e _positionMode is false and is reflected in the values of \e targetVel.
     * \param _armAngleSafetyLimit if >=0, trajectories will be rejected which have an angle (radian) greater
     *     than that between two joint positions. If <0, this feature is disabled.
     * \param _goalTolerance the tolerance to allow when detecting whether the arm is at the last trajectory point.
     * \param _interGoalTolerance tolerance to determine whether intermediate trajectory points are reached
     *      *during* execution of trajectory. This only applies when online trajectory execution control
     *      is performed (using \e _useOnlineTrajectoryControl).
     *      It is ensured that each trajectory point is met with this accuracy. The smaller this tolerance,
     *      the more jerky the trajectory execution might be. If this is > goalTolerance,
     *      at the end of trajectory execution a correction might be done to meet the desired end accuracy.
     * \param _simplifyTrajectoryVelocities if set to true, velocities of each incoming trajectory are
     *      simplified to represent linear moves (fixed velocity between trajectory points).
     *      This disregards velocity values originally based on spline interpolations in the trajectory message.
     *      Execution time of trajectory should still be the same but may be adjusted.
     * \param _useOnlineVelocityControl use the on-line control of the trajectory which continually checks
     *      whether the joint has arrived at its intermediate trajectory position.
     *      The trajectory playing is synchronized at each intermediate trajectory point:
     *      if some joints were slower, others wait for it to be at this intermediate
     *      trajectory point. See also documentation of method
     *      JacoTrajectoryActionServer::playTrajectoryOnlineControlled().
     *      Only works if velocity mode is set, so if \e _positionMode is set to true,
     *      it will be reset to false.
     */
    JacoTrajectoryActionServer(
        ros::NodeHandle &_n,
        std::string& _action_topic_name,
        std::vector<float>& _targetPos,
        std::vector<float>& _targetVel,
        std::vector<float>& _currentAngles,
        std::vector<float>& _currentVels,
        boost::mutex& _lock,
        bool _positionMode,
        std::vector<float>& _maxVelocities,
        float _armAngleSafetyLimit = DEFAULT_ANGLE_SAFETY_LIMIT,
        float _goalTolerance = DEFAULT_GOAL_TOLERANCE,
        bool _simplifyTrajectoryVelocities = true,
        bool _useOnlineVelocityControl = true,
        float _interGoalTolerance = 2 * DEFAULT_GOAL_TOLERANCE);

    ~JacoTrajectoryActionServer();

    /**
     * Starts action server and does internal initialisation.
     */
    bool init();

    /**
     * methods to be executed for shutting down the action server
     */
    void shutdown();

    /**
     * if true, the joint trajectory is controlled by joint angle positions, otherwise it is controlled by velocities.
     * either angles or velocities will be stored in the _target vector which is passed in the constructor.
     */
    bool usePositionMode() const
    {
        return positionMode;
    }

    /**
     * \return true if a trajectory goal is currently being executed.
     */
    bool executingGoal();
    /**
     * \return true if there is currently a goal, even if it has not started executing yet.
     */
    bool hasCurrentGoal();

protected:
    void setHasCurrentGoal(bool flag);

    /**
     * Implementation dependent method which is called to play the trajectory.
     * This does *not* include waiting until the trajectory has executed.
     * This function should return immediately, spinning off the actual execution in a separate thread.
     *
     * **Important:**, when execution in the parallel thread is finished, the function
     * setExecutionFinished(true) has to be called from the thread which
     * does the execution, otherwise the ending of the action might not work properly,
     * as it is only determined by the reaching of the last trajectory point.
     *
     * \param joint_indices must contain one index for each arm / finger joint (in total 9 indices) which
     * indicate at which position in the trajectory arrays this joint can be found, or -1 if this joint does
     * not appear in the trajectory. Such array can be obtained with getJointIndices().
     * \param group set to 0 if both arm and finger angles are given. Set to 1 if only arm joints
     * are present, in which case the size has to be 6, or set to 2 if only finger angles are present,
     * when size is 3. This is return value of getJointIndices().
     */
    virtual bool playTrajectoryImplementation(const trajectory_msgs::JointTrajectory& traj,
            const std::vector<int>& joint_indices, const int group);

    /**
     * Implementation dependent initialisation called from init().
     */
    virtual bool initImpl()
    {
        return true;
    }

    /**
     * Implementation methods to be executed for shutting down the action server
     */
    virtual void shutdownImpl() {}

    /**
     * Abort execution which was started (spinned off in separate thread) in playTrajectoryImplementation.
     */
    virtual void abortExecution();

    /**
     * Joins the thread started within playTrajectoryImplementation(), if applicable.
     */
    virtual void joinExecutionThread();

    /**
     * If implementation-specific details require a manual update of the array current_joint_angles
     * and current_joint_vels, this function is used at the points in this code where a
     * fresh version of the current arm angles are required.
     */
    virtual void updateCurrentState() {}


    /**
     * Plays the trajectory by updating the target values subsequently and
     * then sleeping for the duration of the trajectory point. This method is
     * designed to be spinned off in a separate thread from playTrajectoryImplementation().
     */
    void playTrajectorySimple(const trajectory_msgs::JointTrajectory traj,
            const std::vector<int>& joint_indices, const int group);

    /**
     * Plays the trajectory by setting target velocities (writing to array targetVel) and setting them to 0 when the
     * position target is already reached, or correcting if the target has been overshot.
     *
     * The on-line control of the trajectory which continually checks whether the joint has arrived at its
     * intermediate trajectory position. The trajectory playing is synchronized at each intermediate trajectory point:
     * if some joints were slower, others wait for it to be at this intermediate trajectory point. 
     * Only works if velocity mode is set (usePositionMode()==false).
     *
     * \param inter_tolerance target angle tolerance for points inbetween the trajectory.
     *      Can be larger than variable \e GOAL_TOLERANCE
     * \param lagTime the lag it takes for a command to actually apply at the arm.
     *      The trajectory execution will "look ahead" in time to change the velocities.
     */
    void playTrajectoryOnlineControlled(const trajectory_msgs::JointTrajectory traj,
                                        const std::vector<int>& joint_indices,
                                        const int group, const float inter_tolerance, const float lagTime);


    /**
     * Helper function for playTrajectoryOnlineControlled() which waits until all angular poses in
     * \e _targetPos are reached, using \e _initialTargetVel velocities set to get to the goal.
     * Velocities (in field \e targetVel) may be adapted by this function if goal position has to
     * be corrected (to reach accuracy or when goal overshot).
     * To correct small inaccuracies, velocities between \e min_correct_vel and \e max_correct_vel are used.
     */
    bool waitUntilPointReached(const std::vector<float>& _targetPos,
                               const std::vector<float>& _initialTargetVel, float recheckTime,
                               float maxWaitTime, float tolerance, float lagTime,
                               float min_correct_vel, float max_correct_vel);

    /**
     * Helper function which waits repeatedly (maximum \e numTries) until the
     * joint target pose (in \e _targetPos) is reached,
     * then sets velocities to 0, waits until the arm has stopped, and then checks for target pose again.
     * This is for arms which incur lots of inaccuracy in velocities (e.g. 0 detected velocity was not really 0,
     * it was only fluctuating around 0 and then went up again. This is so with the Jaco arm.)
     * \param maxWaitForExact maximum wait for call of waitUntilPointReached().
     * \param maxWaitForZero maximum wait for call of waitUntilVelocitiesZero().
     */
    bool repeatedWaitUntilPointReached(const std::vector<float>& _targetPos, float recheckTime,
        float maxWaitForExact, float maxWaitForZero, float tolerance, float lagTime,
        float min_correct_vel, float max_correct_vel, int numTries);


    /**
     * Helper function which idles (up to maxWaitTime, rechecking every recheckTime seconds)
     * until actual velocities of the arm are 0 within tolerance.
     */
    bool waitUntilVelocitiesZero(float recheckTime, float maxWaitTime, float tolerance);

    /**
     * Set the flag that execution of the current trajectory has finished in the implementation.
     */
    void setExecutionFinished(bool flag, bool success);

    /**
     * Returns flag indicating if execution of trajectory has finished in implementation.
     */
    bool executionFinished(bool& success);

    /**
     * Waits for execution of trajectory. Uses trajectoryFinished() for checking if the target is reached.
     * \param timeout_secs set to timeout in seconds, or -1 if infinite
     * returns 1 on success, 0 if execution was cancelled while waiting, -1 if timed out, -2 on error
     */
    int waitForExecution(float timeout_secs = -1);


    /**
     * Checks if array current_joint_angles (constructor parameter _current) corresponds to the last point in the trajectory.
     * \param tolerance tolerance (in radians) for all angles to be at the target position
     */
    bool trajectoryFinished(float tolerance = 1e-02);


    /**
     * Returns maximum angle distance (and index of which) to tarjectory target
     */
    void maxEndpointDiff(float& maxAngle, int& maxJoint, const trajectory_msgs::JointTrajectory& traj,
            const std::vector<int>& joint_indices, const int group);

    /**
     * Checks whether the robot is at the start of this trajectory
     */
    bool atTrajectoryStart(const trajectory_msgs::JointTrajectory& traj,
            const std::vector<int>& joint_indices, int group, float tolerance);

    /**
     * Checks wheter the current target (set in array targetAngles) is reached by
     * field \e current_joint_angles values. This is not necessarily the
     * end of the trajectory, but can be an intermediate point of the trajectory.
     */
    bool currentTargetReached(const std::vector<float>& targetAngles, float tolerance);

    /**
     * Checks whether target angles in this trajectory point are reached
     */
    bool currentTargetReached(const trajectory_msgs::JointTrajectoryPoint& point,
        const std::vector<int>& joint_indices, const int group, float tolerance);

    /**
     * Checks if trajectory is eligible for execution from the current robot state.
     * If there is a too large distance between two trajectory points or the current
     * robot position and the first trajectory point, this function returns false.
     * \param joint_indices must contain one index for each arm / finger joint (in total 9 indices)
     * which indicate at which position in the trajectory arrays this joint can be found
     * \param group set to 0 if both arm and finger angles are given. Set to 1 if only arm joints
     * are present, in which case the size has to be 6, or set to 2 if only finger angles are present, when size is 3.
     */
    bool checkTrajectory(const trajectory_msgs::JointTrajectory& traj,
        const std::vector<int>& joint_indices, int group);

    /**
     * Removes angles of exactly PI (180 degrees) from trajectory by slightly altering
     * (by value diff) affected points according to velocities in the
     * trajectory. This is to make the direction the arm has to move unambiguous if
     * joint execution is controlled by mere position(angle).
     */
    void adaptTrajectoryAngles(trajectory_msgs::JointTrajectory& traj,
        double diff = 0.02, double epsilon = 1e-03) const;

    /**
     * Because at this stage the velocities coming from MoveIt are not supported
     * (not sure if they are based on some spline interpolation?),
     * this transforms the Joint trajectory such that velocities are based on the
     * time it takes to the next point and the velocity it requires
     * to reach the position in the next point. No interpolation is assumed.
     * If no timings are given in the trajectory, times are added and maximum velocities
     * are assumed.
     */
    bool adaptTrajectoryVelocitiesToLinear(trajectory_msgs::JointTrajectory& traj,
                    const std::vector<int>& joint_indices) const;

    /**
     * Helper function which interpolates between vectors v1 and v2 (must be of same size):
     * 
     *      result[i] = v1[i] + t * (v2[i]-v1[i])
     *
     * which is equivalent to
     *
     *      result[i]=(1-t)*v1[i] + t*v2[i]
     */
    bool interpolate(const std::vector<float>& v1, const std::vector<float>& v2,
                const float t, std::vector<float>& result);


     /**
     * Checks whether angles distances in joint states \e j1 and \e j2 are too far away for given
     * safety measures (\e maxAngle - this is to prevent the arm from doing unpredictable movements).
     * \e j1 and \e j2 can be two trajectory points, or the current state and the first trajectory point.
     * \param check set to 0 if both arm and finger angles are given. In that case, both \e j1 and \e j2
     * have to be of size 9. Set to 1 if only arm joints are present, in which case the size has to
     * be 6, or set to 2 if only finger angles are present, when size is 3.
     */
    bool angleDistanceOK(std::vector<float>& j1, std::vector<float>& j2, int check, float maxAngle) const;

    /**
     * Checks if any of the velcotities specified in the array exceeds maximum velocities
     * specified in constructor.
     * \param check set to 0 if both arm and finger angles are given.
     *      In that case, both j1 and j2 have to be of size 9. Set to 1 if only arm joints
     *      are present, in which case the size has to be 6, or set to 2 if only finger
     *      angles are present, when size is 3.
     */
    bool maxVelExceeded(std::vector<float>& vels, int check, float tolerance_above = 1e-02) const;


    /**
     * Receive a new goal
     */
    void actionCallback(GoalHandle& goal);

    /**
     * Receive a cancel trajectory instruction
     */
    void cancelCallback(GoalHandle& goal);


    bool equalFlt(float first, float second, float tolerance);

    /**
     * checks whether the joint state values are equal. If \e useMinFingerVal is true,
     * the tolerance for the finger positions (6,7,8) is
     * going to be increased to at least a certain value if necessary,
     * as finger accuracy on the real robot is not as good.
     */
    bool equalJointFloats(const std::vector<float>& first, const std::vector<float>& second,
            float tolerance, bool useMinFingerVal);

    /**
     *
     */
    void maxAngleDiff(const std::vector<float>& first, const std::vector<float>& second, float& max, int& idx);

    /**
     * Sets the values in the \e target array according to the trajectory point.
     * \e targetState will be of size 9, and contain either the trajectory point values, or
     * the current state if the trajectory doesn't specify this joint.
     * \param currentState the current state of the jaco arm which is to be used
     * when in position mode, for joints which are not specified in the trajectory,
     * to maintain the current state.
     * \param usePositions use the positions array of the point, or use velocities otherwise
     */
    void setTargetValues(const trajectory_msgs::JointTrajectoryPoint& p,
            const std::vector<int>& joint_indices, const int group,
            const std::vector<float>& currentState,
            std::vector<float>& targetState, bool usePositions);

    /**
     * Returns target pose of the arm specified in trajectory point (in parameter target).
     * First 6 positions are arm angles, last 3 the fingers. If joint angles are
     * not specified in the trajectory point, \e j_idx should contain -1 at the index for the joint.
     * Returns false if one of the indices in \e j_idx is out of range.
     * \param j_idx must contain one index for each arm / finger joint (in total 9 indices)
     *      which indicate at which position in the trajectory point where this joint can be found.
     *      If the index is -1, this joint is not part of the trajectory and will be skipped,
     *      reducing resulting size of target.
     */
    bool getTargetAngles(const trajectory_msgs::JointTrajectoryPoint& p,
            const std::vector<int>& j_idx, std::vector<float>& target) const;

    /**
     * As getTargetAngles(), but instead of positions returns velocities.
     */
    bool getTargetVelocities(const trajectory_msgs::JointTrajectoryPoint& p,
            const std::vector<int>& j_idx, std::vector<float>& target) const;


    /**
     * Removes from values all entries which have a value < 0 in array idx at the
     * same position in the array. Returns false if both arrays are not same size.
     */
    bool removeIrrelevantStates(const std::vector<int>& idx, std::vector<float>& values) const;

    JacoJointManager joints;

    /**
     * Multiply execution time of trajectories by this factor
     */
    float TRAJECTORY_PLAY_FACTOR;

    /**
     * Thread which can be used to play trajectory.
     * Can be used by subclasses classes within playTrajectoryImplementation().
     */
    boost::thread * playThread;

    /**
     * tolerance (in radian) at which a goal is considered reached
     */
    float GOAL_TOLERANCE;

    /**
     * Tolerance of intermediate trajectory points to achieve during execution of trajectory.
     * This only applies when online trajectory execution control is performed
     * (using playTrajectoryOnlineControlled()), which depends on what
     * the class does in playTrajectoryImplementation().
     * It is ensured that each trajectory point is met with this accuracy.
     * The smaller this tolerance,the more jerky the trajectory execution might be.
     * If this is > goalTolerance, at the end of trajectory execution a correction
     * might be done to meet the desired end accuracy.
     */
    float INTER_GOAL_TOLERANCE;

    /**
     * maximum distance allowed between two trajectory points (radians),
     * when \e enableMaxAngleDistSafety is true.
     */
    float ANGLE_SAFETY_LIMIT;

    std::vector<float> maxVelocities;


    /**
     * Use the on-line control of the trajectory
     * (method JacoTrajectoryActionServer::playTrajectoryOnlineControlled).
     * Only works if velocity mode is set.
     */
    bool useOnlineVelocityControl;

protected:
    bool has_goal;
    bool executionIsFinished;
    bool executionSuccessful;
    GoalHandle current_goal;

    trajectory_msgs::JointTrajectory current_traj;

    /**
     * joint indices for current trajectory, as return from getJointIndices()
     */
    std::vector<int> current_traj_idx;

    /**
     * group of joints for current trajectory, as returned from getJointIndices()
     */
    int current_traj_group;

    /**
     * to lock access to executionIsFinished, current_goal,
     * current_traj, current_traj_idx, current_traj_group and has_goal
     */
    boost::mutex goalLock;

private:
    /**
     * to lock access to the fields target and current
     */
    boost::mutex& valueLock;

    /**
     * reference to structure to store target angles/velocities
     */
    std::vector<float>& targetPos;
    std::vector<float>& targetVel;

    /**
     * reference to current joint angle values
     */
    std::vector<float>& current_joint_angles;

    /**
     * reference to current joint velocities
     */
    std::vector<float>& current_joint_vels;

    JTActionServerT * action_server;

    /**
     * If set to true, the target values are angle positions, and the joint trajectory
     * is to be played using target position values.
     * If set to false, target values will be positions
     */
    bool positionMode;

    /**
     * set to false to disable the safety stop (maximum angle in radians that
     * trajectory points are allowed to be apart)
     */
    bool enableMaxAngleDistSafety;

    bool initialized;

    /**
     * if set to true, velocities of each incoming trajectory are simplified to
     * represent linear moving (in given time) from one point to the
     * other. This disregards velocity values originally based on spline
     * interpolations. Execution time of trajectory should still be the same.
     */
    bool simplifyTrajectoryVelocities;
};

typedef boost::shared_ptr<JacoTrajectoryActionServer> JacoTrajectoryActionServerPtr;


#endif  // JACO_JOINTS_JACOTRAJECTORYACTIONSERVER_H
