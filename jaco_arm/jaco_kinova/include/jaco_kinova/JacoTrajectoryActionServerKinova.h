#ifndef JACO_KINOVA_JACOTRAJECTORYACTIONSERVERKINOVA_H
#define JACO_KINOVA_JACOTRAJECTORYACTIONSERVERKINOVA_H

#include <joint_trajectory_execution/TrajectoryActionServer.h>
#include <jaco_joints/JacoJointManager.h>

#include <kinova/KinovaTypes.h>
#include <kinova/Kinova.API.UsbCommandLayerUbuntu.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/FingerPosition.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/controller_info.h>

#include <controller_manager/controller_manager.h>

#include <ros/callback_queue.h>

//velocity limitation (in degrees per second) for joints 1,2,3
#define MAX_SPEED_123_DEG 10
//velocity limitation (in degrees per second) for joints 4,5,6
#define MAX_SPEED_456_DEG 20

namespace jaco_kinova
{

/**
 * \brief Implementation of JacoTrajectoryActionServer which can be used on the real Kinova arm.
 *
 * This class uses the kinova drivers to control the arm, and therefore conflicts with any other
 * libraries which directly use the kinova drivers. It is meant to use *in place of* the kinova_driver::JacoArm
 * (specifically any class using kinova_driver::JacoAPI). Therefore, it supports also the publishing
 * of sensor_msgs::JointState and the support of kinova_msgs/ArmJointAngles.action and
 * kinova_msgs/SetFingersPositionAction.action
 *
 * **Important:**    
 * This class uses jaco_joints::JacoJointManager, which requires the joint names to be loaded via
 * ROS parameter. Please see JacoJointManager class (and its base class) for documentation.
 * 
 * \author Jennifer Buehler
 * \date 2014, revised 05/2016
 */
class JacoTrajectoryActionServerKinova: public joint_trajectory_execution::TrajectoryActionServer, public hardware_interface::RobotHW
{
private:
    typedef actionlib::ActionServer<kinova_msgs::ArmJointAnglesAction> JacoArmAnglesServerT;
    typedef actionlib::ActionServer<kinova_msgs::SetFingersPositionAction> JacoFingerAnglesServerT;
    typedef JacoArmAnglesServerT::GoalHandle AnglesGoalHandle;
    typedef JacoFingerAnglesServerT::GoalHandle FingersGoalHandle;
    enum StateInfoType {POSE, VELOCITY, FORCE};

public:

    const static double DEG_TO_RAD = M_PI / 180.0;
    const static double RAD_TO_DEG = 180.0 / M_PI;

    /**
     * \param _maxSpeed123 velocity limitation (in RAD per second) for joints 1,2,3, sent to kinova driver as limitation
     * \param _maxSpeed456 velocity limitation (in RAD per second) for joints 4,5,6, sent to kinova driver as limitation
     * \param _allMaxVelocities array of size 9 containing all maximum velocities allowed for the joints in a trajectory.
     * \param jointStateTopic if non-empty, this class will also publish JointState on this topic.
     * \param jointStateFreq if jointStateTopic set, this is the publish frequency.
     * \param _useRosControllers use the ros_control interfaces to control the robot angles.
     * \param _useOnlineVelocityControl use the on-line control of the trajectory (method JacoTrajectoryActionServer::playTrajectoryOnlineControlled). Only
     * works if velocity mode is set.
     * \param _interGoalTolerance see superclass. For this implementation, it is not guaranteed to stay within these bounds, but they usually won't be exceeded by much, if they are.
     * allow some safety buffer to ensure accuracy, but the lower this value, the jerkier the trajectory execution.
     */
    JacoTrajectoryActionServerKinova(
        const arm_components_name_manager::ArmComponentsNameManager& armComponents,
        std::string& action_topic_name,
        bool _positionMode,
        std::vector<float>& _allMaxVelocities,
        double _maxSpeed123 = MAX_SPEED_123_DEG * DEG_TO_RAD,
        double _maxSpeed456 =  MAX_SPEED_456_DEG * DEG_TO_RAD,
        const std::string& jointStateTopic = "",
        const float jointStateFreq = 25,
        float _armAngleSafetyLimit = DEFAULT_ANGLE_SAFETY_LIMIT,
        float goalTolerance = DEFAULT_GOAL_TOLERANCE,
        const std::string& arm_angles_topic_name = "/jaco_arm_angles",
        const std::string& finger_angles_topic_name = "/jaco_arm_angles",
        bool _useRosControllers = false,
        bool _useOnlineVelocityControl = true,
        float _interGoalTolerance = 2 * DEFAULT_GOAL_TOLERANCE);

    ~JacoTrajectoryActionServerKinova();

    /**
     * Returns the last updated current state
     * \param type set the type of information to get, either angular pose, velocity of effort
     * \param correct do the angle correction required to translate angles read from jaco arm, and return in radians.
     * \return false if last update not succeeded or if \e type is unknown
     */
    bool getCurrentState(AngularPosition& currP, StateInfoType type, bool correct);

    /**
     * Calls the API to obtain current state of the arm directly (the most recent state possible).
     * Returns false if API call failed.
     * \param type set the type of information to get, either angular pose, velocity of effort
     * \param correct do the angle correction required to translate angles read from jaco arm, and return in radians.
     */
    bool getCurrentStateKinova(AngularPosition& currP, StateInfoType type, bool correct);

protected:

    virtual bool initImpl();

    virtual void shutdownImpl();

    void eraseTrajectories();

    virtual void abortExecutionImpl();

    virtual void joinExecutionThreadImpl();

    virtual void updateCurrentState();

private:

    virtual bool playTrajectoryImplementation(const trajectory_msgs::JointTrajectory& traj, const std::vector<int>& joint_indices, const int group);

    //wrapper method around super::playTrajectoryKinovaSimple. This is needed so that it can be launched in a separate thread with boost::bind despite the protected attribute.
    void _playTrajectoryOnlineControlled(const trajectory_msgs::JointTrajectory traj, const std::vector<int>& joint_indices,
                                         const int group, const float inter_tolerance, const float lagTime);


    void playTrajectoryKinovaSimple(const trajectory_msgs::JointTrajectory traj, const std::vector<int>& joint_indices, const int group);

    void targetsUpdate(const ros::TimerEvent& t);


    /**
     * For use in on-line velocity control. This functions hould be regularly called wiht target velocity values.
     * \param clearPoints clears existing trajectory points before adding the velocities (resets all ongoing actions).
     *   otherwise, ADDS a trajectory point, so if you need to clear other points first,
     *   do so with eraseTrajectories().
     * \return 0 on success, -1 if these angles can't be set, -2 if execution failed
     */
    int setKinovaVelocities(const std::vector<float>& target_vels, float delayTime, bool clearPoints);

    /**
     * Sends a command to kinova to set these target angles.
     * \param useAdvancedMode if true, uses KnvSendAdvanceTrajectory for each trajectory point, else
     *      it uses KnvSendBasicTrajectory
     * \param clearPreviousTrajectories clear any trajectory which may currently be executed
     * \param resetAngularControl reset the arm to "angular" control using KnvSetAngularControl
     * \param tolerance function blocks until the target values are reached with an accuracy of this tolerance
     * \return 0 on success, -1 if these angles can't be set, -2 if execution failed
     */
    int sendKinovaAngles(std::vector<float>& target_angles, const bool& stopWaitFlag, bool clearPreviousTrajectories,
            float tolerance, bool useAdvancedMode = true, bool resetAngularControl = false);

    /**
     * Sends the trajectory points to the kinova driver
     */
    bool sendTrajectoryToKinova(const trajectory_msgs::JointTrajectory& traj, const std::vector<int>& joint_indices, const int group);


    /**
     * \param useAdvanceTrajectory if true, uses KnvSendAdvanceTrajectory for each trajectory point, else
     *      it uses KnvSendBasicTrajectory
     * \param clearExistingExecution clear any trajectory which may currently be executed
     * \param resetAngularControl reset the arm to "angular" control using KnvSetAngularControl
     */
    bool executeOnKinova(const std::vector<TrajectoryPoint*>& kinovaTrajectory, bool clearExistingExecution,
                         bool useAdvanceTrajectory, bool resetAngularControl = false);


    int remainingTrajectoryPoints();


    bool trajectoryPlayed();


    void getCurrentRawState(std::vector<float>& vals);

    void getCurrentCorrectedState(std::vector<float>& vals, const StateInfoType type);

    float capCloseZeros(const float& val, const float tolerance = 1e-04) const;

    /**
     * transforms the joint trajectory point to the Kinova type and adds it to the points, or returns false if joint_indices does not contain all joint indices.
     * \param values has to be of size 6 at least, containing all arm angles or velocities. If of size 9, also finger angles are contained within.
     * \param delayTime if >=0, this time delay is going to be added directly after this point (it will be scaled according to TRAJECTORY_PLAY_FACTOR though!).
     * This only has an effect if asAngularPosition=false, and velocities are used. If this is -1, no wait is added.
     * parameter position is set to false.
     * \param asAngularPosition set to true if the values passed are angular positions, or false if they are velocities.
     */
    bool addTrajectoryPoint(const std::vector<float>& values, std::vector<TrajectoryPoint*>& points, bool asAngularPosition, float delayTime = -1) const;

    /**
     */
    void armAnglesCancelCallback(AnglesGoalHandle& goal);

    /**
     */
    void armAnglesCallback(AnglesGoalHandle& goal);


    /**
     */
    void fingerAnglesCancelCallback(FingersGoalHandle& goal);


    /**
     */
    void fingerAnglesCallback(FingersGoalHandle& goal);


    void waitForTrajectoryPlayed(FingersGoalHandle& fingerGoal);


    void addDelay(const double secs, std::vector<TrajectoryPoint*>& points) const;

    void correctToWrite(std::vector<float>& a, bool positionMode = true) const;

    void correctToWrite(AngularInfo &a, bool positionMode = true) const;


    void correctToWrite(FingersPosition &p, bool positionMode = true) const;

    /**
     * Jaco angles need to be converted. See page 10 of documentation (DH Parameters).
     */
    void correctFromRead(AngularInfo &a, bool isPosition) const;

    void correctFromRead(AngularPosition &a, bool isPosition) const;


    /**
     * Calls the API to obtain current state of the arm. Only pointers which are not NULL will be requested.
     * Returns false if API call failed.
     */
    bool getCurrentStateKinova(AngularPosition * angles, AngularPosition * velocities, AngularPosition * forces);


    /**
     * To be called at a certain rate, calls KINOVA API to read the current state.
     */
    void refreshCurrentState(const ros::TimerEvent& t);

    void jointStatePublish(const ros::TimerEvent& t);

    /**
     * Adapts trajectory point  values. This is needed because the arm does not take the short path when wrapping around the 0 and 2*PI boundaries,
     * e.g. +10 to 350, we have to make it move from +10 to -10 instead. Likewise, from +350 to +10 would have to be transformed to +350 to + 270.
     * \param prevPoint point before the trajectory point
     */
    void adaptKinovaAngles(const TrajectoryPoint& prevPoint, TrajectoryPoint * point) const;

    /**
     * Adapts trajectory point values. This is needed because the arm does not take the short path when wrapping around the 0 and 2*PI boundaries,
     * e.g. +10 to 350, we have to make it move from +10 to -10 instead. Likewise, from +350 to +10 would have to be transformed to +350 to + 270.
     * \param pPrev point before the trajectory point, in Kinova corrected values (see correctToWrite) as obtained wiht getValues().
     * \param pThis values of the trajectory point, in Kinova corrected values (see correctToWrite) as obtained with getValues(). This array will also be corrected
     */
    void adaptKinovaAngles(const std::vector<float>& pPrev, std::vector<float>& pThis, TrajectoryPoint& point) const;

    /**
     * Adapts trajectory point  values. This is needed because the arm does not take the short path when wrapping around the 0 and 2*PI boundaries,
     * e.g. +10 to 350, we have to make it move from +10 to -10 instead. Likewise, from +350 to +10 would have to be transformed to +350 to + 270.
     * \param startState Uncorrected start state of the robot, as read from getCurrentState(startState,POSE,false).
     */
    void adaptKinovaAngles(const AngularPosition& startState, std::vector<TrajectoryPoint*>& traj) const;

    std::string toString(const TrajectoryPoint& p) const;

    std::string toString(const AngularPosition& p) const;

    std::string toString(const AngularInfo& p) const;

    std::string toString(const FingersPosition& p) const;

    std::string toString(const UserPosition& p) const;


    // node for advertising joint states and running the arm and finger servers
    ros::NodeHandle node;

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    double cmdPos[9];
    double cmdVel[9];
    double pos[9];
    double vel[9];
    double eff[9];

    // for online velocity commanding of Jaco.
    // to avoid having to update the newest velocities frequently on the jaco (that causes
    // jerky movement), the last command sent to the arm is saved here, so the same command
    // needs not be updated.
    std::vector<float> lastCmdVel;

    // the time the last velocity command was sent to Jaco. If equals 0 time point / duration,
    // no velocities were sent to the robot yet.
    ros::Time lastCmdVelTime;

    ros::Timer hw_timer; //timer for hwUpdate
    boost::shared_ptr<controller_manager::ControllerManager> cm;

    ros::NodeHandle hw_node;
    ros::CallbackQueue hw_callback_queue;
    //controller manager needs its own callback spinner, otherwise node freezes!
    boost::shared_ptr<ros::AsyncSpinner> hw_spinner;
    bool interfacesRegistered;

    void registerHardwareInterface(const std::string& jointName, int idx);

    void registerHardwareInterfaces();

    // Implement robot-specific resouce management
    bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const;

    void hwUpdate(const ros::TimerEvent& t);

    void printVersion();

    //all joint names of the arm and fingers
    std::vector<std::string> all_joint_names;

    std::vector<float> targetPosValues;
    std::vector<float> targetVelValues;
    std::vector<float> currentAngValues;
    std::vector<float> currentVelValues;
    AngularPosition currentAnglesRaw;
    AngularPosition currentVelocitiesRaw;
    AngularPosition currentEffortsRaw;
    bool lastRawUpdateOK;
    boost::mutex lock; //lock for target[Pos/Vel]Values and current[Ang/Vel]Values
    boost::mutex rawLock; //lock for current[Angles/Velocities/Efforst]Raw and lastRawUpdateOK
    double maxSpeed123, maxSpeed456;

    // Topic where to publish joint states (JointState of the whole arm and fingers).
    // If set to empty string, this class won't publish any joint states.
    std::string JOINT_STATES_TOPIC;
    // publish frequency for joint states, if JOINT_STATES_TOPIC is not empty
    double JOINT_STATES_FREQ;
    // publisher for joint states, used if JOINT_STATES_TOPIC is not empty
    ros::Publisher JointState_pub;
    ros::Timer state_pub_timer; //timer for JointState_pub
    ros::Timer state_update_timer; //timer to update current state
    ros::Timer vel_update_timer; //timer for updating the kinova velocities / positions according to targetPosValues and targetVelValues

    bool initAPI;

    // for sendKinovaAngles() to abort waiting for the completion
    bool stopKinovaAngles;
    // to stop the regular velocity control. This is to make it work with ArmAnglesAction at the same time.
    bool suspendVelocityUpdates;

    // Kinova handles and functions

    //Handle for the library's command layer.
    void * commandLayer_handle;

    //Function pointers to the functions we need
    int (*KnvInitAPI)();
    int (*KnvCloseAPI)();
    int (*KnvSendAdvanceTrajectory)(TrajectoryPoint command);
    int (*KnvStartControlAPI)();
    int (*KnvStopControlAPI)();
    int (*KnvSendBasicTrajectory)(TrajectoryPoint);
    int (*KnvMoveHome)(int&);
    int (*KnvInitFingers)();
    int (*KnvSetAngularControl)();
    int (*KnvGetAngularPosition)(AngularPosition &);
    int (*KnvGetAngularVelocity)(AngularPosition &);
    int (*KnvGetAngularForce)(AngularPosition &);
    int (*KnvGetCartesianPosition)(CartesianPosition &);
    int (*KnvEraseAllTrajectories)();
    int (*KnvGetGlobalTrajectoryInfo)(TrajectoryFIFO &);
    int (*KnvGetActualTrajectoryInfo)(TrajectoryPoint &);
    int (*KnvGetCodeVersion)(int[CODE_VERSION_COUNT]);

    //protects all API calls
    boost::mutex knv_lock;

    //for testing purposes, we also integrate ArmJointAnglesAction
    JacoArmAnglesServerT * angle_server;
    JacoFingerAnglesServerT * finger_server;

    // extra thread that can be used for angle_server and finger_server action reception
    boost::thread * actionThread;

    bool useRosControllers;
    // finger_angle_conv_ratio used to display finger position properly in Rviz
    // Approximative conversion ratio from finger position (0..6400) to joint angle
    // in radians (0.. 1.4) for 3 finger hand
    float finger_conv_ratio;
};

}  //namespace
#endif  // JACO_KINOVA_JACOTRAJECTORYACTIONSERVERKINOVA_H
