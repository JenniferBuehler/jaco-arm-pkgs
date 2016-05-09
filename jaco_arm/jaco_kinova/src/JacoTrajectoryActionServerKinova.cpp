#include <jaco_kinova/JacoTrajectoryActionServerKinova.h>
#include <convenience_math_functions/MathFunctions.h>

#include <kinova/KinovaTypes.h>

#include <dlfcn.h>
#include <math.h>

#define JACO_USB_LIBRARY "Kinova.API.USBCommandLayerUbuntu.so"

//value obtained experimentally that is added to the amount of wait time
//between trajectory points when using velocities (we set a velocity at the
//kinova drivers and then wait the amount of time that is allocated between two
//trajectory points). It was found that kinova usually overshoots the goal, and this
//can be corrected by waiting less time. It is not the best solution, but it shall
//do for now
#define ADD_WAIT_TIME -0.015

//this is the maximum difference between time points at which a new update velocities
//command is sent to the kinova driver, if we are using the online velocity control
//(see setKinovaVelocities)
#define MAX_UPDATE_VELOCITIES_TIMEDIFF 1

//XXX HACK: We need to publish values not between -PI and PI for joint 1 and 2, because
//its lower limit is < -PI. It was not possible to change this specify lower limit > higher limit, e.g. 2.5..0.73.
//MoveIt however will detect an angle of 3.0 as invalid if we have limits -3.9..0.73. So we need to publish angles
//between -3.9..0.73 for revolute joint with such limits!
#define DO_JOINT_1_2_PUBLISH_FIX
//low/high joint limit. At some stage, we should read this in from URDF
#define LOW_LIMIT_1 -3.943
#define HIGH_LIMIT_1 0.81
#define LOW_LIMIT_2 -4.395
#define HIGH_LIMIT_2 1.252

using jaco_kinova::JacoTrajectoryActionServerKinova;
using convenience_math_functions::MathFunctions;

/********************************
 ** Helper functions
 ********************************/
namespace jaco_kinova
{

template<typename FloatT>
void getValues(const FingersPosition& a, std::vector<FloatT>& values) 
{
    values.push_back(a.Finger1);
    values.push_back(a.Finger2);
    values.push_back(a.Finger3);
}

template<typename FloatT>
void getValues(const AngularInfo& a, std::vector<FloatT>& values) 
{
    values.push_back(a.Actuator1);
    values.push_back(a.Actuator2);
    values.push_back(a.Actuator3);
    values.push_back(a.Actuator4);
    values.push_back(a.Actuator5);
    values.push_back(a.Actuator6);
}



template<typename FloatT>
void getValues(const UserPosition& p, std::vector<FloatT>& values) 
{
    values.clear();
    getValues(p.Actuators, values);
    getValues(p.Fingers, values);
}


template<typename FloatT>
void getValues(const AngularPosition& p, std::vector<FloatT>& values) 
{
    values.clear();
    getValues(p.Actuators, values);
    getValues(p.Fingers, values);
}

template<typename FloatT>
bool setValues(const std::vector<FloatT>& values, AngularPosition& p) 
{
    if (values.size() != 9) return false;
    setValues(values, p.Actuators);
    setValues(values, p.Fingers);
    return true;
}


template<typename FloatT>
bool setValues(const std::vector<FloatT>& values, FingersPosition& a) 
{
    if (values.size() != 9) return false;
    a.Finger1 = values[6];
    a.Finger2 = values[7];
    a.Finger3 = values[8];
    return true;
}

template<typename FloatT>
bool setValues(const FingersPosition& a, std::vector<FloatT>& values) 
{
    if (values.size() != 9) return false;
    values[6] = a.Finger1;
    values[7] = a.Finger2;
    values[8] = a.Finger3;
    return true;
}


template<typename FloatT>
bool setValues(const std::vector<FloatT>& values, jaco_msgs::FingerPosition& a) 
{
    if (values.size() != 9) return false;
    a.finger1 = values[6];
    a.finger2 = values[7];
    a.finger3 = values[8];
    return true;
}

template<typename FloatT>
bool setValues(const jaco_msgs::FingerPosition& a, std::vector<FloatT>& values) 
{
    if (values.size() != 9) return false;
    values[6] = a.finger1;
    values[7] = a.finger2;
    values[8] = a.finger3;
    return true;
}

template<typename FloatT>
bool setValues(const std::vector<FloatT>& values, AngularInfo& a) 
{
    if (values.size() != 9) return false;
    a.Actuator1 = values[0];
    a.Actuator2 = values[1];
    a.Actuator3 = values[2];
    a.Actuator4 = values[3];
    a.Actuator5 = values[4];
    a.Actuator6 = values[5];
    return true;
}

template<typename FloatT>
void setValues(const std::vector<FloatT>& values, jaco_msgs::JointAngles& a) 
{
    a.joint1 = values[0];
    a.joint2 = values[1];
    a.joint3 = values[2];
    a.joint4 = values[3];
    a.joint5 = values[4];
    a.joint6 = values[5];
}

template<typename FloatT>
void setValues(const jaco_msgs::JointAngles& a, std::vector<FloatT>& values) 
{
    if (values.size() < 6) values.resize(6, 0);
    values[0] = a.joint1;
    values[1] = a.joint2;
    values[2] = a.joint3;
    values[3] = a.joint4;
    values[4] = a.joint5;
    values[5] = a.joint6;
}

template<typename FloatT>
bool setValues(const std::vector<FloatT>& values, UserPosition& p) 
{
    if (values.size() != 9) return false;
    setValues(values, p.Actuators);
    setValues(values, p.Fingers);
    return true;
}




//cut back values to 0..2PI
template<typename FloatT>
FloatT normalize(const FloatT& value) {
    if (fabs(value) > 100000) ROS_ERROR("Too high value in normalize(): %f",value);
    FloatT val=value;
    static FloatT pi_2=2.0*M_PI;
    while (val > pi_2)
        val -= pi_2;
    while (val < 0)
        val += pi_2;

    return val;
}

//cut back values to 0..2PI
void normalize(FingersPosition &a) 
{
    a.Finger1 = normalize(a.Finger1);
    a.Finger2 = normalize(a.Finger2);
    a.Finger3 = normalize(a.Finger3);
}


//cut back values to 0..2PI
void normalize(AngularInfo &a) 
{
    a.Actuator1 = normalize(a.Actuator1);
    a.Actuator2 = normalize(a.Actuator2);
    a.Actuator3 = normalize(a.Actuator3);
    a.Actuator4 = normalize(a.Actuator4);
    a.Actuator5 = normalize(a.Actuator5);
    a.Actuator6 = normalize(a.Actuator6);
}

//cut back values to 0..2PI
void normalize(AngularPosition &a) 
{
    normalize(a.Actuators);
    normalize(a.Fingers);
}


//enforces the bound of the angle to be 0 and 2*PI
template<typename FloatT>
FloatT capTo2PI(const FloatT _v)
{
    return normalize(_v);
    /*static FloatT pi_2=2.0*M_PI;
    FloatT v=_v;
    if ((v < 0) || (v > pi_2)){
        v = fmod(v, pi_2); //cut out mulitples of pi_2
        if (v < 0)
            v += pi_2;
    }
    return v;*/
}

template<typename FloatT>
void capTo2PI(std::vector<FloatT>& v) 
{
    for (typename std::vector<FloatT>::iterator it = v.begin(); it != v.end(); ++it)
    {
        *it = capTo2PI(*it);
    }
}

template<typename FloatT>
bool getCurrentStateKinova(std::vector<FloatT>& states, JacoTrajectoryActionServerKinova::StateInfoType type, bool correct, JacoTrajectoryActionServerKinova* _this)
{
    AngularPosition currP;
    if (!_this->getCurrentState(currP, type, correct))
    {
        return false;
    }
    getValues(currP, states);
    return true;
}



template<typename FloatT>
bool getCurrentState(std::vector<FloatT>& states, JacoTrajectoryActionServerKinova::StateInfoType type, bool correct, JacoTrajectoryActionServerKinova * _this)
{
    AngularPosition _curr;
    if (!_this->getCurrentState(_curr, type, correct))
    {
        ROS_ERROR("State could not be retrieved");
        return false;
    }
    getValues(_curr, states);
    return true;
}




}  // namespace





/*******************************************
 **  JacoTrajectoryActionServerKinova   ****
 *******************************************/






JacoTrajectoryActionServerKinova::JacoTrajectoryActionServerKinova(
    ros::NodeHandle &n,
    std::string& action_topic_name,
    bool _positionMode,
    std::vector<float>& _allMaxVelocities,
    double _maxSpeed123,
    double _maxSpeed456,
    const std::string& jointStateTopic,
    const float jointStateFreq,
    float _armAngleSafetyLimit,
    float goalTolerance,
    const std::string& arm_angles_topic_name,
    const std::string& finger_angles_topic_name,
    bool _useRosControllers,
    bool _useOnlineVelocityControl,
    float _interGoalTolerance):

    JacoTrajectoryActionServer(n, action_topic_name, targetPosValues, targetVelValues,
                               currentAngValues,
                               currentVelValues,
                               lock, _positionMode,
                               _allMaxVelocities,
                               _armAngleSafetyLimit,
                                goalTolerance, true,
                               _useOnlineVelocityControl, _interGoalTolerance),
    joints(), // use default constructor to read from parameters
    maxSpeed123(_maxSpeed123),
    maxSpeed456(_maxSpeed456),
    JOINT_STATES_TOPIC(jointStateTopic),
    JOINT_STATES_FREQ(jointStateFreq),
    initAPI(false),
    stopKinovaAngles(false),
    interfacesRegistered(false),
    lastCmdVelTime(0),
    useRosControllers(_useRosControllers),
    lastRawUpdateOK(true),
    actionThread(NULL)
{
    if (JOINT_STATES_TOPIC != "")
    {
        ROS_INFO("Advertising joint states at %s", JOINT_STATES_TOPIC.c_str());
        JointState_pub = n.advertise<sensor_msgs::JointState>(JOINT_STATES_TOPIC, 5);
        state_pub_timer = n.createTimer(ros::Duration(1.0 / (double)JOINT_STATES_FREQ), &JacoTrajectoryActionServerKinova::jointStatePublish, this);
    }

    std::string prepend = "";
    joints.getJointNames(all_joint_names, true, prepend);

    targetPosValues.resize(9, 0);
    targetVelValues.resize(9, 0);
    currentAngValues.resize(9, 0);
    currentVelValues.resize(9, 0);


    currentAnglesRaw.Actuators.InitStruct();
    currentAnglesRaw.Fingers.InitStruct();
    currentVelocitiesRaw.Actuators.InitStruct();
    currentVelocitiesRaw.Fingers.InitStruct();
    currentEffortsRaw.Actuators.InitStruct();
    currentEffortsRaw.Fingers.InitStruct();

    suspendVelocityUpdates = false;

    angle_server = new JacoArmAnglesServerT(n,
                                            arm_angles_topic_name,
                                            boost::bind(&JacoTrajectoryActionServerKinova::armAnglesCallback, this, _1),
                                            boost::bind(&JacoTrajectoryActionServerKinova::armAnglesCancelCallback, this, _1),
                                            false);

    finger_server = new JacoFingerAnglesServerT(n,
            finger_angles_topic_name,
            boost::bind(&JacoTrajectoryActionServerKinova::fingerAnglesCallback, this, _1),
            boost::bind(&JacoTrajectoryActionServerKinova::fingerAnglesCancelCallback, this, _1),
            false);


    if (useRosControllers)
    {
        hw_timer = n.createTimer(ros::Duration(0.01), &JacoTrajectoryActionServerKinova::hwUpdate, this);
    }

    if (useOnlineVelocityControl || useRosControllers)
    {
        vel_update_timer = n.createTimer(ros::Duration(0.003), &JacoTrajectoryActionServerKinova::targetsUpdate, this);
    }

    state_update_timer = n.createTimer(ros::Duration(0.05), &JacoTrajectoryActionServerKinova::refreshCurrentState, this);

    if (useOnlineVelocityControl && useRosControllers)
    {
        ROS_ERROR("Online velocity control and ros controllers interfere, as they both use the target velocities array");
    }

    lastCmdVel.resize(9, 0);
}

JacoTrajectoryActionServerKinova::~JacoTrajectoryActionServerKinova()
{
    if (initAPI)(*KnvStopControlAPI)();
}

bool JacoTrajectoryActionServerKinova::initImpl()
{
    if (useOnlineVelocityControl && useRosControllers)
    {
        ROS_ERROR("Online velocity control and ros controllers interfere, as they both use the target velocities array");
        return false;
    }

    ROS_INFO("Loading USB library %s", JACO_USB_LIBRARY);

    //load the library
    commandLayer_handle = dlopen(JACO_USB_LIBRARY, RTLD_NOW | RTLD_GLOBAL);
    if (commandLayer_handle == NULL)
    {
        ROS_ERROR("Could not open USB lib!");
        ROS_WARN("%s", dlerror());
        return false;
    }
    else
    {
        ROS_INFO("Opened USB library.");
    }


    //load the functions from the library (Under Windows, use GetProcAddress)
    KnvInitAPI = (int (*)()) dlsym(commandLayer_handle, "InitAPI");
    KnvCloseAPI = (int (*)()) dlsym(commandLayer_handle, "CloseAPI");
    KnvSendBasicTrajectory = (int (*)(TrajectoryPoint))dlsym(commandLayer_handle, "SendBasicTrajectory");
    KnvSendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendAdvanceTrajectory");
    KnvStartControlAPI = (int (*)()) dlsym(commandLayer_handle, "StartControlAPI");
    KnvStopControlAPI = (int (*)()) dlsym(commandLayer_handle, "StopControlAPI");
    KnvMoveHome = (int (*)(int&)) dlsym(commandLayer_handle, "MoveHome");
    KnvInitFingers = (int (*)()) dlsym(commandLayer_handle, "InitFingers");
    KnvSetAngularControl = (int (*)())dlsym(commandLayer_handle, "SetAngularControl");
    KnvGetCartesianPosition = (int (*)(CartesianPosition &))dlsym(commandLayer_handle, "GetCartesianPosition");
    KnvGetAngularPosition = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularPosition");
    KnvGetAngularVelocity = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularVelocity");
    KnvGetAngularForce = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularForce");
    KnvEraseAllTrajectories = (int (*)())dlsym(commandLayer_handle, "EraseAllTrajectories");
    KnvGetGlobalTrajectoryInfo = (int (*)(TrajectoryFIFO &)) dlsym(commandLayer_handle, "GetGlobalTrajectoryInfo");
    KnvGetActualTrajectoryInfo = (int (*)(TrajectoryPoint &)) dlsym(commandLayer_handle, "GetActualTrajectoryInfo");
    KnvGetCodeVersion = (int (*)(std::vector<int> &)) dlsym(commandLayer_handle, "GetCodeVersion");

    //If the was loaded correctly
    if ((KnvInitAPI == NULL) || (KnvCloseAPI == NULL) || (KnvSendAdvanceTrajectory == NULL) || (KnvSendBasicTrajectory == NULL) || (KnvStartControlAPI == NULL)
            || (KnvStopControlAPI == NULL) || (KnvMoveHome == NULL) || (KnvInitFingers == NULL) || (KnvSetAngularControl == NULL) || (KnvEraseAllTrajectories == NULL) ||
            (KnvGetAngularPosition == NULL) || (KnvGetAngularVelocity == NULL) || (KnvGetAngularForce == NULL) || (KnvGetCartesianPosition == NULL)
            || (KnvGetGlobalTrajectoryInfo == NULL) || (KnvGetActualTrajectoryInfo == NULL)
       )
    {
        ROS_ERROR("Unable to initialize the command layer.");
        if (KnvInitAPI == NULL) ROS_ERROR("Init NULL");
        if (KnvCloseAPI == NULL) ROS_ERROR("Close NULL");
        if (KnvSendAdvanceTrajectory == NULL) ROS_ERROR("AdvancedTraj NULL");
        if (KnvStartControlAPI == NULL) ROS_ERROR("StartControl NULL");
        return false;
    }

    {   // scope for mutex
        boost::unique_lock<boost::mutex> lock(knv_lock);
        ROS_INFO("The Kinova command has been initialized correctly. Now calling InitAPI().");
        int result = (*KnvInitAPI)();
        ROS_INFO("result of InitAPI() = %i", result);
        if (result != 1)
        {
            ROS_ERROR("Error initializing Kinova API");
            return false;
        }

        ROS_INFO("Using version: ");
        printVersion();

        result = (*KnvStartControlAPI)();
        ROS_INFO("result of StartControlAPI() = %i", result);
        if (result != 1)
        {
            ROS_ERROR("Could not start control API");
            return false;
        }
        ROS_INFO("TEST: Moving home");
        int data;
        //result=(*KnvMoveHome)(data);

        ROS_INFO("TEST: Init fingers");
        result = (*KnvInitFingers)();

        (*KnvSetAngularControl)();
    }

    //test();
    //test3();
    //test4();

    angle_server->start();
    finger_server->start();

    if (useRosControllers)
    {
        registerHardwareInterfaces();
    }
    initAPI = true;
    return true;
}

void JacoTrajectoryActionServerKinova::shutdownImpl()
{
    if (!initAPI) return;
    ROS_INFO("Calling the method CloseAPI()");
    knv_lock.lock();
    int result = (*KnvCloseAPI)();
    knv_lock.unlock();
    ROS_INFO("result of CloseAPI() = %i", result);
}




bool JacoTrajectoryActionServerKinova::getCurrentStateKinova(AngularPosition * angles, AngularPosition * velocities, AngularPosition * forces)
{
    bool success = false;
    knv_lock.lock();
    int retA = angles == NULL ? 1 : (*KnvGetAngularPosition)(*angles);
    if (retA == 1)
    {
        int retV = velocities == NULL ? 1 : (*KnvGetAngularVelocity)(*velocities);
        if (retV == 1)
        {
            int retF = forces == NULL ? 1 : (*KnvGetAngularForce)(*forces);
            success = (retF == 1);
        }
        else
        {
            ROS_ERROR("Could not obtain velocities");
        }
    }
    else
    {
        ROS_ERROR("Could not obtain angles");
    }
    knv_lock.unlock();
    return success;
}


bool JacoTrajectoryActionServerKinova::getCurrentStateKinova(AngularPosition& currP, StateInfoType type, bool correct)
{
    switch (type)
    {
    case POSE:
    {
        knv_lock.lock();
        int ret = (*KnvGetAngularPosition)(currP);
        knv_lock.unlock();
        if (ret != 1)
        {
            ROS_ERROR("Could not obtain current position of arm");
            return false;
        }
        if (correct)
        {
            correctFromRead(currP, true); //angles coming from jaco arm have to be corrected.
        }
        break;
    }
    case VELOCITY:
    {
        knv_lock.lock();
        int ret = (*KnvGetAngularVelocity)(currP);
        knv_lock.unlock();
        if (ret != 1)
        {
            ROS_ERROR("Could not obtain current velocity of arm");
            return false;
        }
        if (correct)
        {
            correctFromRead(currP, false);
        }
        break;
    }
    case FORCE:
    {
        knv_lock.lock();
        int ret = (*KnvGetAngularForce)(currP);
        knv_lock.unlock();
        if (ret != 1)
        {
            ROS_ERROR("Could not obtain current force of arm");
            return false;
        }
        if (correct)
        {
            correctFromRead(currP, false);
        }
        break;
    }
    default:
    {
        ROS_ERROR("Unknown StateInfoType");
        return false;
    }
    }
    return true;
}


bool JacoTrajectoryActionServerKinova::getCurrentState(AngularPosition& currP, StateInfoType type, bool correct)
{
    rawLock.lock();
    if (!lastRawUpdateOK)
    {
        rawLock.unlock();
        ROS_ERROR("Last raw update was not ok");
        return false;
    }
    switch (type)
    {
    case POSE:
    {
        currP = currentAnglesRaw;
        break;
    }
    case VELOCITY:
    {
        currP = currentVelocitiesRaw;
        break;
    }
    case FORCE:
    {
        currP = currentEffortsRaw;
        break;
    }
    default:
    {
        ROS_ERROR("Unknown StateInfoType");
        break;
    }
    }
    rawLock.unlock();

    if (correct) correctFromRead(currP, type == POSE);

    return true;
}


void JacoTrajectoryActionServerKinova::refreshCurrentState(const ros::TimerEvent& t)
{
    AngularPosition _currentAnglesRaw;
    AngularPosition _currentVelocitiesRaw;
    AngularPosition _currentEffortsRaw;
    bool updAng = true;
    bool updVel = false;
    bool updEff = false;

    //we have to be sure that we really want all the values regularly. Getting the values will block the knvLock, which can
    //prevent regular velocity updated (with sendBasicTrajectory) to be too slow.
    bool ok = getCurrentStateKinova(updAng ? &_currentAnglesRaw : NULL, updVel ? &_currentVelocitiesRaw : NULL, updEff ? &_currentEffortsRaw : NULL);

    if (!updVel)
    {
        _currentVelocitiesRaw.Actuators.InitStruct();
        _currentVelocitiesRaw.Fingers.InitStruct();
    }
    if (!updEff)
    {
        _currentEffortsRaw.Actuators.InitStruct();
        _currentEffortsRaw.Fingers.InitStruct();
    }

    rawLock.lock();
    currentAnglesRaw = _currentAnglesRaw;
    currentVelocitiesRaw = _currentVelocitiesRaw;
    currentEffortsRaw = _currentEffortsRaw;
    lastRawUpdateOK = ok;
    rawLock.unlock();

    //ROS_INFO("Uncorrected: %f %f %f %f %f %f",currentAnglesRaw.Actuators.Actuator1, currentAnglesRaw.Actuators.Actuator2, currentAnglesRaw.Actuators.Actuator3,
    //  currentAnglesRaw.Actuators.Actuator4, currentAnglesRaw.Actuators.Actuator5, currentAnglesRaw.Actuators.Actuator6);
}





void JacoTrajectoryActionServerKinova::eraseTrajectories()
{
    knv_lock.lock();
    (*KnvEraseAllTrajectories)();
    knv_lock.unlock();
}

void JacoTrajectoryActionServerKinova::abortExecutionImpl()
{
    eraseTrajectories();
}

void JacoTrajectoryActionServerKinova::joinExecutionThreadImpl()
{
    if (!goalActive()) return;

    while (!trajectoryPlayed())
    {
        sleep(0.2);
        ROS_INFO("joinExecutionThread: Waiting for trajectory to be played by Kinova");
    }
    ROS_INFO("Kinova thread joined");
}

void JacoTrajectoryActionServerKinova::updateCurrentState()
{
    std::vector<float> currAng;
    getCurrentCorrectedState(currAng, POSE);
    std::vector<float> currVel;
    getCurrentCorrectedState(currVel, VELOCITY);
    lock.lock();
    currentAngValues = currAng;
    currentVelValues = currVel;
    lock.unlock();
}


bool JacoTrajectoryActionServerKinova::playTrajectoryImplementation(const trajectory_msgs::JointTrajectory& traj, const std::vector<int>& joint_indices, const int group)
{
    if (playThread != NULL)
    {
        playThread->interrupt();
        playThread->join();
        delete playThread;
    }

    float lagTime = 0.12;
    if (useOnlineVelocityControl)
    {
        playThread = new boost::thread(boost::bind(&JacoTrajectoryActionServerKinova::_playTrajectoryOnlineControlled, this, traj, joint_indices, group, INTER_GOAL_TOLERANCE, lagTime));
    }
    else
    {
        playThread = new boost::thread(boost::bind(&JacoTrajectoryActionServerKinova::playTrajectoryKinovaSimple, this, traj, joint_indices, group));
    }
    return true;
}

void JacoTrajectoryActionServerKinova::_playTrajectoryOnlineControlled(const trajectory_msgs::JointTrajectory traj, const std::vector<int>& joint_indices, const int group, const float inter_tolerance, const float lagTime)
{
    JacoTrajectoryActionServer::playTrajectoryOnlineControlled(traj, joint_indices, group, inter_tolerance, lagTime);
}

void JacoTrajectoryActionServerKinova::playTrajectoryKinovaSimple(const trajectory_msgs::JointTrajectory traj, const std::vector<int>& joint_indices, const int group)
{

    ROS_INFO("Send off trajectory to Kinova drivers");

    //ROS_INFO("Indices (group %i)",group);
    //for (int k=0; k<joint_indices.size(); ++k) ROS_INFO("IDX%i: %i",k,joint_indices[k]);

    //first, kick off the trajectory
    if (!sendTrajectoryToKinova(traj, joint_indices, group))
    {
        ROS_ERROR("Could not send trajectory to Kinova");
        setExecutionFinished(true, false);
        return;
    }

    ROS_INFO("Waiting for execution of KINOVA driver");

    //now keep track of execution
    float tolerance = GOAL_TOLERANCE;
    if (!usePositionMode()) tolerance = 0.15; //velocities need more tolerance
    if (tolerance > ANGLE_SAFETY_LIMIT) tolerance = ANGLE_SAFETY_LIMIT - 1e-03;

    bool success = true;

    float waitTime = 0;
    float waitDur = 0.05;
    float maxWaitFact = 2;
    float maxWait = traj.points.back().time_from_start.toSec() * maxWaitFact;
    while (!trajectoryFinished(tolerance))
    {
        ros::Duration(waitDur).sleep();
        waitTime += waitDur;
        if (waitTime > maxWait)
        {
            float maxAngle = 0;
            int maxJoint = -1;
            maxEndpointDiff(maxAngle, maxJoint, traj, joint_indices, group);
            ROS_ERROR("Action has not finished executing in time (have waited %fs). max distance to target is %f at joint %i", waitTime, maxAngle, maxJoint);
            success = false;
            break;
        }
        if (!goalActive())
        {
            float maxAngle = 0;
            int maxJoint = -1;
            maxEndpointDiff(maxAngle, maxJoint, traj, joint_indices, group);
            ROS_WARN("Action has been considered to end already, so stopping execution in Kinova driver. Max distance to target is %f at joint %i", maxAngle, maxJoint);
            success = false;
            break;
        }
        //can check if the number of remaining trajectory points is 0, the trajectory on kinova might have been played already.
        if (trajectoryPlayed())
        {
            ROS_INFO("Remaining trajectory points are 0.");
            if (!trajectoryFinished(tolerance))
            {
                ROS_ERROR("Remaining trajectory points are 0, but we are not within goal tolerance of %f rad.", tolerance);
                //success=false;
            }
            break;
        }
    }


    //if we use velocity mode, accuracy might not be that great. Hence, we add the last taret angle
    //position to make last corrections, if we are at least close enough to the goal.
    if (success && !usePositionMode() && !trajectoryFinished(GOAL_TOLERANCE))
    {
        ROS_INFO("Velocities execution of KINOVA almost at target, now check if we can add the last position target at the end, and wait.");

        AngularPosition _currStateUncorr;
        if (!getCurrentState(_currStateUncorr, POSE, false))
        {
            ROS_ERROR("Could not get current state of robot arm to assess if Trajectory is ok");
            return;
        }

        AngularPosition _currState = _currStateUncorr;
        correctFromRead(_currState, true);
        std::vector<float> currState;
        getValues(_currState, currState);
        std::vector<float> currStateUncorr;
        getValues(_currStateUncorr, currStateUncorr);

        trajectory_msgs::JointTrajectoryPoint lastPoint = traj.points.back();
        std::vector<float> endPoint;
        setTargetValues(lastPoint, joint_indices, group, currState, endPoint, true);

        if (!angleDistanceOK(currState, endPoint, group, ANGLE_SAFETY_LIMIT))
        {
            ROS_ERROR("Can't correct last point due to too much inaccuracy.");
        }
        else
        {
            std::vector<TrajectoryPoint*> kinovaTrajectory;
            if (!addTrajectoryPoint(endPoint, kinovaTrajectory, true, -1))
            {
                ROS_ERROR("Could not use last trajectory point to correct last point: not all joints specified");
            }
            else
            {
                //ROS_INFO_STREAM("From "<<toString(_currStateUncorr));
                TrajectoryPoint * added = kinovaTrajectory.back();
                //ROS_INFO_STREAM("To:: "<<toString(*added));
                correctToWrite(endPoint, true);
                adaptKinovaAngles(currStateUncorr, endPoint, *added);
                //ROS_INFO_STREAM("Corr: "<<toString(*added));
                if (!executeOnKinova(kinovaTrajectory, true, true))
                {
                    ROS_WARN("Could not execute last point on trajectory to correct possible velocity inaccuracies in earlier points.");
                }
            }
        }
        ROS_INFO("Now waiting for last point in trajectory to be reached exactly");
        tolerance = GOAL_TOLERANCE;
        while (!trajectoryFinished(tolerance))
        {
            if (!goalActive())
            {
                ROS_WARN("Action has been considered to end already, so stopping execution");
                break;
            }
            //ROS_INFO("Have to wait as current target is not reached yet by arm");
            ros::Duration(waitDur).sleep();
            waitTime += waitDur;
            if (waitTime > maxWait)
            {
                float maxAngle = 0;
                int maxJoint = -1;
                maxEndpointDiff(maxAngle, maxJoint, traj, joint_indices, group);
                ROS_ERROR("Action has not finished executing in time (have waited %fs). max distance to target is %f at joint %i", waitTime, maxAngle, maxJoint);
                success = false;
                break;
            }
            //can check if the number of remaining trajectory points is 0, the trajectory on kinova might have been played already.
            if (trajectoryPlayed())
            {
                ROS_INFO("Remaining trajectory points are 0.");
                if (!trajectoryFinished(tolerance))
                {
                    ROS_ERROR("Remaining trajectory points are 0, but we are not within goal tolerance of %f rad.", tolerance);
                    success = false;
                }
                break;
            }

        }
    }

    ROS_INFO("Trajectory execution on KINOVA finished.");
    setExecutionFinished(true, success);
}

void JacoTrajectoryActionServerKinova::targetsUpdate(const ros::TimerEvent& t)
{


    ros::Duration elapsed_time = ros::Duration(t.current_real - t.last_real);

    static float sumTime = 0;
    static int numTimeSum = 0;

    float elapsed = elapsed_time.toSec();
    sumTime += elapsed;
    ++numTimeSum;

    float avgRate = sumTime / numTimeSum;
    //if (avgRate > 0.01) ROS_WARN("target update for Kinova velocities not fast enough (avg): %f > 0.01",avgRate);
    //if (elapsed > 0.01) ROS_WARN("target update for Kinova velocities not fast enough: %f > 0.01",elapsed);

    if (numTimeSum == 150)
    {
        ROS_INFO("Average joint velocities update rate: %f ", avgRate);
        if (avgRate > 0.01) ROS_WARN("target update for Kinova velocities not fast enough (avg): %f > 0.01", avgRate);
        sumTime = 0;
        numTimeSum = 0;
    }

    if (suspendVelocityUpdates) return;


    if (usePositionMode())
    {
        std::vector<float> target_angles;
        float tolerance = GOAL_TOLERANCE;
        lock.lock();
        target_angles = targetPosValues;
        lock.unlock();

        std::vector<float> current_angles;
        if (!jaco_kinova::getCurrentState(current_angles, POSE, true, this))
        {
            ROS_ERROR("Could not obtain current state");
            return;
        }

        bool targetReached = equalJointFloats(current_angles, target_angles, tolerance, true);
        if (!targetReached)
        {
            bool stop = true;
            ROS_ERROR("Develop me: We can't wait for finishing execution here. Instead, check if we sent the same command already!");
            bool clearPrevious = true;
            int ret = sendKinovaAngles(target_angles, stop, clearPrevious, GOAL_TOLERANCE);

            if (ret < 0)
            {
                ROS_ERROR("Failed to set target angles");
            }
        }
    }
    else
    {
        std::vector<float> target_vels;
        lock.lock();
        target_vels = targetVelValues;
        lock.unlock();
        bool clearPrevious = true;
        if (setKinovaVelocities(target_vels, -1, clearPrevious) < 0)
        {
            ROS_ERROR("Failed to set target velocities");
        }
    }
}


/**
 * For use in on-line velocity control. This functions hould be regularly called wiht target velocity values.
 */
#if 0
bool setKinovaVelocities(const std::vector<float>& current_vels, const std::vector<float>& target_vels)
{

    float toleranceLast = 1e-03;

    ros::Time timeNow = ros::Time::now();

    bool commandChanged = !equalJointFloats(lastCmdVel, target_vels, toleranceLast);

    bool updateSinceLastRequired =
        (lastCmdVelTime.toSec() < 1e-07) ||
        ((timeNow - lastCmdVelTime).toSec() >= MAX_UPDATE_VELOCITIES_TIMEDIFF) ||
        commandChanged;

    float toleranceCurrent = 1e-03;

    if (commandChanged || (!equalJointFloats(current_vels, target_vels, toleranceCurrent) && updateSinceLastRequired))
    {
        /*for (int i=0; i<target_vels.size(); ++i){
            ROS_INFO("Dist %f",MathFunctions::angleDistance(start_vels[i],target_vels[i]));
        }*/

        //ROS_INFO("Velocity update. cmdChanged=%u",commandChanged);
        //ROS_INFO("Cmd: %f %f %f %f %f %f", target_vels[0], target_vels[1], target_vels[2], target_vels[3], target_vels[4], target_vels[5]);

        //ROS_INFO("Velocity update since t=%f (passed: %f). cmdChanged=%i",lastCmdVelTime.toSec(),((timeNow-lastCmdVelTime).toSec()),commandChanged);
        //if (commandChanged) ROS_INFO("Command changed: %f / %f",lastCmdVel[0],target_vels[0]);

        //bool equals=equalJointFloats(current_vels, target_vels, toleranceCurrent);
        //float dist=fabs(MathFunctions::angleDistance(current_vels[0],target_vels[0]));
        //ROS_INFO("Curr vel: %f / %f, equals %i, dist %f",current_vels[0],target_vels[0],equals,dist);


        lastCmdVel = target_vels;
        lastCmdVelTime = ros::Time::now();

        //NOTE: The waitTime is not very accurate on the kinova. It only worked to now accumulate (if !deletePrevious) when using a much shorter
        //time (about 0.5 elapsed_time). This is consistent with the timings being a bit off (see ADD_WAIT_TIME).
        //After some testing: The jerking (and it seems, also random pausing) seems to be happening no matter whether we use or don't use EraseAllTrajectories.
        //The only situation when it does not happen is when we set one long wait and don't interrupt it, or if we set several points and waits.
        //Seems kinova doesn't like to get to the end of a trajectory. It also gets better if we only delete points off the queue when we are really sure
        //they are accumulating.
        float waitTime = MAX_UPDATE_VELOCITIES_TIMEDIFF;
        bool deletePrevious = false;
        int numRem = remainingTrajectoryPoints();
        if (commandChanged || (numRem > 1))
        {
            //avoid accumulating points. It is safer to do numRem>=1 here, but if we trust the last point is
            //about to be finished shortly, the movement gets smoother.
            deletePrevious = true;
        }
        //ROS_INFO("Got remaining: %i",numRem);
        int ret = setKinovaVelocities(target_vels, waitTime, deletePrevious);
        //ROS_INFO("Vel: %f ",target_vels[0]);
        if (ret < 0)
        {
            ROS_ERROR("Failed to set target velocities");
            ROS_INFO("Vel: %f %f %f %f %f %f %f %f %f",
                     target_vels[0], target_vels[1], target_vels[2], target_vels[3],
                     target_vels[4], target_vels[5], target_vels[6], target_vels[7], target_vels[8]);
            return false;
        }
    }
    return true;
}
#endif

int JacoTrajectoryActionServerKinova::setKinovaVelocities(const std::vector<float>& target_vels, float delayTime, bool clearPoints)
{

    //ROS_INFO("Target: %f",target.joint1);
    std::vector<TrajectoryPoint*> kinovaTrajectory;
    if (!addTrajectoryPoint(target_vels, kinovaTrajectory, false, delayTime))
    {
        ROS_ERROR("Could not add kinova trajectory point");
        return -1;
    }

    //ROS_INFO("Executing target velocities");
    if (!executeOnKinova(kinovaTrajectory, clearPoints, false))
    {
        ROS_ERROR("Error with execution of kinova velocities");
        return -2;
    }

    for (std::vector<TrajectoryPoint*>::iterator it = kinovaTrajectory.begin(); it != kinovaTrajectory.end(); ++it) delete *it;

    return 0;

}

int JacoTrajectoryActionServerKinova::sendKinovaAngles(std::vector<float>& target_angles, const bool& stopWaitFlag, bool clearPreviousTrajectories, float tolerance)
{

    AngularPosition startStateUncorr;
    if (!getCurrentState(startStateUncorr, POSE, false))
    {
        ROS_ERROR("Could not get current state of robot arm to assess if Trajectory is ok");
        return -1;
    }


    //ROS_INFO("Target: %f",target.joint1);
    std::vector<TrajectoryPoint*> kinovaTrajectory;
    if (!addTrajectoryPoint(target_angles, kinovaTrajectory, true, -1))
    {
        ROS_ERROR("Could not add kinova trajectory point");
        return -1;
    }

    adaptKinovaAngles(startStateUncorr, kinovaTrajectory);

    ROS_INFO("Executing angles goal");
    if (!executeOnKinova(kinovaTrajectory, clearPreviousTrajectories, true))
    {
        ROS_ERROR("Error with execution of arm angles");
        return -2;
    }

    for (std::vector<TrajectoryPoint*>::iterator it = kinovaTrajectory.begin(); it != kinovaTrajectory.end(); ++it) delete *it;

    ros::Rate r(10);

    bool success = true;

    while (true)
    {
        if (stopWaitFlag || !ros::ok())
        {
            ROS_WARN("Aborting ArmAction execution because cancellation was requested or ROS is not initialized");
            success = false;
            break;
        }

        //ROS_INFO("Correctign: %s",joints.toString().c_str());
        //ROS_INFO("Getting current angles");
        std::vector<float> state;
        if (!jaco_kinova::getCurrentState(state, POSE, true, this))
        {
            ROS_ERROR("Could not obtain current state");
            success = false;
            break;
        }

        MathFunctions::capToPI(state);

        //ROS_INFO("Target: %f - curr %f ",targetA.joint1,curr_angles.joint1);
        //setValues(state,feedback.angles);
        //angle_server->publishFeedback(feedback);

        if (equalJointFloats(state, target_angles, tolerance, true))
        {
            ROS_INFO("Angular control complete.");
            /*ROS_INFO("Target : %f %f %f %f %f %f",
                target_angles[0], target_angles[1], target_angles[2], target_angles[3], target_angles[4],target_angles[5]);
            ROS_INFO("State : %f %f %f %f %f %f",
                state[0], state[1], state[2], state[3], state[4],state[5]);
            */
            /*ROS_INFO("Target differences direct: %f %f %f %f %f %f",
                target_angles[0]-state[0],
                target_angles[1]-state[1],
                target_angles[2]-state[2],
                target_angles[3]-state[3],
                target_angles[4]-state[4],
                target_angles[5]-state[5]);*/

            ROS_INFO("Target differences: %f %f %f %f %f %f",
                     MathFunctions::angleDistance(target_angles[0], state[0]),
                     MathFunctions::angleDistance(target_angles[1], state[1]),
                     MathFunctions::angleDistance(target_angles[2], state[2]),
                     MathFunctions::angleDistance(target_angles[3], state[3]),
                     MathFunctions::angleDistance(target_angles[4], state[4]),
                     MathFunctions::angleDistance(target_angles[5], state[5]));
            break;
        }
        /*if (trajectoryPlayed()) {
            ROS_WARN("Trajectory played, but target not reached yet with tolerance %f",tolerance);
            //success=false;
        }*/
        /*ROS_INFO("Still waiting for target to be reached");
        ROS_INFO("Target : %f %f %f %f %f %f %f %f %f",
            target_angles[0], target_angles[1], target_angles[2], target_angles[3], target_angles[4],target_angles[5], target_angles[6],target_angles[7], target_angles[8]);
        ROS_INFO("State : %f %f %f %f %f %f %f %f %f",
            state[0], state[1], state[2], state[3], state[4],state[5], state[6],state[7], state[8]);*/

        r.sleep();
    }

    ros::Duration(0.2).sleep(); //XXX temporary hack: sleep just in case there is a rest movement!
    ROS_INFO("Execution done.");

    return success ? 0 : -2;
}


bool JacoTrajectoryActionServerKinova::sendTrajectoryToKinova(const trajectory_msgs::JointTrajectory& traj, const std::vector<int>& joint_indices, const int group)
{

    AngularPosition startStateUncorr;
    if (!getCurrentState(startStateUncorr, POSE, false))
    {
        ROS_ERROR("Could not get current state of robot arm to assess if Trajectory is ok");
        return false;
    }

    AngularPosition _startState = startStateUncorr;
    correctFromRead(_startState, true);
    std::vector<float> startState;
    getValues(_startState, startState);

    ROS_INFO("------- sending trajectory to kinova ------");

    bool success = true;
    std::vector<TrajectoryPoint*> kinovaTrajectory;

    float lastTime = 0.0;
    //Transform all trajectory points into Kinova data type
    for (int i = 0; i < traj.points.size(); i++)
    {
        const trajectory_msgs::JointTrajectoryPoint& currPoint = traj.points[i];
        float pointTime = currPoint.time_from_start.toSec();

        std::vector<float> targetState;
        setTargetValues(currPoint, joint_indices, group, startState, targetState, usePositionMode());
        //ROS_INFO("Targets (group %i",group);
        //for (int k=0; k<joint_indices.size(); ++k) ROS_INFO("IDX%i: %i",k,joint_indices[k]);
        //for (int k=0; k<currPoint.positions.size(); ++k) ROS_INFO("POS%i: %f",k,currPoint.positions[k]);
        //for (int k=0; k<currPoint.positions.size(); ++k) ROS_INFO("PsS%i: %f",k,currPoint.positions[joint_indices[k]]);
        //for (int k=0; k<targetState.size(); ++k) ROS_INFO("Target Angle%i: %f",k,targetState[k]);

        bool notLastPointForVels = !usePositionMode() && (i < traj.points.size() - 1);

        float waitTime = (pointTime - lastTime) / (double)TRAJECTORY_PLAY_FACTOR;
        if (notLastPointForVels)
        {
            waitTime = (traj.points[i + 1].time_from_start.toSec() - pointTime) / (double)TRAJECTORY_PLAY_FACTOR;
        }
        ROS_INFO("Wait time: %f", waitTime);
        waitTime += ADD_WAIT_TIME;
        if (waitTime < 0) waitTime = 0;
        if (!addTrajectoryPoint(targetState, kinovaTrajectory, usePositionMode(), waitTime))
        {
            ROS_ERROR("Could not use trajectory point, not all joints specified");
            success = false;
            break;
        }

        if (!success) break;

        lastTime = pointTime;
    }


    if (!success)
    {
        return false;
    }



    //Have to adapt values, because the arm does not take the short path when wrapping around the 0 and 2*PI boundaries,
    //e.g. +10 to 350, we have to make it move from +10 to -10 instead. Likewise, from +350 to +10 would have to be transformed to +350 to + 270.
    if (usePositionMode())
    {
        /*ROS_INFO("Start: %f %f %f %f %f %f",startStateUncorr.Actuators.Actuator1,
            startStateUncorr.Actuators.Actuator2,startStateUncorr.Actuators.Actuator3,
            startStateUncorr.Actuators.Actuator4,startStateUncorr.Actuators.Actuator5,
            startStateUncorr.Actuators.Actuator6);

        for (std::vector<TrajectoryPoint*>::iterator it=kinovaTrajectory.begin(); it!=kinovaTrajectory.end(); ++it) {
            TrajectoryPoint * tp=*it;
            ROS_INFO("UPositions: %f %f %f %f %f %f",tp->Position.Actuators.Actuator1,
                tp->Position.Actuators.Actuator2,tp->Position.Actuators.Actuator3,
                tp->Position.Actuators.Actuator4,tp->Position.Actuators.Actuator5,
                tp->Position.Actuators.Actuator6);
        }*/


        adaptKinovaAngles(startStateUncorr, kinovaTrajectory);

        /*for (std::vector<TrajectoryPoint*>::iterator it=kinovaTrajectory.begin(); it!=kinovaTrajectory.end(); ++it) {
            TrajectoryPoint * tp=*it;
            ROS_INFO("CPositions: %f %f %f %f %f %f",tp->Position.Actuators.Actuator1,
                tp->Position.Actuators.Actuator2,tp->Position.Actuators.Actuator3,
                tp->Position.Actuators.Actuator4,tp->Position.Actuators.Actuator5,
                tp->Position.Actuators.Actuator6);
        }*/
    }




    ROS_INFO("Executing goal");
    success = executeOnKinova(kinovaTrajectory, true, usePositionMode());

    for (std::vector<TrajectoryPoint*>::iterator it = kinovaTrajectory.begin(); it != kinovaTrajectory.end(); ++it) delete *it;

    return success;
}


bool JacoTrajectoryActionServerKinova::executeOnKinova(const std::vector<TrajectoryPoint*>& kinovaTrajectory, bool clearExistingExecution, bool useAdvanceTrajectory, bool resetAngularControl)
{
    bool success = true;
    knv_lock.lock();
    if (clearExistingExecution)(*KnvEraseAllTrajectories)();
    if (resetAngularControl)(*KnvSetAngularControl)();
    for (std::vector<TrajectoryPoint*>::const_iterator it = kinovaTrajectory.begin(); it != kinovaTrajectory.end(); ++it)
    {
        //ROS_INFO_STREAM("Sending target "<<std::endl<<toString((*it)->Position));
        int ret = -1;
        if (useAdvanceTrajectory) ret = (*KnvSendAdvanceTrajectory)(**it);
        else ret = (*KnvSendBasicTrajectory)(**it);
        if (ret != 1)
        {
            ROS_ERROR("Failed sending one trajectory");
            success = false;
            break;
        }
    }
    knv_lock.unlock();

    //int rem=remainingTrajectoryPoints();
    //if (rem > 1) ROS_INFO("Remaining points: %i",rem);

    return success;
}


int JacoTrajectoryActionServerKinova::remainingTrajectoryPoints()
{
    TrajectoryFIFO data;
    TrajectoryPoint point;
    knv_lock.lock();
    int result = (*KnvGetGlobalTrajectoryInfo)(data);
    int result2 = (*KnvGetActualTrajectoryInfo)(point);
    knv_lock.unlock();

    if ((result != 1) || (result2 != 1))
    {
        ROS_ERROR("Could not retrieve arm status");
        return -1;
    }
    return data.TrajectoryCount;
}




bool JacoTrajectoryActionServerKinova::trajectoryPlayed()
{
    TrajectoryFIFO data;
    TrajectoryPoint point;
    knv_lock.lock();
    int result = (*KnvGetGlobalTrajectoryInfo)(data);
    int result2 = (*KnvGetActualTrajectoryInfo)(point);
    knv_lock.unlock();

    if ((result != 1) || (result2 != 1))
    {
        ROS_ERROR("Could not retrieve arm status");
        return false;
    }

    /*std::vector<float> s;
    getCurrentState(s,POSE, false);
    std::cout<<std::endl;
    for (int i=0; i<s.size(); ++i) std::cout<<" "<<s[i];
    std::cout<<std::endl;*/

    //ROS_INFO_STREAM( "Robot internal trajectory FIFO's max size : " << data.MaxSize << " trajectory points" );
    //ROS_INFO_STREAM("Remaining trajectory point count : " << data.TrajectoryCount << " trajectory points" );
    //ROS_INFO_STREAM("Current point: " << std::endl<<toString(point.Position));
    //ROS_INFO_STREAM( "                                    Usage : " << data.UsedPercentage << "%" );

    return (data.TrajectoryCount == 0);
}


void JacoTrajectoryActionServerKinova::getCurrentRawState(std::vector<float>& vals)
{
    AngularPosition startStateUncorr;
    if (!getCurrentState(startStateUncorr, POSE, false))
    {
        ROS_ERROR("Could not get current state of robot arm to assess if Trajectory is ok");
        return;
    }

    AngularPosition startState = startStateUncorr;

    vals.clear();
    getValues(startState, vals);
}



void JacoTrajectoryActionServerKinova::getCurrentCorrectedState(std::vector<float>& vals, const StateInfoType type)
{
    AngularPosition stateUncorr;
    if (!getCurrentState(stateUncorr, type, false))
    {
        ROS_ERROR("Could not get current state of robot arm to assess if Trajectory is ok");
        return;
    }

    AngularPosition state = stateUncorr;
    correctFromRead(state, type == POSE);

    vals.clear();
    getValues(state, vals);

    MathFunctions::capToPI(vals);

}

float JacoTrajectoryActionServerKinova::capCloseZeros(const float& val, const float tolerance) const
{
    if (fabs(val) > tolerance) return val;
    return 0.0;
}


bool JacoTrajectoryActionServerKinova::addTrajectoryPoint(const std::vector<float>& values, std::vector<TrajectoryPoint*>& points, bool asAngularPosition, float delayTime) const
{

    if (values.size() < 6)
    {
        ROS_ERROR("Have to have at least 6 values");
        return false;
    }

    TrajectoryPoint * trajectoryPoint = new TrajectoryPoint;
    trajectoryPoint->InitStruct();


    //Initializing the point.
    /*trajectoryPoint->Limitations.accelerationParameter1 = 0.0f; //Not implemented yet but will be in a future release.
    trajectoryPoint->Limitations.accelerationParameter2 = 0.0f; //Not implemented yet but will be in a future release.
    trajectoryPoint->Limitations.accelerationParameter3 = 0.0f; //Not implemented yet but will be in a future release.
    trajectoryPoint->Limitations.forceParameter1 = 0.0f; //Not implemented yet but will be in a future release.
    trajectoryPoint->Limitations.forceParameter2 = 0.0f; //Not implemented yet but will be in a future release.
    trajectoryPoint->Limitations.forceParameter3 = 0.0f; //Not implemented yet but will be in a future release.
    */
    if (asAngularPosition)
    {
        trajectoryPoint->Limitations.speedParameter1 = maxSpeed123 * RAD_TO_DEG; //We limit the translation velocity of joints 1,2,3 to x deg. per second.
        trajectoryPoint->Limitations.speedParameter2 = maxSpeed456 * RAD_TO_DEG; //joints 4,5,6
        trajectoryPoint->Limitations.speedParameter3 = 0.0f; //not used for now
        trajectoryPoint->LimitationsActive = 1;
    }

    if (asAngularPosition) trajectoryPoint->Position.Type = ANGULAR_POSITION;
    else trajectoryPoint->Position.Type = ANGULAR_VELOCITY;

    trajectoryPoint->Position.Actuators.Actuator1 = capCloseZeros(values[0], 1e-04);
    trajectoryPoint->Position.Actuators.Actuator2 = capCloseZeros(values[1], 1e-04);
    trajectoryPoint->Position.Actuators.Actuator3 = capCloseZeros(values[2], 1e-04);
    trajectoryPoint->Position.Actuators.Actuator4 = capCloseZeros(values[3], 1e-04);
    trajectoryPoint->Position.Actuators.Actuator5 = capCloseZeros(values[4], 1e-04);
    trajectoryPoint->Position.Actuators.Actuator6 = capCloseZeros(values[5], 1e-04);

    //ROS_INFO("PositionsNC: %f %f %f %f %f %f",trajectoryPoint->Position.Actuators.Actuator1,trajectoryPoint->Position.Actuators.Actuator2,trajectoryPoint->Position.Actuators.Actuator3,trajectoryPoint->Position.Actuators.Actuator4,trajectoryPoint->Position.Actuators.Actuator5,trajectoryPoint->Position.Actuators.Actuator6);

    correctToWrite(trajectoryPoint->Position.Actuators, asAngularPosition);

    //ROS_INFO("Positions: %f %f %f %f %f %f",trajectoryPoint->Position.Actuators.Actuator1,trajectoryPoint->Position.Actuators.Actuator2,trajectoryPoint->Position.Actuators.Actuator3,trajectoryPoint->Position.Actuators.Actuator4,trajectoryPoint->Position.Actuators.Actuator5,trajectoryPoint->Position.Actuators.Actuator6);

    //The delay value is only used if the position type is TIME_DELAY but we initialize it anyway.
    trajectoryPoint->Position.Delay = 0.0f;

    //ROS_INFO_STREAM("Target "<<std::endl<<toString(trajectoryPoint->Position)<<" delay time "<<delayTime);

    if (values.size() < 9)
    {
        trajectoryPoint->Position.HandMode = HAND_NOMOVEMENT;
    }
    else
    {
        if (asAngularPosition) trajectoryPoint->Position.HandMode = POSITION_MODE;
        else trajectoryPoint->Position.HandMode = VELOCITY_MODE;
        trajectoryPoint->Position.Fingers.Finger1 = capCloseZeros(values[6], 1e-04);
        trajectoryPoint->Position.Fingers.Finger2 = capCloseZeros(values[7], 1e-04);
        trajectoryPoint->Position.Fingers.Finger3 = capCloseZeros(values[8], 1e-04);

        correctToWrite(trajectoryPoint->Position.Fingers, asAngularPosition);
    }

    points.push_back(trajectoryPoint);

    //if we use velocities, we have to add a delay as well.
    if (!asAngularPosition && (delayTime >= 0))
    {
        //ROS_INFO("Adding delay %f",delayTime / (double)TRAJECTORY_PLAY_FACTOR);
        addDelay(delayTime / (double)TRAJECTORY_PLAY_FACTOR, points);
    }
    return true;
}


void JacoTrajectoryActionServerKinova::armAnglesCancelCallback(AnglesGoalHandle& goal)
{
    ROS_INFO("Cancelling arm action");
    stopKinovaAngles = true;
}


void JacoTrajectoryActionServerKinova::armAnglesCallback(AnglesGoalHandle& goal)
{

    if (hasCurrentGoal())
    {
        ROS_ERROR("Currently has a goal, can't accept");
        goal.setRejected();
        return;
    }
    setHasCurrentGoal(true);

    typename JacoArmAnglesServerT::Feedback feedback;
    typename JacoArmAnglesServerT::Result result;

    ROS_INFO("Got an angular goal for the arm.");// Joints: %s",joints.toString().c_str());


    std::vector<float> target_angles;
    if (!jaco_kinova::getCurrentState(target_angles, POSE, true, this))
    {
        ROS_ERROR("Could not obtain current state");
        goal.setAborted();
        return;
    }

    setValues(goal.getGoal()->angles, target_angles);
    MathFunctions::capToPI(target_angles);

    goal.setAccepted();

    suspendVelocityUpdates = true;

    ROS_INFO("Sending target values...");
    bool clearPrevious = true;
    int ret = sendKinovaAngles(target_angles, stopKinovaAngles, clearPrevious, GOAL_TOLERANCE);
    ROS_INFO("... values reached");

    suspendVelocityUpdates = false;

    if (ret < 0)
    {
        goal.setAborted();
    }
    else
    {
        std::vector<float> state;
        if (!jaco_kinova::getCurrentState(state, POSE, true, this))
        {
            ROS_ERROR("Could not obtain current state");
            goal.setAborted();
        }
        else
        {
            setValues(state, result.angles);
            goal.setSucceeded(result);
        }
    }
    setHasCurrentGoal(false);
}


void JacoTrajectoryActionServerKinova::fingerAnglesCancelCallback(FingersGoalHandle& goal)
{
    ROS_INFO("Cancelling fingers action");
    stopKinovaAngles = true;
    if (!usePositionMode())
    {
        abortExecution(); //stop fake trajectory execution
        if (actionThread)
        {
            actionThread->interrupt();
            actionThread->join();
            delete actionThread;
            actionThread = NULL;
        }
    }
}


void JacoTrajectoryActionServerKinova::fingerAnglesCallback(FingersGoalHandle& goal)
{

    if (hasCurrentGoal())
    {
        ROS_ERROR("Currently has a goal, can't accept");
        goal.setRejected();
        return;
    }
    setHasCurrentGoal(true);

    typename JacoFingerAnglesServerT::Feedback feedback;
    typename JacoFingerAnglesServerT::Result result;

    ROS_INFO("Got an angular goal for the fingers.");// Joints: %s",joints.toString().c_str());

    std::vector<float> curr_angles;
    if (!jaco_kinova::getCurrentState(curr_angles, POSE, true, this))
    {
        ROS_ERROR("Could not obtain current state");
        goal.setAborted();
        return;
    }

    std::vector<float> target_angles = curr_angles;
    jaco_msgs::FingerPosition fp = goal.getGoal()->fingers;
    if (!setValues(fp, target_angles))
    {
        ROS_ERROR("Could not set target finger angles");
        goal.setAborted();
        return;
    }

    MathFunctions::capToPI(target_angles);

    ROS_INFO("Setting finger goal to accepted");
    goal.setAccepted();
    ROS_INFO("Done.");

    if (usePositionMode())
    {
        //use sendKinvoaAngles to move the fingers. The problem with this is that we will use the angular positioning mode
        //(not velocity mode) for the whole arm. This causes the arm to drop a little, as the angular positioning mode is not
        //accurate at all. Hence, it is better to use velocity mode.
        //However, since in position mode we are using sendKinovaAngles() anyway, we might as well do the same here.
        suspendVelocityUpdates = true;
        ROS_INFO("Sending target values...");
        bool clearPrevious = true;
        int ret = sendKinovaAngles(target_angles, stopKinovaAngles, clearPrevious, std::max(GOAL_TOLERANCE, (float)3e-02)); //this function can't be more accurate than 3e-01.
        ROS_INFO("... values reached");
        suspendVelocityUpdates = false;
        if (ret < 0)
        {
            goal.setAborted();
        }
        else
        {
            std::vector<float> state;
            if (!jaco_kinova::getCurrentState(state, POSE, true, this))
            {
                ROS_ERROR("Could not obtain current state");
                goal.setAborted();
            }
            else
            {
                setValues(state, result.fingers);
                goal.setSucceeded(result);
            }
        }
        setHasCurrentGoal(false);
    }
    else
    {
        float diff1 = target_angles[6] - curr_angles[6];
        float diff2 = target_angles[7] - curr_angles[7];
        float diff3 = target_angles[8] - curr_angles[8];
        float maxDiff = std::max(fabs(diff1), std::max(fabs(diff2), fabs(diff3)));

        //max allowed velocity for all fingers
        float maxVel = std::min(maxVelocities[6], std::min(maxVelocities[7], maxVelocities[8]));
        //with the given velocity, determine how long it will take to get to reach the target
        float time = maxDiff / maxVel;

        //just make a simple hack and construct a trajectory in order to use existing functions.
        trajectory_msgs::JointTrajectory j;
        joints.getJointNames(j.joint_names, true, "");
        std::vector<int> idx;

        //set values for the two trajectory points: current value and target value
        trajectory_msgs::JointTrajectoryPoint p1, p2;
        for (int i = 0; i < 9; ++i)
        {
            idx.push_back(i);
            p1.positions.push_back(curr_angles[i]);
            p1.velocities.push_back(0);
            p1.effort.push_back(0);
            p2.positions.push_back(target_angles[i]);
            p2.velocities.push_back(0);
            p2.effort.push_back(0);
        }

        p1.velocities[6] = diff1 > 0 ? maxVelocities[6] : -maxVelocities[6];
        p1.velocities[7] = diff2 > 0 ? maxVelocities[7] : -maxVelocities[7];
        p1.velocities[8] = diff3 > 0 ? maxVelocities[8] : -maxVelocities[8];

        p1.time_from_start = ros::Duration(0);
        p2.time_from_start = ros::Duration(time);
        j.points.push_back(p1);
        j.points.push_back(p2);

        if (!playTrajectoryImplementation(j, idx, 0))
        {
            ROS_ERROR("Could not play trajectory");
            goal.setAborted();
        }
        else
        {
            if (actionThread)
            {
                actionThread->join();
                delete actionThread;
            }
            actionThread = new boost::thread(boost::bind(&JacoTrajectoryActionServerKinova::waitForTrajectoryPlayed, this, goal));
        }
    }
}


void JacoTrajectoryActionServerKinova::waitForTrajectoryPlayed(FingersGoalHandle& fingerGoal)
{
    bool playSuccess = false;
    //wait for execution to be finished. Ideally we should launch the wait in a separate thread...
    while (!executionFinished(playSuccess))
    {
        ros::Duration(0.2).sleep();
    }
    if (!playSuccess)
    {
        fingerGoal.setAborted();
    }
    else
    {
        std::vector<float> state;
        if (!jaco_kinova::getCurrentState(state, POSE, true, this))
        {
            ROS_ERROR("Could not obtain current state");
            fingerGoal.setAborted();
        }
        else
        {
            typename JacoFingerAnglesServerT::Result result;
            setValues(state, result.fingers);
            ROS_INFO("Setting finger goal succeeded");
            fingerGoal.setSucceeded(result);
            ROS_INFO("Done.");
        }
    }
    setHasCurrentGoal(false);
}


void JacoTrajectoryActionServerKinova::addDelay(const double secs, std::vector<TrajectoryPoint*>& points) const
{
    //just send a delay to the arm
    TrajectoryPoint * delay = new TrajectoryPoint();
    delay->InitStruct();
    delay->LimitationsActive = 0;
    delay->Position.Type = TIME_DELAY;
    delay->Position.Delay = secs;
    points.push_back(delay);
}

void JacoTrajectoryActionServerKinova::correctToWrite(std::vector<float>& a, bool positionMode) const
{
    if (a.size() != 9)
    {
        ROS_ERROR("Can onlys upport correctToWrite() for all 9 values at this stage");
        return;
    }
    if (positionMode)
    {
        a[0] = MathFunctions::capToPI(a[0]);
        a[1] = MathFunctions::capToPI(a[1]);
        a[2] = MathFunctions::capToPI(a[2]);
        a[3] = MathFunctions::capToPI(a[3]);
        a[4] = MathFunctions::capToPI(a[4]);
        a[5] = MathFunctions::capToPI(a[5]);
        a[6] = MathFunctions::capToPI(a[6]);
        a[7] = MathFunctions::capToPI(a[7]);
        a[8] = MathFunctions::capToPI(a[8]);
        //ROS_INFO("Positions2PI: %f %f %f %f %f %f",a[,a[,a[,a[,a[,a[);
        a[0] = fmod(180.0 - (a[0] * RAD_TO_DEG), 360);
        a[1] = fmod((a[1] * RAD_TO_DEG) + 270.0, 360);
        a[2] = fmod(90.0 - (a[2] * RAD_TO_DEG), 360);
        a[3] = fmod(180.0 - (a[3] * RAD_TO_DEG), 360);
        a[4] = fmod(180.0 - (a[4] * RAD_TO_DEG), 360);
        a[5] = fmod(260.0 - (a[5] * RAD_TO_DEG), 360);
        a[6] = fmod(a[6] * RAD_TO_DEG, 360);
        a[7] = fmod(a[7] * RAD_TO_DEG, 360);
        a[8] = fmod(a[8] * RAD_TO_DEG, 360);
    }
    else
    {
        a[0] = -a[0] * RAD_TO_DEG;
        a[1] = a[1] * RAD_TO_DEG;
        a[2] = -a[2] * RAD_TO_DEG;
        a[3] = -a[3] * RAD_TO_DEG;
        a[4] = -a[4] * RAD_TO_DEG;
        a[5] = -a[5] * RAD_TO_DEG;
        a[6] = a[6] * RAD_TO_DEG;
        a[7] = a[7] * RAD_TO_DEG;
        a[8] = a[8] * RAD_TO_DEG;
    }
}


void JacoTrajectoryActionServerKinova::correctToWrite(AngularInfo &a, bool positionMode) const
{
    if (positionMode)
    {
        a.Actuator1 = MathFunctions::capToPI((double) a.Actuator1);
        a.Actuator2 = MathFunctions::capToPI((double) a.Actuator2);
        a.Actuator3 = MathFunctions::capToPI((double) a.Actuator3);
        a.Actuator4 = MathFunctions::capToPI((double) a.Actuator4);
        a.Actuator5 = MathFunctions::capToPI((double) a.Actuator5);
        a.Actuator6 = MathFunctions::capToPI((double) a.Actuator6);
        //ROS_INFO("Positions2PI: %f %f %f %f %f %f",a.Actuator1,a.Actuator2,a.Actuator3,a.Actuator4,a.Actuator5,a.Actuator6);
        a.Actuator1 = fmod(180.0 - ((double) a.Actuator1 * RAD_TO_DEG), 360);
        a.Actuator2 = fmod(((double) a.Actuator2 * RAD_TO_DEG) + 270.0, 360);
        a.Actuator3 = fmod(90.0 - ((double) a.Actuator3 * RAD_TO_DEG), 360);
        a.Actuator4 = fmod(180.0 - ((double) a.Actuator4 * RAD_TO_DEG), 360);
        a.Actuator5 = fmod(180.0 - ((double) a.Actuator5 * RAD_TO_DEG), 360);
        a.Actuator6 = fmod(268.0 - ((double) a.Actuator6 * RAD_TO_DEG), 360); //XXX in documentation it says use 260, but that gives wrong angles!
    }
    else
    {
        a.Actuator1 = -(double) a.Actuator1 * RAD_TO_DEG;
        a.Actuator2 = (double) a.Actuator2 * RAD_TO_DEG;
        a.Actuator3 = -(double) a.Actuator3 * RAD_TO_DEG;
        a.Actuator4 = -(double) a.Actuator4 * RAD_TO_DEG;
        a.Actuator5 = -(double) a.Actuator5 * RAD_TO_DEG;
        a.Actuator6 = -(double) a.Actuator6 * RAD_TO_DEG;
    }
}


void JacoTrajectoryActionServerKinova::correctToWrite(FingersPosition &p, bool positionMode)const
{
    if (positionMode)
    {
        p.Finger1 = MathFunctions::capToPI((double) p.Finger1);
        p.Finger2 = MathFunctions::capToPI((double) p.Finger2);
        p.Finger3 = MathFunctions::capToPI((double) p.Finger3);
        p.Finger1 = fmod(((double) p.Finger1 * RAD_TO_DEG), 360);
        p.Finger2 = fmod(((double) p.Finger2 * RAD_TO_DEG), 360);
        p.Finger3 = fmod(((double) p.Finger3 * RAD_TO_DEG), 360);
    }
    else
    {
        p.Finger1 = (double)p.Finger1 * RAD_TO_DEG;
        p.Finger2 = (double)p.Finger2 * RAD_TO_DEG;
        p.Finger3 = (double)p.Finger3 * RAD_TO_DEG;
    }
}

void JacoTrajectoryActionServerKinova::correctFromRead(AngularInfo &a, bool isPosition)const
{
    if (isPosition)
    {
        a.Actuator1 = (double)(180.0 - (double)a.Actuator1) * DEG_TO_RAD;
        a.Actuator2 = (double)((double) a.Actuator2 - 270.0) * DEG_TO_RAD;
        a.Actuator3 = (double)(90.0 - (double) a.Actuator3)  * DEG_TO_RAD;
        a.Actuator4 = (double)(180.0 - (double) a.Actuator4) * DEG_TO_RAD;
        a.Actuator5 = (double)(180.0 - (double) a.Actuator5) * DEG_TO_RAD;
        a.Actuator6 = (double)(268.0 - (double) a.Actuator6) * DEG_TO_RAD; //XXX in documentation it says use 260, but that gives wrong angles!
        normalize(a);
    }
    else
    {
        a.Actuator1 = -(double) a.Actuator1 * DEG_TO_RAD;
        a.Actuator2 = (double) a.Actuator2 * DEG_TO_RAD;
        a.Actuator3 = -(double) a.Actuator3 * DEG_TO_RAD;
        a.Actuator4 = -(double) a.Actuator4 * DEG_TO_RAD;
        a.Actuator5 = -(double) a.Actuator5 * DEG_TO_RAD;
        a.Actuator6 = -(double) a.Actuator6 * DEG_TO_RAD;
    }
}

void JacoTrajectoryActionServerKinova::correctFromRead(AngularPosition &a, bool isPosition) const
{
    correctFromRead(a.Actuators, isPosition);
    a.Fingers.Finger1 *= DEG_TO_RAD;
    a.Fingers.Finger2 *= DEG_TO_RAD;
    a.Fingers.Finger3 *= DEG_TO_RAD;
    normalize(a.Fingers);
}




void JacoTrajectoryActionServerKinova::jointStatePublish(const ros::TimerEvent& t)
{


    if (JointState_pub.getNumSubscribers() < 1)
    {
        //ROS_INFO("No subscribers");
        return;
    }

    //ROS_INFO("Publishing joint state");
    sensor_msgs::JointState joint_state;

    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = ""; //for angles, the frame does not matter

    joint_state.name = all_joint_names;

    if (!jaco_kinova::getCurrentState(joint_state.position, POSE, true, this))
    {
        ROS_ERROR("Could not obtain current pose");
        return;
    }

    MathFunctions::capToPI(joint_state.position);

#ifdef DO_JOINT_1_2_PUBLISH_FIX
    double lowLimit = LOW_LIMIT_1;
    double upLimit = HIGH_LIMIT_1;
    double currAngle = joint_state.position[1];
    //ROS_INFO("Joint limits: %f, %f",lowLimit,upLimit);
    if (currAngle > upLimit)
    {
        double newAngle = -M_PI - (M_PI - currAngle);
        if (fabs(currAngle - upLimit) < 1e-02) newAngle = upLimit; //correct rounding errors which can be significant on kinova
        //ROS_INFO("Correcting %f to %f",currAngle,newAngle);
        currAngle = newAngle;
    }

    joint_state.position[1] = currAngle;

    lowLimit = LOW_LIMIT_2;
    upLimit = HIGH_LIMIT_2;
    currAngle = joint_state.position[2];
    if (currAngle > upLimit)
    {
        double newAngle = -M_PI - (M_PI - currAngle);
        if (fabs(currAngle - upLimit) < 1e-02) newAngle = upLimit; //correct rounding errors which can be significant on kinova
        //ROS_INFO("Correcting %f to %f",currAngle,newAngle);
        currAngle = newAngle;
    }
    joint_state.position[2] = currAngle;
#endif

    if (!jaco_kinova::getCurrentState(joint_state.velocity, VELOCITY, true, this))
    {
        ROS_ERROR("Could not obtain current velocity");
        return;
    }

    if (!jaco_kinova::getCurrentState(joint_state.effort, FORCE, true, this))
    {
        ROS_ERROR("Could not obtain current force");
        return;
    }

    JointState_pub.publish(joint_state);
}


void JacoTrajectoryActionServerKinova::adaptKinovaAngles(const TrajectoryPoint& prevPoint, TrajectoryPoint * point) const
{
    std::vector<float> pPrev;
    getValues(prevPoint.Position, pPrev);
    std::vector<float> pThis;
    getValues(point->Position, pThis);
    adaptKinovaAngles(pPrev, pThis, *point);
}

void JacoTrajectoryActionServerKinova::adaptKinovaAngles(const std::vector<float>& pPrev, std::vector<float>& pThis, TrajectoryPoint& point) const
{
    for (int i = 0; i < 9; ++i)
    {
        float p1 = capTo2PI(pPrev[i] * DEG_TO_RAD);
        float p2 = capTo2PI(pThis[i] * DEG_TO_RAD);
        float dist = MathFunctions::angleDistance(p1, p2);
        pThis[i] = pPrev[i] + dist * RAD_TO_DEG;
        if ((i != 1) && (i != 2) && fabs(pThis[i] > 10000))
        {
            //all angles except 2 and 3 have limits -10.000 .. 10.000. For 2 and 3, limits won't exceed due
            //to physical limitations which should be respected by the motion command.
            ROS_INFO("Kinova angle %i exceeded 10.000, correcting value %f", i, pThis[i]);
            if (pThis[i] > 10000) pThis[i] -= 20000;
            if (pThis[i] < -10000) pThis[i] += 20000;
            ROS_INFO("Corrected, now %f", pThis[i]);
        }
    }
    setValues(pThis, point.Position);
}

void JacoTrajectoryActionServerKinova::adaptKinovaAngles(const AngularPosition& startState, std::vector<TrajectoryPoint*>& traj) const
{

    std::vector<float> startPoint;
    getValues(startState, startPoint);

    for (std::vector<TrajectoryPoint*>::iterator it = traj.begin(); it != traj.end(); ++it)
    {
        TrajectoryPoint * point = *it;
        std::vector<float> vals;
        getValues(point->Position, vals);
        adaptKinovaAngles(startPoint, vals, *point);
        /*for (int i=0; i<9; ++i) {
            float p1=capTo2PI(startPoint[i]*DEG_TO_RAD);
            float p2=capTo2PI(vals[i]*DEG_TO_RAD);
            float dist=MathFunctions::angleDistance(p1,p2);
            //if (fabs(p1-p2)>1e-02) ROS_INFO("%f   %f   (distance %f / %f) -->   %f",point[i],vals[i],dist,dist*RAD_TO_DEG,point[i]+dist*RAD_TO_DEG);
            vals[i]=startPoint[i]+dist*RAD_TO_DEG;
        }
        setValues(vals,point->Position);*/
        startPoint = vals;
    }
}


std::string JacoTrajectoryActionServerKinova::toString(const TrajectoryPoint& p) const
{
    std::stringstream str;
    str << toString(p.Position);
    return str.str();
}

std::string JacoTrajectoryActionServerKinova::toString(const AngularPosition& p) const
{
    std::stringstream str;
    str << toString(p.Actuators) << toString(p.Fingers);
    return str.str();
}

std::string JacoTrajectoryActionServerKinova::toString(const AngularInfo& p) const
{
    std::stringstream str;
    str << "Actuator 1 : " << p.Actuator1 << std::endl;
    str << "Actuator 2 : " << p.Actuator2 << std::endl;
    str << "Actuator 3 : " << p.Actuator3 << std::endl;
    str << "Actuator 4 : " << p.Actuator4 << std::endl;
    str << "Actuator 5 : " << p.Actuator5 << std::endl;
    str << "Actuator 6 : " << p.Actuator6 << std::endl;
    return str.str();
}
std::string JacoTrajectoryActionServerKinova::toString(const FingersPosition& p) const
{
    std::stringstream str;
    str << " Finger 1 : " << p.Finger1 << std::endl;
    str << " Finger 2 : " << p.Finger2 << std::endl;
    str << " Finger 3 : " << p.Finger3 << std::endl;
    return str.str();
}



std::string JacoTrajectoryActionServerKinova::toString(const UserPosition& p) const
{
    std::stringstream str;
    str << toString(p.Actuators);
    /*  str << "        X : " << p.CartesianPosition.X << " m" << std::endl;
        str << "        Y : " << p.CartesianPosition.Y << " m" << std::endl;
        str << "        Z : " << p.CartesianPosition.Z << " m" << std::endl;
        str << "   ThetaX : " << p.CartesianPosition.ThetaX << " RAD" << std::endl;
        str << "   ThetaY : " << p.CartesianPosition.ThetaY << " RAD" << std::endl;
        str << "   ThetaZ : " << p.CartesianPosition.ThetaZ << " RAD" << std::endl;*/
    str << toString(p.Fingers);

    str << "     Type : " << (POSITION_TYPE)p.Type << std::endl;

    return str.str();
}

void JacoTrajectoryActionServerKinova::registerHardwareInterface(const std::string& jointName, int idx)
{
    // connect and register the joint state interface
    hardware_interface::JointStateHandle jh(jointName, &pos[idx], &vel[idx], &eff[idx]);
    jnt_state_interface.registerHandle(jh);

    // connect and register the joint position interface
    hardware_interface::JointHandle ph(jnt_state_interface.getHandle(jointName), &cmdPos[idx]);
    jnt_pos_interface.registerHandle(ph);

    hardware_interface::JointHandle vh(jnt_state_interface.getHandle(jointName), &cmdVel[idx]);
    jnt_vel_interface.registerHandle(vh);
}

void JacoTrajectoryActionServerKinova::registerHardwareInterfaces()
{

    ROS_INFO("Registering ros_control Hardwar Interfaces");

    std::vector<float> s;
    if (!jaco_kinova::getCurrentState(s, POSE, true, this))
    {
        ROS_ERROR("Could not obtain current state");
        return;
    }
    for (int i = 0; i < 9; ++i) cmdPos[i] = s[i];

    s.clear();
    if (!jaco_kinova::getCurrentState(s, VELOCITY, true, this))
    {
        ROS_ERROR("Could not obtain current state");
        return;
    }
    for (int i = 0; i < 9; ++i) cmdVel[i] = s[i];


    hw_spinner.reset(new ros::AsyncSpinner(0, &hw_callback_queue));
    hw_spinner->start();
    hw_node.setCallbackQueue(&hw_callback_queue);

    cm.reset(new controller_manager::ControllerManager(this, hw_node));

    int i = 0;
    for (std::vector<std::string>::iterator it = all_joint_names.begin(); it != all_joint_names.end(); ++it)
    {
        //ROS_INFO("Register %s",it->c_str());
        std::string name;//="jaco/";
        name.append(*it);
        registerHardwareInterface(*it, i);
        //ROS_INFO("Registered.");
        ++i;
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
    registerInterface(&jnt_vel_interface);
    interfacesRegistered = true;
}

// Implement robot-specific resouce management
bool JacoTrajectoryActionServerKinova::checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const
{
    return false;
}

void JacoTrajectoryActionServerKinova::hwUpdate(const ros::TimerEvent& t)
{
    if (!interfacesRegistered) return;

    std::vector<float> s;
    if (!jaco_kinova::getCurrentState(s, POSE, true, this))
    {
        ROS_ERROR("Could not obtain current state");
        return;
    }
    for (int i = 0; i < 9; ++i) pos[i] = s[i];
    std::vector<float> start_angles = s;

    s.clear();
    if (!jaco_kinova::getCurrentState(s, VELOCITY, true, this))
    {
        ROS_ERROR("Could not obtain current state");
        return;
    }
    for (int i = 0; i < 9; ++i) vel[i] = s[i];
    std::vector<float> start_vels = s;

    s.clear();
    if (!jaco_kinova::getCurrentState(s, FORCE, true, this))
    {
        ROS_ERROR("Could not obtain current state");
        return;
    }
    for (int i = 0; i < 9; ++i) eff[i] = s[i];

    //for (int i=0; i<9; ++i) ROS_INFO("Vel: %f, Eff: %f",vel[i],eff[i]);
    ros::Duration elapsed_time = ros::Duration(t.current_real - t.last_real);
    if (cm.get()) cm->update(ros::Time::now(), elapsed_time);


    bool positional = false;
    if (positional)
    {
        std::vector<float> target_angles;
        target_angles.resize(9, 0);
        for (int i = 0; i < 9; ++i) target_angles[i] = cmdPos[i];

        lock.lock();
        targetPosValues = target_angles;
        lock.unlock();
    }
    else
    {
        std::vector<float> target_vels;
        target_vels.resize(9, 0);
        for (int i = 0; i < 9; ++i) target_vels[i] = cmdVel[i];

        lock.lock();
        targetVelValues = target_vels;
        lock.unlock();
    }

    //ROS_INFO("Calling:");
    hw_callback_queue.callAvailable(ros::WallDuration(0));
    //ROS_INFO("exit call");
}


void JacoTrajectoryActionServerKinova::printVersion()
{
    std::cout << "The command has been initialized correctly." << std::endl;

    std::vector<int> data;
    int result = (*KnvGetCodeVersion)(data);

    std::cout << "DSP's firmware version is : " << std::hex << data[0] << "." << std::hex <<  data[1] << "." << std::hex <<  data[2] << "." << std::hex <<  data[30] << std::endl;

    std::cout << "Actuator 1's firmware version is : " << std::hex <<  data[3] << "." << std::hex <<  data[4] << "." << std::hex <<  data[5]  << std::endl;
    std::cout << "Actuator 2's firmware version is : " << std::hex <<  data[6] << "." << std::hex <<  data[7] << "." << std::hex <<  data[8]  << std::endl;
    std::cout << "Actuator 3's firmware version is : " << std::hex <<  data[9] << "." << std::hex <<  data[10] << "." << std::hex <<  data[11] << std::endl;
    std::cout << "Actuator 4's firmware version is : " << std::hex <<  data[12] << "." << std::hex <<  data[13] << "." << std::hex <<  data[14] << std::endl;
    std::cout << "Actuator 5's firmware version is : " << std::hex <<  data[15] << "." << std::hex <<  data[16] << "." << std::hex <<  data[17] << std::endl;
    std::cout << "Actuator 6's firmware version is : " << std::hex <<  data[18] << "." << std::hex <<  data[19] << "." << std::hex <<  data[20] << std::endl;

    std::cout << "Finger 1's firmware version is : " << std::hex <<  data[21] << "." << std::hex <<  data[22] << "." << std::hex <<  data[23] << std::endl;
    std::cout << "Finger 2's firmware version is : " << std::hex <<  data[24] << "." << std::hex <<  data[25] << "." << std::hex <<  data[26] << std::endl;
    std::cout << "Finger 3's firmware version is : " << std::hex <<  data[27] << "." << std::hex <<  data[28] << "." << std::hex <<  data[29] << std::endl;

    std::cout << "CAN interface 1's firmware version is : " << std::hex <<  data[31] << "." << std::hex <<  data[32] << "." << std::hex <<  data[33] << std::endl;
    std::cout << "CAN interface 2's firmware version is : " << std::hex <<  data[34] << "." << std::hex <<  data[35] << "." << std::hex <<  data[36] << std::endl;
}

