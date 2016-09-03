#include <jaco_kinova/JacoTrajectoryActionServerKinova.h>

#define MAX_SPEED_FINGERS_DEG 20

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "jaco_joint_trajectory_action");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle priv("~");
    ros::NodeHandle pub("");

    if (!priv.hasParam("trajectory_params_namespace"))
    {
        ROS_ERROR("'trajectory_params_namespace' parameter not specified.");
        return 0;
    }
    if (!priv.hasParam("arm_components_namespace"))
    {
        ROS_ERROR("'arm_components_namespace' parameter not specified.");
        return 0;
    }

    // see which namespace to read arm components and trajectory parameters from
    std::string trajNamespace, armNamespace;
    priv.param<std::string>("trajectory_params_namespace", trajNamespace, "jaco");
    priv.param<std::string>("arm_components_namespace", armNamespace, "jaco");
    
    ROS_INFO_STREAM("Creating trajectory action server, reading ROS parameters from namespaces "<<trajNamespace<<" and "<<armNamespace);

    JacoJointManager armNames(armNamespace, false);
    if (armNames.loadParameters(true)<0)
    {
        ROS_ERROR_STREAM("Could not load arm components parameters from ROS parameter server, namespace "<<armNamespace);
        return 0;
    }

    std::string joint_trajectory_action_topic;
    double goal_angles_tolerance;
    double intermediateTrajectoryAnglesTolerance;
    double angles_safety_limit;
    bool trajectory_position_mode;
    bool useOnlineVelocityControl;
    double exceed_duration_wait_factor;
    std::vector<float> _maxVelIgnore, _targetPosIgnore, 
        _targetVelIgnore, _currentAnglesIgnore, _currentVelsIgnore;  // all these fields won't be used but are needed for convenience function called next
    if (!joint_trajectory_execution::TrajectoryActionServer::InitFromROSParameters(trajNamespace, armNames,
            joint_trajectory_action_topic,
            goal_angles_tolerance,
            intermediateTrajectoryAnglesTolerance,
            angles_safety_limit,
            trajectory_position_mode,
            useOnlineVelocityControl,
            exceed_duration_wait_factor,
            _maxVelIgnore,
            _targetPosIgnore,
            _targetVelIgnore,
            _currentAnglesIgnore,
            _currentVelsIgnore))
    {
        ROS_ERROR_STREAM("Could not read parameters from namespace "<<trajNamespace);
        return 0;
    }

    bool useRosControllers = false;
    priv.param<bool>("use_ros_controllers", useRosControllers, useRosControllers);

    if (useOnlineVelocityControl)
    {
        useRosControllers = false;
    }

    std::string CURRENT_JOINT_STATES_TOPIC;
    priv.param<std::string>("joint_states_topic", CURRENT_JOINT_STATES_TOPIC, "/jaco/joint_state");
    ROS_INFO("Got Joint States Topic Name: <%s>", CURRENT_JOINT_STATES_TOPIC.c_str());

    std::string ARM_ANGLES_ACTION_TOPIC = "jaco/arm_joint_angles";
    priv.param<std::string>("arm_action_topic", ARM_ANGLES_ACTION_TOPIC, ARM_ANGLES_ACTION_TOPIC);
    ROS_INFO("Got arm angles action Topic Name: <%s>", ARM_ANGLES_ACTION_TOPIC.c_str());

    std::string FINGERS_ANGLES_ACTION_TOPIC = "jaco/finger_joint_angles";
    priv.param<std::string>("fingers_action_topic", FINGERS_ANGLES_ACTION_TOPIC, FINGERS_ANGLES_ACTION_TOPIC);
    ROS_INFO("Got fingers action Topic Name: <%s>", FINGERS_ANGLES_ACTION_TOPIC.c_str());

    double maxSpeed123, maxSpeed456, maxSpeedFingers;
    //velocity limitation (in RAD per second) for joints 1,2,3 and 4,5,6
    priv.param<double>("max_speed_123", maxSpeed123, MAX_SPEED_123_DEG * jaco_kinova::JacoTrajectoryActionServerKinova::DEG_TO_RAD);
    priv.param<double>("max_speed_456", maxSpeed456, MAX_SPEED_456_DEG * jaco_kinova::JacoTrajectoryActionServerKinova::DEG_TO_RAD);
    priv.param<double>("max_speed_fingers", maxSpeedFingers, MAX_SPEED_FINGERS_DEG * jaco_kinova::JacoTrajectoryActionServerKinova::DEG_TO_RAD);
    ROS_INFO("Overwriting velocity limits for Kinova drivers: {123} = %lf {456} = %lf {Fingers} = %lf", maxSpeed123, maxSpeed456, maxSpeedFingers);
    std::vector<float> maxVel;
    maxVel.insert(maxVel.end(), 3, maxSpeed123);
    maxVel.insert(maxVel.end(), 3, maxSpeed456);
    maxVel.insert(maxVel.end(), 3, maxSpeedFingers);

    double CURRENT_JOINT_STATES_FREQ;
    priv.param<double>("joint_states_freq", CURRENT_JOINT_STATES_FREQ, 20.0);
    ROS_INFO("Got Joint States Frequency: <%f>", CURRENT_JOINT_STATES_FREQ);

    jaco_kinova::JacoTrajectoryActionServerKinova jte(
                     armNames,
                     joint_trajectory_action_topic,
                     trajectory_position_mode,
                     maxVel,
                     maxSpeed123,
                     maxSpeed456,
                     CURRENT_JOINT_STATES_TOPIC,
                     CURRENT_JOINT_STATES_FREQ,
                     angles_safety_limit,
                     goal_angles_tolerance,
                     ARM_ANGLES_ACTION_TOPIC,
                     FINGERS_ANGLES_ACTION_TOPIC,
                     useRosControllers, useOnlineVelocityControl,
                     intermediateTrajectoryAnglesTolerance);

    ROS_INFO("Now initializing trajectory server");
    if (!jte.init())
    {
        ROS_ERROR("Failed to initialize JacoTrajectoryActionServerKinova. Exit.");
    }
    else
    {
        ROS_INFO("Joint trajectory action server running.");
        ros::spin();
    }
    jte.shutdown();
}
