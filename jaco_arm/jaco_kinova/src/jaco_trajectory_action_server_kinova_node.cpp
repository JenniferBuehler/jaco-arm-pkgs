#include <jaco_kinova/JacoTrajectoryActionServerKinova.h>

#define MAX_SPEED_FINGERS_DEG 20

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "jaco_joint_trajectory_action");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle priv("~");
    ros::NodeHandle pub("");

    std::string JOINT_TRAJECTORY_ACTION_TOPIC;
    priv.param<std::string>("joint_trajectory_action_topic", JOINT_TRAJECTORY_ACTION_TOPIC, "/jaco/joint_trajectory_action");
    ROS_INFO("Got Trajectory action Topic Name: <%s>", JOINT_TRAJECTORY_ACTION_TOPIC.c_str());

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
    priv.param<double>("max_arm_speed_123", maxSpeed123, MAX_SPEED_123_DEG * jaco_kinova::JacoTrajectoryActionServerKinova::DEG_TO_RAD);
    priv.param<double>("max_arm_speed_456", maxSpeed456, MAX_SPEED_456_DEG * jaco_kinova::JacoTrajectoryActionServerKinova::DEG_TO_RAD);
    priv.param<double>("max_arm_speed_fingers", maxSpeedFingers, MAX_SPEED_FINGERS_DEG * jaco_kinova::JacoTrajectoryActionServerKinova::DEG_TO_RAD);
    ROS_INFO("Speed limits: {123} = %lf {456} = %lf {Fingers} = %lf", maxSpeed123, maxSpeed456, maxSpeedFingers);
    std::vector<float> maxVel;
    maxVel.insert(maxVel.end(), 3, maxSpeed123);
    maxVel.insert(maxVel.end(), 3, maxSpeed456);
    maxVel.insert(maxVel.end(), 3, maxSpeedFingers);

    double goalTolerance = DEFAULT_GOAL_TOLERANCE;
    priv.param<double>("goal_angles_tolerance", goalTolerance, goalTolerance);

    double interGoalTolerance = 0.025;
    priv.param<double>("goal_angles_intermediate_tolerance", interGoalTolerance, interGoalTolerance);

    ROS_INFO("Using intermediate trajectory point accuracy: %f", interGoalTolerance);

    double angleSafetyLimit = DEFAULT_ANGLE_SAFETY_LIMIT;
    priv.param<double>("trajectory_angle_safety_limit", angleSafetyLimit, angleSafetyLimit);
    ROS_INFO("Got angle parameters: goal tolerance %lf, safety limit %lf", goalTolerance, angleSafetyLimit);

    double CURRENT_JOINT_STATES_FREQ;
    priv.param<double>("joint_states_freq", CURRENT_JOINT_STATES_FREQ, 20.0);
    ROS_INFO("Got Joint States Frequency: <%f>", CURRENT_JOINT_STATES_FREQ);

    bool useAnglePoses = true;
    priv.param<bool>("use_angle_poses", useAnglePoses, useAnglePoses);

    bool useOnlineVelocityControl = false;
    priv.param<bool>("use_online_trajectory_control", useOnlineVelocityControl, useOnlineVelocityControl);

    bool useRosControllers = false;
    priv.param<bool>("use_ros_controllers", useRosControllers, useRosControllers);

    if (useOnlineVelocityControl)
    {
        useAnglePoses = false;
        useRosControllers = false;
    }

    jaco_kinova::JacoTrajectoryActionServerKinova jte(pub, JOINT_TRAJECTORY_ACTION_TOPIC,
                     useAnglePoses, maxVel, maxSpeed123, maxSpeed456,
                     CURRENT_JOINT_STATES_TOPIC, CURRENT_JOINT_STATES_FREQ, angleSafetyLimit,
                     goalTolerance,
                     ARM_ANGLES_ACTION_TOPIC,
                     FINGERS_ANGLES_ACTION_TOPIC,
                     useRosControllers, useOnlineVelocityControl, interGoalTolerance);
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


