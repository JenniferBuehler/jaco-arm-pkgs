
/**
 * Test client for ArmJointAngles action
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <kinova_msgs/ArmJointAnglesGoal.h>
#include <kinova_msgs/JointAngles.h>


#define ACTION_TOPIC_NAME "jaco/arm_joint_angles"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "jaco_arm_action_client");
    
 if (argc != 7) ROS_ERROR("Needs exactly 6 values as arguments");
  
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> ac(ACTION_TOPIC_NAME, true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  kinova_msgs::JointAngles jointAngles;
  jointAngles.joint1 = atof(argv[1]);
  jointAngles.joint2 = atof(argv[2]);
  jointAngles.joint3 = atof(argv[3]);
  jointAngles.joint4 = atof(argv[4]);
  jointAngles.joint5 = atof(argv[5]);
  jointAngles.joint6 = atof(argv[6]);
 
  kinova_msgs::ArmJointAnglesGoal goal;
  goal.angles = jointAngles;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  //exit
  return 0;
}
