
/**
 * Test client for SetFingerPosition action
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <jaco_msgs/SetFingersPositionAction.h>
#include <jaco_msgs/ArmJointAnglesGoal.h>
#include <jaco_msgs/JointAngles.h>

#define FINGER_POS_ACTION_TOPIC "/jaco/finger_joint_angles"

int main (int argc, char **argv)
{
  if (argc !=4) ROS_ERROR("Needs exactly 3 arguments"); 
 
  ros::init(argc, argv, "jaco_finger_position_client");
    
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> ac(FINGER_POS_ACTION_TOPIC, true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  // send a goal to the action
  jaco_msgs::FingerPosition jointAngles;
  jointAngles.finger1 = atof(argv[1]);
  jointAngles.finger2 = atof(argv[2]);
  jointAngles.finger3 = atof(argv[3]);
 
  ROS_INFO("Action server started, sending goal.");
  ROS_INFO_STREAM(jointAngles);
 
  jaco_msgs::SetFingersPositionGoal goal;
  goal.fingers = jointAngles;
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
