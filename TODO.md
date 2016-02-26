- I had to adapt the URDF model in order to fit the actual robot. This is described in [jaco_arm/jaco\_description/README.md](https://github.com/JenniferBuehler/jb-ros-packs/blob/master/jaco_arm/jaco_description/README.md). The meshes around the finger bases are not well fitted. Designing new meshes it part of future work.
- New implementation of DefaultRobotHWSim could be used to replace/supplement JacoGazeboJointControl classes.
- Migrate trajectory execution and joint controllers to more robot-general and create Jaco adapters/specific implementations.

jaco_gazebo:
- the joint velocities still do not correspond to the ones set, it only fluctuates around the target value. Tried fine-tuning PID gains to  no avail.
   So far, using Joint::SetVelocity() is the best solution found so far, both on Indigo and Jade. But it should be made to work with SetForce()
   at some stage, and the values should be stabilized (e.g. a joint rotating towards/against gravity should always meet the target velocity
   *v*, with a low error margin around *v*).

jaco_joints:

- JacoTrajectoryActionServer can be improved to read tolerances from FollowJointTrajectoryAction, after recent migration from JointTrajectoryAction.


# Possible future contributions

gazebo:
    - the higher-level controller with position and velocity
    - a program which helps find PID values, either with neural network or Ziegler-Nichols method
