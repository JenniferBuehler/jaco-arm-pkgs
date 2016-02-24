I have previously had issues controlling the Jaco joints with efforts (Joint::SetForce())
in older Gazebo versions which could not be addressed by fine-tuning model/joint parameters in the URDF
and PID values. The current [gazebo_ros_control][gazebo_ros_control] implementation
controls the joints with efforts only, so this could not work with older Gazebo versions.    
This has to be tested again as the Jaco URDF has changed since I last had this issue and I have
not tested the new model on Gazebo 2 yet.

You can swith to using Joint::SetVelocity() instead of Joint::SetForce() by setting
the parameter "jaco/gazebo_use_set_velocity" to true. Note that this also requires loading
different configurations from the .yaml file. See also launch file:

``rosed jaco_gazebo jaco_on_table_gazebo``
