## Quick HOWTO use the real Jaco arm

The package jaco_kinova provides a ROS joint trajectory action server for the Kinova Jaco arm.


###Installation

You will need to add the [kinova-ros](https://github.com/Kinovarobotics/kinova-ros.git) repository to your catkin workspace.
It is only required for the message types (package jaco_msgs) and for the [kinova](https://github.com/Kinovarobotics/kinova-ros/tree/master/jaco_driver/include/kinova) 
API and library.

**You are not meant to use both jaco_driver together with this package to control the arm!**

*Reason:* This trajectory driver uses the kinova drivers to control the arm directly, and therefore conflicts with any other
packages which directly use the kinova drivers. It is meant to use *in place of* the API jaco_driver::JacoArm
(specifically any class using jaco_driver::JacoAPI).
Having two separate ROS nodes which control the arm directly
running at the same time would be a conflict in itself.
Therefore, *jaco_kinova* supports also the publishing sensor_msgs/JointState and the support of jaco_msgs/ArmJointAngles.action and
jaco_msgs/SetFingersPositionAction.action.    
Some time in hopefully near future, a better way can be found to combine functionality of jaco_driver and this package.


**[optional] Setting the rules for USB access**

You *may* need to set the rules for the USB port, but try without this optional step first.

Edit /etc/udev/rules.d/10-kinova.rules, and rename to 99-kinova.rules.

Alternatively, copy the udev rule file 99-jaco-arm.rules from ``<kinova-ros-repo>/jaco_driver/udev`` to ``/etc/udev/rules.d/``

In case you create a new one, the following contents have been working:
```
SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device",ATTRS{idVendor}=="22cd", MODE="0666", SYMLINK+="kinova/jaco%n", GROUP="plugdev"
```
The symlink specified in the rules file will enable you to look up the device under /dev/kinova/jaco/.
You might need to rename the .rules file to start with a higher number, so it has higher priority,
e.g. rename to ``/etc/udev/rules.d/99-kinova.rules``.

After you created the rules, you have to restart udev:

``sudo service udev restart``

/etc/udev/rules.d should have created an entry for kinova.

Troubleshooting tips:    
* With ``dmesg`` you can print some information about the new device found, including "Jaco" in the specs.
    If dmesg prints some errors, try another physical USB port, I had problems on one (it probably needs to be USB-2).
* With ``lsusb`` you can also see if the arm is listed, and if 22cd appears as vendor.
* You can find out more details about currently applied udev rules with where the device is currently mounted with
      ``udevadm info -a -n /dev/<current-mount>``

**Exposing the USB library**

The library ``<kinova-ros-repo>/jaco_driver/lib/<architecture>/Kinova.API.USBCommandLayerUbuntu.so`` has to 
be in a path where it can be found. Add the path to your LD_LIBRARY_PATH.

``export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path-to-kinova-ros>/jaco_driver/lib/x86_64-linux-gnu/``

or do it in the .bashrc.

###Testing

**Connecting the arm**

Connect the arm to the USB. You should try a USB-2 port first.

**General advice**

During testing, it is recommended to always have the hand on the **on/off switch**
on the back of the arm, to shut it down immediately if it starts behaving strange.
This applies in particular when starting up the arm, but also when it
executes trajectories: watch out when it moves so you can anticipate a collision
and shut it down quickly, especially when you use MoveIt! - see also [this issue](https://github.com/JenniferBuehler/jaco-arm-pkgs/issues/4).


**Test 1**

Launch *only* the trajectory action server:

``roslaunch jaco_kinova jaco_trajectory_action_kinova.launch``

If everything works fine, the arm should maintain its current pose.
Now, see if the joint states are being published, the default topic should be *jaco/joint_state*:

``rostopic echo /jaco/joint_state``

Sanity check the values. Do they make sense? Control the arm with the joystick and see if the values change
accordingly.

**Test 2**

Again, launch *only* the trajectory action server:

``roslaunch jaco_kinova jaco_trajectory_action_kinova.launch``

Now use the test client to send a jaco_msgs/ArmJointAngles.action to the drivers.
Start with a simple one which will set only the position of the last arm joint (the "wrist"):

``rosrun jaco_kinova jaco_arm_action_client_node 0 0 0 0 0 0.5`` 

Play around with the values and see if it makes sense. Compare the joint values to 
how they look like in Gazebo, see also [this tutorial](https://github.com/JenniferBuehler/jaco-arm-pkgs/wiki/Jaco-example-in-Gazebo).    
To launch Jaco in Gazebo with the joint state publisher:    
``roslaunch jaco_on_table jaco_on_table_gazebo_controlled.launch load_joint_state_publisher:=true``

After testing the arm joints, try the same for the fingers:

``rosrun jaco_kinova jaco_fingers_action_client_node 0 0 0.2``

Again, try different values and compare to how it looks like in Gazebo. 

**Further tests**

Please don't test the trajectory execution yet before reporting back to me if the previous two tests worked.
This branch is in a testing state and the previous version of the code using the Kinova drivers is being
ported to the new code structure. I don't have a Kinova arm here to re-test the functionality right now,
so I rely on the outside world to help me re-integrate this.    
Thanks!
