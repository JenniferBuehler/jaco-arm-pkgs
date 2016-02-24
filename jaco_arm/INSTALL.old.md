# Note about the official kinova-ros package

End of 2014, when I developed the first version of these packages, I was still using an old version
of the [kinova-ros](https://github.com/Kinovarobotics/kinova-ros) package (I believe also named differently 
back then). 

I used the packages on the real arm. I had some problems with using the kinova packages (specifically
the stuff in jaco_driver). 
E.g. the fingers were simply not usable---the API call failed altogether. This lead me to do some fixes:
I needed to update the firmware and add new API calls to the kinova-ros/jaco_driver source.

However, it looks like many things have been fixed in a new version of the kinova ROS packages. 

So I decided to **start off afresh with the most recent version of the [kinova-ros](https://github.com/Kinovarobotics/kinova-ros)
respository**.

Unfortunately, I do not have access to the real robot arm at the moment to test. So there is no guarantee that
it will work smoothly on the real arm without the fixes I had applied back then on the jaco_drivers codebase.
Please report back if there are any issues.

An insight into which changes were necessary back then to the jaco_drivers source may help to identify similar
problems with the new jaco_driver sources:

* Often, the fingers would not react any more once the arm has been moved. The best way to solve this was to not use cartesian control at all, and only use angular control. A few updates to the ROS packages were made back then.

* Fingers were not initialized properly back then, which lead me to change the kinova driver package. This may have been solved now.

* I had added function to print the version (incl. firmware) of all motors/joints. Not sure this is implemented providing the same amount of information now. It helps when testing.

* Using sendBasicTrajectory() instead of sendAdvancedTrajectory() in the old JacoJomm::SetAngles() and JacoComm::SetFingers() was needed to get it to work.

* Had to update the 'Home' position (actuator angles) to the new firmware I had put on the robot. 
  I also had to add a function to home the arm, which has been added since then to the kinova-ros package by the looks of it.

* In the action servers:
    * jaco_angles_action.cpp
        - Added timeout for action if the arm has not moved. Value read through private ROS parameter ~arm_action_timeout
        - Had to adjust accuracy to 3 degrees, because accuracy was often not better than that in the API, and action would fail.
    * jaco_fingers_action.cpp
        - Added timeout for action if the fingers has not moved. Value read through private ROS parameter ~fingers_action_timeout


# Installation

1. Installing kinova drivers (ATTENTION: maybe this is optional?)

Go to http://runswift.cse.unsw.edu.au/confluence/display/RaCSER/JacoArm

Follow instructions to install Kinova drivers. 

To run the real arm, you may also need to do the following:

Edit /etc/udev/rules.d/10-kinova.rules (rename to 99-kinova.rules):
SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device",ATTRS{idVendor}=="22cd", MODE="0666", SYMLINK+="kinova/jaco%n", GROUP="plugdev"
The symlink will enable you to look up the device under /dev/kinova/jaco/
You might need to rename the .rules file to start with a higher number, so it has higher priority

 /etc/udev/rules.d should have created an entry for kinova.
With dmesg it should print some information about the new device found, including "Jaco" in the specs.
If dmesg prints some errors, try another port, I had problems on one.

With lsusb you can also see if the arm is listed, if 22cd appears as vencor.

You can find out more details about currently applied udev rules with where the device is currently mounted:
udevadm info -a -n /dev/<current-mount>

After you created the rules, you have to restart udev:
sudo service udev restart


2. Optional (and actually not recommended): Installing jaco ros drivers: http://wiki.ros.org/jaco

(See also instructions on 
  https://bitbucket.org/LukeGT/comp3431-project
which seems to refer to the old jaco package. 
IMPORTANT: Actually, it is not necessary to copy any .so libraries around any more (maybe only for the 32 bit version, copy to devel/ or install/ ?)
)

A few modifications were done to the jaco_driver (see version in cse-jaco-ros) because the finger initialisation wasn't working properly.
Hence, it is recommended to use the CSE version of kinova's jaco-ros package, until they fixed the problem.

MODIFICATIONS APPLIED TO SOURCE CODE IN jaco-ros/jaco_driver: See JACO_DRIVER_MODIFICATIONS.md


3. Use the trajectory action server implemented in jaco_ros package (joint_trajectory_action_kinova.cpp). This uses the Kinoav API calls in a safe way
(the official ROS drivers don't use mutexes so the drivers can crash at any time).

To make sure you use the newest libraries, download the most recent kinova developer kit. From the folder API in the package, locate the Kinova.API.USBCommandLayerUbuntu.so
file and the KinovaTypes.h file and copy it to jaco_ros/kinvoa/lib/<architecture> and jaco_ros/kinova/include/kinova.
The safest way is to just copy files for both 32 and 64 bit architecture. 


4. Make sure the USB cable is connected when you start the robot. It did happen that the ROS node would not work properly then.

5. { roslaunch jaco_ros jaco_bringup.launch } will start up the robot which accepts JoinTrajectoryAction commands and publishes to joint_states. See the launch file for documentation of arguments. This is your most important launch file.

To compile these packages, you will need to link the following directories into you catkin_ws:

- jaco_arm
- jaco_msgs (this is from the official ROS drivers at  http://wiki.ros.org/jaco, but you only should use the jaco_msgs package).
- gazebo and moveit ROS packages
- path_navigation_msgs and navigation_actions (this is needed for MoveIt controllers to know how to execute a moveit_msgs::RobotTrajectory which can involve the actual robot/virtual_joint moving)


During run-time, if you want to use predefined gazebo worlds (which support extra stuff):
- gazebo_worlds
- gazebo_ros_interface

6. See jaco_description/ README's for more info.
