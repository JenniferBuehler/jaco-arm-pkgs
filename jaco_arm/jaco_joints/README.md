# jaco_joints

This package provides a few C++ classes for the Jaco arm.
So far, this includes:

- A manager for the joint names, reading from ROS parameter server, and
- A trajectory execution ROS action server.

# Building documentation

Documentation spread throughout the API documentation. Go to
your catkin workspace and use cmake to generate it with doxygen.

```
cd <your_catkin_ws>
catkin_make reset && catkin_make jaco_joints_doc
```

The API will be generated in ``<your_catkin_ws>/build/jaco-arm-pkgs/jaco_arm/jaco_gazebo/docs/html``.
