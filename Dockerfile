# In order to support graphical interfaces,
# this should be run with 
# docker run -it --rm \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \  
#     ros-indigo-full-catkin <cmd> 
#
# The -e and -v commands are needed to display on the host X server.
# For hardware support, you will also need:
#  --privileged   (to access the graphics card) 
#  It may also be required to call
#  $ xhost +
#  before running the container.

FROM jenniferbuehler/ros-indigo-full-catkin 

MAINTAINER Jennifer Buehler

# Install system essentials
RUN apt-get update && apt-get install -y \
    cmake \
    sudo \
    vim \
    && rm -rf /var/lib/apt/lists/*

# need g++ for compiling with cmake even if gcc
# is already installed
RUN apt-get update && apt-get install -y g++ \
    && rm -rf /var/lib/apt/lists/*

# Install required ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-indigo-gazebo-ros \
    ros-indigo-gazebo-ros-control \
    ros-indigo-roslint \
    && rm -rf /var/lib/apt/lists/


# install git
RUN apt-get update && apt-get install -y git

COPY jaco_arm /catkin_ws/src/jaco_arm
COPY jaco_tutorial /catkin_ws/src/jaco_tutorial

# Get the repository convenience-pkgs as well
RUN bin/bash -c "cd /catkin_ws/src \
    && git clone https://github.com/JenniferBuehler/convenience-pkgs.git"

# Build
RUN bin/bash -c "source /.bashrc \
    && cd /catkin_ws \
    && catkin_make \
    && catkin_make install"

RUN bin/bash -c "source .bashrc"

CMD ["bash","-l"]
