  
# Build:
#  docker build --rm -f Dockerfile -t xaialab/ur-planners .
#
# Usage:
#  docker pull humalab/ros-ecl-planner

# Use the base image
FROM gramaziokohler/ros-noetic-dfab
LABEL maintainer "Joseph Kenny <joseph.kenny@princeton.edu>"

SHELL ["/bin/bash","-c"]
ENV CATKIN_WS=/root/catkin_ws
ENV ROS_DISTRO=noetic
RUN mkdir -p $CATKIN_WS/src

# Clone the private repository and set up the catkin workspace
RUN echo "Building UR Planners for ROS" \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    # Update apt-get
    && apt-get update && apt-get -y upgrade \
    # Remove any existing universal_robot directory to avoid conflict
    && rm -rf $CATKIN_WS/src/universal_robot \
    # Clone the repositories from ROS-Industrial
    && git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git \
    # Cloning the UR ROS driver... Enables real-time ROS data exchange, but has conflicts with RTDE.
    # && git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git \
    # Install dependencies
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} \
    # Build catkin workspace
    && catkin_make -j1 \
    # Clean up
    && rm -rf /var/lib/apt/lists/*