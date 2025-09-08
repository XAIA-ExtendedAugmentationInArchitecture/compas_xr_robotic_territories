# # syntax=docker/dockerfile:1
    
# # Build:
# #  docker build --rm -f Dockerfile -t xaialab/ros-abb-irb4600s-planner .
# #
# # Usage:
# #  docker pull xaialab/ros-abb-irb4600s-planner

# # Use the base image
# FROM gramaziokohler/ros-noetic-dfab
# LABEL maintainer = "Eleni Alexi <eleni.alexi@princeton.edu>"

# SHELL ["/bin/bash","-c"]
# ENV CATKIN_WS=/root/catkin_ws
# ENV ROS_DISTRO=noetic
# RUN mkdir -p $CATKIN_WS/src

# # Temporarily disable any old ROS sources so apt update won't hit an invalid key
# RUN for f in /etc/apt/sources.list.d/*ros*.list; do \
#       mv "$f" "$f.disabled" || true; \
#     done

# # Install the ros-apt-source package (pulls in the new key + repo config)
# RUN apt-get update \
#  && apt-get install -y ros-apt-source \
#  && rm -rf /var/lib/apt/lists/*

# # (Optional) Remove the legacy list & key so you won't ever reimport the old one
# RUN rm -f /etc/apt/sources.list.d/*ros*.list.disabled \
#  && apt-key del "C1CF 6E31 E6BA DE88 68B1  72B4 F42E D6FB AB17 C654" || true


# RUN rm /etc/apt/sources.list.d/ros1-latest.list \
#   && rm /usr/share/keyrings/ros1-latest-archive-keyring.gpg

# RUN apt-get update \
#   && apt-get install -y ca-certificates curl

# RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
#     curl -L -s -o /tmp/ros-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
#     && apt-get update \
#     && apt-get install /tmp/ros-apt-source.deb \
#     && rm -f /tmp/ros-apt-source.deb

# RUN apt-get update \
#   && apt-get install -y ros-noetic-roscpp-tutorials

# # Now install your other tools normally
# RUN apt-get update \
#  && apt-get install -y \
#       git \
#       ssh \
#       build-essential \
#  && rm -rf /var/lib/apt/lists/*

# # 1) remove the old ROS list & keyring
# RUN rm -f \
#     /etc/apt/sources.list.d/ros-latest.list \
#     /usr/share/keyrings/ros-archive-keyring.gpg

# # 2) install curl + lsb-release
# RUN apt-get update && apt-get install -y \
#       ca-certificates \
#       curl \
#       lsb-release \
#     && rm -rf /var/lib/apt/lists/*

# # 3) download & install ros-apt-source package
# RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
#       | grep -F '"tag_name":' | awk -F\" '{print $4}') && \
#     curl -L -s -o /tmp/ros-apt-source.deb \
#       "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}_$(lsb_release -sc)_all.deb" && \
#     dpkg -i /tmp/ros-apt-source.deb && \
#     rm /tmp/ros-apt-source.deb

# # 4) test with a ROS package
# RUN apt-get update && apt-get install -y ros-noetic-roscpp-tutorials

# # 5) now install your other tools
# RUN apt-get update && apt-get install -y \
#       git \
#       ssh \
#       build-essential \
#     && rm -rf /var/lib/apt/lists/*

# syntax=docker/dockerfile:1

# Use the base image
FROM gramaziokohler/ros-noetic-dfab
LABEL maintainer="Eleni Alexi <eleni.alexi@princeton.edu>"

SHELL ["/bin/bash","-c"]
ENV CATKIN_WS=/root/catkin_ws
ENV ROS_DISTRO=noetic

RUN mkdir -p $CATKIN_WS/src

# 0) Disable any existing ROS apt sources & remove the old keyring
RUN for f in /etc/apt/sources.list.d/ros*.list; do \
      mv "$f" "$f.disabled" || true; \
    done && \
    rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# 1) Install key-management tools from Ubuntu only
RUN apt-get update && apt-get install -y \
      ca-certificates \
      curl \
      lsb-release \
    && rm -rf /var/lib/apt/lists/*

# 2) Import the fresh ROS GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | apt-key add - \
    && rm -rf /var/lib/apt/lists/*

# 3) Re-enable the ROS apt sources
RUN for f in /etc/apt/sources.list.d/ros*.list.disabled; do \
      mv "$f" "${f%.disabled}"; \
    done

# 4) Verify by installing a ROS package
RUN apt-get update && apt-get install -y ros-noetic-roscpp-tutorials \
    && rm -rf /var/lib/apt/lists/*

# 5) Install your other development tools
RUN apt-get update && apt-get install -y \
      git \
      ssh \
      build-essential \
    && rm -rf /var/lib/apt/lists/*

# Set up SSH configuration
RUN mkdir -p /root/.ssh \
    && chmod 0700 /root/.ssh \
    && ssh-keyscan github.com >> /root/.ssh/known_hosts

# Copy SSH key for accessing private repository (make sure to use BuildKit)
# Ensure BuildKit is enabled by setting DOCKER_BUILDKIT=1 during the build process
# This line assumes the SSH key is provided as a secret during the build
RUN --mount=type=secret,id=ssh_key \
    mkdir -p /root/.ssh \
    && cp /run/secrets/ssh_key /root/.ssh/id_ed25519 \
    && chmod 600 /root/.ssh/id_ed25519


# Clone the private repository and set up the catkin workspace
RUN echo "Building ABB IRB4600s Planner for ROS" \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    # Update apt-get
    && apt-get update && apt-get -y upgrade \
    # Clone the private repository for ECL packages
    && git clone git@github.com:XAIA-ExtendedAugmentationInArchitecture/abb_irb4600s_description.git \
    # Install dependencies
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} \
    # Build catkin workspace
    && catkin_make -j1\
    # Clean up
    && rm -rf /var/lib/apt/lists/*
