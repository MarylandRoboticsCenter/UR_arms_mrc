##############
# modified full ubuntu image #
##############
FROM osrf/ros:humble-desktop AS humble-mod_desktop

# Set default shell
SHELL ["/bin/bash", "-c"]

WORKDIR ${HOME}

ENV DEBIAN_FRONTEND=noninteractive

# Basic setup
RUN sudo apt-get update && sudo apt-get dist-upgrade && sudo apt-get install -y --no-install-recommends --allow-unauthenticated \
    autoconf \
    automake \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    g++ \
    git \
    iproute2 \
    iputils-ping \
    libxext-dev \
    libx11-dev \
    make \
    mc \
    mesa-utils \
    nano \
    pkg-config \
    software-properties-common \
    sudo \
    tmux \
    tzdata \
    xclip \
    x11proto-gl-dev && \
    sudo rm -rf /var/lib/apt/lists/*

# Set datetime and timezone correctly
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' | sudo tee -a /etc/timezone

ENV DEBIAN_FRONTEND=dialog

##############
# Aux ROS2 packages #
##############
FROM humble-mod_desktop AS humble-dev

# Install ROS packages
RUN sudo apt-get update && sudo apt-get install -y \
    ros-dev-tools \
    python-is-python3 \
    python3-pip \
    python3-colcon-common-extensions python3-vcstool && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# upgrading colcon package to fix symlink issues
RUN pip3 install setuptools==58.2.0

# Install auxilary ROS packages
RUN sudo apt-get update && sudo apt-get install -y \
    ros-humble-gazebo-* \
    ros-humble-usb-cam \
    ros-humble-moveit-* && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*


##############
# user with matching uid and gid#
##############
FROM humble-dev AS humble-user

ARG WS_DIR="dir_ws"
ARG USERNAME=user
ARG userid=1111
ARG groupid=1111
ARG PW=user@123

RUN groupadd -g ${groupid} -o ${USERNAME}
RUN useradd --system --create-home --home-dir /home/${USERNAME} --shell /bin/bash --uid ${userid} -g ${groupid} --groups sudo,video ${USERNAME} && \ 
    echo "${USERNAME}:${PW}" | chpasswd && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ENV USER=${USERNAME} \
    WS_DIR=${WS_DIR} \
    LANG=en_US.UTF-8 \
    HOME=/home/${USERNAME} \
    XDG_RUNTIME_DIR=/run/user/${userid} \
    TZ=America/New_York

USER ${USERNAME}
WORKDIR ${HOME}

# custom Bash prompt
RUN { echo && echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \\\$ '" ; } >> .bashrc

RUN sudo mkdir -p -m 0700 /run/user/${userid} && \
    sudo chown ${USERNAME}:${USERNAME} /run/user/${userid}

# Setup tmux config
ADD --chown=${USERNAME}:${USERNAME} https://raw.githubusercontent.com/MarylandRoboticsCenter/someConfigs/refs/heads/master/.tmux_K.conf $HOME/.tmux.conf

#####################
# UR ROS2 drivers#
#####################
FROM humble-user AS humble-ur_mrc

# Install binary UR drivers (not from source)
RUN sudo apt-get update && sudo apt-get install -y \
    ros-humble-ur && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# Setup UR ROS2 Drivers
RUN source /opt/ros/humble/setup.bash && \
    mkdir -p $HOME/ros_ur_driver/src && \
    cd ~/ros_ur_driver && \
    git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git src/Universal_Robots_ROS2_Gazebo_Simulation && \
    sudo apt update -qq && \
    rosdep update && \
    rosdep install --ignore-src --from-paths src -y && \
    colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

#####################
# UR MRC workspace #
#####################
FROM humble-ur_mrc AS humble-ur_ws


# Set up UR5e workspace
RUN source /opt/ros/humble/setup.bash && \
	mkdir -p $HOME/${WS_DIR}/src && \
    cd $HOME/${WS_DIR} && \
	colcon build --symlink-install


# Set up working directory and bashrc
WORKDIR ${HOME}/${WS_DIR}/
RUN echo 'source /opt/ros/humble/setup.bash' >> $HOME/.bashrc && \
    echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >> $HOME/.bashrc && \
    echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> $HOME/.bashrc && \
    echo >> $HOME/.bashrc && \
    echo 'source /usr/share/gazebo/setup.bash' >> $HOME/.bashrc  && \
    echo 'source $HOME/ros_ur_driver/install/setup.bash' >> $HOME/.bashrc && \
    echo "source $HOME/${WS_DIR}/install/setup.bash" >> $HOME/.bashrc && \
    echo "source $HOME/${WS_DIR}/config/ur_setup.bash" >> $HOME/.bashrc    


WORKDIR ${HOME}
    
CMD /bin/bash
