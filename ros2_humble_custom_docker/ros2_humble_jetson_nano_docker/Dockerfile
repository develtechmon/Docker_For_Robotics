FROM dustynv/ros:humble-desktop-l4t-r36.2.0

RUN apt-get update \
 && apt-get install -y \
 nano \
 vim \
#  bash-completion \
#  python3-argcomplete \
 terminator \
 joystick jstest-gtk evtest \
 && rm -rf /var/lib/apt/lists/*

# Example of copying a file
COPY config/ /site_config/

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Programs for testing devices
RUN apt-get update \
    && apt-get install -y \
    evtest \
    jstest-gtk \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout ${USERNAME}

# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# Set up entrypoint and default command
ENTRYPOINT [ "/bin/bash","/entrypoint.sh" ]
CMD ["bash"]
