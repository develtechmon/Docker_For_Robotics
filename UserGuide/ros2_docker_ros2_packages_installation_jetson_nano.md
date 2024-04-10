# Getting Started

In this userguide i'm going to show you how to install ros2 humble in Jetson Nano
and install our `ros2 packages`

## Step 1: Git Clone below directory
Open Jetson Nano and open new terminal and run below command
```
git clone https://github.com/develtechmon/Docker_For_Robotics.git
```

## Step 2: Build our docker image
Before building our docker image, please follow below step
```
cd /Docker_For_Robotics/ros2_humble_custom_docker/ros2_humble_jetson_nano_docker
```
Inside `ros2_humble_jetson_nano_docker`, you should see following directory and files.
Here we're interested at `Dockerfile` which is the source of build image.
```
├── bashrc
├── config
│   └── my_config.yaml
├── Dockerfile
└── entrypoint.sh
```
Then run below command to build our image
```
docker build -t my_image .
```
To check if our image is successfully created, run below command. 
This will show you `my_image` from the output to indicate our image successfully created.
```
docker image ls
```
## Step 3: Run our `my_image` container

Run below command to start our docker container of `my_image`.
Here i create my image with `lukas` to make it easy to remember and easy to restart later when we close our docker.
Here, `-it` command to ensure we enter the docker interactively
Also, i mount the graphic volume to ensure that we can run graphiz inside the docker.
Also i mount all the devices in host with docker to ensure the devices is recognizable and accessible inside the docker with privileged mode.

In other `Jetson Nano` terminal, run below command.
```
docker run -it --user ros --name lukas --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /dev:/dev --privileged my_image
```

At this point, you should be inside the docker of `my_image`. Once we're in the container, run following command first to get all the update
```
sudo apt-get update
```

You will encounter with opencv issues later due to default opencv2 installation that come from the `humble` image
of jetson nano. To solve the issue with opencv later, we need to remove this package to ensure it is easy to install
rest of the `ros2 packages`.

Follow the step below and you might to try each of the command below as long you can remove the package.
```
sudo apt-get purge '*opencv*'
sudo find / -name "*opencv*" -exec rm -rf {} \;
sudo apt remove libopencv-dev
sudo apt-get autoremove
```
And install `opencv` packages
```
pip install opencv-contrib-python
```

## Step 4: Install `ros2_packages`

In the docker terminal, install following packages.
```
sudo apt-get install tree -y
sudo apt-get install ros-humble-xacro -y
sudo apt install ros-humble-joint-state-publisher-gui -y
sudo apt-get install ros-humble-teleop-twist-keyboard -y
sudo apt-get install ros-humble-teleop-twist-joy -y
sudo apt-get install joystick jstest-gtk evtest -y
sudo apt install ros-humble-joy-tester -y
sudo apt-get install v4l-utils -y
sudo apt-get install ros-humble-twist-mux -y
sudo apt-get install v4l-utils ros-humble-v4l2-camera -y
sudo apt-get install ros-humble-image-transport-plugins ros-humble-rqt-image-view -y
sudo apt-get install ros-humble-rqt-image-view -y

sudo apt-get install ros-humble-cv-bridge -y
sudo apt install ros-humble-rmw-cyclonedds-cpp -y
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo apt-get install ros-humble-ros2-control -y
sudo apt-get install ros-humble-ros2-controllers -y
sudo apt install ros-humble-controller-manager -y

sudo apt-get install ros-humble-slam-toolbox -y
sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* -y
sudo apt-get install ros-humble-nav2* -y
sudo apt-get install ros-humble-tf2* -y
```

## Step 5: Source our `ros2 file`
You might notice, `setup.sh` file is not store outside of `install`. To ensure our installed packages work, 
we will need to source our ros2 and add it to `bashrc` file.
```
source /opt/ros/humble/setup.sh
```
Add this `source` file into `bashrc`
```
sudo vi ~ros/.bashrc
```
Add following line into `.bashrc` as follow and save and quit
```
source /opt/ros/humble/install/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /opt/ros/humble/setup.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
You should be able to use ros2 package in your docker environment.

## Step 6: Stop the docker 
To stop the docker without losing our previous data, do the following

While you're in the container type below command to quit from the docker. This will enable you
to exit from the `my_image` docker
```
exit
```

To restart our previous closed docker `my_image`. Follow below step

Run below command to check if `my_image` container still availabe.
```
docker ps -a
```

Outpout will be as follow. See status `Up` which mean running.
```
CONTAINER ID   IMAGE      COMMAND                  CREATED          STATUS          PORTS     NAMES
d6654b5e7e26   my_image   "/bin/bash /entrypoi…"   53 minutes ago   Up 29 minutes             lukas
```
To stop our container, run below command
```
docker stop d6654b5e7e26

or

docker stop lukas
```
At this point your docker should be closed. and when you write down above command
```
docker ps -a
```
Output will be as follow, this indicate no container is running. See status `Exited`
```
CONTAINER ID   IMAGE      COMMAND                  CREATED          STATUS                            PORTS     NAMES
d6654b5e7e26   my_image   "/bin/bash /entrypoi…"   56 minutes ago   Exited (137) About a minute ago             lukas
```
## Step 7: Restart our docker
To restart previously closed container, run the following command
```
docker restart lukas
```
And when you run below comand
```
docker ps -a
```
Output will be as follow. See status `Up` which mean it is running again with `lukas` name and `my_image`.
```
CONTAINER ID   IMAGE      COMMAND                  CREATED             STATUS         PORTS     NAMES
d6654b5e7e26   my_image   "/bin/bash /entrypoi…"   About an hour ago   Up 3 seconds             lukas
```
To attach with our running docker, write the following command
```
docker exec -it d6654b5e7e26 bash

or

docker exec -it lukas bash
```

At this point you should be inside the same docker again without losing your previous data.
