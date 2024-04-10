# Getting Started

In this userguide i'm going to show you how to install ros2 humble in Jetson Nano
and run `lidar slamtec`.

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
## Step 4: Install our `lidar packages`

While you're inside the docker terminal, go to following directory
```
cd ~

or

cd /home/ros
```

And follow rest of the step below
```
mkdir -p lidar_slamtec/src
cd lidar_slamtec/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd lidar_slamtec/
colcon build --symlink-install
source install/setup.bash
```
Then plug your `lidar sensor` to the jetson nano and run below command step
```
ls /dev/ttyUSB0 <---- This is my device id

and change the mode to ensure we can access this later.
sudo chmod 777 /dev/ttyUSB0
```

## Step 5: Run our `lidar_ros` package

Before start running our lidar, first we need to add the `source` command into our `bashrc` command
so that we don't need to source the `lidar_slamtec` package everytime
```
sudo vi ~ros/.bashrc
```
Add following line into the this file and save and quit
```
source /home/ros/lidar_slamtec/install/setup.bash 
```

Then use below command to run the `lidar_slamtec`
```
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
```
You should see `rviz2` appear and our sensor working.

## Step 6: Stop the docker 

To stop the docker without losing our previous data, do the following

While you're in the container type below command to quit from the docker
```
exit
```
```

```
