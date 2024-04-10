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


