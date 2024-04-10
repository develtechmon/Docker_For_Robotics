# Getting Started

In this userguide i'm going to show you how to install ros2 humble in Jetson Nano
and run `lidar slamtec`.

## Step 1: Git Clone below directory
Open Jetson Nano and open new terminal and run below command
```
git clone https://github.com/develtechmon/Docker_For_Robotics.git
```
In other terminal of `Jetson Nano`, you'll need to install `CH341SER` driver.
Follow below step accordingly.
```
cd /home/jlukas-jetson/
git clone https://github.com/juliagoda/CH341SER.git

cd CH341SER
make
sudo make load
ls
sudo adduser $USER dialout
sudo usermod -a -G dialout jlukas 
```
At this point you've installed the `usb driver` successfully

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
sudo find / -name "*opencv*" -exec rm -rf {} \;
sudo find / -name "*opencv*" -exec rm -i {} \;
sudo find / -name "*opencv*" -exec rm -i {} \;

sudo find / -name "*opencv*" -exec rm -i {} \;

sudo apt-get purge '*opencv*'
sudo find / -name "*opencv*" -exec rm -rf {} \;
sudo find / -name "*opencv*";
sudo find / -name "*opencv*" -exec rm -rf {} \;
sudo apt remove libopencv-dev
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

## Step 8: To check our `lidar_slamtec` again - if it still available or not

run following command
```
cd ~
```
This will show you following directory our our previously created packages
```
lidar_slamtec
```
run our `lidar_ros` package again
```
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
```

The rviz should work again and our sensor.

Next you can stop and restart the container with no issue
