# Getting Started

To setup my docker environment. Follow the following

## This is `jetson nano` image for `ros-humble`

Check Link below
```
link : https://hub.docker.com/r/dustynv/ros/tags
```

Below is the image for `linux/arm64/v8`
```
Image : docker pull dustynv/ros:humble-desktop-l4t-r36.2.0
```

## This is `PC` image for `ros-humble`

Check Link below
```
link : https://hub.docker.com/r/osrf/ros/tags
```

Below is the image for `linux/amd64`
```
Image : docker pull osrf/ros:humble-desktop-full
```
Now let's build our custom image either using jetson nano or pc. Follow below step.
For Jetson Nano, you will have to build your docker using specific file which i've prepared below in my repo.
```
Docker_For_Robotics
├── my_code/source
├── ros2_humble_docker
└── ros2_humble_jetson_nano_docker (Use this one for Jetson Nano)
```
You just need to build the docker as follow for jetson nano
```
docker image build -t my_image .
```
For PC, follow the step below.

## Step 1: Create `My_Project` for us to build `my_image`
```
cd /home/jlukas/Desktop/My_Project/Docker_For_Robotics/ros2_humble_custom_docker/
mkdir ros2_humble_docker
```

Inside this directory, it should consist the following.
This is the directory you use to build the `my_image`.
```
.
├── config
│   └── my_config.yaml
├── docker_command.md
├── Dockerfile
└── entrypoint.sh
```

## Step 2: Build `my_image` using command
Run below command to build the `my_image`
```
docker image build -t my_image .
```

## Step 3: Mount Directory between `host` and `docker` mount

Below is the directory you will use to mount your `my_image`.

To create `mount` directory and please do this step in `host` pc. 
Follow below step.
```
cd /home/jlukas/Desktop/My_Project/Docker_For_Robotics/ros2_humble_custom_docker/
mkdir my_code
cd my_code 
```

Under `my_code` create `source` and `something.py` 
```
.
└── source
    └── something.py
```

## Step 4: Map `source` (host) to `my_souce_code` (container)

We're going to map following directory to our docker directory

Host PC
```
/home/jlukas/Desktop/My_Project/Docker_For_Robotics/ros2_humble_custom_docker/
└── my_code
    └── source <---------------- Map source to 1
        └── something.py
```

docker 
```
/my_source_code/ --------------> 1 my_source_cde
└── something.py
```

## Step 5: run docker command as `root` to start the mapping.

Inside `my_code` run following command.

Here we're running as root instead of `ros` user.
```
docker run -v <absol_path_on_host>:<absol_path_in_container>
```
Actual command as follow
```
docker run -it -v $PWD/source:/my_source_code my_image
```

You should be able to login successfully, and you should able to see `my_source_code` directory in the terminal.
This is directory mapped from `source` (host) to `my_source_code` (docker)

## Step 6 : Create file inside `my_source_code`

Run following command and ensure you're in `root`
```
cd my_source_code
touch new_file.yaml
```

If you open from `folder browser` you'll see this `new_file.yaml` file is locked and when
your run below command, You will see this file `new_file.yaml` is belong to `root`
```
ls -la
```

And you run below command. You'll see this file UID `1000`
```
ls -ln
```

## Step 7: Add USER for `1000` into dockerfile
To solve this problem, we need to `USER` with `ros` details in `dockerfile` as follow
```
# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config
```

## Step 8: run docker command with `ros` user
```
docker run -it --user ros -v $PWD/source:/my_source_code my_image
```
You should be able to login successfully with `ros` as a user, and you should able to see `my_source_code` directory in the terminal.
This is directory mapped from `source` (host) to `my_source_code` (docker) we did earlier.


## Step 9: Create new file inside `my_source_code`

Run following command and ensure you're in `ros` now as the user
```
cd my_source_code
touch a_new_file.yaml
```

If you open from `folder browser` you'll see this `a_new_file.yaml` file is no longer locked and when
your run below command, You will see this file `a_new_file.yaml` is belong to `ros` and also `root` as well
```
ls -la
```

And you run below command. You'll see this file UID `1000`
```
ls -ln
```


# This is docker command used in this tutorial

## To list Image
```
docker image ls
```

## To list Running Container
```
docker container ls
```

## To Start docker with interactive command
```
docker run -it ros:humble
```

## To Start docker with existing`id`
```
docker start <id>
```

## To Stop runnning docker with `id`
```
docker stop <id>
```

## To attach docker with running container
```
docker exec -it <id> bash
```

## To Start docker with image name and interactively
```
docker run -it -d osrf/ros:humble-desktop bin/bash
```

## To build docker with `my_image` name
```
docker build -t my_image .
```

## To Start docker with `my_image` and mount the volume interactively inside `/source:/my_source_code`

```
docker run -it --user ros -v $PWD/source:/my_source_code my_image
```

## To Start docker with `my_image` and mount the volume interactively inside `/source:/my_source_code` and share the `network` with local host

IPC stands for - Inter-Process Communication
```
docker run -it --user ros --network=host --ipc=host -v $PWD/source:/my_source_code my_image
```
You  will see now you terminal named as `ros@jlukas`


## To Start docker with `my_image` and mount the volume interactively inside `/source:/my_source_code` and share the `network` with local host and set the `Graphic` display to run `rviz2`
```
docker run -it --user ros --network=host --ipc=host -v $PWD/source:/my_source_code -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY my_image
```

## To delete docker image
```
docker image ls
docker rmi -f <image id>
```

# To access our `Steam Controller` and get it recognize by docker, follow below step

1. Plugin the receiver of `steam controller` into your host laptop
2. Inside `host` pc, open new terminal and install the following joystick command
```
sudo apt-get install joystick jstest-gtk evtest
```

and run `jstest-gtk` command to test our `controller`. It should work in the host pc.

3. Once, you confirm it's working, now run this command to see the `link` of device
```
ls -l /dev/input/by-id/
```

This should output the result as follow
```
jlukas@jlukas:~/Desktop/My_Project/Docker_Tutorial/My_Project$ ls -l /dev/input/by-id/
total 0
usb-SunplusIT_Inc_Integrated_Camera-event-if00 -> ../event8
usb-Valve_Software_Steam_Controller-event-mouse -> ../event18
usb-Valve_Software_Steam_Controller-if01-event-joystick -> ../event19
usb-Valve_Software_Steam_Controller-if01-joystick -> ../js0
usb-Valve_Software_Steam_Controller-mouse -> ../mouse4
usb-Wacom_Co._Ltd._Pen_and_multitouch_sensor-event-if00 -> ../event6
usb-Wacom_Co._Ltd._Pen_and_multitouch_sensor-if01-event-mouse -> ../event7
usb-Wacom_Co._Ltd._Pen_and_multitouch_sensor-if01-mouse -> ../mouse3
```

3. run below command to see the `major id` of our device
```
ls -l /dev/input
```

This should output result as follow. You will notice device `major id` is start at `13`. This number
indicate `input` and `output` device.

And you'll notice value of `32 --63`. This is `minor id` value.
```
crw-rw-r--+ 1 root input 13,  0 Apr   7 17:53 js0 <---------------
crw-rw----  1 root input 13, 63 Apr   7 15:44 mice
crw-rw----  1 root input 13, 32 Apr   7 15:44 mouse0
crw-rw----  1 root input 13, 33 Apr   7 15:44 mouse1
crw-rw----  1 root input 13, 34 Apr   7 15:44 mouse2
crw-rw----  1 root input 13, 35 Apr   7 15:44 mouse3
crw-rw----  1 root input 13, 36 Apr   7 17:52 mouse4
```

Now we're going to add all id with major id `13` from local host to container using below command in order to let docker recognize this device.
This will only map `/dev/input` with `c 13` ID
```
docker run -it --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /dev/input:/dev/input --device-cgroup-rule='c 13:* rmw' my_image
```

Run below command to test our `steam-controller` and `joystick gui` should appear. At this point our controller should work
```
jstest-gtk
```

To map everything in `/dev` and all `ID` with wildcard `*`. Run the following command
```
docker run -it --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /dev:/dev --device-cgroup-rule='c *:* rmw' my_image
```

Run below command to test our `steam-controller` and `joystick gui` should appear. At this point our controller should work again
```
jstest-gtk
```

You can see all the `devices` in our `docker containe` by running below command
```
ls /dev/input/by-id/
```
Result as follow
```
usb-SunplusIT_Inc_Integrated_Camera-event-if00
usb-Valve_Software_Steam_Controller-event-mouse
usb-Valve_Software_Steam_Controller-if01-event-joystick
usb-Valve_Software_Steam_Controller-if01-joystick
usb-Valve_Software_Steam_Controller-mouse
usb-Wacom_Co._Ltd._Pen_and_multitouch_sensor-event-if00
usb-Wacom_Co._Ltd._Pen_and_multitouch_sensor-if01-event-mouse
usb-Wacom_Co._Ltd._Pen_and_multitouch_sensor-if01-mouse
```

# To access our `Pico microcontroller` and overall `serial interface` and get it recognize by docker.

Here i add `privileged` into the command line
```
docker run -it --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /dev:/dev --privileged my_image
```

To see our `pico` port run following. Here `ACM0` is referring to our `pico`.
```
ls /dev/ttyACM0 
```

Then run below command to read from the `serial`.
```
pyserial-miniterm
```

And select index `1`. It should automatically recognize `Pico Microcontroller` port.

# To access specific port 

To access specific port let's say `lidar` or `arduino` port, write the following command. 
Here i specify `--device=/dev/ttyACM0` of `arduino` port
```
docker run -it --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY --device=/dev/ttyACM0 my_image
```

# To access specific port with `root dialout 166` which is referring to `ACM USB modems`. 

Write following command 
```
ls -l /dev/tty*
```
To access specific port let's say `lidar` or `arduino` port, write the following command. 
Here i specify `--device=/dev/ttyACM0` of `arduino` port
```
docker run -it --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY --device-cgroup-rule='c 166:1 rmw' my_image
```
# To start docker with our custom name
```
docker run -it --user ros --name lukas --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY --device-cgroup-rule='c 166:1 rmw' my_image
```

To stop and restart the docker 
```
sudo docker stop [name_of_container]
sudo docker restart [name_of_container]

sudo docker restart lukas
```

# To remove docker container
```
docker ps -a
docker rm -f f6e5466272a
```

# To remove unused docker container
```
docker container prune
```
