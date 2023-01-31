# Mission-Planning-with-Obstacle-Avoidance-and-Human-Detection-Algorithms

## PLEASE HAVE AN UBUNTU 20.04 or USE WSLG IN WINDOWS 11 AND DOWNLOAD UBUNTU 20.04 IN THE MICROSOFT STORE

# Installing Ardupilot and MAVProxy Ubuntu 20.04

## Clone ArduPilot

In home directory:
```
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

## Install dependencies:
```
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

reload profile
```
. ~/.profile
```

## Checkout Latest Copter Build
```
git checkout Copter-4.3.1
git submodule update --init --recursive
```

Run SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

# Installing Gazebo and ArduPilot Plugin


## Overview 

Robot simulation is an essential tool in every roboticist's toolbox. A well-designed simulator makes it possible to rapidly test algorithms, design robots, perform regression testing, and train AI system using realistic scenarios. Gazebo offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. At your fingertips is a robust physics engine, high-quality graphics, and convenient programmatic and graphical interfaces. Best of all, Gazebo is free with a vibrant community.

for more infromation on gazebo checkout http://gazebosim.org/

## Install Gazebo [***20.04***]

Setup your computer to accept software from http://packages.osrfoundation.org:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Reload software list:
```
sudo apt update
```

Install Gazebo:
### Ubuntu [***20.04***]
```
sudo apt-get install gazebo11 libgazebo11-dev
```

for more detailed instructions for installing gazebo checkout http://gazebosim.org/tutorials?tut=install_ubuntu


## Install Gazebo plugin for APM (ArduPilot Master) :
```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
```

build and install plugin
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```
Set paths for models:
```
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

## Run Simulator

**NOTE the iris_arducopter_runway is not currently working in gazebo11. The sim worlds DO work**

In one Terminal (Terminal 1), run Gazebo:
```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

In another Terminal (Terminal 2), run SITL:
```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
ctrl + c to exit the program


# Install ROS and Setup Catkin  

## 1. Install ROS

   - Do _Desktop-full Install_
   - Follow until _Step 1.7_ at the end of the page

   First, install **ROS Noetic** using the following instructions: http://wiki.ros.org/noetic/Installation/Ubuntu


## 2. Set Up Catkin workspace

We use `catkin build` instead of `catkin_make`. Please install the following:
```
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon
```

Then, initialize the catkin workspace:
```
cd "folder where you cloned the ardupilot"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

## 3. Dependencies installation

Install `mavros` and `mavlink` from source:
```
cd ~/catkin_ws
wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```
Add a line to end of `~/.bashrc` by running the following command:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

update global variables
```
source ~/.bashrc
```

install geographiclib dependancy 
```
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```


## 4. Clone Simulation ROS package 

```
cd ~/catkin_ws/src
git clone https://github.com/Krome00/sims.git
```
Our repository should now be copied to `~/catkin_ws/src/sims/` (don't run this line. This is just saying that if you browse in the file manager, you will see those folders).

run the following to tell gazebo where to look for the sims models 
```
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/sims/models" >> ~/.bashrc
```

## 5. Build instructions
Inside `catkin_ws`, run `catkin build`:

```
cd ~/catkin_ws
catkin build
```
update global variables
```
source ~/.bashrc
```
## Make sure Install ROS plugins for Gazebo:
```
sudo apt install ros-noetic-gazebo-ros ros-noetic-gazebo-plugins
```
## copy and paste this to the terminal to use my script on using sitl
```
cp ~/catkin_ws/src/sims/scripts/startsitl.sh ~
```

# Cloning Guidance Navigation and Control

## Make sure you have a text editor
Please make sure you have a text editor. Our prefered text editor is sublime. You can download it by running the below commands
```
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text
```

## Clone the GNC ROS package
```
git clone https://github.com/Krome00/gnc.git

```
## Build Repository
```
cd ~/catkin_ws
catkin build
source ~/.bashrc
```

# Gazebo World Modeling

## Add the Open Gazebo Models Database

Use git to get a bunch of open source gazebo models from the Open Source Robotics Foundation (OSRF) 

```
git clone https://github.com/osrf/gazebo_models.git
```
Add Models path to the bashrc
```
echo 'export GAZEBO_MODEL_PATH=~/gazebo_ws/gazebo_models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
source ~/.bashrc
```

# Installing YOLO/Darknet Image Recognition 


## Install CUDA 
Cuda is a library that allows programs to take advantage of your GPU as a computing resource. YOLO will run without Cuda, but the algorithm is up to 500 x more quick with Cuda. To install Cuda, run 

```
sudo apt install nvidia-cuda-toolkit
```

## **Ubuntu 20.04**
### Clone Darknet/YOLO 
```
cd ~/catkin_ws/src
git clone https://github.com/kunaltyagi/darknet_ros.git
cd darknet_ros
git checkout opencv4
git submodule update --init --recursive
```
### Build Darknet 
```
catkin build -DCMAKE_BUILD_TYPE=Release 
```
if you run into errors try running the following 
```
catkin build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-8
```


## Configure YOLO/Darknet
## PLEASE IN THIS CODE CHANGE YOLO VERSION TO YOLOV3, FOLLOW INSTRUCTIONS

in the file `ros.yaml` specifies ros parameters. You can find this file under `darknet_ros/darknet_ros/config`. You will need to change the image topic from `/camera/rgb/image_raw` to 

```
/webcam/image_raw
```

The file `darknet_ros.launch` will launch the darknet/yolo ros node. You can find this file under `darknet_ros/darknet_ros/launch`

in this file you can choose which version of yolo you would like to run by changing
```
<arg name="network_param_file"         default="$(find darknet_ros)/config/yolov2-tiny.yaml"/>
```
the options are as follows

- yolov1: Not recommended. this model is old 
- yolov2: more accurate, and faster. 
- yolov3: about as fast as v2, but more accurate. Yolo v3 has a high GPU ram requirement to train and run. If your graphics card does not have enough ram, use yolo v2 
- tiny-yolo: Very fast yolo model. Would recommend for application where speed is most important. Works very well on Nvidia Jetson

---
### References 

```
https://pjreddie.com/darknet/yolo/
```

# NOW WE WILL LAUNCH THE PROGRAMS FOR THE SIMULATION

## Terminal 1
```
roslaunch sims testing.world
```
## Terminal 2
```
./startsitl.sh
```
## Terminal 3
```
roslaunch sims apm.launch
```
## Terminal 4
```
roslaunch darknet_ros darknet_ros.launch
```
## Terminal 5
```
rosrun gnc avoidance
```
## Terminal 6
```
rosrun gnc sub
```
The Last terminal is a script that tells the location of the person when the camera of the drone detects them.
