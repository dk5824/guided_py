# ArduPilot_MAVROS
the ROS Package to control UAV in **ArduPilot GUIDED Mode** using **MAVROS**

this package is written using **Python**

System Environment:
Ubuntu 20.04 / ROS Noetic / Gazebo 11 / ArduCopter v 4.3.0


## Install ArduPilot and Gazebo Plugin

### Install ArduPilot Firmware
```bash
git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
```
### Install ArduPilot - Gazebo Plugin
```bash
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

### Solve Python Dependency Error(up to Ububtu 20.04)
```bash
cd ardupilot/modules/waf
gedit waf-light.py
```

on the top of the source, change #!usr/bin/env/python to #!usr/bin/env/**python3**.

```bash
cd ardupilot/Tools/autotest
gedit sim_vehicle.py
```

similarly, in this source you should change the line..

### install extend package
first, install pip tools.

```bash
sudo apt update
sudo apt install python3-pip
```

and using pip tools, install some packages.

```bash
pip3 install pymavlink

sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
pip3 install PyYAML mavproxy --user
echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
```

## Install ROS and MAVROS

### Install ROS Noetic
- ROS Installation: [ROS WIKI ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu)

### Install MAVROS
```bash
sudo apt-get update
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

#install geographicdatasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

##  Build Environment Composition

### Install catkin tools

```bash
sudo apt update
sudo apt install python3-osrf-pycommon
sudo apt install python3-catkin-tools
```

### Create Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd .. #move to ~/catkin_ws
catkin_make
source devel/setup.bash

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### Pull This Repository


```bash
git clone https://github.com/dk5824/ArduPilot_MAVROS
cd ArduPilot_MAVROS
mv guided_py ~/catkin_ws/src #move guided_py package to your workspace..

# grant execute permission
cd ~/catkin_ws/src/guided_py/scripts
chmod +x test_callback_functions.py
chmod +x test_local_servoing.py

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```


## Launch Simulation

### Terminal 1 : Launch Gazebo Simulation
```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```

### Terminal 2 : Run ArduPilot SITL
```bash
cd ardupilot/ArduCopter
python3 ../Tools/autotest/sim_vehicle.py -f gazebo-iris --console
```

### Terminal 3 : Launch MAVROS Node
```bash
roslaunch mavros apm.launch fcu_url:=udp://:14550@
```

### Terminal 4 : Run guided_py Node (callback test)
```bash
cd ~/catkin_ws
source devel/setup.bash

rosrun guided_py test_callback_functions.py
```

## Result
![Result](https://img1.daumcdn.net/thumb/R1280x0/?scode=mtistory2&fname=https%3A%2F%2Fblog.kakaocdn.net%2Fdn%2F92AMQ%2FbtrValB3ztY%2FK5KgEmUWlA3qdtpmlnAZW1%2Fimg.png)


## Result for running test_callback_functions node
![Result](https://img1.daumcdn.net/thumb/R1280x0/?scode=mtistory2&fname=https%3A%2F%2Fblog.kakaocdn.net%2Fdn%2Fdsvs4X%2FbtrVfrBphav%2FjvKgE5yt48aE5Cko5Wc5J1%2Fimg.png)


## Links for overall test video about this package (click this thumbnail!)
[![Video Label](http://img.youtube.com/vi/r29FpbyiQU0/0.jpg)](https://www.youtube.com/watch?v=r29FpbyiQU0)
